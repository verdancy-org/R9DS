#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: FrSky R9DS SBUS receiver input module
constructor_args:
  - data_topic_name: "r9ds_data"
  - signal_timeout_ms: 50
  - task_stack_depth: 1024
template_args: []
required_hardware:
  - sbus_uart
  - ramfs
depends:
  - AnotcCommon
=== END MANIFEST === */
// clang-format on

#include "AnotcCommon.hpp"
#include "app_framework.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>

class R9DS : public LibXR::Application {
 public:
  R9DS(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
       const char* data_topic_name, uint32_t signal_timeout_ms,
       size_t task_stack_depth)
      : signal_timeout_ms_(signal_timeout_ms),
        topic_(data_topic_name, sizeof(data_)),
        uart_(hw.template FindOrExit<LibXR::UART>({"sbus_uart"})),
        cmd_file_(LibXR::RamFS::CreateFile("r9ds", CommandFunc, this)) {
    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    auto ans = uart_->SetConfig({100000, LibXR::UART::Parity::EVEN, 8, 2});
    ASSERT(ans == LibXR::ErrorCode::OK);

    thread_.Create(this, ThreadFunc, "r9ds_thread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  void OnMonitor() override {
    if (data_.no_signal) {
      XR_LOG_WARN("R9DS: No valid frame.");
    }
  }

 private:
  static constexpr std::array<uint8_t, 4> FRAME_END = {0x04, 0x14, 0x24, 0x34};

  bool ParseByte(uint8_t byte) {
    const uint32_t now_us =
        static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
    if ((now_us - last_byte_time_us_) > 2500U) {
      frame_count_ = 0;
    }
    last_byte_time_us_ = now_us;

    frame_buffer_[frame_count_++] = byte;

    if (frame_count_ < frame_buffer_.size()) {
      return false;
    }

    frame_count_ = frame_buffer_.size() - 1;

    const bool valid_frame =
        frame_buffer_[0] == 0x0F &&
        (frame_buffer_[24] == 0x00 ||
         frame_buffer_[24] == FRAME_END[frame_end_index_]);

    if (!valid_frame) {
      for (size_t i = 0; i < frame_buffer_.size() - 1; ++i) {
        frame_buffer_[i] = frame_buffer_[i + 1];
      }
      return false;
    }

    DecodeFrame();
    frame_count_ = 0;
    frame_end_index_ = (frame_end_index_ + 1U) % FRAME_END.size();
    return true;
  }

  void DecodeFrame() {
    uint16_t raw[16] = {};
    raw[0] = (static_cast<uint16_t>(frame_buffer_[2] & 0x07) << 8) |
             frame_buffer_[1];
    raw[1] = (static_cast<uint16_t>(frame_buffer_[3] & 0x3F) << 5) |
             (frame_buffer_[2] >> 3);
    raw[2] = (static_cast<uint16_t>(frame_buffer_[5] & 0x01) << 10) |
             (static_cast<uint16_t>(frame_buffer_[4]) << 2) |
             (frame_buffer_[3] >> 6);
    raw[3] = (static_cast<uint16_t>(frame_buffer_[6] & 0x0F) << 7) |
             (frame_buffer_[5] >> 1);
    raw[4] = (static_cast<uint16_t>(frame_buffer_[7] & 0x7F) << 4) |
             (frame_buffer_[6] >> 4);
    raw[5] = (static_cast<uint16_t>(frame_buffer_[9] & 0x03) << 9) |
             (static_cast<uint16_t>(frame_buffer_[8]) << 1) |
             (frame_buffer_[7] >> 7);
    raw[6] = (static_cast<uint16_t>(frame_buffer_[10] & 0x1F) << 6) |
             (frame_buffer_[9] >> 2);
    raw[7] = (static_cast<uint16_t>(frame_buffer_[11]) << 3) |
             (frame_buffer_[10] >> 5);
    raw[8] = (static_cast<uint16_t>(frame_buffer_[13] & 0x07) << 8) |
             frame_buffer_[12];
    raw[9] = (static_cast<uint16_t>(frame_buffer_[14] & 0x3F) << 5) |
             (frame_buffer_[13] >> 3);
    raw[10] = (static_cast<uint16_t>(frame_buffer_[16] & 0x01) << 10) |
              (static_cast<uint16_t>(frame_buffer_[15]) << 2) |
              (frame_buffer_[14] >> 6);
    raw[11] = (static_cast<uint16_t>(frame_buffer_[17] & 0x0F) << 7) |
              (frame_buffer_[16] >> 1);
    raw[12] = (static_cast<uint16_t>(frame_buffer_[18] & 0x7F) << 4) |
              (frame_buffer_[17] >> 4);
    raw[13] = (static_cast<uint16_t>(frame_buffer_[20] & 0x03) << 9) |
              (static_cast<uint16_t>(frame_buffer_[19]) << 1) |
              (frame_buffer_[18] >> 7);
    raw[14] = (static_cast<uint16_t>(frame_buffer_[21] & 0x1F) << 6) |
              (frame_buffer_[20] >> 2);
    raw[15] = (static_cast<uint16_t>(frame_buffer_[22]) << 3) |
              (frame_buffer_[21] >> 5);

    data_.flags = frame_buffer_[23];
    data_.fail_safe = (data_.flags & 0x08U) != 0U;
    for (size_t i = 0; i < data_.channels_us.size(); ++i) {
      data_.channels_us[i] = static_cast<int16_t>(
          0.644f * (static_cast<float>(raw[i]) - 1024.0f) + 1500.0f);
    }

    const uint32_t now_ms =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if (!data_.fail_safe) {
      last_good_frame_ms_ = now_ms;
      data_.no_signal = false;
      frame_counter_window_++;
    }

    if (now_ms - freq_window_start_ms_ >= 1000U) {
      data_.signal_frequency_hz = frame_counter_window_;
      frame_counter_window_ = 0;
      freq_window_start_ms_ = now_ms;
    }

    if (data_.fail_safe &&
        (now_ms - last_good_frame_ms_ >= signal_timeout_ms_)) {
      data_.no_signal = true;
    }

    topic_.Publish(data_);
  }

  void HandleSignalTimeout() {
    const uint32_t now_ms =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if ((now_ms - last_good_frame_ms_) < signal_timeout_ms_) {
      return;
    }

    if (!data_.no_signal) {
      data_.no_signal = true;
      data_.fail_safe = true;
      data_.signal_frequency_hz = 0;
      data_.channels_us.fill(0);
      topic_.Publish(data_);
    }
  }

  static void ThreadFunc(R9DS* r9ds) {
    LibXR::Semaphore read_sem;
    uint8_t byte = 0;
    r9ds->freq_window_start_ms_ =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    r9ds->last_good_frame_ms_ = r9ds->freq_window_start_ms_;

    while (true) {
      LibXR::ReadOperation op(read_sem, 10);
      auto ans = r9ds->uart_->Read({&byte, 1}, op);
      if (ans == LibXR::ErrorCode::OK) {
        (void)r9ds->ParseByte(byte);
      } else {
        r9ds->HandleSignalTimeout();
      }
    }
  }

  static int CommandFunc(R9DS* r9ds, int argc, char** argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf(
          "  show [time_ms] [interval_ms] - Print the first 10 R9DS channels.\r\n");
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "show") == 0) {
      int time_ms = std::atoi(argv[2]);
      int interval_ms = std::atoi(argv[3]);
      interval_ms = std::clamp(interval_ms, 10, 1000);

      while (time_ms > 0) {
        LibXR::STDIO::Printf(
            "R9DS: %d %d %d %d %d %d %d %d %d %d | freq=%u | flags=0x%02X | fs=%d | no_signal=%d\r\n",
            r9ds->data_.channels_us[0], r9ds->data_.channels_us[1],
            r9ds->data_.channels_us[2], r9ds->data_.channels_us[3],
            r9ds->data_.channels_us[4], r9ds->data_.channels_us[5],
            r9ds->data_.channels_us[6], r9ds->data_.channels_us[7],
            r9ds->data_.channels_us[8], r9ds->data_.channels_us[9],
            r9ds->data_.signal_frequency_hz, r9ds->data_.flags,
            static_cast<int>(r9ds->data_.fail_safe),
            static_cast<int>(r9ds->data_.no_signal));
        LibXR::Thread::Sleep(interval_ms);
        time_ms -= interval_ms;
      }
      return 0;
    }

    LibXR::STDIO::Printf("Error: Invalid arguments.\r\n");
    return -1;
  }

  uint32_t signal_timeout_ms_ = 50;
  uint32_t last_byte_time_us_ = 0;
  uint32_t last_good_frame_ms_ = 0;
  uint32_t freq_window_start_ms_ = 0;
  uint16_t frame_counter_window_ = 0;
  size_t frame_count_ = 0;
  size_t frame_end_index_ = 0;
  std::array<uint8_t, 25> frame_buffer_ = {};
  Anotc::R9DSData data_;
  LibXR::Topic topic_;
  LibXR::UART* uart_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
