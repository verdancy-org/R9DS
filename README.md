# R9DS

FrSky R9DS SBUS receiver input and RC processing module.

## Required Hardware

- `sbus_uart`
- `ramfs`

## Constructor Arguments

- `data_topic_name`: `r9ds_data`
- `rc_state_topic_name`: `rc_state`
- `signal_timeout_ms`: `50`
- `task_stack_depth`: `1024`

## Outputs

- `R9DS::Data`
- `R9DS::State`

## Notes

- Decodes SBUS frames from the R9DS receiver.
- Publishes both raw channel timing and normalized RC state used by the
  upper-level flight logic.
