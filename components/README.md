# tilt_cover — ESPHome external component

An external component version of a tilt-blind controller originally written
as inline `globals:` + `script:` + `lambda:` blocks. Targets ESPHome
2026.7.0 conventions (entity `*_schema()` helpers, `cover.new_cover`,
`cg.register_component`, etc.).

## Layout

```
components/tilt_cover/
├── __init__.py        # tilt_cover: hub (pins, angle sensor ref, glue)
├── cover.py            # cover: platform tilt_cover
├── binary_sensor.py     # binary_sensor: platform tilt_cover (Calibrated)
├── sensor.py            # sensor: platform tilt_cover (Calibrate Step)
├── text_sensor.py       # text_sensor: platform tilt_cover (Calibrate Message)
├── switch.py            # switch: platform tilt_cover (use_angle_sensor / has_problem)
├── button.py            # button: platform tilt_cover (Calibrate)
├── tilt_cover.h
└── tilt_cover.cpp
example.yaml            # full working example, drop-in replacement for the original
```

Drop the `components/` folder into your ESPHome config directory (or a
GitHub repo) and reference it via `external_components:` — see
`example.yaml`.

## How it maps to the original script

| Original YAML                                   | Now lives in                                   |
|--------------------------------------------------|-------------------------------------------------|
| `globals:` (pos_low, pos_high, step_total, ...)  | private members of `TiltCover` (tilt_cover.h)   |
| `binary_sensor: ph_a` / `ph_b` (unnamed)         | polled directly in `TiltCover::loop()`          |
| `switch: cw` / `ccw` (unnamed)                   | `cw_pin_` / `ccw_pin_` driven directly, no entity |
| `sensor: rotary_sensor` (unnamed)                | `TiltCover::compute_rotary_value_()`            |
| `sensor: gyro` / `accel_x` filter lambda          | `TiltCover::compute_angle_value_()`             |
| `binary_sensor: calibrated`                      | `binary_sensor: platform: tilt_cover`           |
| `sensor: calibrate_step`                          | `sensor: platform: tilt_cover`                  |
| `text_sensor: calibrate_msg`                      | `text_sensor: platform: tilt_cover`             |
| `switch: Use Angle Sensor` / `Has Problem`        | `switch: platform: tilt_cover` (`type:`)        |
| `button: Calibrate`                               | `button: platform: tilt_cover`                  |
| `cover: mycover`                                  | `cover: platform: tilt_cover`                   |
| `interval: 0.1s` block                            | `TiltCover::loop()` (100 ms gate via `millis()`) |
| `script: switch_on`                               | `TiltCover::start_motor_()`                      |
| `script: initialize`                              | `TiltCover::initialize_()` (called from `setup()`) |
| `script: rotary_update`                           | removed, see below                               |

## Deliberate changes from the original

- **Mode selection.** The original decided encoder-vs-angle mode at boot by
  checking whether the MPU6050 I2C component had failed
  (`get_component_state() == COMPONENT_STATE_FAILED`), timed via an
  `on_boot: priority: -100` hook. This version just checks whether you gave
  it an `angle_sensor:` id in YAML. Simpler, and doesn't depend on boot
  ordering. You can still flip modes at runtime with the "Use Angle Sensor"
  switch.
- **`script: rotary_update` is gone.** It existed to speed up the MPU6050's
  polling interval to 100 ms while the motor was moving, and slow it back
  down afterwards. Since I2C reads are cheap and don't wear anything out,
  the example config just sets `update_interval: 100ms` on the `mpu6050`
  sensor directly — no dynamic interval juggling needed.
- **`ph_a` / `ph_b` and `cw` / `ccw` are no longer separate entities.**
  They never had a `name:` in the original config either (i.e. they were
  already HA-invisible), so folding them into the hub's internal pin
  handling removes four component instances without losing any exposed
  functionality.
- **Calibration values only persist once actually calibrated.** Defaults
  are `0`, not the original's arbitrary `-8` / `10` guesses, since those
  were never meant to be used before the first calibration pass anyway.

## Before you flash

- Update `CODEOWNERS` and the `url:` in `example.yaml` if you're publishing
  this to your own GitHub repo.
- This hasn't been compiled against a real ESPHome checkout (no network
  access in this environment) — please run `esphome config example.yaml`
  and `esphome compile example.yaml` before flashing, and treat this as a
  strong first draft rather than field-tested code. The pieces most worth
  double-checking against your installed ESPHome version: `cover.cover_schema`,
  `switch.switch_schema`, and `button.button_schema` signatures (these
  entity-schema helpers have changed shape a few times across ESPHome
  releases).
