[Русская версия](README.ru.md)

# jalouzee_blinds — an ESPHome external component

The component implements a `cover` platform for controlling the tilt angle
of venetian blind slats with a DC motor, with automatic detection of the
current angle from one of three sources: **MPU6050** (accelerometer), a
**Hall sensor** (encoder on the motor shaft), or a **resistor
(potentiometer)** on the gearbox's output shaft.

Supported platforms: **ESP32** and **ESP8266**.

## What you need

- ESP32 or ESP8266.
- Any DC motor with a gearbox and an H-bridge/driver on 2 digital pins (full
  speed, no PWM) — the specific motor model doesn't matter, the component
  isn't tied to any one of them. The examples below use the GA12-N20 simply
  as a common, cheap option, but pretty much any gearmotor with a 2-pin
  driver will work.
- One angle source (you can use several at once):
  - **MPU6050** over I2C — used as a ready-made `sensor` (a specific
    accelerometer axis, or your own template sensor with an already-computed
    angle);
  - a **Hall sensor** on the motor shaft — 2 digital pins (A/B). Usually
    comes built into the motor (e.g. the GA12-N20 in its "with encoder"
    variant);
  - a **resistor (potentiometer)** on the gearbox's output shaft (between
    the motor and the coupling that holds the blinds' tilt rod) — 1 ADC pin.
    The full "closed↔open" travel on this shaft is less than one full turn,
    so a regular single-turn potentiometer with a mechanical stop works
    fine.

  The Hall sensor and the resistor are mutually exclusive variants of the
  same "encoder" measurement approach (you can't specify both at once);
  MPU6050 can be used alongside either of them as a backup/primary source.

### Hardware examples

| Source | Example |
|---|---|
| MPU6050 (ready-made GY-521 module) | ![MPU6050](pics/mpu6050.png?raw=true) |
| GA12-N20 with a built-in Hall sensor | ![GA12-N20](pics/ga12-n20-hall.png?raw=true) |
| GA12-N20 without a Hall sensor + potentiometer on the output shaft | ![GA12-N20](pics/ga12-n20-potentiometer.png?raw=true) |

## Installation

Copy the `components/jalouzee_blinds` folder into your ESPHome project's
`external_components` (see `example/example.yaml`), or reference the
repository directly via `external_components: - source: github://...`.

## Example configuration

```yaml
esp32:
  board: esp32dev
  framework:
    type: esp-idf

external_components:
  - source:
      type: git
      url: https://github.com/shammysha/esphome-jalouzee-blinds/
    refresh: 1s

i2c:
  sda: GPIO21
  scl: GPIO22

sensor:
  - platform: mpu6050
    address: 0x68
    accel_x:
      id: acc_x
    update_interval: 100ms

cover:
  - platform: jalouzee_blinds
    id: blinds
    name: "Living Room Blinds"
    motor:
      in1: GPIO25
      in2: GPIO26
    encoder:
      # OPTION 1: Hall sensor on the motor shaft
      a: GPIO32
      b: GPIO33
      # OPTION 2 (mutually exclusive with a/b): resistor on the motor shaft
      # adc: GPIO34
    angle: acc_x
    position: auto   # auto | angle | encoder
    fault_timeout: 10s
```

## Configuration parameters

| Parameter | Required | Description |
|---|---|---|
| `motor.in1`, `motor.in2` | yes | two digital output pins to the motor driver |
| `encoder.a`, `encoder.b` | no* | Hall sensor pins (both together) |
| `encoder.adc` | no* | ADC pin of the resistor (mutually exclusive with `a`/`b`). Valid pins depend on the platform (e.g. on ESP8266 it may only be `A0`/`GPIO17`) — an invalid pin will already error out at compile time |
| `angle` | no* | id of an already-configured `sensor` (an MPU6050 accelerometer axis, or a ready-made angle computed by a separate sensor) |
| `position` | no, defaults to `auto` | `auto` / `angle` / `encoder` — which source to use (see "Angle source priority" below) |
| `fault_timeout` | no, defaults to `10s` | how many seconds the angle may stay unchanged while the motor is actively moving before a fault is declared (1s to 300s) |

\* you need to specify the `encoder:` block and/or the `angle:` parameter —
at least one angle source is required.

## Automatically created entities

None of these need to be (or can be) declared in YAML — the component
creates them itself on startup:

| Entity | Type | Purpose |
|---|---|---|
| `<name> Calibration` | button | calibration steps — see below |
| `<name> Cancel Calibration` | button | cancels calibration if it's currently running |
| `<name> Reset Fault` | button | clears the fault state |
| `<name> Angle Source` | select | `auto` / `angle` / `encoder` — same as `position` in YAML, but changeable on the fly |
| `<name> Fault Timeout (s)` | number | same as `fault_timeout`, editable on the fly (1–300) |
| `<name> Calibration Message` | text_sensor | a text hint for each calibration step |
| `<name> Calibrated` | binary_sensor | on if at least one source is calibrated |
| `<name> Fault` | binary_sensor | on while a fault is active (see "Fault Timeout" above) |
| `<name> Hall Calibrated` | binary_sensor (diagnostic) | only if `encoder.a`/`b` is configured — calibration status of that specific source |
| `<name> ADC Calibrated` | binary_sensor (diagnostic) | only if `encoder.adc` is configured — calibration status of that specific source |
| `<name> Angle Calibrated` | binary_sensor (diagnostic) | only if `angle` is configured — calibration status of that specific source |

## How to calibrate

1. Press the **"Calibration"** button.
2. Using the regular ▲/▼ arrows on the cover card in Home Assistant, move
   the slats to the fully **closed** position (overshooting and correcting
   with the other arrow is fine — only the point at the moment of the next
   press matters). Press **"Calibration"** again.
3. Likewise, move the slats to the fully **open** position and press
   **"Calibration"** again.
4. Calibration is complete. The "Calibration Message" sensor will show a
   confirmation for a few seconds.

Calibration can be interrupted at any time with the **"Cancel Calibration"**
button — blind control returns to normal mode without any changes.

Calibration captures points for **all connected sources** at once — i.e. if
you have both Hall and MPU6050 configured, a single pass calibrates both.

## Control

- Regular open/close (arrows in HA, or voice commands like "open the
  blinds") move the blinds through three fixed positions: **Closed → 50% →
  Open** (and back), one step per press. This also applies to the main
  position slider dragged to exactly 0% or 100% — see "Position vs. tilt"
  below for why.
- The **tilt** control (a separate slider/dial in HA, since this cover
  supports both position and tilt) always moves directly to the exact
  percentage requested, with no stepping — use it for precise intermediate
  angles.
- The Stop button halts movement immediately.

---

## Behavior details

### Angle source priority (`auto` mode)

1. **`angle`** (e.g. MPU6050) — if configured, calibrated, and has fresh
   readings.
2. Otherwise — the **Hall sensor or resistor** (whichever is set in
   `encoder:`), if calibrated.

If no source is calibrated and available — blind control is fully blocked
(buttons and the slider don't work; the arrows are shown in HA, but the
command is rejected) until calibration is performed.

The `angle` and `encoder` modes force the use of only that specific source
(no automatic fallback to the other one, except for the case described
below under "Power loss behavior").

### When a source's calibration is accepted, and when it isn't

When capturing calibration points, each source is checked separately for
**actual movement** between "closed" and "open" (a sufficiently large
difference in raw readings). If the difference is too small for a specific
source — the sensor most likely didn't move, or is physically disconnected
— that source remains **uncalibrated**, even if the other sources
calibrated successfully. Nothing extra needs to be pressed: that source's
calibration simply won't be accepted, and it won't be used until it's
calibrated correctly.

### Automatic re-calibration

If, during normal (non-calibration) operation, the blinds actually reach 0%
or 100% per an already-trusted source, the component opportunistically
captures the current raw value of any **other available but not-yet-
calibrated** source. As soon as both endpoints have accumulated for it, it
automatically becomes calibrated, without a repeat manual calibration pass.

Practical example: if the MPU6050 was physically disconnected from the
slats during calibration (and therefore didn't calibrate — see the point
above), once it's reconnected, a couple of regular open/close cycles are
enough for it to calibrate itself.

### Power loss behavior

The component precisely tracks whether motor movement was interrupted by a
power loss (as opposed to a normal stop — reaching the target, the Stop
button, a fault, or entering calibration).

- If movement was **interrupted** by a power loss and the **Hall sensor**
  is in use as the source — its current position is considered
  untrustworthy (the encoder is cumulative, not absolute, and can't confirm
  on its own that nothing changed while power was off). In this case:
  - if **MPU6050** is available and calibrated — the component temporarily
    (for this session, without changing saved settings) switches to it;
  - otherwise — blind control is **blocked** until manual calibration.
- If movement wasn't interrupted (a clean reboot) — the Hall position is
  restored from the last saved value, and nothing else needs to be done.
- The **resistor (ADC)** doesn't depend on this protection at all — it's an
  absolute sensor (the current voltage IS the current position right now),
  so it's always trusted right after power-up, regardless of whether
  movement was interrupted.

### Fault ("stuck" sensor)

While the motor is actively moving toward a target, the component checks
that the measured angle is actually changing. If the angle doesn't change
for longer than `fault_timeout` seconds — this is treated as a fault: the
motor stops, the "Fault" binary sensor turns on, and further blind control
is blocked until the "Reset Fault" button is pressed.

This protection **doesn't apply during calibration** — movement there is
manually controlled by the user via the jog buttons.

### Three fixed positions

The regular open/close arrows always move the blinds in steps between three
positions — 0% / 50% / 100%, one step per press, regardless of where the
blinds currently are between steps. Setting an arbitrary percentage via the
**tilt** control (see "Position vs. tilt" below) moves the blinds straight
to that point and updates the "current step" accordingly for subsequent
arrow presses.

This stepping isn't just a convenience — for a full ~180° tilt mechanism,
both 0% and 100% are physically **closed** (slats rotated to opposite
extremes), with the genuinely open, light-passing state at ~50% in between.
Moving straight from 0% to 100% would sweep through the open state and end
up closed again on the other side — the opposite of what "open the blinds"
should do. Stepping one position at a time guarantees an open/close command
always lands on the correct state.

### Position vs. tilt

Home Assistant's cover domain doesn't let a component tell an open/close
arrow tap, a voice command ("open the blinds"), and the main position
slider dragged to exactly 0%/100% apart — all of them arrive identically as
`cover.set_cover_position` with position 1.0 or 0.0 (this is true even for
the arrows and voice commands specifically — see
`homeassistant/components/esphome/cover.py`, `async_open_cover`/
`async_close_cover` always call `cover_command(position=1.0/0.0)`
regardless of what the entity supports). That's why the position slider
uses the stepped closed→50%→open logic described above at its extremes —
it has to, since there's no way to know whether a position=1.0 call came
from an arrow or from the slider itself.

The **tilt** control is a separate HA service
(`cover.set_cover_tilt_position`), sent over a distinct field from
position, and can never originate from an open/close arrow or voice
command. So it always moves directly to the requested percentage, with no
stepping — use it when you need a precise angle, especially at/near the
0%/100% extremes. Both controls drive the exact same underlying angle; they
just differ in how 0%/100% requests are handled.

### What gets saved, and how

Flash stores: each source's calibration data, the selected angle-source
mode, and the last known position. Fault status is **intentionally not
saved** — after a reboot, a fault is always cleared.

The position is saved immediately upon reaching a movement target, and no
more than once every 5 minutes while idle (if it has changed noticeably) —
deliberately infrequent, since this project doesn't assume manual
intervention with the slat position outside of the component's own
commands.

### Publishing position to Home Assistant

During movement, the position update in the UI happens no more than once a
second (the final value is published immediately upon stopping, with no
delay) — this protects against overloading the Home Assistant connection
with frequent updates and doesn't affect control accuracy itself.

During calibration, the UI forcibly shows 50% (so both ▲/▼ arrows stay
active for jogging) — the real position is restored immediately once
calibration finishes or is cancelled.
