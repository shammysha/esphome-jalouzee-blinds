#pragma once

#include <cstdint>

namespace esphome {
namespace jalouzee_blinds {

// Data persisted to flash (NVS). Fault status is NOT included here — it
// should not survive a reboot.
struct JalouzeeBlindsStore {
  bool hall_calibrated;
  bool adc_calibrated;
  bool mpu_calibrated;
  // true between the start of an actual motor movement and its normal
  // completion (target reached / explicit stop / fault / entering
  // calibration) — if this flag is still true on the next boot, the
  // movement was interrupted by a power loss, and the Hall position cannot
  // be trusted (see JalouzeeBlinds::setup()).
  bool movement_in_progress;

  float hall_closed;  // "raw" accumulated pulse count with slats closed
  float hall_open;
  float adc_closed;  // "raw" ADC value with slats closed
  float adc_open;
  float mpu_closed;  // MPU6050 sensor value with slats closed
  float mpu_open;

  uint8_t angle_source_mode;  // user's choice: auto/angle/encoder
  float last_angle_percent;   // last known tilt angle, 0..100%
} __attribute__((packed));

// Detected motor wiring polarity (in1/in2 possibly swapped at installation —
// see JalouzeeBlinds::detect_motor_polarity_()). Deliberately its own tiny
// preference object, separate from JalouzeeBlindsStore above: adding a field
// to JalouzeeBlindsStore changes its size, which invalidates ESPHome's flash
// preference blob checksum and silently wipes ALL existing hall/adc/mpu
// calibration on every already-deployed blind (see
// JalouzeeBlinds::load_from_flash_()) — not an acceptable cost for a feature
// this small.
struct MotorPolarityStore {
  bool inverted;
} __attribute__((packed));

}  // namespace jalouzee_blinds
}  // namespace esphome
