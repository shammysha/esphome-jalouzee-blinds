#pragma once

#include <cstdint>
#include "store.h"
#include "motor_sensor.h"
#include "mpu_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// The angle-source mode chosen by the user (persisted to flash)
enum AngleSourceMode : uint8_t {
  ANGLE_SOURCE_AUTO = 0,     // 1) MPU6050  2) Hall/ADC (by priority)
  ANGLE_SOURCE_MPU6050 = 1,  // force MPU6050
  ANGLE_SOURCE_ENCODER = 2,  // force Hall or ADC (whichever is set in YAML)
};

// The source actually in use right now (after resolving priority,
// availability, and calibration)
enum ActiveAngleSource : uint8_t {
  ACTIVE_SOURCE_NONE = 0,
  ACTIVE_SOURCE_MPU6050 = 1,
  ACTIVE_SOURCE_HALL = 2,
  ACTIVE_SOURCE_ADC = 3,
};

// Owns the angle-source mode and each source's calibration data (per-source
// closed/open + calibrated, in JalouzeeBlindsStore), their validation
// against a minimum delta (see .cpp), resolving the active source
// (resolve_active_source), converting the raw value to a percentage
// (raw_to_percent), and opportunistic auto-calibration of rejected sources
// (try_auto_calibrate_at_endpoint).
class Controller {
 public:
  void set_sensors(MotorSensor *hall_adc, MpuSensor *mpu) {
    this->hall_adc_ = hall_adc;
    this->mpu_ = mpu;
  }
  // store outlives the whole component lifecycle (owned by JalouzeeBlinds) —
  // the pointer is safe to keep without extra synchronization.
  void set_store(JalouzeeBlindsStore *store) { this->store_ = store; }

  uint8_t mode() const { return this->store_->angle_source_mode; }
  void set_mode(uint8_t mode) { this->store_->angle_source_mode = mode; }

  // Uncalibrated sources must have closed/open == NAN (not 0.0 from
  // zero-init/stale data) — otherwise auto_calibrate_capture_() would
  // wrongly conclude one of the points was already captured. Call right
  // after loading the store from flash.
  void normalize_uncalibrated();

  bool is_any_calibrated() const;
  // Diagnostics — is this specific source calibrated (see the per-source
  // diagnostic binary sensors in SubEntities).
  bool is_calibrated(ActiveAngleSource src) const { return this->is_source_calibrated_(src); }

  // hall_untrusted — the Hall encoder is not considered reliable this
  // session (usually due to a detected power-loss-interrupted movement, see
  // JalouzeeBlinds::setup()) — this is general logic, not calibration data
  // per se, so it's passed as a parameter. Gates ONLY the Hall branch (ADC
  // is an absolute sensor and doesn't need this protection) and works the
  // same in any mode (auto/encoder).
  ActiveAngleSource resolve_active_source(bool hall_untrusted) const;
  float read_raw(ActiveAngleSource src) const;
  float raw_to_percent(ActiveAngleSource src, float raw) const;
  // The inverse of raw_to_percent — reconstructs the raw value
  // corresponding to a percentage from the source's calibration points
  // (used to restore hall_pulse_count_ from the saved position after a
  // reboot — see JalouzeeBlinds::setup()). Returns NAN for an uncalibrated
  // source.
  float percent_to_raw(ActiveAngleSource src, float percent) const;

  // Tries to accept a source's calibration from its two captured points
  // (see finish_calibration_ in jalouzee_blinds.cpp) — accepts if
  // |open-closed| is not below the minimum threshold; otherwise the source
  // stays/becomes uncalibrated (closed/open are reset to NAN). Returns true
  // if accepted.
  bool try_finish_calibration(ActiveAngleSource src, float closed, float open);

  // When the blinds actually reach 0%/100% (per the already-trusted active
  // source) during normal operation, we opportunistically capture that same
  // point for any other available but not-yet-calibrated source (e.g. an
  // MPU whose manual calibration was rejected due to no movement) — so it
  // can reach calibrated=true on its own if it's later physically
  // reconnected, without a repeat manual calibration pass. Returns true if
  // at least one source just became calibrated=true (the caller should save
  // to flash and update the "Calibrated" binary sensor).
  bool try_auto_calibrate_at_endpoint(bool is_closed_point);

 protected:
  bool is_source_calibrated_(ActiveAngleSource src) const;
  bool is_source_available_(ActiveAngleSource src) const;
  bool auto_calibrate_capture_(ActiveAngleSource src, bool is_closed_point, float raw);

  MotorSensor *hall_adc_{nullptr};
  MpuSensor *mpu_{nullptr};
  JalouzeeBlindsStore *store_{nullptr};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
