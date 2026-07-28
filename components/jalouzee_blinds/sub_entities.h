#pragma once

#include <string>
#include <vector>
#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace jalouzee_blinds {

class JalouzeeBlinds;

// ---------------------------------------------------------------------------
// Helper entities created automatically by the component (not configurable
// in YAML — see "Entities created automatically" in the README).
// press_action()/control() just forward the event to the parent
// (JalouzeeBlinds) — the decision of what to do with it stays in the
// general logic.
// ---------------------------------------------------------------------------

class CalibrationButton : public button::Button {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void press_action() override;
  JalouzeeBlinds *parent_{nullptr};
};

class CancelCalibrationButton : public button::Button {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void press_action() override;
  JalouzeeBlinds *parent_{nullptr};
};

class FaultResetButton : public button::Button {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void press_action() override;
  JalouzeeBlinds *parent_{nullptr};
};

class AngleSourceSelect : public select::Select {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void control(const std::string &value) override;
  JalouzeeBlinds *parent_{nullptr};
};

class FaultTimeoutNumber : public number::Number {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void control(float value) override;
  JalouzeeBlinds *parent_{nullptr};
};

// Creates/registers the nested entities and publishes their state — "sensor
// bookkeeping". Reacting to presses/changes (see the classes above) stays
// in JalouzeeBlinds (general logic).
class SubEntities {
 public:
  void setup(JalouzeeBlinds *parent, bool has_hall, bool has_adc, bool has_mpu, const char *initial_angle_source,
             uint32_t initial_fault_timeout_s);

  void set_calibration_message(const std::string &msg) { this->calibration_text_sensor_->publish_state(msg); }
  const std::string &calibration_message_state() const { return this->calibration_text_sensor_->state; }

  // Numeric mirror of cal_state_ (CalibrationState) — 0 = outside calibration
  // (CAL_IDLE), 1 = waiting for "closed" (CAL_WAIT_CLOSED), 2 = waiting for
  // "open" (CAL_WAIT_OPEN). Same information as the message text_sensor
  // above, just as a plain number for automations/dashboards that would
  // otherwise have to pattern-match the English message text.
  void set_calibration_step(uint8_t step) { this->calibration_step_sensor_->publish_state(step); }

  void set_calibrated(bool calibrated) { this->calibrated_binary_sensor_->publish_state(calibrated); }
  void set_fault(bool active) { this->fault_binary_sensor_->publish_state(active); }
  void set_angle_source_state(const std::string &value) { this->angle_source_select_->publish_state(value); }
  void set_fault_timeout_state(uint32_t seconds) { this->fault_timeout_number_->publish_state(seconds); }

  // Diagnostic calibration sensors for each CONFIGURED source (only created
  // for the ones actually set up in YAML — see has_hall/has_adc/has_mpu in
  // setup()). Publishing is a no-op if the corresponding source isn't
  // configured (nullptr pointer).
  void set_hall_calibrated(bool calibrated) {
    if (this->hall_calibrated_binary_sensor_ != nullptr) this->hall_calibrated_binary_sensor_->publish_state(calibrated);
  }
  void set_adc_calibrated(bool calibrated) {
    if (this->adc_calibrated_binary_sensor_ != nullptr) this->adc_calibrated_binary_sensor_->publish_state(calibrated);
  }
  void set_angle_calibrated(bool calibrated) {
    if (this->angle_calibrated_binary_sensor_ != nullptr) this->angle_calibrated_binary_sensor_->publish_state(calibrated);
  }

 protected:
  // configure_entity_() only stores a StringRef (no copy) — keep the built name
  // strings alive here for the lifetime of the device. setup() reserves exactly
  // the number of make_name() calls it makes so the vector never reallocates
  // (which would invalidate the c_str() pointers already handed to entities).
  std::vector<std::string> entity_name_storage_;

  CalibrationButton *calibration_button_{nullptr};
  CancelCalibrationButton *cancel_calibration_button_{nullptr};
  FaultResetButton *fault_reset_button_{nullptr};
  AngleSourceSelect *angle_source_select_{nullptr};
  FaultTimeoutNumber *fault_timeout_number_{nullptr};
  text_sensor::TextSensor *calibration_text_sensor_{nullptr};
  sensor::Sensor *calibration_step_sensor_{nullptr};
  binary_sensor::BinarySensor *calibrated_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *fault_binary_sensor_{nullptr};
  // Per-source diagnostics (nullptr if the source isn't configured).
  binary_sensor::BinarySensor *hall_calibrated_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *adc_calibrated_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *angle_calibrated_binary_sensor_{nullptr};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
