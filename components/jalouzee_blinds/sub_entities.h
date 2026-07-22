#pragma once

#include <string>
#include <vector>
#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

class JalouzeeBlinds;

// ---------------------------------------------------------------------------
// Вспомогательные сущности, создаваемые компонентом автоматически
// (в YAML не конфигурируются, см. п. "Внутри компонента создаются автоматически").
// press_action()/control() лишь пересылают событие родителю (JalouzeeBlinds) —
// решение, что с ним делать, остаётся в общей логике.
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

// Создание/регистрация вложенных сущностей и публикация их состояния —
// "работа с сенсорами". Реакция на нажатия/изменения (см. классы выше)
// остаётся в JalouzeeBlinds (общая логика).
class SubEntities {
 public:
  void setup(JalouzeeBlinds *parent, const std::string &base_name, bool has_hall, bool has_adc, bool has_mpu,
             const char *initial_angle_source, uint32_t initial_fault_timeout_s);

  void set_calibration_message(const std::string &msg) { this->calibration_text_sensor_->publish_state(msg); }
  const std::string &calibration_message_state() const { return this->calibration_text_sensor_->state; }

  void set_calibrated(bool calibrated) { this->calibrated_binary_sensor_->publish_state(calibrated); }
  void set_fault(bool active) { this->fault_binary_sensor_->publish_state(active); }
  void set_angle_source_state(const std::string &value) { this->angle_source_select_->publish_state(value); }
  void set_fault_timeout_state(uint32_t seconds) { this->fault_timeout_number_->publish_state(seconds); }

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
  binary_sensor::BinarySensor *calibrated_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *fault_binary_sensor_{nullptr};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
