#include "sub_entities.h"
#include "jalouzee_blinds.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace jalouzee_blinds {

// Категория "diagnostic" для per-source сенсоров калибровки — см.
// ENTITY_FIELD_ENTITY_CATEGORY_SHIFT/EntityCategory в esphome/core/entity_base.h.
static const uint32_t DIAGNOSTIC_ENTITY_FIELDS = static_cast<uint32_t>(ENTITY_CATEGORY_DIAGNOSTIC)
                                                  << ENTITY_FIELD_ENTITY_CATEGORY_SHIFT;

void CalibrationButton::press_action() { this->parent_->on_calibration_button_pressed(); }
void CancelCalibrationButton::press_action() { this->parent_->on_cancel_calibration_button_pressed(); }
void FaultResetButton::press_action() { this->parent_->on_fault_reset_button_pressed(); }
void AngleSourceSelect::control(const std::string &value) { this->parent_->on_angle_source_select_changed(value); }
void FaultTimeoutNumber::control(float value) { this->parent_->on_fault_timeout_changed(value); }

void SubEntities::setup(JalouzeeBlinds *parent, const std::string &base_name, bool has_hall, bool has_adc,
                         bool has_mpu, const char *initial_angle_source, uint32_t initial_fault_timeout_s) {
  this->entity_name_storage_.reserve(10);
  auto make_name = [this](std::string name) -> const char * {
    this->entity_name_storage_.push_back(std::move(name));
    return this->entity_name_storage_.back().c_str();
  };

  this->calibration_button_ = new CalibrationButton();
  this->calibration_button_->set_parent(parent);
  App.register_button(this->calibration_button_, make_name(base_name + " Калибровка"), 0, 0);

  this->cancel_calibration_button_ = new CancelCalibrationButton();
  this->cancel_calibration_button_->set_parent(parent);
  // Всегда видна в HA — ESPHome не поддерживает динамическое отключение/скрытие
  // кнопки в рантайме. Нажатие вне калибровки безопасно игнорируется в
  // on_cancel_calibration_button_pressed().
  App.register_button(this->cancel_calibration_button_, make_name(base_name + " Отменить калибровку"), 0, 0);

  this->fault_reset_button_ = new FaultResetButton();
  this->fault_reset_button_->set_parent(parent);
  App.register_button(this->fault_reset_button_, make_name(base_name + " Сброс аварии"), 0, 0);

  this->angle_source_select_ = new AngleSourceSelect();
  this->angle_source_select_->set_parent(parent);
  {
    FixedVector<const char *> options;
    options.init(3);
    options.push_back("auto");
    if (has_mpu) options.push_back("angle");
    if (has_hall || has_adc) options.push_back("encoder");
    this->angle_source_select_->traits.set_options(options);
  }
  App.register_select(this->angle_source_select_, make_name(base_name + " Источник угла наклона"), 0, 0);
  this->angle_source_select_->publish_state(initial_angle_source);

  this->fault_timeout_number_ = new FaultTimeoutNumber();
  this->fault_timeout_number_->set_parent(parent);
  this->fault_timeout_number_->traits.set_min_value(1);
  this->fault_timeout_number_->traits.set_max_value(300);
  this->fault_timeout_number_->traits.set_step(1);
  App.register_number(this->fault_timeout_number_, make_name(base_name + " Таймаут аварии (сек)"), 0, 0);
  this->fault_timeout_number_->publish_state(initial_fault_timeout_s);

  this->calibration_text_sensor_ = new text_sensor::TextSensor();
  App.register_text_sensor(this->calibration_text_sensor_, make_name(base_name + " Сообщение калибровки"), 0, 0);

  this->calibrated_binary_sensor_ = new binary_sensor::BinarySensor();
  App.register_binary_sensor(this->calibrated_binary_sensor_, make_name(base_name + " Откалибровано"), 0, 0);

  this->fault_binary_sensor_ = new binary_sensor::BinarySensor();
  App.register_binary_sensor(this->fault_binary_sensor_, make_name(base_name + " Авария"), 0, 0);
  this->fault_binary_sensor_->publish_state(false);

  // Диагностика калибровки по каждому НАСТРОЕННОМУ источнику (см. cover.py —
  // ровно столько же дополнительных binary_sensor учтено в platform_counts).
  if (has_hall) {
    this->hall_calibrated_binary_sensor_ = new binary_sensor::BinarySensor();
    App.register_binary_sensor(this->hall_calibrated_binary_sensor_, make_name(base_name + " Hall откалиброван"), 0,
                                DIAGNOSTIC_ENTITY_FIELDS);
  }
  if (has_adc) {
    this->adc_calibrated_binary_sensor_ = new binary_sensor::BinarySensor();
    App.register_binary_sensor(this->adc_calibrated_binary_sensor_, make_name(base_name + " ADC откалиброван"), 0,
                                DIAGNOSTIC_ENTITY_FIELDS);
  }
  if (has_mpu) {
    this->angle_calibrated_binary_sensor_ = new binary_sensor::BinarySensor();
    App.register_binary_sensor(this->angle_calibrated_binary_sensor_, make_name(base_name + " Angle откалиброван"), 0,
                                DIAGNOSTIC_ENTITY_FIELDS);
  }
}

}  // namespace jalouzee_blinds
}  // namespace esphome
