#pragma once

#include "esphome/core/hal.h"
#include "esphome/components/adc/adc_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// Датчики, физически расположенные на/у оси мотора: квадратурный Hall-энкодер
// (7 PPR x передаточное число редуктора) и/или резистор на оси, читаемый через
// штатный ADC-компонент ESPHome. Оба взаимоисключающие способы определения
// угла через энкодер/резистор на быстром валу мотора (см. Controller).
class MotorSensor {
 public:
  void set_hall_pins(InternalGPIOPin *a, InternalGPIOPin *b) {
    this->encoder_a_pin_ = a;
    this->encoder_b_pin_ = b;
    this->has_hall_ = true;
  }
  void set_adc_pin(InternalGPIOPin *pin) {
    this->adc_gpio_pin_ = pin;
    this->has_adc_ = true;
  }

  bool has_hall() const { return this->has_hall_; }
  bool has_adc() const { return this->has_adc_; }

  // Восстанавливает счётчик импульсов из значения, сохранённого во flash (см.
  // JalouzeeBlinds::setup()) — иначе после ребута счётчик стартует с 0, теряя
  // привязку к калибровочным точкам closed/open. Вызывать ДО setup().
  void seed_hall_pulse_count(int32_t value) { this->hall_pulse_count_ = value; }

  void setup();

  float read_hall_raw() const { return static_cast<float>(this->hall_pulse_count_); }
  float read_adc_raw();

  // диагностика (см. jalouzee_blinds.cpp::loop())
  int32_t hall_pulse_count() const { return this->hall_pulse_count_; }
  bool hall_pin_a_level() const { return this->encoder_a_pin_->digital_read(); }
  bool hall_pin_b_level() const { return this->encoder_b_pin_->digital_read(); }

 protected:
  static void hall_isr_(MotorSensor *arg);

  InternalGPIOPin *encoder_a_pin_{nullptr};
  InternalGPIOPin *encoder_b_pin_{nullptr};
  // ISR-safe копии encoder_a_pin_/encoder_b_pin_ (см. hall_isr_) — обычный
  // InternalGPIOPin::digital_read() это виртуальный вызов и не гарантированно
  // безопасен из прерывания.
  ISRInternalGPIOPin encoder_a_isr_;
  ISRInternalGPIOPin encoder_b_isr_;
  volatile int32_t hall_pulse_count_{0};
  // Текущее 2-битное состояние (A<<1|B) для полного (4x) квадратурного декода
  // в hall_isr_ — см. .cpp.
  volatile uint8_t hall_last_state_{0};

  InternalGPIOPin *adc_gpio_pin_{nullptr};
  // Внутренний экземпляр штатного ADC-сенсора ESPHome (драйвер зависит от
  // платформы). Не регистрируется в App (нет периодического update()) — читаем
  // значение вручную через sample() когда нужно (см. read_adc_raw()).
  adc::ADCSensor *adc_sensor_{nullptr};

  bool has_hall_{false};
  bool has_adc_{false};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
