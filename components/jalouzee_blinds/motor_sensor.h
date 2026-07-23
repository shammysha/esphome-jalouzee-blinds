#pragma once

#include "esphome/core/hal.h"
#include "esphome/components/adc/adc_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// Датчики, физически расположенные на быстром валу мотора: квадратурный
// Hall-энкодер (7 PPR x передаточное число редуктора) и/или endless-
// потенциометр (без механического упора, крутится многооборотно вместе с
// валом), читаемый через штатный ADC-компонент ESPHome. Оба — взаимоисключающие
// способы определения угла (см. Controller) и оба ОТНОСИТЕЛЬНЫЕ/накопительные:
// endless-потенциометр каждый оборот "перескакивает" через границу диапазона
// АЦП, поэтому read_adc_raw() не возвращает сырое напряжение, а разворачивает
// эти перескоки в непрерывно накапливаемую величину — см. .cpp.
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
  // То же самое для накопленной позиции endless-потенциометра — см.
  // seed_hall_pulse_count() и JalouzeeBlinds::setup(). Вызывать ДО setup().
  void seed_adc_position(float value) { this->adc_unwrapped_ = value; }

  void setup();

  float read_hall_raw() const { return static_cast<float>(this->hall_pulse_count_); }
  // Возвращает НЕ сырое напряжение АЦП, а накопленную (развёрнутую через
  // перескоки оборота) величину — см. класс-комментарий выше и .cpp.
  float read_adc_raw();

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
  // Предыдущий сырой отсчёт (для вычисления delta и детекта перескока через
  // границу оборота) и сама накопленная развёрнутая позиция — см. read_adc_raw().
  float adc_last_raw_{0};
  float adc_unwrapped_{0};

  bool has_hall_{false};
  bool has_adc_{false};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
