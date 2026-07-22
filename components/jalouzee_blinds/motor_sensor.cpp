#include <cmath>
#include "motor_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// Таблица переходов для полного (4x) квадратурного декода датчика Холла (A/B).
// Индекс — (предыдущее_состояние << 2) | новое_состояние, где состояние = (A<<1)|B.
// Значение — направление счёта: 0 для "невозможных" переходов (пропущенный
// фронт/дребезг), ±1 для валидных соседних переходов по коду Грея. Надёжнее,
// чем угадывать направление по значению соседнего канала в момент прерывания —
// см. hall_isr_().
static const int8_t HALL_QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,   //
    1, 0, 0, -1,   //
    -1, 0, 0, 1,   //
    0, 1, -1, 0,   //
};

void MotorSensor::setup() {
  if (this->has_hall_) {
    this->encoder_a_pin_->setup();
    this->encoder_b_pin_->setup();
    this->encoder_a_isr_ = this->encoder_a_pin_->to_isr();
    this->encoder_b_isr_ = this->encoder_b_pin_->to_isr();
    // Стартовое состояние — до первого реального фронта, чтобы не засчитать
    // фантомный переход на первом прерывании.
    this->hall_last_state_ =
        (this->encoder_a_pin_->digital_read() ? 2 : 0) | (this->encoder_b_pin_->digital_read() ? 1 : 0);
    // Полный (4x) квадратурный декод требует прерываний на ОБОИХ каналах, не только A.
    this->encoder_a_pin_->attach_interrupt(&MotorSensor::hall_isr_, this, gpio::INTERRUPT_ANY_EDGE);
    this->encoder_b_pin_->attach_interrupt(&MotorSensor::hall_isr_, this, gpio::INTERRUPT_ANY_EDGE);
  }
  // --- ADC (резистор на оси мотора) ---
  // Используем штатный ADC-компонент ESPHome (ESP-IDF adc_oneshot драйвер,
  // включая калибровку по эталонной кривой/линии, если она доступна для
  // конкретного чипа). Объект создаём и настраиваем сами, в App не
  // регистрируем (не нужен периодический update()) — читаем sample() вручную.
  if (this->has_adc_) {
    this->adc_sensor_ = new adc::ADCSensor();  // NOLINT(cppcoreguidelines-owning-memory)
    this->adc_sensor_->set_pin(this->adc_gpio_pin_);
#ifdef USE_ESP32
    this->adc_sensor_->set_attenuation(adc::ADC_ATTEN_DB_12_COMPAT);
#endif
    this->adc_sensor_->setup();
  }
}

float MotorSensor::read_adc_raw() {
  if (this->adc_sensor_ == nullptr) return NAN;
  // sample() выполняет одиночное измерение через ESP-IDF adc_oneshot API
  // (с калибровкой, если она доступна) и возвращает напряжение в вольтах.
  // Для наших целей единица измерения неважна — калибровка "закрыто/открыто"
  // работает с любой монотонной величиной.
  return this->adc_sensor_->sample();
}

void MotorSensor::hall_isr_(MotorSensor *arg) {
  // Полный (4x) квадратурный декод по обоим каналам (см. HALL_QUADRATURE_TABLE) —
  // прерывание срабатывает на любом фронте A ИЛИ B, читаем оба уровня и по
  // таблице переходов получаем ±1 либо 0 (для "невозможных"/дребезговых
  // переходов, которые не отбрасывались надёжно при декоде только по одному
  // каналу).
  uint8_t a = arg->encoder_a_isr_.digital_read() ? 1 : 0;
  uint8_t b = arg->encoder_b_isr_.digital_read() ? 1 : 0;
  uint8_t new_state = (a << 1) | b;
  uint8_t index = (arg->hall_last_state_ << 2) | new_state;
  arg->hall_pulse_count_ += HALL_QUADRATURE_TABLE[index];
  arg->hall_last_state_ = new_state;
}

}  // namespace jalouzee_blinds
}  // namespace esphome
