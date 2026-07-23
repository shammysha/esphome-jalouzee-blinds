#pragma once

#include <cstdint>

namespace esphome {
namespace jalouzee_blinds {

// Данные, сохраняемые во flash (NVS). Статус аварии сюда НЕ входит — она не
// должна переживать перезагрузку.
struct JalouzeeBlindsStore {
  bool hall_calibrated;
  bool adc_calibrated;
  bool mpu_calibrated;
  // true между началом реального движения мотора и его штатным завершением
  // (достижение цели / явный stop / авария / вход в калибровку) — если при
  // следующей загрузке этот флаг всё ещё true, значит движение было прервано
  // потерей питания, и позиции Hall доверять нельзя (см. JalouzeeBlinds::setup()).
  bool movement_in_progress;

  float hall_closed;  // "сырое" значение накопленных импульсов при закрытых ламелях
  float hall_open;
  float adc_closed;  // "сырое" значение АЦП при закрытых ламелях
  float adc_open;
  float mpu_closed;  // значение sensor'а MPU6050 при закрытых ламелях
  float mpu_open;

  uint8_t angle_source_mode;  // выбор пользователя: auto/angle/encoder
  float last_angle_percent;   // последний известный угол наклона, 0..100%
} __attribute__((packed));

}  // namespace jalouzee_blinds
}  // namespace esphome
