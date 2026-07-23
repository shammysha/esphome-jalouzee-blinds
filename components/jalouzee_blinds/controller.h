#pragma once

#include <cstdint>
#include "store.h"
#include "motor_sensor.h"
#include "mpu_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// Режим определения угла, который выбрал пользователь (хранится во flash)
enum AngleSourceMode : uint8_t {
  ANGLE_SOURCE_AUTO = 0,     // 1) MPU6050  2) Hall/ADC (по приоритету)
  ANGLE_SOURCE_MPU6050 = 1,  // принудительно MPU6050
  ANGLE_SOURCE_ENCODER = 2,  // принудительно Hall либо ADC (что задано в YAML)
};

// Реально используемый в данный момент источник (после разрешения приоритетов,
// доступности и калибровки)
enum ActiveAngleSource : uint8_t {
  ACTIVE_SOURCE_NONE = 0,
  ACTIVE_SOURCE_MPU6050 = 1,
  ACTIVE_SOURCE_HALL = 2,
  ACTIVE_SOURCE_ADC = 3,
};

// Владеет режимом определения угла и калибровочными данными источников
// (per-source closed/open + calibrated, в JalouzeeBlindsStore), их валидацией
// по минимальной разнице (см. .cpp), разрешением активного источника
// (resolve_active_source), пересчётом сырого значения в проценты
// (raw_to_percent), и оппортунистической автокалибровкой отклонённых
// источников (try_auto_calibrate_at_endpoint).
class Controller {
 public:
  void set_sensors(MotorSensor *hall_adc, MpuSensor *mpu) {
    this->hall_adc_ = hall_adc;
    this->mpu_ = mpu;
  }
  // store переживает весь жизненный цикл компонента (владеет им JalouzeeBlinds) —
  // указатель безопасен хранить без доп. синхронизации.
  void set_store(JalouzeeBlindsStore *store) { this->store_ = store; }

  uint8_t mode() const { return this->store_->angle_source_mode; }
  void set_mode(uint8_t mode) { this->store_->angle_source_mode = mode; }

  // Некалиброванные источники должны иметь closed/open == NAN (а не 0.0 из
  // zero-init/старых данных) — иначе auto_calibrate_capture_() ошибочно решит,
  // что одна из точек уже поймана. Вызывать сразу после загрузки store из flash.
  void normalize_uncalibrated();

  bool is_any_calibrated() const;
  // Диагностика — калиброван ли конкретный источник (см. диагностические
  // бинарные сенсоры per-source в SubEntities).
  bool is_calibrated(ActiveAngleSource src) const { return this->is_source_calibrated_(src); }

  // hall_untrusted — Hall-энкодер не считается надёжным в этой сессии (обычно
  // из-за обнаруженного прерванного питанием движения, см.
  // JalouzeeBlinds::setup()) — общая логика, не калибровочные данные как
  // таковые, поэтому передаётся параметром. Гасит ТОЛЬКО ветку Hall (ADC —
  // абсолютный датчик, в этой защите не нуждается) и работает одинаково в
  // любом режиме (auto/encoder).
  ActiveAngleSource resolve_active_source(bool hall_untrusted) const;
  float read_raw(ActiveAngleSource src) const;
  float raw_to_percent(ActiveAngleSource src, float raw) const;
  // Обратное преобразование raw_to_percent — по калибровочным точкам источника
  // восстанавливает сырое значение, соответствующее проценту (используется для
  // восстановления hall_pulse_count_ из сохранённой позиции после ребута —
  // см. JalouzeeBlinds::setup()). Для некалиброванного источника вернёт NAN.
  float percent_to_raw(ActiveAngleSource src, float percent) const;

  // Пытается принять калибровку источника по двум зафиксированным точкам (см.
  // finish_calibration_ в jalouzee_blinds.cpp) — принимает, если |open-closed|
  // не меньше минимального порога; иначе источник остаётся/становится
  // некалиброванным (closed/open сбрасываются в NAN). Возвращает true, если принято.
  bool try_finish_calibration(ActiveAngleSource src, float closed, float open);

  // Когда жалюзи реально доходят до 0%/100% (по уже доверенному активному
  // источнику) в обычном режиме работы, ловим эту же точку для любого
  // доступного, но ещё не откалиброванного источника (например, MPU, чья
  // ручная калибровка была отклонена из-за отсутствия движения) — так он
  // сможет самостоятельно доехать до calibrated=true, если его позже
  // физически восстановят, без повторного ручного прохода. Возвращает true,
  // если хотя бы один источник только что стал calibrated=true (вызывающий
  // должен сохранить flash и обновить бинарный сенсор "Откалибровано").
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
