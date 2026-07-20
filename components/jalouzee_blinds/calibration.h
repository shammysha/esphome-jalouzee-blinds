#pragma once

#include "types.h"

namespace esphome {
namespace jalouzee_blinds {

class Calibration {
 public:
  Calibration();

  // Начать новую калибровку
  void start();

  // Следующий шаг калибровки
  CalibrationResult next(const SensorData &sensor);

  // Отмена процесса
  void cancel();

  // Применить подготовленную калибровку
  // (вызывается ПОСЛЕ успешной записи в Storage)
  void apply_pending();

  // Загрузить рабочую калибровку
  void load(const CalibrationData &data);

  bool active() const;

  bool commit();

  CalibrationStage stage() const;

  // Действующая калибровка
  const CalibrationData &data() const;

  // Подготовленная калибровка
  const CalibrationData &pending() const;

 protected:
  CalibrationResult validate() const;

  CalibrationResult build_pending();

  void calculate_signs(CalibrationData &data);

  void reset_runtime();

 protected:
  // Рабочая калибровка
  CalibrationData calibration_;

  // Подготовленная, но ещё не применённая
  CalibrationData pending_;

  // Временные значения процесса
  CalibrationRuntime runtime_;

  CalibrationStage stage_ = CalibrationStage::NONE;
};

}  // namespace jalouzee_blinds
}  // namespace esphome
