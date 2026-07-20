#pragma once

#include "types.h"

namespace esphome {
namespace jalouzee {


class Calibration {

 public:

  Calibration();


  /*
   * Запуск режима калибровки
   *
   * Если уже есть активная калибровка,
   * она не изменяется.
   */
  void start();


  /*
   * Нажатие основной кнопки CAL
   *
   * Переход:
   *
   * WAIT_CLOSED
   *        |
   *        v
   * WAIT_OPEN
   *        |
   *        v
   * COMMIT
   */
  bool next(
      const SensorData &sensor
  );


  /*
   * Немедленная отмена
   *
   * Старые настройки остаются.
   */
  void cancel();


  /*
   * Завершение калибровки
   *
   * Проверяет данные и
   * формирует новую CalibrationData
   */
  bool commit();


  /*
   * Получить текущий этап
   */
  CalibrationStage stage() const;


  /*
   * Активна ли калибровка
   */
  bool active() const;


  /*
   * Получить текущие рабочие данные
   */
  const CalibrationData &data() const;


  /*
   * Установить сохранённые данные
   *
   * вызывается при старте ESP
   */
  void load(
      const CalibrationData &data
  );


 protected:


  /*
   * Проверка корректности
   */
  bool validate() const;


  /*
   * Определение направления изменения
   */
  void calculate_signs();


  /*
   * Текущая рабочая калибровка
   *
   * Меняется только после commit()
   */
  CalibrationData calibration_;


  /*
   * Временные данные процесса
   */
  CalibrationRuntime runtime_;


  CalibrationStage stage_ =
      CalibrationStage::NONE;


};


}  // namespace jalouzee
}  // namespace esphome
