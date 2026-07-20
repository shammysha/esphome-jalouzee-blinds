#pragma once



#include "types.h"



namespace esphome {
namespace jalouzee {



class Calibration
{


 public:


  Calibration();




  /*
   * Вход в режим
   */
  void start();




  /*
   * Следующее нажатие
   * основной кнопки
   */
  CalibrationResult next(
      const SensorData &sensor
  );




  /*
   * Отмена
   */
  void cancel();





  /*
   * Получить рабочую
   * калибровку
   */
  const CalibrationData &
  data() const;




  /*
   * Получить подготовленную
   * к сохранению
   */
  const CalibrationData &
  pending() const;




  /*
   * Сделать pending
   * рабочей
   */
  void apply_pending();




  /*
   * Загрузка из NVS
   */
  void load(
      const CalibrationData &data
  );





  CalibrationStage stage() const;



  bool active() const;





 protected:


  /*
   * Подготовка данных
   */
  CalibrationResult commit();




  /*
   * Проверка
   */
  CalibrationResult validate()
      const;





  /*
   * Определение сторон
   */
  void calculate_signs(
      CalibrationData &data
  ) const;




  void reset_runtime();





 protected:




  /*
   * Последняя рабочая
   */
  CalibrationData calibration_;




  /*
   * Новая, но ещё
   * не применённая
   */
  CalibrationData pending_data_;




  /*
   * Текущий процесс
   */
  CalibrationRuntime runtime_;




  CalibrationStage stage_ =
      CalibrationStage::NONE;



};



} // namespace jalouzee
} // namespace esphome
