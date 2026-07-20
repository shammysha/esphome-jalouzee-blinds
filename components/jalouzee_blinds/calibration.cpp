#include "calibration.h"

#include <cmath>
#include <cstdlib>


namespace esphome {
namespace jalouzee {


Calibration::Calibration()
{
}


void Calibration::start()
{
  /*
   * Старые данные не трогаем.
   *
   * Начинаем новый временный цикл.
   */

  runtime_ = CalibrationRuntime();

  stage_ = CalibrationStage::WAIT_CLOSED;
}


bool Calibration::next(
    const SensorData &sensor
)
{
  switch (stage_)
  {

    case CalibrationStage::WAIT_CLOSED:
    {
      /*
       * Первое подтверждение положения.
       *
       * Только RAM.
       */

      runtime_.closed_angle =
          sensor.angle;

      runtime_.closed_encoder =
          sensor.encoder;


      stage_ =
          CalibrationStage::WAIT_OPEN;


      return true;
    }


    case CalibrationStage::WAIT_OPEN:
    {
      /*
       * Второе подтверждение положения.
       *
       * Только RAM.
       */

      runtime_.open_angle =
          sensor.angle;

      runtime_.open_encoder =
          sensor.encoder;


      stage_ =
          CalibrationStage::READY_TO_SAVE;


      return true;
    }


    case CalibrationStage::READY_TO_SAVE:
    {
      /*
       * Третье нажатие.
       *
       * Пытаемся завершить.
       */

      return commit();
    }


    default:

      return false;
  }
}



void Calibration::cancel()
{
  /*
   * Важно:
   *
   * calibration_ НЕ изменяется.
   *
   * Пользователь просто
   * выбросил временные данные.
   */

  runtime_ =
      CalibrationRuntime();

  stage_ =
      CalibrationStage::NONE;
}



bool Calibration::commit()
{

  if (!validate())
  {
    cancel();

    return false;
  }


  CalibrationData new_data;


  new_data.valid = true;


  new_data.closed_angle =
      runtime_.closed_angle;


  new_data.open_angle =
      runtime_.open_angle;



  new_data.encoder_range =
      std::abs(
          runtime_.open_encoder -
          runtime_.closed_encoder
      );


  /*
   * Определяем направления
   */

  if (runtime_.open_angle >
      runtime_.closed_angle)
  {
    new_data.imu_sign = 1;
  }
  else
  {
    new_data.imu_sign = -1;
  }



  if (runtime_.open_encoder >
      runtime_.closed_encoder)
  {
    new_data.encoder_sign = 1;
  }
  else
  {
    new_data.encoder_sign = -1;
  }



  /*
   * Только здесь меняем
   * рабочую калибровку.
   */

  calibration_ =
      new_data;


  runtime_ =
      CalibrationRuntime();


  stage_ =
      CalibrationStage::NONE;


  return true;
}



bool Calibration::validate() const
{

  /*
   * Проверка углового диапазона
   */

  float angle_range =
      std::abs(
          runtime_.open_angle -
          runtime_.closed_angle
      );


  if (angle_range < 5.0f)
  {
    return false;
  }


  /*
   * Проверка энкодера
   */

  int32_t encoder_range =
      std::abs(
          runtime_.open_encoder -
          runtime_.closed_encoder
      );


  /*
   * Минимум условный.
   *
   * Потом можно вынести
   * в настройки.
   */

  if (encoder_range < 10)
  {
    return false;
  }


  return true;
}



CalibrationStage Calibration::stage() const
{
  return stage_;
}



bool Calibration::active() const
{
  return
      stage_ != CalibrationStage::NONE;
}



const CalibrationData &
Calibration::data() const
{
  return calibration_;
}



void Calibration::load(
    const CalibrationData &data
)
{
  calibration_ =
      data;
}



void Calibration::calculate_signs()
{
  /*
   * Оставлено отдельной функцией
   *
   * для дальнейшего расширения.
   *
   * Например:
   * проверка по нескольким движениям.
   */
}



} // namespace jalouzee
} // namespace esphome
