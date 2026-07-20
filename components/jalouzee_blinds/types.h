#pragma once


#include <cstdint>



namespace esphome {
namespace jalouzee_blinds {



/*
 * Состояние основного автомата
 */

enum class SystemState : uint8_t
{

  IDLE = 0,

  MOVING_OPEN,

  MOVING_CLOSE,

  CALIBRATION,

  ERROR

};








/*
 * Этапы калибровки
 *
 * Важно:
 * данные здесь временные.
 * NVS на этих этапах не меняется.
 */

enum class CalibrationStage : uint8_t
{

  NONE = 0,


  WAIT_CLOSED,


  WAIT_OPEN,


  WAIT_COMMIT

};









/*
 * Результат операции калибровки
 */

enum class CalibrationResult : uint8_t
{

  NOT_READY = 0,


  OK,


  INVALID_ANGLE_RANGE,


  INVALID_ENCODER_RANGE,


  INVALID_SENSOR_DATA

};









/*
 * Рабочая калибровка
 *
 * Только эти данные считаются
 * действующими.
 *
 * Именно они сохраняются в NVS.
 */

struct CalibrationData
{

  bool valid = false;




  /*
   * MPU
   */

  float closed_angle = 0;

  float open_angle = 0;




  /*
   * Encoder
   */

  int32_t closed_encoder = 0;

  int32_t open_encoder = 0;



  int32_t encoder_range = 0;





  /*
   * Коррекция установки
   *
   * Мотор/MPU могут быть:
   *
   * слева
   * справа
   *
   */

  int8_t encoder_sign = 1;

  int8_t imu_sign = 1;


};









/*
 * Временные данные
 *
 * Живут только во время
 * процесса калибровки.
 *
 * В NVS никогда не попадают.
 */

struct CalibrationRuntime
{

  float closed_angle = 0;

  float open_angle = 0;



  int32_t closed_encoder = 0;

  int32_t open_encoder = 0;


};









/*
 * Состояние датчиков
 *
 * Передаётся между слоями.
 */

struct SensorData
{


  /*
   * Итоговая оценка положения
   */

  float angle = 0;





  /*
   * Raw encoder
   */

  int32_t encoder = 0;





  /*
   * Доступность источников
   */

  bool mpu_valid = false;


  bool encoder_valid = false;



};









/*
 * Состояние движения
 */

struct MotionState
{

  bool moving = false;


  int8_t direction = 0;


  float target_position = 0;


};









} // namespace jalouzee_blinds
} // namespace esphome
