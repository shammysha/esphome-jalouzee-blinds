#pragma once

#include <cstdint>

namespace esphome {
namespace jalouzee {

//
// Общее состояние компонента
//
enum class SystemState : uint8_t {
  IDLE,          // ожидание
  MOVING,        // выполняется перемещение
  CALIBRATION,   // режим калибровки
  ERROR          // критическая ошибка
};

//
// Направление движения
//
enum class Direction : int8_t {
  STOP  = 0,
  OPEN  = 1,
  CLOSE = -1
};

//
// Режим работы
//
enum class OperationMode : uint8_t {
  NORMAL,        // оба датчика исправны
  IMU_ONLY,      // только MPU6050
  ENCODER_ONLY,  // только энкодер
  DEGRADED       // работа в деградированном режиме
};

//
// Этап калибровки
//
enum class CalibrationStage : uint8_t {
  NONE,

  WAIT_CLOSED,
  CLOSED_STORED,

  WAIT_OPEN,
  READY_TO_SAVE
};

//
// Состояние датчиков
//
struct SensorStatus {

  bool imu_ok = false;

  bool encoder_ok = false;

  bool motor_ok = true;

  float imu_confidence = 0.0f;

  float encoder_confidence = 0.0f;
};

//
// Текущие данные датчиков
//
struct SensorData {

  // Абсолютный угол ламели (градусы)
  float angle = 0.0f;

  // Текущее положение энкодера
  // (от момента включения ESP)
  int32_t encoder = 0;

  // Скорость вращения
  float speed = 0.0f;

  uint32_t timestamp = 0;
};

//
// Постоянная калибровка
//
struct CalibrationData {

  bool valid = false;

  //
  // Абсолютные углы ламели
  //

  float closed_angle = 0.0f;

  float open_angle = 0.0f;

  //
  // Количество импульсов между
  // крайними положениями
  //

  int32_t encoder_range = 0;

  //
  // Направление изменения
  // +1 или -1
  //

  int8_t imu_sign = 1;

  int8_t encoder_sign = 1;
};

//
// Временные данные калибровки
//
struct CalibrationRuntime {

  CalibrationStage stage =
      CalibrationStage::NONE;

  float closed_angle = 0.0f;

  float open_angle = 0.0f;

  int32_t closed_encoder = 0;

  int32_t open_encoder = 0;
};

//
// Целевое движение
//
struct MotionTarget {

  float target_position = 0.0f;

  float target_angle = 0.0f;

  Direction direction =
      Direction::STOP;

  bool active = false;
};

}  // namespace jalouzee
}  // namespace esphome
