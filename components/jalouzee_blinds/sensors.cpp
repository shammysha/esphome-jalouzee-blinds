#include "sensors.h"


#include "motor_driver.h"


#include "esphome/core/log.h"



namespace esphome {
namespace jalouzee_blinds {



static const char *TAG =
    "jalouzee_sensors";








Sensors::Sensors(
    MotorDriver *motor
)
    :
    motor_(
        motor
    )
{

}









void Sensors::setup()
{

}









void Sensors::update()
{

  update_encoder_state();



  calculate_position();

}









void Sensors::set_mpu_angle(
    float angle
)
{

  /*
   * MPU считается живым
   * только после получения
   * реального значения
   */

  mpu_angle_ =
      angle *
      imu_sign_;



  mpu_valid_ =
      true;

}









void Sensors::set_imu_sign(
    int8_t sign
)
{

  imu_sign_ =
      sign >= 0
      ?
      1
      :
      -1;

}









void Sensors::update_encoder_state()
{

  if(
      motor_ == nullptr
  )
  {

    encoder_valid_ =
        false;


    return;

  }







  EncoderStatus status =
      motor_->encoder_status();





  if(
      status ==
      EncoderStatus::OK
  )
  {

    encoder_position_ =
        motor_->encoder_position();



    encoder_valid_ =
        true;

  }
  else
  {

    encoder_valid_ =
        false;

  }

}









void Sensors::calculate_position()
{


  /*
   * Есть оба источника
   */

  if(
      encoder_valid_
      &&
      mpu_valid_
  )
  {


    /*
     * ВАЖНО:
     *
     * Сейчас оставляем
     * простое объединение.
     *
     * Далее можно заменить
     * фильтром.
     */

    position_ =
        (
          (float)encoder_position_
          +
          mpu_angle_
        )
        /
        2.0f;



    return;

  }







  /*
   * Только Hall
   */

  if(
      encoder_valid_
  )
  {

    position_ =
        encoder_position_;


    return;

  }







  /*
   * Только MPU
   */

  if(
      mpu_valid_
  )
  {

    position_ =
        mpu_angle_;


    return;

  }






  /*
   * Нет источников
   */

  ESP_LOGW(
      TAG,
      "No valid position source"
  );


}









SensorData Sensors::data()
    const
{

  SensorData result;



  result.angle =
      position_;



  result.encoder =
      encoder_position_;



  result.mpu_valid =
      mpu_valid_;



  result.encoder_valid =
      encoder_valid_;




  return result;

}









bool Sensors::position_valid()
    const
{

  return
      encoder_valid_
      ||
      mpu_valid_;

}








} // namespace jalouzee_blinds
} // namespace esphome
