#include "position_controller.h"


#include "motor_driver.h"

#include "sensors.h"

#include "calibration.h"


#include "esphome/core/log.h"



#include <cmath>



namespace esphome {
namespace jalouzee_blinds {



static const char *TAG =
    "jalouzee_position";



static constexpr float POSITION_TOLERANCE =
    1.0f;









PositionController::PositionController(
    Sensors *sensors,
    Calibration *calibration,
    MotorDriver *motor
)
    :
    sensors_(
        sensors
    ),
    calibration_(
        calibration
    ),
    motor_(
        motor
    )
{

}









void PositionController::update()
{

  if(
      !position_available()
  )
  {

    if(
        moving_
    )
    {

      ESP_LOGE(
          TAG,
          "Position lost, stopping"
      );


      stop();

    }


    return;

  }






  current_position_ =
      calculate_position();





  if(
      moving_
  )
  {

    move_to_target();

  }

}









void PositionController::set_position(
    float position
)
{

  if(
      position < 0
  )
      position = 0;



  if(
      position > 100
  )
      position = 100;




  target_position_ =
      position;



  moving_ =
      true;

}









void PositionController::stop()
{

  if(
      motor_
  )
  {

    motor_->stop();

  }



  moving_ =
      false;

}









float PositionController::position()
    const
{

  return current_position_;

}









bool PositionController::position_available()
    const
{

  if(
      sensors_ == nullptr
  )
      return false;



  SensorData data =
      sensors_->data();



  return
      data.encoder_valid
      ||
      data.mpu_valid;

}









float PositionController::calculate_position()
    const
{

  if(
      calibration_ == nullptr ||
      sensors_ == nullptr
  )
      return 0;






  const CalibrationData &cal =
      calibration_->data();





  SensorData sensor =
      sensors_->data();






  /*
   * Основной вариант:
   * Hall encoder
   */

  if(
      sensor.encoder_valid
  )
  {


    float range =
        (
          float
        )
        (
          cal.open_encoder -
          cal.closed_encoder
        );



    if(
        fabs(range)
        > 1
    )
    {


      float value =
          (
            sensor.encoder -
            cal.closed_encoder
          )
          /
          range;




      return
          value *
          100.0f;

    }

  }








  /*
   * Резерв:
   * MPU6050
   */

  if(
      sensor.mpu_valid
  )
  {


    float range =
        cal.open_angle -
        cal.closed_angle;




    if(
        fabs(range)
        > 0.1f
    )
    {


      float value =
          (
            sensor.angle -
            cal.closed_angle
          )
          /
          range;



      return
          value *
          100.0f;

    }

  }






  return 0;

}









void PositionController::move_to_target()
{

  float delta =
      target_position_
      -
      current_position_;





  if(
      fabs(delta)
      <=
      POSITION_TOLERANCE
  )
  {

    stop();

    return;

  }







  if(
      delta > 0
  )
  {

    motor_->move_open();

  }
  else
  {

    motor_->move_close();

  }

}






} // namespace jalouzee_blinds
} // namespace esphome
