#include "jalouzee_blinds.h"


#include "esphome/core/log.h"
#include "esphome/core/hal.h"



namespace esphome {
namespace jalouzee_blinds {



static const char *TAG =
    "jalouzee_blinds";









JalouzeeBlinds::JalouzeeBlinds()
    :
    motor_(),
    sensors_(
        &motor_
    ),
    calibration_(),
    storage_(),
    position_controller_(
        &sensors_,
        &calibration_,
        &motor_
    )
{

}









void JalouzeeBlinds::setup()
{

  ESP_LOGCONFIG(
      TAG,
      "Setting up JalouzeeBlinds"
  );





  /*
   * Storage first
   */

  storage_.setup();






  /*
   * Motor hardware
   */

  motor_.setup();






  /*
   * Sensors
   */

  sensors_.setup();







  /*
   * Restore calibration
   */

  CalibrationData data;



  if(
      storage_.load(
          data
      )
  )
  {


    calibration_.load(
        data
    );



    motor_.set_encoder_sign(
        data.encoder_sign
    );



    sensors_.set_imu_sign(
        data.imu_sign
    );



    ESP_LOGI(
        TAG,
        "Calibration restored"
    );

  }
  else
  {

    ESP_LOGW(
        TAG,
        "Calibration not found"
    );

  }







  /*
   * MPU callback
   */

  if(
      angle_sensor_ != nullptr
  )
  {


    angle_sensor_
        ->add_on_state_callback(
            [this](float value)
            {


              sensors_.set_mpu_angle(
                  value
              );


            }
        );


  }







  if(
      calibration_button_ != nullptr
  )
  {

    calibration_button_
        ->setup();

  }



  if(
      cancel_button_ != nullptr
  )
  {

    cancel_button_
        ->setup();

  }



}









void JalouzeeBlinds::loop()
{

  motor_.loop();



  sensors_.update();




  check_buttons();




  position_controller_
      .update();





  publish_position();





  if(
      motor_.has_error()
  )
  {

    handle_error();

  }


}









void JalouzeeBlinds::control(
    const cover::CoverCall &call
)
{


  if(
      call.get_position()
          .has_value()
  )
  {

    float position =
        *call.get_position();



    position_controller_
        .set_position(
            position *
            100.0f
        );

  }






  if(
      call.get_stop()
  )
  {

    position_controller_
        .stop();

  }


}









cover::CoverTraits
JalouzeeBlinds::get_traits()
    const
{

  auto traits =
      cover::CoverTraits();



  traits.set_supports_position(
      true
  );


  traits.set_supports_stop(
      true
  );



  return traits;

}









void JalouzeeBlinds::set_motor_pins(
    GPIOPin *in1,
    GPIOPin *in2
)
{

  motor_.set_pins(
      in1,
      in2
  );

}









void JalouzeeBlinds::set_encoder_pins(
    GPIOPin *a,
    GPIOPin *b
)
{

  motor_.set_encoder_pins(
      a,
      b
  );

}









void JalouzeeBlinds::set_angle_sensor(
    sensor::Sensor *sensor
)
{

  angle_sensor_ =
      sensor;

}









void JalouzeeBlinds::set_calibration_button(
    GPIOPin *pin
)
{

  calibration_button_ =
      pin;

}









void JalouzeeBlinds::set_cancel_button(
    GPIOPin *pin
)
{

  cancel_button_ =
      pin;

}









void JalouzeeBlinds::check_buttons()
{

  if(
      calibration_button_ != nullptr
  )
  {


    bool pressed =
        !calibration_button_
            ->digital_read();





    if(
        pressed &&
        !last_calibration_button_
    )
    {

      calibration_pressed();

    }



    last_calibration_button_ =
        pressed;


  }






  if(
      cancel_button_ != nullptr
  )
  {


    bool pressed =
        !cancel_button_
            ->digital_read();





    if(
        pressed &&
        !last_cancel_button_
    )
    {

      cancel_pressed();

    }



    last_cancel_button_ =
        pressed;


  }

}









void JalouzeeBlinds::calibration_pressed()
{

  SensorData data =
      sensors_.data();




  CalibrationResult result =
      calibration_.next(
          data
      );






  if(
      result ==
      CalibrationResult::OK
  )
  {


    const CalibrationData &pending =
        calibration_.pending();






    /*
     * Здесь единственное место
     * записи новой калибровки
     */

    if(
        storage_.save(
            pending
        )
    )
    {

      calibration_.apply_pending();




      motor_.set_encoder_sign(
          pending.encoder_sign
      );



      sensors_.set_imu_sign(
          pending.imu_sign
      );




      ESP_LOGI(
          TAG,
          "Calibration committed"
      );


    }
    else
    {

      ESP_LOGE(
          TAG,
          "Calibration save failed"
      );

    }


  }

}









void JalouzeeBlinds::cancel_pressed()
{

  calibration_.cancel();



  ESP_LOGI(
      TAG,
      "Calibration cancelled"

  );

}









void JalouzeeBlinds::publish_position()
{

  float position =
      position_controller_
          .position();




  if(
      fabs(
          position -
          last_position_
      )
      < 0.5f
  )
  {

    return;

  }





  last_position_ =
      position;



  this->position =
      position /
      100.0f;



  publish_state();

}









void JalouzeeBlinds::handle_error()
{

  ESP_LOGE(
      TAG,
      "Motor driver error"
  );


  motor_.stop();


  position_controller_
      .stop();



  state_ =
      SystemState::ERROR;

}






} // namespace jalouzee_blinds
} // namespace esphome
