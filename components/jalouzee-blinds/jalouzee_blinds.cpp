#include "jalouzee_blinds.h"

#include "esphome/core/log.h"



namespace esphome {
namespace jalouzee_blinds {



static const char *const TAG =
    "jalouzee_blinds";





JalouzeeBlinds::JalouzeeBlinds()
{

}









void JalouzeeBlinds::setup()
{

  ESP_LOGI(
      TAG,
      "Starting Jalouzee Blinds"
  );



  /*
   * Motor pins
   */


  if(
      motor_open_pin_ != nullptr
  )
  {
    motor_open_pin_->setup();
  }


  if(
      motor_close_pin_ != nullptr
  )
  {
    motor_close_pin_->setup();
  }


  motor_stop();




  /*
   * Preferences
   */


  preference_ =
      global_preferences->make_preference<PersistentData>(
          DATA_PREF_KEY
      );



  if(
      preference_.load(
          &data_
      )
  )
  {

    if(
        data_.magic != DATA_MAGIC
    )
    {

      ESP_LOGW(
          TAG,
          "Invalid stored data"
      );

      data_ =
          PersistentData();

    }

  }
  else
  {

    ESP_LOGI(
        TAG,
        "No stored calibration data"
    );

  }




  /*
   * Restore runtime state
   */


  source_ =
      static_cast<AngleSource>(
          data_.angle_source
      );



  if(
      data_.position <=
      static_cast<uint8_t>(
          BlindPosition::UNKNOWN
      )
  )
  {
      position_ =
          static_cast<BlindPosition>(
              data_.position
          );
  }
  else
  {
      position_ =
          BlindPosition:HALF;
  }


  if(
      data_.fault
  )
  {

    state_ =
        BlindState::FAULT;

  }

  if(
      data_.calibrated
  )
  {
      calibration_.restore(
          data_.closed_angle,
          data_.open_angle,
          data_.inverted
      );
  }


  angle_sensor_.set_source(
      source_
  );




  publish_entities();

}









float JalouzeeBlinds::get_setup_priority() const
{

  return setup_priority::DATA;


}









cover::CoverTraits JalouzeeBlinds::get_traits()
{

  auto traits =
      cover::CoverTraits();



  traits.set_is_assumed_state(
      false
  );



  traits.set_supports_stop(
      true
  );



  traits.set_supports_position(
      true
  );



  return traits;

}

void JalouzeeBlinds::move_to(
    BlindPosition target
)
{

  if(
      !data_.calibrated
  )
  {

    ESP_LOGW(
        TAG,
        "Cannot move: not calibrated"
    );

    return;

  }


  switch(target)
  {

    case BlindPosition::CLOSED:

      target_angle_ =
          data_.closed_angle;

      break;


    case BlindPosition::OPEN:

      target_angle_ =
          data_.open_angle;

      break;


    case BlindPosition::HALF:

      target_angle_ =
          (
              data_.closed_angle +
              data_.open_angle
          )
          /
          2.0f;

      break;


    default:

      return;

  }



  float current =
      get_angle();



  bool increasing =
      target_angle_ > current;



  if(
      data_.inverted
  )
  {
    increasing =
        !increasing;
  }



  movement_start_time_ =
      millis();


  last_angle_change_time_ =
      millis();


  last_angle_ =
      current;



  if(increasing)
  {

    motor_open();


    state_ =
        BlindState::MOVING_OPEN;


  }
  else
  {

    motor_close();


    state_ =
        BlindState::MOVING_CLOSE;

  }



  ESP_LOGI(
      TAG,
      "Moving to %.2f",
      target_angle_
  );

}


void JalouzeeBlinds::control(
    const cover::CoverCall &call
)
{


  if(
      has_fault()
  )
  {

    ESP_LOGW(
        TAG,
        "Command ignored: FAULT state"
    );

    return;

  }




  /*
   * STOP command
   */


  if(
      call.get_stop()
  )
  {

    motor_stop();


    state_ =
        BlindState::IDLE;


    ESP_LOGI(
        TAG,
        "Movement stopped"
    );


    return;

  }







  /*
   * Position command
   *
   * HA slider:
   *
   * 0.0 = closed
   * 0.5 = half
   * 1.0 = open
   *
   */


  if(
      call.get_position().has_value()
  )
  {

    float pos =
        *call.get_position();



    if(
        pos <= 0.05f
    )
    {

      move_to(
          BlindPosition::CLOSED
      );

    }
    else if(
        pos >= 0.95f
    )
    {

      move_to(
          BlindPosition::OPEN
      );

    }
    else
    {

      move_to(
          BlindPosition::HALF
      );

    }


    return;

  }







  /*
   * OPEN button
   */


  if(
      call.get_command_open()
  )
  {


    switch(position_)
    {


      case BlindPosition::CLOSED:


        move_to(
            BlindPosition::HALF
        );

        break;



      case BlindPosition::HALF:


        move_to(
            BlindPosition::OPEN
        );

        break;



      default:


        move_to(
            BlindPosition::OPEN
        );

        break;

    }


    return;

  }








  /*
   * CLOSE button
   */


  if(
      call.get_command_close()
  )
  {


    switch(position_)
    {


      case BlindPosition::OPEN:


        move_to(
            BlindPosition::HALF
        );


        break;



      case BlindPosition::HALF:


        move_to(
            BlindPosition::CLOSED
        );


        break;



      default:


        move_to(
            BlindPosition::CLOSED
        );


        break;


    }


    return;

  }


}


void JalouzeeBlinds::update_position() {
  this->position =
      (angle - closed) /
      (open - closed);

}

void JalouzeeBlinds::loop()
{

  if(
      state_ != BlindState::MOVING_OPEN &&
      state_ != BlindState::MOVING_CLOSE
  )
  {
    return;
  }





  uint32_t now =
      millis();



  float angle =
      get_angle();

  if(!angle_available())
  {
      if(
         now - movement_start_time_
         >
         3000
      )
      {
          ESP_LOGE(
              TAG,
              "Angle sensor timeout"
          );

          motor_stop();
          set_fault();
      }

      return;
  }

  /*
   * Проверяем изменение угла
   */


  if(
      fabs(
          angle -
          last_angle_
      ) > 0.3f
  )
  {

    last_angle_ =
        angle;


    last_angle_change_time_ =
        now;


    data_.current_angle =
        angle;


  }





  /*
   * Контроль зависания
   */


  if(
      now -
      last_angle_change_time_
      >
      stall_timeout_
  )
  {


    ESP_LOGE(
        TAG,
        "Movement timeout: angle does not change"
    );



    motor_stop();



    set_fault();



    return;

  }








  /*
   * Достигли цели
   */


  bool reached = false;


  if(
      fabs(
          angle - target_angle_
      ) < 1.0f
  )
  {
      reached = true;
  }








  if(
      reached
  )
  {

    motor_stop();



    state_ =
        BlindState::IDLE;



    if(
        fabs(
            target_angle_ -
            data_.closed_angle
        )
        <
        1.0f
    )
    {

      position_ =
          BlindPosition::CLOSED;

    }
    else if(
        fabs(
            target_angle_ -
            data_.open_angle
        )
        <
        1.0f
    )
    {

      position_ =
          BlindPosition::OPEN;

    }
    else
    {

      position_ =
          BlindPosition::HALF;

    }





    data_.position =
        static_cast<uint8_t>(
            position_
        );



    data_.current_angle =
        angle;



    save_state();



    publish_entities();



    ESP_LOGI(
        TAG,
        "Movement complete"
    );


  }


}










void JalouzeeBlinds::set_fault()
{

  state_ =
      BlindState::FAULT;



  data_.fault =
      true;



  motor_stop();



  save_state();



  publish_entities();



  ESP_LOGE(
      TAG,
      "BLINDS FAULT"
  );


}









bool JalouzeeBlinds::has_fault()
{

  return
      state_ ==
      BlindState::FAULT;

}









void JalouzeeBlinds::clear_fault()
{

  data_.fault =
      false;



  state_ =
      BlindState::IDLE;



  save_state();



  publish_entities();



  ESP_LOGW(
      TAG,
      "Fault cleared"
  );


}


void JalouzeeBlinds::set_motor_pins(
    GPIOPin *open_pin,
    GPIOPin *close_pin
)
{

  motor_open_pin_ =
      open_pin;


  motor_close_pin_ =
      close_pin;

}








void JalouzeeBlinds::motor_open()
{

  if(
      motor_close_pin_
  )
  {

    motor_close_pin_->digital_write(false);

  }



  if(
      motor_open_pin_
  )
  {

    motor_open_pin_->digital_write(true);

  }


  ESP_LOGD(
      TAG,
      "Motor OPEN"
  );

}








void JalouzeeBlinds::motor_close()
{

  if(
      motor_open_pin_
  )
  {

    motor_open_pin_->digital_write(false);

  }



  if(
      motor_close_pin_
  )
  {

    motor_close_pin_->digital_write(true);

  }


  ESP_LOGD(
      TAG,
      "Motor CLOSE"
  );

}








void JalouzeeBlinds::motor_stop()
{

  if(
      motor_open_pin_
  )
  {

    motor_open_pin_->digital_write(false);

  }



  if(
      motor_close_pin_
  )
  {

    motor_close_pin_->digital_write(false);

  }


  ESP_LOGD(
      TAG,
      "Motor STOP"
  );

}









void JalouzeeBlinds::set_primary_sensor(
    sensor::Sensor *sensor
)
{

  angle_sensor_.set_primary(
      sensor
  );

}








void JalouzeeBlinds::set_secondary_sensor(
    sensor::Sensor *sensor
)
{

  angle_sensor_.set_secondary(
      sensor
  );

}








void JalouzeeBlinds::set_angle_source(
    AngleSource source
)
{

  source_ =
      source;


  angle_sensor_.set_source(
      source
  );


  data_.angle_source =
      static_cast<uint8_t>(
          source
      );


  save_state();

}








float JalouzeeBlinds::get_angle()
{

  return
      angle_sensor_.get_angle();

}








bool JalouzeeBlinds::angle_available()
{

  return
      angle_sensor_.available();

}









void JalouzeeBlinds::start_calibration()
{

  if(
      state_ ==
      BlindState::FAULT
  )
  {
    return;
  }


  state_ =
      BlindState::CALIBRATION;


  calibration_.start();



  ESP_LOGI(
      TAG,
      "Manual calibration started"
  );


}








void JalouzeeBlinds::save_closed_position()
{

  if(!angle_available())
  {
      ESP_LOGE(
          TAG,
          "Cannot save closed: angle unavailable"
      );

      return;
  }

  calibration_.set_closed_angle(
      get_angle()
  );


}








void JalouzeeBlinds::save_open_position()
{

  if(!angle_available())
  {
      ESP_LOGE(
          TAG,
          "Cannot save closed: angle unavailable"
      );

      return;
  }

  calibration_.set_open_angle(
      get_angle()
  );



  if(
      calibration_.is_complete()
  )
  {


    data_.closed_angle =
        calibration_.closed_angle();



    data_.open_angle =
        calibration_.open_angle();



    data_.inverted =
        calibration_.inverted();



    data_.calibrated =
        true;



    state_ =
        BlindState::IDLE;



    save_state();



    publish_entities();


  }

}








bool JalouzeeBlinds::is_calibrated()
{

  return
      data_.calibrated;

}








void JalouzeeBlinds::set_stall_timeout(
    uint32_t timeout
)
{

  stall_timeout_ =
      timeout;

}









void JalouzeeBlinds::load_state()
{

  preference_.load(
      &data_
  );

}









void JalouzeeBlinds::save_state()
{

  data_.magic =
      DATA_MAGIC;


  preference_.save(
      &data_
  );

}









void JalouzeeBlinds::publish_entities()
{

  /*
   * Cover position
   */


  switch(position_)
  {


    case BlindPosition::CLOSED:

      this->position =
          0.0f;

      break;


    case BlindPosition::HALF:

      this->position =
          0.5f;

      break;


    case BlindPosition::OPEN:

      this->position =
          1.0f;

      break;


    default:

      break;

  }


  this->publish_state();



  if(
      angle_output_
  )
  {

    angle_output_->publish_state(
        get_angle()
    );

  }




  if(
      position_output_
  )
  {

    position_output_->publish_state(
        static_cast<float>(
            static_cast<uint8_t>(
                position_
            )
        )
    );

  }




  if(
      fault_output_
  )
  {

    fault_output_->publish_state(
        has_fault()
    );

  }




  if(
      calibrated_output_
  )
  {

    calibrated_output_->publish_state(
        data_.calibrated
    );

  }

}








void JalouzeeBlinds::set_angle_output(
    sensor::Sensor *sensor
)
{

  angle_output_ =
      sensor;

}








void JalouzeeBlinds::set_position_output(
    sensor::Sensor *sensor
)
{

  position_output_ =
      sensor;

}








void JalouzeeBlinds::set_fault_output(
    binary_sensor::BinarySensor *sensor
)
{

  fault_output_ =
      sensor;

}








void JalouzeeBlinds::set_calibrated_output(
    binary_sensor::BinarySensor *sensor
)
{

  calibrated_output_ =
      sensor;

}


AngleSource JalouzeeBlinds::angle_source() const
{
    return source_;
}


}  // namespace jalouzee_blinds
}  // namespace esphome
