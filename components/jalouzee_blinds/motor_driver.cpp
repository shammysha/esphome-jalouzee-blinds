#include "motor_driver.h"


#include "esphome/core/log.h"
#include "esphome/core/hal.h"



namespace esphome {
namespace jalouzee_blinds {



static const char *TAG =
    "jalouzee_motor";






/*
 * Quadrature decoder
 */

static const int8_t QUAD_TABLE[16] =
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};









MotorDriver::MotorDriver()
{

}









void MotorDriver::setup()
{

  if(
      in1_
  )
      in1_->setup();



  if(
      in2_
  )
      in2_->setup();





  if(
      encoder_a_
  )
      encoder_a_->setup();



  if(
      encoder_b_
  )
      encoder_b_->setup();





  if(
      encoder_a_ &&
      encoder_b_
  )
  {

    encoder_state_ =
        (
          encoder_a_->digital_read()
          << 1
        )
        |
        encoder_b_->digital_read();

  }



  last_encoder_time_ =
      millis();

}









void MotorDriver::loop()
{

  update_encoder();


  check_encoder();

}









void MotorDriver::set_pins(
    GPIOPin *in1,
    GPIOPin *in2
)
{

  in1_ =
      in1;


  in2_ =
      in2;

}









void MotorDriver::set_encoder_pins(
    GPIOPin *a,
    GPIOPin *b
)
{

  encoder_a_ =
      a;


  encoder_b_ =
      b;

}









void MotorDriver::move_open()
{

  if(
      !in1_ ||
      !in2_
  )
      return;



  in1_->digital_write(
      true
  );


  in2_->digital_write(
      false
  );



  motor_direction_ =
      1;

}









void MotorDriver::move_close()
{

  if(
      !in1_ ||
      !in2_
  )
      return;



  in1_->digital_write(
      false
  );


  in2_->digital_write(
      true
  );



  motor_direction_ =
      -1;

}









void MotorDriver::stop()
{

  if(
      in1_
  )
      in1_->digital_write(
          false
      );



  if(
      in2_
  )
      in2_->digital_write(
          false
      );



  motor_direction_ =
      0;

}









int32_t MotorDriver::encoder_position()
    const
{

  return
      encoder_count_ *
      encoder_sign_;

}









void MotorDriver::reset_encoder()
{

  encoder_count_ =
      0;

}









void MotorDriver::set_encoder_sign(
    int8_t sign
)
{

  encoder_sign_ =
      sign >= 0
      ?
      1
      :
      -1;

}









int8_t MotorDriver::encoder_sign()
    const
{

  return encoder_sign_;

}









bool MotorDriver::running()
    const
{

  return
      motor_direction_ != 0;

}









bool MotorDriver::has_error()
    const
{

  return error_;

}









EncoderStatus MotorDriver::encoder_status()
    const
{

  return encoder_status_;

}









void MotorDriver::update_encoder()
{

  if(
      !encoder_a_ ||
      !encoder_b_
  )
  {

    encoder_status_ =
        EncoderStatus::INVALID;


    return;

  }





  uint8_t state =
      (
        encoder_a_->digital_read()
        << 1
      )
      |
      encoder_b_->digital_read();






  uint8_t index =
      (
        encoder_state_
        << 2
      )
      |
      state;






  int8_t delta =
      QUAD_TABLE[index];





  if(
      delta != 0
  )
  {

    encoder_count_ += delta;


    last_encoder_time_ =
        millis();


    encoder_status_ =
        EncoderStatus::OK;

  }





  encoder_state_ =
      state;

}









void MotorDriver::check_encoder()
{

  if(
      motor_direction_ == 0
  )
  {

    return;

  }





  uint32_t now =
      millis();





  if(
      now -
      last_encoder_time_
      >
      ENCODER_TIMEOUT
  )
  {


    /*
     * Это НЕ ошибка мотора.
     *
     * Просто Hall недоступен.
     */

    encoder_status_ =
        EncoderStatus::NO_SIGNAL;




    ESP_LOGW(
        TAG,
        "Encoder signal lost"
    );


  }

}







} // namespace jalouzee_blinds
} // namespace esphome
