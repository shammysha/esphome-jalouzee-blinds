#pragma once


#include "esphome/core/gpio.h"


#include <cstdint>



namespace esphome {
namespace jalouzee_blinds {



enum class EncoderStatus : uint8_t
{

  OK = 0,

  NO_SIGNAL,

  INVALID

};






class MotorDriver
{


 public:


  MotorDriver();




  void setup();


  void loop();






  /*
   * Motor GPIO
   */

  void set_pins(
      GPIOPin *in1,
      GPIOPin *in2
  );





  /*
   * Encoder GPIO
   */

  void set_encoder_pins(
      GPIOPin *a,
      GPIOPin *b
  );






  /*
   * Motor control
   */

  void move_open();


  void move_close();


  void stop();






  /*
   * Encoder position
   */

  int32_t encoder_position()
      const;



  void reset_encoder();






  /*
   * Encoder direction correction
   *
   * Set only after calibration commit
   */

  void set_encoder_sign(
      int8_t sign
  );



  int8_t encoder_sign()
      const;






  /*
   * Status
   */

  bool running()
      const;



  bool has_error()
      const;



  EncoderStatus encoder_status()
      const;






 protected:



  void update_encoder();



  void check_encoder();








 protected:



  GPIOPin *in1_ =
      nullptr;



  GPIOPin *in2_ =
      nullptr;







  GPIOPin *encoder_a_ =
      nullptr;



  GPIOPin *encoder_b_ =
      nullptr;






  volatile int32_t encoder_count_ =
      0;






  /*
   * Correction of mechanical installation
   *
   * +1 / -1
   */

  int8_t encoder_sign_ =
      1;







  uint8_t encoder_state_ =
      0;







  /*
   * Motor direction
   *
   * -1 close
   *  0 stop
   * +1 open
   */

  int8_t motor_direction_ =
      0;







  /*
   * Encoder monitoring
   */

  uint32_t last_encoder_time_ =
      0;



  static constexpr uint32_t ENCODER_TIMEOUT =
      3000;







  EncoderStatus encoder_status_ =
      EncoderStatus::OK;






  /*
   * Real motor driver error
   */

  bool error_ =
      false;



};




} // namespace jalouzee_blinds
} // namespace esphome
