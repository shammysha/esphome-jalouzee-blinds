#include "motor.h"


#include "esphome/core/log.h"



namespace esphome {
namespace jalouzee_blinds {



static const char *TAG =
    "jalouzee_motor";





DualGPIOMotor::DualGPIOMotor()
{

}






void DualGPIOMotor::setup()
{


    if(open_pin_)
    {

        open_pin_->setup();

        open_pin_->digital_write(
            false
        );

    }



    if(close_pin_)
    {

        close_pin_->setup();

        close_pin_->digital_write(
            false
        );

    }



    direction_ =
        MotorDirection::STOP;



}








void DualGPIOMotor::open()
{

    if(
       direction_
       ==
       MotorDirection::OPEN
      )
    {
        return;
    }



    ESP_LOGD(
        TAG,
        "Motor OPEN"
    );



    /*
       Сначала выключаем всё

       чтобы исключить
       короткое замыкание
    */


    stop();



    write_state(
        true,
        false
    );



    direction_ =
        MotorDirection::OPEN;

}








void DualGPIOMotor::close()
{


    if(
       direction_
       ==
       MotorDirection::CLOSE
      )
    {
        return;
    }




    ESP_LOGD(
        TAG,
        "Motor CLOSE"
    );



    stop();



    write_state(
        false,
        true
    );



    direction_ =
        MotorDirection::CLOSE;

}









void DualGPIOMotor::stop()
{

    if(
       open_pin_
    )
    {

        open_pin_->digital_write(
            false
        );

    }



    if(
       close_pin_
    )
    {

        close_pin_->digital_write(
            false
        );

    }



    direction_ =
        MotorDirection::STOP;


}









void DualGPIOMotor::write_state(
    bool open,
    bool close
)
{

    /*
       Защита от ошибки программирования

       никогда не допускаем:

       OPEN=1
       CLOSE=1

    */


    if(
       open &&
       close
      )
    {

        ESP_LOGE(
            TAG,
            "Invalid motor state"
        );


        stop();

        return;

    }





    if(open_pin_)
    {

        open_pin_->digital_write(
            open
        );

    }



    if(close_pin_)
    {

        close_pin_->digital_write(
            close
        );

    }



}



}
}
