#include "motor.h"



namespace esphome {
namespace jalouzee_blinds {



void GPIOMotor::setup()
{

    if(open_pin_)
    {

        open_pin_->setup();

    }



    if(close_pin_)
    {

        close_pin_->setup();

    }



    stop();


}









void GPIOMotor::open()
{

    stop();



    if(open_pin_)
    {

        open_pin_->digital_write(
            true
        );

    }


}









void GPIOMotor::close()
{

    stop();



    if(close_pin_)
    {

        close_pin_->digital_write(
            true
        );

    }


}









void GPIOMotor::stop()
{


    if(open_pin_)
    {

        open_pin_->digital_write(
            false
        );

    }




    if(close_pin_)
    {

        close_pin_->digital_write(
            false
        );

    }


}





}
}
