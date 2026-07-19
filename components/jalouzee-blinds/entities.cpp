#include "entities.h"

#include "jalouzee_blinds.h"


namespace esphome {
namespace jalouzee_blinds {





void CalibrationStartButton::press_action()
{

    if(parent_)
    {
        parent_->
            start_calibration();
    }

}








void SaveClosedButton::press_action()
{

    if(parent_)
    {
        parent_->
            save_closed_position();
    }

}








void SaveOpenButton::press_action()
{

    if(parent_)
    {
        parent_->
            save_open_position();
    }

}








void ClearFaultButton::press_action()
{

    if(parent_)
    {
        parent_->
            clear_fault();
    }

}








void AngleSourceSelect::control(
    const std::string &value
)
{


    if(!parent_)
        return;



    if(value == "AUTO")
    {

        parent_->
            set_angle_source(
                AngleSource::AUTO
            );

    }
    else
    if(value == "MPU6050")
    {

        parent_->
            set_angle_source(
                AngleSource::MPU6050
            );

    }
    else
    if(value == "HALL")
    {

        parent_->
            set_angle_source(
                AngleSource::HALL
            );

    }



    publish_state(
        value
    );


}





}
}
