#include "state_manager.h"



namespace esphome {
namespace jalouzee_blinds {



void StateManager::reset()
{

    status_ =
        BlindStatus::UNKNOWN;



    calibration_state_ =
        CalibrationState::NONE;



    fault_ =
        false;



    calibrated_ =
        false;


}









void StateManager::set_status(
    BlindStatus status
)
{

    status_ =
        status;


}









void StateManager::set_calibration_state(
    CalibrationState state
)
{

    calibration_state_ =
        state;


}









void StateManager::set_fault(
    bool value
)
{

    fault_ =
        value;



    if(value)
    {

        status_ =
            BlindStatus::FAULT;

    }


}









bool StateManager::can_move() const
{

    if(fault_)
        return false;



    return
        status_ == BlindStatus::READY ||
        status_ == BlindStatus::MOVING;


}









bool StateManager::calibrated() const
{

    return calibrated_;

}









void StateManager::set_calibrated(
    bool value
)
{

    calibrated_ =
        value;


}






}
}
