#pragma once


#include <cstdint>


namespace esphome {
namespace jalouzee_blinds {



enum class BlindStatus : uint8_t
{

    UNKNOWN = 0,

    UNCALIBRATED,

    READY,

    MOVING,

    FAULT

};





enum class CalibrationState : uint8_t
{

    NONE = 0,

    WAIT_CLOSED,

    WAIT_OPEN,

    COMPLETE

};









class StateManager
{


public:


    void reset();





    void set_status(
        BlindStatus status
    );



    BlindStatus status() const
    {
        return status_;
    }





    void set_calibration_state(
        CalibrationState state
    );



    CalibrationState calibration_state() const
    {
        return calibration_state_;
    }





    void set_fault(
        bool value
    );



    bool fault() const
    {
        return fault_;
    }





    bool can_move() const;



    bool calibrated() const;



    void set_calibrated(
        bool value
    );







protected:


    BlindStatus status_ =
        BlindStatus::UNKNOWN;



    CalibrationState calibration_state_ =
        CalibrationState::NONE;



    bool fault_{false};



    bool calibrated_{false};



};




}
}
