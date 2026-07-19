#pragma once


#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"


namespace esphome {
namespace jalouzee_blinds {


class JalouzeeBlinds;




class CalibrationStartButton :
    public button::Button
{

public:


    void set_parent(
        JalouzeeBlinds *parent
    )
    {
        parent_ = parent;
    }



protected:


    void press_action() override;



    JalouzeeBlinds *parent_{nullptr};

};







class SaveClosedButton :
    public button::Button
{

public:


    void set_parent(
        JalouzeeBlinds *parent
    )
    {
        parent_ = parent;
    }



protected:


    void press_action() override;



    JalouzeeBlinds *parent_{nullptr};

};








class SaveOpenButton :
    public button::Button
{

public:


    void set_parent(
        JalouzeeBlinds *parent
    )
    {
        parent_ = parent;
    }



protected:


    void press_action() override;



    JalouzeeBlinds *parent_{nullptr};

};









class ClearFaultButton :
    public button::Button
{

public:


    void set_parent(
        JalouzeeBlinds *parent
    )
    {
        parent_ = parent;
    }



protected:


    void press_action() override;



    JalouzeeBlinds *parent_{nullptr};

};









class AngleSourceSelect :
    public select::Select
{

public:


    void set_parent(
        JalouzeeBlinds *parent
    )
    {
        parent_ = parent;
    }




protected:


    void control(
        const std::string &value
    ) override;



    JalouzeeBlinds *parent_{nullptr};

};




}
}
