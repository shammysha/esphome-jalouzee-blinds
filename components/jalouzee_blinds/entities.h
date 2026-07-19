#pragma once

#include "esphome/core/component.h"

#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"

#include "jalouzee_blinds.h"

namespace esphome {
  namespace jalouzee_blinds {

    class JalouzeeBlinds;

    class JalouzeeButton: public button::Button {

      public:

        enum class Action : uint8_t {

          START_CALIBRATION,

          SAVE_CLOSED,

          SAVE_OPEN,

          CLEAR_FAULT

        };

        void set_parent(JalouzeeBlinds *parent);

        void set_action(Action action);

      protected:

        void press_action() override;

      protected:

        JalouzeeBlinds *parent_ { nullptr };

        Action action_ { Action::START_CALIBRATION };

    };

    class AngleSourceSelect: public select::Select {

      public:

        void set_parent(JalouzeeBlinds *parent);

        void setup() override;

      protected:

        void control(const std::string &value) override;

      protected:

        JalouzeeBlinds *parent_ { nullptr };

    };

  } // namespace jalouzee_blinds
} // namespace esphome
