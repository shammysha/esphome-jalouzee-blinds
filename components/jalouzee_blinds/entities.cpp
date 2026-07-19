#include "entities.h"
#include "jalouzee_blinds.h"
#include "esphome/core/log.h"

namespace esphome {
  namespace jalouzee_blinds {

    static const char *const TAG = "jalouzee_blinds.entities";

    void JalouzeeButton::set_parent(JalouzeeBlinds *parent) {

      parent_ = parent;

    }

    void JalouzeeButton::set_action(Action action) {

      action_ = action;

    }

    void JalouzeeButton::press_action() {

      if (parent_ == nullptr) {
        return;
      }

      switch (action_) {

        case Action::START_CALIBRATION:

          parent_->start_calibration();

          ESP_LOGI(TAG, "Start calibration requested");

          this->set_action(Action::SAVE_CLOSED);

          break;

        case Action::SAVE_CLOSED:

          ESP_LOGI(TAG, "Save closed requested");

          parent_->save_closed_position();

          this->set_action(Action::SAVE_OPEN);

          break;

        case Action::SAVE_OPEN:

          ESP_LOGI(TAG, "Save open requested");

          parent_->save_open_position();

          this->set_action(Action::START_CALIBRATION);

          break;

        case Action::CLEAR_FAULT:
          ESP_LOGI(TAG, "Clear fault requested");

          parent_->clear_fault();

          break;

      }

    }

    void AngleSourceSelect::set_parent(JalouzeeBlinds *parent) {

      parent_ = parent;

    }

    void AngleSourceSelect::setup() {

      this->traits.set_options( { "AUTO", "PRIMARY", "SECONDARY" });

      if (parent_ == nullptr) {
        ESP_LOGW(TAG, "Angle source select without parent");

        return;
      }

      switch (parent_->angle_source()) {

        case AngleSource::AUTO:

          this->publish_state("AUTO");

          break;

        case AngleSource::PRIMARY:

          this->publish_state("PRIMARY");

          break;

        case AngleSource::SECONDARY:

          this->publish_state("SECONDARY");

          break;

      }

    }

    void AngleSourceSelect::control(const std::string &value) {

      if (parent_ == nullptr) {
        return;
      }

      bool accepted = false;

      if (value == "AUTO") {

        parent_->set_angle_source(AngleSource::AUTO);

        accepted = true;

      } else if (value == "PRIMARY") {

        parent_->set_angle_source(AngleSource::PRIMARY);

        accepted = true;

      } else if (value == "SECONDARY") {

        parent_->set_angle_source(AngleSource::SECONDARY);

        accepted = true;

      }

      if (accepted) {
        this->publish_state(value);
      }
    }

  } // namespace jalouzee_blinds
} // namespace esphome
