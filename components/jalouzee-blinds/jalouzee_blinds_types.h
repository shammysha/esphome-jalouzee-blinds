#pragma once

#include <cstdint>

namespace esphome {
  namespace jalouzee_blinds {

    static constexpr uint32_t DATA_PREF_KEY = 0x4A424C42;
    static constexpr uint32_t DATA_MAGIC = 0x4A424C31;

    enum class BlindPosition : uint8_t {

      CLOSED = 0,

      HALF = 1,

      OPEN = 2,

      UNKNOWN = 3

    };

    enum class BlindState : uint8_t {

      IDLE = 0,

      MOVING_OPEN = 1,

      MOVING_CLOSE = 2,

      CALIBRATION = 3,

      FAULT = 4

    };

    enum class AngleSource : uint8_t {

      AUTO = 0,

      PRIMARY = 1,

      SECONDARY = 2

    };

  }  // namespace jalouzee_blinds
}  // namespace esphome
