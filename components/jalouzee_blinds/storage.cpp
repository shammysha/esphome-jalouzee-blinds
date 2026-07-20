#include "storage.h"

#include "esphome/core/log.h"

#include <cstring>

namespace esphome {
  namespace jalouzee_blinds {

    static const char *TAG = "jalouzee_storage";

    /*
     * В дальнейшем сюда можно
     * добавить ESPHome Preferences
     *
     * Сейчас оставляем слой
     * изолированным.
     */

    struct StoredCalibration {

        uint32_t magic;

        CalibrationData data;

        uint32_t crc;

    };

    Storage::Storage() {

    }

    void Storage::setup() {

    }

    bool Storage::load(CalibrationData &data) {

      /*
       * Заглушка уровня хранения.
       *
       * Реальная реализация
       * будет через ESPHome
       * global_preferences.
       */

      StoredCalibration stored;

      /*
       * Здесь:
       *
       * preferences->load()
       *
       */

      if (stored.magic != MAGIC) {

        ESP_LOGI(TAG, "No calibration stored");

        return false;

      }

      if (stored.crc != checksum(stored.data)) {

        ESP_LOGW(TAG, "Calibration CRC error");

        return false;

      }

      data = stored.data;

      return data.valid;

    }

    bool Storage::save(const CalibrationData &data) {

      if (!data.valid) {

        ESP_LOGW(TAG, "Refuse invalid calibration");

        return false;

      }

      StoredCalibration stored;

      stored.magic = MAGIC;

      stored.data = data;

      stored.crc = checksum(data);

      /*
       *
       * Реальная запись:
       *
       * preferences->save()
       *
       */

      ESP_LOGI(TAG, "Calibration saved");

      return true;

    }

    uint32_t Storage::checksum(const CalibrationData &data) {

      const uint8_t *ptr = reinterpret_cast<const uint8_t*>(&data);

      uint32_t crc = 0;

      for (size_t i = 0; i < sizeof(data); i++) {

        crc = crc * 31 + ptr[i];

      }

      return crc;

    }

  } // namespace jalouzee_blinds
} // namespace esphome
