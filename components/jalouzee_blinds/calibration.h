#pragma once

#include "types.h"

namespace esphome {
  namespace jalouzee_blinds {

    class Calibration {

      public:

        Calibration();

        /*
         * Запуск новой калибровки
         */

        void start();

        /*
         * Обработка очередного
         * шага калибровки
         */

        CalibrationResult next(const SensorData &sensor);

        /*
         * Отмена в любой момент
         *
         * Старые данные остаются
         */

        void cancel();

        /*
         * Применение подготовленной
         * калибровки после успешной
         * записи Storage
         */

        void apply_pending();

        /*
         * Загрузка готовой
         * калибровки из Storage
         */

        void load(const CalibrationData &data);

        bool commit();

        /*
         * Состояние
         */

        bool active() const;

        CalibrationStage stage() const;

        /*
         * Рабочая калибровка
         */

        const CalibrationData& data() const;

        /*
         * Подготовленная
         *
         * Ещё НЕ сохранена
         */

        const CalibrationData& pending() const;

      protected:

        /*
         * Проверка собранных
         * данных
         */

        CalibrationResult validate() const;

        /*
         * Создание pending_
         */

        CalibrationResult build_pending();

        /*
         * Расчёт направления
         */

        void calculate_signs(CalibrationData &data);

        /*
         * Очистка временных
         * данных
         */

        void reset_runtime();

        /*
         * Текущая рабочая
         * калибровка
         *
         * Только она считается
         * действительной
         */

        CalibrationData calibration_;

        /*
         * Новая калибровка
         *
         * Ожидает подтверждения
         */

        CalibrationData pending_;

        /*
         * Временные точки
         *
         * Вообще не сохраняются
         */

        CalibrationRuntime runtime_;

        CalibrationStage stage_ = CalibrationStage::NONE;

        bool commit_requested_ = false;

    };

  } // namespace jalouzee_blinds
} // namespace esphome
