#pragma once


#include "types.h"



namespace esphome {
namespace jalouzee_blinds {



class Storage
{


 public:


  Storage();





  /*
   * Инициализация
   */
  void setup();






  /*
   * Загрузка последней
   * подтвержденной калибровки
   */

  bool load(
      CalibrationData &data
  );







  /*
   * Сохранение новой
   *
   * Вызывается только
   * после завершения
   * калибровки
   */

  bool save(
      const CalibrationData &data
  );






 protected:


  uint32_t checksum(
      const CalibrationData &data
  );





 protected:


  static constexpr uint32_t MAGIC =
      0x4A4C5A42; // "JLZB"



};




} // namespace jalouzee_blinds
} // namespace esphome
