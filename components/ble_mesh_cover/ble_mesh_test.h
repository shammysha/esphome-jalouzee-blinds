#pragma once

#include "esphome/core/component.h"

#ifdef USE_ESP32

namespace esphome {
namespace ble_mesh_cover {

// Step 1 bring-up for BLE Mesh integration (see docs/ble-remote-design.md) —
// gets a node provisioned and able to receive+ack a custom vendor-model
// message. Not wired into JalouzeeBlinds::control() yet — that's step 2,
// once provisioning and the message round-trip are confirmed working on
// real hardware.
class BleMeshTest : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }
};

}  // namespace ble_mesh_cover
}  // namespace esphome

#endif  // USE_ESP32
