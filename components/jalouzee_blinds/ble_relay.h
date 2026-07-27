#pragma once

#include <array>
#include <cstdint>

#include "esphome/core/component.h"

#ifdef USE_ESP32

namespace esphome {
namespace jalouzee_blinds {

class JalouzeeBlinds;

// Custom BLE advertising-flood-relay transport for the physical remote --
// replaces the earlier esp_ble_mesh-based design (see
// docs/plans/vivid-noodling-gray.md for why: esp_ble_mesh enforces a
// persisted, mutually-exclusive Node/Provisioner role per device with no
// live-switch path, which made "any blind can help with enrollment" not
// achievable on this ESP-IDF version). This talks to NimBLE directly:
// every blind both scans (to receive/relay packets) and advertises (to
// relay/send its own) continuously, with no role exclusivity at all.
//
// Owned by JalouzeeBlinds as a plain member, not a separate ESPHome
// component -- same convention the old BleMesh class used, see its header
// for why (single BLE Mesh/BLE-relay node per firmware, callbacks are plain
// C function pointers bound via a static owner pointer).
class BleRelay {
 public:
  // Must be called (from codegen, see cover.py's CONF_NET_KEY) before
  // setup() -- the shared fleet-wide NetKey every AppKey is derived from and
  // every KEY_ANNOUNCE/KEY_REVOKE is authenticated with. No safe compiled-in
  // default: cover.py requires this whenever ble_relay is enabled.
  void set_net_key(std::array<uint8_t, 16> key);

  void setup(JalouzeeBlinds *owner);

  // Periodic (throttled internally) status dump -- NOT just setup()-time
  // logging. Boot-time logs race esphome logs' WiFi/API client attaching and
  // are easily lost from the small early log buffer; a periodic dump in
  // loop() sidesteps that race entirely. Call every JalouzeeBlinds::loop().
  void loop();

  // Immediately floods this blind's current position to the fleet (see
  // ble_relay.cpp's broadcast_own_status_()) -- call right after movement
  // genuinely ends (target reached, explicit Stop, a fault), so a phone
  // watching this blind's position doesn't have to wait for the next
  // periodic tick. No-op if not yet provisioned.
  void notify_movement_stopped();
};

}  // namespace jalouzee_blinds
}  // namespace esphome

#endif  // USE_ESP32
