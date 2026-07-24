#pragma once

#include "packet.h"
#include "cmac.h"

namespace esphome {
namespace ble_adv_codec {

// Serializes `pkt` and appends a truncated CMAC computed over the header
// (magic..slot_id) with `key`, writing the full BLE_ADV_PACKET_LEN-byte wire
// payload to `out`. Returns false only on a crypto backend failure (see
// compute_cmac) — `out` is left partially written in that case, callers
// should not transmit it.
bool encode(const BleAdvPacket &pkt, const uint8_t key[BLE_ADV_KEY_LEN], uint8_t out[BLE_ADV_PACKET_LEN]);

// Parses `in` into `out_pkt` and verifies its trailing MAC against `key`.
// Returns false if the MAC doesn't match (wrong/foreign key, corrupted
// packet, or a crypto backend failure) — `out_pkt` is still filled either
// way, but callers must check the return value before acting on it.
//
// Does NOT check replay/counter freshness — that requires per-device_id
// state (last-seen counter, persisted in NVS) that this stateless codec
// doesn't own. See is_counter_fresh() below; the caller is expected to call
// it against its own stored last_counter after a successful decode_and_verify.
bool decode_and_verify(const uint8_t in[BLE_ADV_PACKET_LEN], const uint8_t key[BLE_ADV_KEY_LEN],
                        BleAdvPacket &out_pkt);

// Replay protection: a received counter is only accepted if strictly
// greater than the last one seen for that device_id. Plain integer
// comparison, no wraparound handling — 24 bits (16M messages) isn't
// expected to wrap in the device's lifetime.
inline bool is_counter_fresh(uint32_t counter, uint32_t last_counter) { return counter > last_counter; }

}  // namespace ble_adv_codec
}  // namespace esphome
