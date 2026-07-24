#pragma once

#include <cstdint>
#include <cstddef>

namespace esphome {
namespace ble_adv_codec {

// Commands carried in a packet — see docs/ble-remote-design.md.
enum BleAdvCmd : uint8_t {
  BLE_ADV_CMD_OPEN = 0,
  BLE_ADV_CMD_CLOSE = 1,
  BLE_ADV_CMD_STOP = 2,
  BLE_ADV_CMD_TILT_OPEN = 3,
  BLE_ADV_CMD_TILT_CLOSE = 4,
  BLE_ADV_CMD_TILT_STOP = 5,
  BLE_ADV_CMD_SET_POSITION = 6,
  BLE_ADV_CMD_SET_TILT = 7,
};

// Protocol marker checked before any crypto work, to cheaply discard
// non-protocol BLE advertising traffic.
static constexpr uint8_t BLE_ADV_MAGIC = 0x4A;  // 'J' for jalouzee

// Length of the signed header (magic..slot_id) that goes into the CMAC, and
// of the full wire payload (header + truncated MAC), in bytes.
static constexpr size_t BLE_ADV_HEADER_LEN = 8;
static constexpr size_t BLE_ADV_MAC_LEN = 4;
static constexpr size_t BLE_ADV_PACKET_LEN = BLE_ADV_HEADER_LEN + BLE_ADV_MAC_LEN;

// Logical (unpacked) view of a command — the counter is transmitted as 3
// bytes on the wire (see pack_header/unpack_header) but kept as a normal
// 32-bit value here; the top 8 bits are always 0.
struct BleAdvPacket {
  uint8_t magic{BLE_ADV_MAGIC};
  uint8_t device_id{0};
  uint8_t cmd{0};
  uint8_t param{0};
  uint32_t counter{0};  // 24-bit on the wire, see BLE_ADV_COUNTER_MAX
  uint8_t slot_id{0};
};

static constexpr uint32_t BLE_ADV_COUNTER_MAX = 0xFFFFFF;  // 3 bytes

// Serializes magic..slot_id into `out` (BLE_ADV_HEADER_LEN bytes) — this is
// exactly the range that gets authenticated by the CMAC (see codec.h). The
// 24-bit counter is written big-endian; counter values above
// BLE_ADV_COUNTER_MAX are truncated (callers should never produce those).
void pack_header(const BleAdvPacket &pkt, uint8_t out[BLE_ADV_HEADER_LEN]);

// Inverse of pack_header — does not touch/validate the MAC, see
// codec.h::decode_and_verify() for that.
void unpack_header(const uint8_t in[BLE_ADV_HEADER_LEN], BleAdvPacket &out);

}  // namespace ble_adv_codec
}  // namespace esphome
