#include "packet.h"

namespace esphome {
namespace ble_adv_codec {

void pack_header(const BleAdvPacket &pkt, uint8_t out[BLE_ADV_HEADER_LEN]) {
  out[0] = pkt.magic;
  out[1] = pkt.device_id;
  out[2] = pkt.cmd;
  out[3] = pkt.param;
  // Big-endian 24-bit counter — arbitrary but fixed choice, must match
  // between remote and cover (see unpack_header).
  out[4] = static_cast<uint8_t>((pkt.counter >> 16) & 0xFF);
  out[5] = static_cast<uint8_t>((pkt.counter >> 8) & 0xFF);
  out[6] = static_cast<uint8_t>(pkt.counter & 0xFF);
  out[7] = pkt.slot_id;
}

void unpack_header(const uint8_t in[BLE_ADV_HEADER_LEN], BleAdvPacket &out) {
  out.magic = in[0];
  out.device_id = in[1];
  out.cmd = in[2];
  out.param = in[3];
  out.counter = (static_cast<uint32_t>(in[4]) << 16) | (static_cast<uint32_t>(in[5]) << 8) |
                static_cast<uint32_t>(in[6]);
  out.slot_id = in[7];
}

}  // namespace ble_adv_codec
}  // namespace esphome
