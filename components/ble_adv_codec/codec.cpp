#include "codec.h"
#include <cstring>

namespace esphome {
namespace ble_adv_codec {

bool encode(const BleAdvPacket &pkt, const uint8_t key[BLE_ADV_KEY_LEN], uint8_t out[BLE_ADV_PACKET_LEN]) {
  pack_header(pkt, out);  // out[0..BLE_ADV_HEADER_LEN)
  return compute_cmac(key, out, BLE_ADV_HEADER_LEN, out + BLE_ADV_HEADER_LEN, BLE_ADV_MAC_LEN);
}

bool decode_and_verify(const uint8_t in[BLE_ADV_PACKET_LEN], const uint8_t key[BLE_ADV_KEY_LEN],
                        BleAdvPacket &out_pkt) {
  unpack_header(in, out_pkt);

  uint8_t expected_mac[BLE_ADV_MAC_LEN];
  if (!compute_cmac(key, in, BLE_ADV_HEADER_LEN, expected_mac, BLE_ADV_MAC_LEN)) return false;

  return memcmp(expected_mac, in + BLE_ADV_HEADER_LEN, BLE_ADV_MAC_LEN) == 0;
}

}  // namespace ble_adv_codec
}  // namespace esphome
