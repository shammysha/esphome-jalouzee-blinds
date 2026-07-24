// Pure-logic test for packet.h's pack/unpack round-trip and codec.h's
// is_counter_fresh() — no crypto backend needed, see README.md in this
// directory for how to build and run it.
#include <cassert>
#include <cstdio>
#include "../packet.h"
#include "../codec.h"

using namespace esphome::ble_adv_codec;

static void test_roundtrip(uint8_t device_id, uint8_t cmd, uint8_t param, uint32_t counter, uint8_t slot_id) {
  BleAdvPacket pkt;
  pkt.device_id = device_id;
  pkt.cmd = cmd;
  pkt.param = param;
  pkt.counter = counter;
  pkt.slot_id = slot_id;

  uint8_t buf[BLE_ADV_HEADER_LEN];
  pack_header(pkt, buf);

  BleAdvPacket out;
  unpack_header(buf, out);

  assert(out.magic == BLE_ADV_MAGIC);
  assert(out.device_id == device_id);
  assert(out.cmd == cmd);
  assert(out.param == param);
  assert(out.counter == counter);
  assert(out.slot_id == slot_id);
}

int main() {
  static_assert(BLE_ADV_HEADER_LEN == 8, "header length changed");
  static_assert(BLE_ADV_MAC_LEN == 4, "mac length changed");
  static_assert(BLE_ADV_PACKET_LEN == 12, "packet length changed");

  test_roundtrip(0, BLE_ADV_CMD_OPEN, 0, 0, 0);
  test_roundtrip(1, BLE_ADV_CMD_CLOSE, 0, 1, 1);
  test_roundtrip(255, BLE_ADV_CMD_SET_TILT, 73, BLE_ADV_COUNTER_MAX, 3);
  test_roundtrip(42, BLE_ADV_CMD_TILT_STOP, 0, 0x123456, 2);
  printf("pack/unpack round-trip: OK\n");

  // Exact byte layout check — locks in the wire format so a future refactor
  // can't silently change it without this test catching it. Also matches
  // the worked CMAC example in test/README.md.
  {
    BleAdvPacket pkt;
    pkt.device_id = 0x11;
    pkt.cmd = BLE_ADV_CMD_SET_POSITION;
    pkt.param = 0x64;
    pkt.counter = 0x010203;
    pkt.slot_id = 0x02;
    uint8_t buf[BLE_ADV_HEADER_LEN];
    pack_header(pkt, buf);
    uint8_t expected[BLE_ADV_HEADER_LEN] = {BLE_ADV_MAGIC, 0x11, BLE_ADV_CMD_SET_POSITION, 0x64,
                                             0x01, 0x02, 0x03, 0x02};
    for (size_t i = 0; i < BLE_ADV_HEADER_LEN; i++) {
      if (buf[i] != expected[i]) {
        printf("byte layout mismatch at [%zu]: got 0x%02x expected 0x%02x\n", i, buf[i], expected[i]);
        return 1;
      }
    }
    printf("byte layout: OK\n");
  }

  // Replay protection semantics.
  assert(is_counter_fresh(1, 0) == true);
  assert(is_counter_fresh(0, 0) == false);
  assert(is_counter_fresh(0, 1) == false);
  assert(is_counter_fresh(5, 5) == false);
  assert(is_counter_fresh(BLE_ADV_COUNTER_MAX, BLE_ADV_COUNTER_MAX - 1) == true);
  printf("is_counter_fresh: OK\n");

  printf("ALL TESTS PASSED\n");
  return 0;
}
