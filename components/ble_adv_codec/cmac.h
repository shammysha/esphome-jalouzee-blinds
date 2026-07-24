#pragma once

#include <cstdint>
#include <cstddef>

namespace esphome {
namespace ble_adv_codec {

static constexpr size_t BLE_ADV_KEY_LEN = 16;  // AES-128

// Computes AES-128-CMAC (RFC 4493) over `data`/`len` using `key` (16 bytes),
// via ESP-IDF's bundled mbedtls (hardware-accelerated AES on ESP32), and
// writes the first `out_len` bytes of the resulting 16-byte tag to `out`.
// Returns false only on an mbedtls backend failure — not expected in
// practice with a valid 16-byte AES-128 key.
bool compute_cmac(const uint8_t key[BLE_ADV_KEY_LEN], const uint8_t *data, size_t data_len, uint8_t *out,
                   size_t out_len);

}  // namespace ble_adv_codec
}  // namespace esphome
