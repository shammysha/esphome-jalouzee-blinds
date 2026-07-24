#include "cmac.h"

// BLE (and therefore this whole protocol) only exists on ESP32 targets — see
// docs/ble-remote-design.md. Kept as a runtime-false stub rather than
// omitting the .cpp on other platforms, so callers don't need their own
// #ifdef just to link against this component.
#ifdef USE_ESP32

#include <cstring>
#include <mbedtls/cmac.h>
#include <mbedtls/cipher.h>

namespace esphome {
namespace ble_adv_codec {

bool compute_cmac(const uint8_t key[BLE_ADV_KEY_LEN], const uint8_t *data, size_t data_len, uint8_t *out,
                   size_t out_len) {
  const mbedtls_cipher_info_t *cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
  if (cipher_info == nullptr) return false;

  uint8_t full_tag[16];
  // keylen is in BITS per mbedtls's convention, not bytes.
  int ret = mbedtls_cipher_cmac(cipher_info, key, BLE_ADV_KEY_LEN * 8, data, data_len, full_tag);
  if (ret != 0) return false;

  memcpy(out, full_tag, out_len);
  return true;
}

}  // namespace ble_adv_codec
}  // namespace esphome

#else  // !USE_ESP32

namespace esphome {
namespace ble_adv_codec {

bool compute_cmac(const uint8_t *, const uint8_t *, size_t, uint8_t *, size_t) { return false; }

}  // namespace ble_adv_codec
}  // namespace esphome

#endif  // USE_ESP32
