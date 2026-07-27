#include "ble_relay.h"

#ifdef USE_ESP32

#include <cmath>
#include <cstring>
#include <cinttypes>

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "jalouzee_blinds.h"

#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_att.h"
#include "host/ble_hs_mbuf.h"
#include "host/util/util.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "mbedtls/ccm.h"
#include "mbedtls/cmac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Custom BLE advertising-flood-relay protocol -- replaces esp_ble_mesh
// entirely. See docs/plans/vivid-noodling-gray.md for the full design and
// why: esp_ble_mesh's persisted Node/Provisioner role exclusivity made
// "any blind can help enroll a phone" unachievable, so we're not using the
// Bluetooth SIG Mesh Profile stack at all -- just NimBLE's raw GAP
// advertise/scan APIs, which have no such role restriction (confirmed:
// broadcaster + observer run concurrently on one radio, unlike Node vs.
// Provisioner).

namespace esphome {
namespace jalouzee_blinds {

static const char *const TAG = "jalouzee_blinds.ble_relay";

// --- Packet format (see plan for the byte-level table) --------------------
// AD: Flags (3B) + AD: Manufacturer Specific Data (len + 0xFF + 2B CID + payload)
// Payload: msg_type(1) ttl(1) msg_id(4,BE) src(2,BE) dst(2,BE) app_idx(1) ciphertext(+4B MIC)
static constexpr uint16_t COMPANY_ID = 0x1234;
static constexpr uint16_t ADDR_BROADCAST = 0xFFFF;

enum RelayMsgType : uint8_t {
  MSG_CMD = 0x01,
  MSG_STATUS = 0x02,
  MSG_KEY_ANNOUNCE = 0x03,
  MSG_KEY_REVOKE = 0x04,
  // Unlike MSG_STATUS (a direct reply to one CMD, encrypted with that
  // requester's own AppKey -- only they can read it), this is a blind
  // announcing its own current position to the whole fleet, authenticated
  // with NetKey like MSG_KEY_ANNOUNCE/REVOKE -- any enrolled phone can read
  // it, not just whoever last commanded this specific blind. See
  // docs/plans/vivid-noodling-gray.md's position-collection design.
  MSG_STATUS_BROADCAST = 0x05,
};

enum CoverCmd : uint8_t {
  COVER_CMD_OPEN = 0,              // param ignored
  COVER_CMD_CLOSE = 1,             // param ignored
  COVER_CMD_STOP = 2,              // param ignored
  COVER_CMD_SET_POSITION = 3,      // param = target position, 0..100 (%)
  COVER_CMD_CALIBRATE = 4,         // param ignored -- same button HA's "Calibration" maps to
  COVER_CMD_CANCEL_CALIBRATE = 5,  // param ignored -- same as HA's "Cancel Calibration"
  COVER_CMD_RESET_FAULT = 6,       // param ignored -- same button HA's "Reset Fault" maps to
};

static constexpr uint8_t TTL_START = 4;
static constexpr uint8_t MIC_LEN = 4;
static constexpr uint8_t PAYLOAD_HEADER_LEN = 11;  // msg_type+ttl+msg_id+src+dst+app_idx
static constexpr uint8_t MAX_ADV_LEN = 31;

// How many times a relayed MSG_CMD gets enqueued at each hop (see
// process_relay_payload_()) -- there's no ack/retry anywhere else in this
// flood-relay design, so a single lost radio transmission between two
// directly adjacent blinds means the command just never arrives. Observed
// on real hardware (2026-07-25): ~20% of commands relayed from one test
// board to its direct neighbor silently never showed up, with nothing
// logged anywhere -- plain RF loss, not a dedup/TTL/queueing bug.
static constexpr uint8_t CMD_RELAY_REPEAT_COUNT = 3;

// Shared fleet-wide NetKey -- every AppKey is derived from this (see
// derive_app_key_()) and every KEY_ANNOUNCE/KEY_REVOKE is authenticated with
// it, so this is the one piece of key material that actually needs to be
// shared out-of-band across the fleet. All-zero (as here) until this blind
// is provisioned -- see provisioned_/provision_net_key_() further down for
// how it actually gets set, either at compile time (BleRelay::set_net_key())
// or over the air.
static uint8_t NET_KEY[16] = {0};

// The single owning JalouzeeBlinds instance -- see ble_relay.h for why this
// is a static pointer rather than a capture.
static JalouzeeBlinds *owner_ = nullptr;

// --- NimBLE host bring-up (unchanged from the old BleMesh -- see
// docs/plans/vivid-noodling-gray.md, this part was never Mesh-specific).
// Placed early in the file since everything below (scan/adv setup) needs
// own_addr_type/own_addr_. ---
static SemaphoreHandle_t mesh_sem;
// Guards adv_state_/phone_connected_ and every ble_gap_adv_start()/stop()
// call -- these are touched from two different FreeRTOS tasks (the ESPHome
// main loop task via pump_tx_queue_(), and the NimBLE host task via
// gap_event_handler_()'s CONNECT/DISCONNECT/ADV_COMPLETE cases). Without
// this, a flood burst starting in pump_tx_queue_() can race a
// disconnect-triggered start_idle_adv_() and fail outright (observed on
// real hardware: "ble_gap_adv_start failed" right at phone disconnect,
// silently dropping the queued STATUS reply). Recursive because
// gap_event_handler_() takes it and then calls start_idle_adv_(), which
// takes it again.
static SemaphoreHandle_t adv_mutex_;
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};

// This device's own short address, derived from its BLE MAC's low 2 bytes
// (see mesh_on_sync()) -- no allocation/registration scheme needed at this
// scale (a handful of devices in one home), collision risk is negligible.
// Top bit forced to 0 so blind addresses (0x0000-0x7FFF) can never collide
// with phone addresses (0x8000-0xFFFE, see enroll_new_phone_()), which are
// assigned independently by whichever blind handles the enrollment.
static uint16_t own_addr_ = 0;

static void mesh_on_reset(int reason) { ESP_LOGI(TAG, "NimBLE host reset, reason=%d", reason); }

static void start_idle_adv_();
static int gap_event_handler_(struct ble_gap_event *event, void *arg);

static void mesh_on_sync() {
  ble_hs_util_ensure_addr(0);
  if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
    ESP_LOGE(TAG, "Failed to infer own BLE address type");
    return;
  }
  ble_hs_id_copy_addr(own_addr_type, addr_val, nullptr);
  own_addr_ = ((addr_val[4] << 8) | addr_val[5]) & 0x7FFF;
  // NOT ble_gatts_start() here -- ble_hs_start() (ble_hs.c) already calls it
  // internally, immediately before invoking this sync_cb (see ble_hs.c's
  // ble_hs_start()/ble_hs_sync()). A second call corrupts the GATT attribute
  // table (crashed with a NULL-uuid deref inside ble_uuid_cmp, called from
  // ble_gatts_start()'s own CCCD-indexing walk) -- confirmed by comparing
  // against ESP-IDF's own bleprph example, which never calls it from app code.
  xSemaphoreGive(mesh_sem);
}

static void mesh_host_task(void *) {
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run();  // returns only after nimble_port_stop()
  nimble_port_freertos_deinit();
}

// From the NimBLE host's "store/config" component -- persists bonding data
// to NVS. No public header in this ESP-IDF layout, hence the forward
// declaration instead of an #include.
extern "C" void ble_store_config_init(void);

// --- GATT server: enrollment + command injection ---------------------------
// Custom service (NOT a standard Bluetooth SIG service, since this isn't BLE
// Mesh) served by every blind identically -- any blind can enroll a phone,
// relay an injected command, or revoke an app_idx, there's no privileged
// node. UUIDs must match ble_transport.dart's relayServiceUuid/
// enrollCharUuid/cmdInjectCharUuid/revokeCharUuid exactly. BLE_UUID128_INIT
// wants the bytes in reverse (little-endian) order relative to the UUID
// string, e.g. "4a420001-0000-1000-8000-00805f9b34fb" reversed byte-by-byte
// -- computed with a script rather than by hand to avoid a transcription
// error that would silently break discovery.
static const ble_uuid128_t svc_uuid_ =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x42, 0x4a);
static const ble_uuid128_t enroll_chr_uuid_ =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x42, 0x4a);
static const ble_uuid128_t cmd_inject_chr_uuid_ =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x42, 0x4a);
static const ble_uuid128_t revoke_chr_uuid_ =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x04, 0x00, 0x42, 0x4a);
static const ble_uuid128_t provision_chr_uuid_ =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x05, 0x00, 0x42, 0x4a);
static const ble_uuid128_t own_addr_chr_uuid_ =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x06, 0x00, 0x42, 0x4a);

static int gatt_access_cb_(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_chr_def gatt_chrs_[] = {
    {
        .uuid = &enroll_chr_uuid_.u,
        .access_cb = gatt_access_cb_,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = &cmd_inject_chr_uuid_.u,
        .access_cb = gatt_access_cb_,
        .flags = BLE_GATT_CHR_F_WRITE,
    },
    {
        // Write-only: one byte, the app_idx to revoke. Any phone enrolled on
        // this blind's fleet can revoke any app_idx (including its own or
        // another phone's) -- same trust model as enrollment itself: nothing
        // gates this beyond "you can reach a blind's GATT server", because
        // the actual security boundary is NetKey possession (see
        // flood_key_event_()/this file's KEY_ANNOUNCE comment), not a
        // permission check here. Revoking a nonexistent/already-invalid
        // app_idx is a harmless no-op.
        .uuid = &revoke_chr_uuid_.u,
        .access_cb = gatt_access_cb_,
        .flags = BLE_GATT_CHR_F_WRITE,
    },
    {
        // Write-only: 16 raw NetKey bytes. Only accepted while this blind is
        // unprovisioned (see provisioned_/NetKeyStore below) -- an
        // already-provisioned blind rejects this outright, which is the only
        // thing standing between "any nearby BLE client" and hijacking a
        // working blind's NetKey. This is how a blind with no compiled-in
        // !secret net_key (i.e. factory firmware for a real customer, not
        // this maintainer's own ESPHome-built installs) joins a fleet
        // without anyone needing ESPHome access -- see docs/plans/
        // vivid-noodling-gray.md's "Provisioning NetKey без ESPHome".
        .uuid = &provision_chr_uuid_.u,
        .access_cb = gatt_access_cb_,
        .flags = BLE_GATT_CHR_F_WRITE,
    },
    {
        // Read-only: this blind's own 2-byte relay address (own_addr_).
        // Unlike the enroll response's ownAddr (the *phone's* freshly-minted
        // address, and only returned on an actual enroll -- skipped on
        // reconnect when this phone already has saved keys), this is
        // readable unconditionally on every connection. Needed to target a
        // CMD at exactly one specific blind (dst == this value) rather than
        // broadcast -- e.g. the calibration flow's "which physical blind is
        // this candidate" identify step, see docs/plans/
        // vivid-noodling-gray.md's "Калибровка и Reset Fault недоступны без HA".
        .uuid = &own_addr_chr_uuid_.u,
        .access_cb = gatt_access_cb_,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {0},
};

static const struct ble_gatt_svc_def gatt_svcs_[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid_.u,
        .characteristics = gatt_chrs_,
    },
    {0},
};

static bool nimble_bringup() {
  mesh_sem = xSemaphoreCreateBinary();
  adv_mutex_ = xSemaphoreCreateRecursiveMutex();
  if (mesh_sem == nullptr || adv_mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create bring-up semaphore/mutex");
    return false;
  }
  if (nimble_port_init() != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed");
    return false;
  }
  ble_hs_cfg.reset_cb = mesh_on_reset;
  ble_hs_cfg.sync_cb = mesh_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  ble_store_config_init();

  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_svc_gap_device_name_set("jalouzee-blind");
  if (ble_gatts_count_cfg(gatt_svcs_) != 0 || ble_gatts_add_svcs(gatt_svcs_) != 0) {
    ESP_LOGE(TAG, "Failed to register GATT enroll/cmd-inject service");
    return false;
  }

  nimble_port_freertos_init(mesh_host_task);
  xSemaphoreTake(mesh_sem, portMAX_DELAY);
  return true;
}

// --- AES-CCM/CMAC helpers (mbedTLS directly -- no esp_ble_mesh crypto needed) --
static bool ccm_encrypt_(const uint8_t key[16], const uint8_t nonce[13], const uint8_t *plain, uint8_t plain_len,
                          uint8_t *out_cipher, uint8_t *out_mic) {
  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);
  bool ok = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, 128) == 0 &&
            mbedtls_ccm_encrypt_and_tag(&ctx, plain_len, nonce, 13, nullptr, 0, plain, out_cipher, out_mic,
                                         MIC_LEN) == 0;
  mbedtls_ccm_free(&ctx);
  return ok;
}

static bool ccm_decrypt_(const uint8_t key[16], const uint8_t nonce[13], const uint8_t *cipher, uint8_t cipher_len,
                          const uint8_t *mic, uint8_t *out_plain) {
  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);
  bool ok = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, 128) == 0 &&
            mbedtls_ccm_auth_decrypt(&ctx, cipher_len, nonce, 13, nullptr, 0, cipher, out_plain, mic, MIC_LEN) == 0;
  mbedtls_ccm_free(&ctx);
  return ok;
}

static void build_nonce_(uint8_t msg_type, uint16_t src, uint32_t msg_id, uint8_t out[13]) {
  out[0] = msg_type;
  out[1] = (uint8_t) (src >> 8);
  out[2] = (uint8_t) src;
  out[3] = (uint8_t) (msg_id >> 24);
  out[4] = (uint8_t) (msg_id >> 16);
  out[5] = (uint8_t) (msg_id >> 8);
  out[6] = (uint8_t) msg_id;
  memset(out + 7, 0, 6);
}

// Every phone's AppKey is derived from the shared NetKey rather than
// generated randomly and flooded to the fleet -- a flooded KEY_ANNOUNCE
// physically can't carry a raw 16-byte AppKey (see the plan's "Формат
// пакета": legacy advertising's 31-byte cap leaves only 9 plaintext bytes
// per packet after our header+MIC overhead). This costs nothing security-
// wise: anyone who can forge a valid KEY_ANNOUNCE already holds NetKey, and
// could mint an arbitrary AppKey under either scheme -- NetKey secrecy was
// always the actual trust boundary. The phone still gets its concrete
// AppKey bytes handed to it directly over GATT (ATT payload, not
// advertising -- much bigger budget) in the enroll response, so this is
// invisible to the app; see EnrollResult/enroll_new_phone_() below. One
// AES-CMAC call over a 16-byte block gives exactly a 16-byte AES key.
static bool derive_app_key_(uint8_t app_idx, uint8_t out_key[16]) {
  uint8_t input[16] = {0};
  input[0] = 0x01;  // derivation label: "app key"
  input[1] = app_idx;
  const mbedtls_cipher_info_t *cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
  return cipher_info != nullptr && mbedtls_cipher_cmac(cipher_info, NET_KEY, 128, input, sizeof(input), out_key) == 0;
}

// --- Enrolled app_idx validity (persisted) ----------------------------------
// What actually needs to be shared across the fleet isn't key material (see
// derive_app_key_() above) -- it's which app_idx values are currently
// authorized. One bit per possible app_idx (0..255) covers that in 32 bytes,
// flat, no slot/eviction bookkeeping needed.
struct AppKeyValidityStore {
  uint8_t valid_bitmap[32];
} __attribute__((packed));
static AppKeyValidityStore key_validity_{};
static ESPPreferenceObject key_validity_pref_;

static bool app_idx_valid_(uint8_t app_idx) {
  return (key_validity_.valid_bitmap[app_idx / 8] >> (app_idx % 8)) & 1;
}

static void app_idx_set_valid_(uint8_t app_idx, bool valid) {
  uint8_t mask = 1 << (app_idx % 8);
  if (valid) {
    key_validity_.valid_bitmap[app_idx / 8] |= mask;
  } else {
    key_validity_.valid_bitmap[app_idx / 8] &= (uint8_t) ~mask;
  }
}

static void save_key_validity_() { key_validity_pref_.save(&key_validity_); }

// --- NetKey provisioning (persisted) ----------------------------------------
// True as soon as NET_KEY holds a real value -- either set at compile time
// (BleRelay::set_net_key(), called from codegen before setup() whenever
// cover.py's net_key is configured -- this maintainer's own ESPHome-built
// installs) or over the air via provision_chr_uuid_ (factory firmware with
// no compiled-in key -- the path a real customer with no ESPHome access
// uses, see docs/plans/vivid-noodling-gray.md's "Provisioning NetKey без
// ESPHome"). Gates two things: whether the enroll characteristic will hand
// out enrollment material at all (see gatt_access_cb_), and whether the
// provision characteristic accepts a write -- once true, a NetKey can never
// be overwritten over BLE again, compile-time or not, which is the whole
// point (otherwise any nearby BLE client could hijack a working blind).
static bool provisioned_ = false;
struct NetKeyStore {
  uint8_t net_key[16];
} __attribute__((packed));
static NetKeyStore net_key_store_{};
static ESPPreferenceObject net_key_pref_;

// Only reachable while !provisioned_ (see gatt_access_cb_'s guard) -- no
// fleet-wide flood needed here, unlike KEY_ANNOUNCE: provisioning is strictly
// pairwise (this phone, this blind), other blinds don't need to agree on
// anything for it to take effect.
static bool provision_net_key_(const uint8_t key[16]) {
  memcpy(NET_KEY, key, 16);
  memcpy(net_key_store_.net_key, key, 16);
  if (!net_key_pref_.save(&net_key_store_)) return false;
  provisioned_ = true;
  ESP_LOGI(TAG, "Provisioned with a new NetKey");
  return true;
}

// --- Factory reset via rapid power-cycle counter ----------------------------
// The only way to un-provision a blind (see provision_net_key_() above) once
// it's provisioned -- e.g. the buyer made a mistake, or wants to move a
// blind to a different fleet -- otherwise requires a USB reflash, which a
// real customer with no ESPHome access can't do. Standard pattern for
// mains-powered devices with no dedicated reset button reachable after
// install: N power cycles in a row, each within a short grace window of the
// previous boot, triggers a reset. A boot counter alone can't tell "5 quick
// power cycles" apart from "5 ordinary power cycles over the device's
// lifetime" -- what actually distinguishes them is that a *stable* boot
// (stayed powered long enough to matter) clears the counter back to 0, so
// it only ever reaches the threshold if every one of the last N boots was
// itself cut short quickly.
static constexpr uint8_t FACTORY_RESET_BOOT_COUNT = 5;
static constexpr uint32_t FACTORY_RESET_GRACE_MS = 10000;  // stay powered this long to clear the counter

struct BootCounterStore {
  uint8_t count;
} __attribute__((packed));
static BootCounterStore boot_counter_{};
static ESPPreferenceObject boot_counter_pref_;
static bool boot_count_cleared_ = false;
static uint32_t setup_millis_ = 0;

// Back to exactly the as-shipped state: no NetKey, nothing authorized. Does
// NOT reboot -- takes effect immediately for the rest of this boot too
// (called from setup(), before scanning/advertising start), matching what a
// factory-fresh device would do from cold boot.
static void factory_reset_() {
  ESP_LOGW(TAG, "Factory reset: %u quick power cycles in a row -- clearing NetKey and all authorizations",
           FACTORY_RESET_BOOT_COUNT);
  provisioned_ = false;
  memset(NET_KEY, 0, sizeof(NET_KEY));
  net_key_store_ = NetKeyStore{};
  net_key_pref_.save(&net_key_store_);

  key_validity_ = AppKeyValidityStore{};
  save_key_validity_();

  boot_counter_ = BootCounterStore{};
  boot_counter_pref_.save(&boot_counter_);
}

// --- Dedup cache ------------------------------------------------------------
struct DedupEntry {
  uint16_t src;
  uint32_t msg_id;
  bool used;
};
static constexpr uint8_t DEDUP_CACHE_SIZE = 48;
static DedupEntry dedup_cache_[DEDUP_CACHE_SIZE] = {};
static uint8_t dedup_next_ = 0;

static bool dedup_seen_(uint16_t src, uint32_t msg_id) {
  for (auto &e : dedup_cache_) {
    if (e.used && e.src == src && e.msg_id == msg_id) return true;
  }
  return false;
}

static void dedup_remember_(uint16_t src, uint32_t msg_id) {
  dedup_cache_[dedup_next_] = {src, msg_id, true};
  dedup_next_ = (dedup_next_ + 1) % DEDUP_CACHE_SIZE;
}

// --- Outgoing relay queue (jittered, one at a time) ------------------------
struct PendingTx {
  uint8_t adv_bytes[MAX_ADV_LEN];
  uint8_t adv_len;
  uint32_t send_at_ms;
  bool valid;
};
// Was 8 -- too small once a single enroll (3x KEY_ANNOUNCE) can land in the
// same tick as ordinary CMD/STATUS traffic and gossip's recent-event
// reminders (up to RECENT_ANNOUNCE_SIZE + RECENT_REVOKE_SIZE single-shot
// entries): observed "TX queue full, dropping a relay packet" firing
// repeatedly on real hardware once repeats were added. Each slot is ~37
// bytes (MAX_ADV_LEN + 3 small fields), so doubling costs under 300 bytes
// of RAM -- trivial against this chip's budget.
static constexpr uint8_t TX_QUEUE_SIZE = 16;
static PendingTx tx_queue_[TX_QUEUE_SIZE] = {};
static uint32_t own_msg_id_next_ = 1;

static void enqueue_tx_(const uint8_t *adv_bytes, uint8_t adv_len, uint32_t jitter_max_ms) {
  for (auto &slot : tx_queue_) {
    if (slot.valid) continue;
    memcpy(slot.adv_bytes, adv_bytes, adv_len);
    slot.adv_len = adv_len;
    slot.send_at_ms = millis() + (jitter_max_ms > 0 ? (esphome::random_uint32() % jitter_max_ms) : 0);
    slot.valid = true;
    return;
  }
  ESP_LOGW(TAG, "TX queue full, dropping a relay packet");
}

// Builds the full AD byte sequence (Flags + Manufacturer Specific Data) for
// one payload. Returns the total length, or 0 if it doesn't fit.
static uint8_t build_adv_(uint8_t msg_type, uint8_t ttl, uint32_t msg_id, uint16_t src, uint16_t dst,
                           uint8_t app_idx, const uint8_t *ciphertext_and_mic, uint8_t cipher_mic_len,
                           uint8_t out[MAX_ADV_LEN]) {
  uint8_t payload_len = PAYLOAD_HEADER_LEN + cipher_mic_len;
  uint8_t mfg_len = 1 /*0xFF*/ + 2 /*CID*/ + payload_len;
  uint8_t total = 3 /*flags*/ + 1 /*len byte*/ + mfg_len;
  if (total > MAX_ADV_LEN) return 0;

  uint8_t *p = out;
  *p++ = 0x02;
  *p++ = 0x01;  // BLE_HS_ADV_TYPE_FLAGS
  *p++ = 0x06;  // BR/EDR not supported, LE General Discoverable
  *p++ = mfg_len;
  *p++ = 0xFF;  // BLE_HS_ADV_TYPE_MFG_DATA
  *p++ = (uint8_t) (COMPANY_ID & 0xFF);
  *p++ = (uint8_t) (COMPANY_ID >> 8);
  *p++ = msg_type;
  *p++ = ttl;
  *p++ = (uint8_t) (msg_id >> 24);
  *p++ = (uint8_t) (msg_id >> 16);
  *p++ = (uint8_t) (msg_id >> 8);
  *p++ = (uint8_t) msg_id;
  *p++ = (uint8_t) (src >> 8);
  *p++ = (uint8_t) src;
  *p++ = (uint8_t) (dst >> 8);
  *p++ = (uint8_t) dst;
  *p++ = app_idx;
  memcpy(p, ciphertext_and_mic, cipher_mic_len);
  p += cipher_mic_len;
  return (uint8_t) (p - out);
}

// Idle-state advertisement: connectable, carries the 128-bit relay service
// UUID so findNearestRelayNode() (ble_transport.dart) can find any blind for
// enrollment/cmd-inject. Reuses svc_uuid_.value so the AD bytes and the GATT
// UUID can never drift apart. Mutually exclusive with flood-packet bursts
// (see AdvState below) -- the radio only advertises one thing at a time.
static uint8_t build_idle_adv_(uint8_t out[MAX_ADV_LEN]) {
  uint8_t *p = out;
  *p++ = 0x02;
  *p++ = 0x01;
  *p++ = 0x06;
  *p++ = 17;    // length: 1 (type) + 16 (uuid)
  *p++ = 0x07;  // Complete List of 128-bit Service UUIDs
  memcpy(p, svc_uuid_.value, 16);
  p += 16;
  return (uint8_t) (p - out);
}

enum AdvState : uint8_t {
  ADV_STATE_NONE = 0,
  ADV_STATE_IDLE_BEACON,
  ADV_STATE_FLOOD_BURST,
};
static AdvState adv_state_ = ADV_STATE_NONE;
static bool phone_connected_ = false;

// RAII guard for adv_mutex_ -- see its declaration for why this exists.
// Recursive, so nesting (e.g. gap_event_handler_() holding it across a call
// into start_idle_adv_(), which takes it again) is safe.
class AdvLock {
 public:
  AdvLock() { xSemaphoreTakeRecursive(adv_mutex_, portMAX_DELAY); }
  ~AdvLock() { xSemaphoreGiveRecursive(adv_mutex_); }
};

// Starts (or resumes) the connectable idle beacon -- no-op if a phone is
// currently connected (own advertising would be pointless/wasteful, iOS/
// Android already show the device as connected) or if the radio is already
// advertising something.
static void start_idle_adv_() {
  AdvLock lock;
  if (phone_connected_ || ble_gap_adv_active()) return;
  uint8_t adv[MAX_ADV_LEN];
  uint8_t len = build_idle_adv_(adv);
  if (ble_gap_adv_set_data(adv, len) != 0) {
    ESP_LOGW(TAG, "idle adv set_data failed");
    return;
  }
  struct ble_gap_adv_params adv_params = {};
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.itvl_min = 400;  // 250ms -- a discoverable beacon, not urgent like a flood burst
  adv_params.itvl_max = 500;
  if (ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER, &adv_params, gap_event_handler_, nullptr) != 0) {
    ESP_LOGW(TAG, "idle adv start failed");
    return;
  }
  adv_state_ = ADV_STATE_IDLE_BEACON;
}

static void dispatch_command_(uint8_t cmd, uint8_t param) {
  if (owner_ == nullptr) return;
  switch (cmd) {
    case COVER_CMD_OPEN:
      owner_->make_call().set_command_open().perform();
      break;
    case COVER_CMD_CLOSE:
      owner_->make_call().set_command_close().perform();
      break;
    case COVER_CMD_STOP:
      owner_->make_call().set_command_stop().perform();
      break;
    case COVER_CMD_SET_POSITION:
      // .set_tilt(), not .set_position() -- found on real hardware
      // (2026-07-26): dragging the phone's slider fully to 0%/100% landed
      // on the exact same call.get_position() values (0.0/1.0) that a
      // plain open()/close() button press produces, so control() couldn't
      // tell them apart and ran the stepped closed->50%->open logic
      // instead of moving straight there (e.g. closed -> slider to 100% ->
      // stopped at 50%). get_tilt() is the dedicated, always-unambiguous
      // channel jalouzee_blinds.cpp's get_traits()/control() already built
      // for exactly this (a tilt call can never originate from an
      // open/close command) -- it just wasn't wired up to this BLE command
      // yet. See docs/plans/vivid-noodling-gray.md.
      owner_->make_call().set_tilt(param / 100.0f).perform();
      break;
    case COVER_CMD_CALIBRATE:
      owner_->on_calibration_button_pressed();
      break;
    case COVER_CMD_CANCEL_CALIBRATE:
      owner_->on_cancel_calibration_button_pressed();
      break;
    case COVER_CMD_RESET_FAULT:
      owner_->on_fault_reset_button_pressed();
      break;
    default:
      ESP_LOGW(TAG, "Unknown cover command: %u", cmd);
      break;
  }
}

// Shared by send_status_() and flood_key_event_() -- encrypts a 2-byte
// plaintext payload with [key] and enqueues it as a new flood packet from
// this device. header_app_idx is the packet header's app_idx field, which
// only means anything for CMD/STATUS (which AppKey to use) -- KEY_ANNOUNCE/
// KEY_REVOKE pass 0 there since they're authenticated with NetKey, not an
// AppKey, and carry the *announced* app_idx in the plaintext payload instead.
static void send_encrypted_(uint8_t msg_type, uint16_t dst, uint8_t header_app_idx, const uint8_t key[16],
                             const uint8_t plain[2]) {
  uint8_t nonce[13];
  uint32_t msg_id = own_msg_id_next_++;
  build_nonce_(msg_type, own_addr_, msg_id, nonce);
  uint8_t cipher[2], mic[MIC_LEN];
  if (!ccm_encrypt_(key, nonce, plain, 2, cipher, mic)) {
    ESP_LOGW(TAG, "Failed to encrypt msg_type=%u", msg_type);
    return;
  }
  uint8_t cipher_and_mic[2 + MIC_LEN];
  memcpy(cipher_and_mic, cipher, 2);
  memcpy(cipher_and_mic + 2, mic, MIC_LEN);

  uint8_t adv[MAX_ADV_LEN];
  uint8_t len = build_adv_(msg_type, TTL_START, msg_id, own_addr_, dst, header_app_idx, cipher_and_mic,
                            sizeof(cipher_and_mic), adv);
  if (len == 0) {
    ESP_LOGW(TAG, "Packet too large, msg_type=%u", msg_type);
    return;
  }
  dedup_remember_(own_addr_, msg_id);
  enqueue_tx_(adv, len, 100);
}

// Replies with our current position, encrypted with the same AppKey the
// triggering CMD used (requester_app_idx) -- only the requesting phone (or
// others sharing that app_idx) can decrypt it, matching the plan's STATUS
// design.
static void send_status_(uint8_t requester_app_idx) {
  uint8_t app_key[16];
  if (!derive_app_key_(requester_app_idx, app_key)) return;
  uint8_t status = owner_ != nullptr ? static_cast<uint8_t>(lroundf(owner_->position * 100.0f)) : 0;
  uint8_t plain[2] = {status, 0};
  send_encrypted_(MSG_STATUS, ADDR_BROADCAST, requester_app_idx, app_key, plain);
}

// Bitmask for MSG_STATUS_BROADCAST's plain[1] (see broadcast_own_status_()
// below) -- room for more flags later without another wire format change.
// Must match main.dart's statusFlagCalibrated/statusFlagMoving constants.
// STATUS_FLAG_MOVING: the periodic tick fires on a plain timer with no
// idea whether anything is actually moving, so a phone can't otherwise
// tell "this broadcast reflects a blind at rest" apart from "this just
// happened to land mid-movement" (observed on real hardware: the position
// control's thumb would visibly jump back to a stale in-transit position
// because *some* broadcast arrived after the command, even though the
// blind hadn't actually stopped yet -- see main.dart's _isSettled, which
// now specifically requires a broadcast with this bit clear). Only whether
// the motor is moving at all matters here, not which direction -- see
// JalouzeeBlinds::motor_direction()'s doc comment.
// STATUS_FLAG_FAULT: whether trigger_fault_() is currently blocking normal
// control (see JalouzeeBlinds::control()'s fault_active_ check) -- there's
// no other way for the app to learn this short of a command silently doing
// nothing, since CMD packets don't carry a reply the broadcasting app can
// see (see docs/plans/vivid-noodling-gray.md, "нужна иконка с
// восклицательным знаком на устройстве с аварией", 2026-07-26). Lets
// DevicesScreen show a warning icon per blind and offer to reset it
// (COVER_CMD_RESET_FAULT) without the user having to guess why a blind
// stopped responding.
enum StatusBroadcastFlag : uint8_t {
  STATUS_FLAG_CALIBRATED = 1 << 0,
  STATUS_FLAG_MOVING = 1 << 1,
  STATUS_FLAG_FAULT = 1 << 2,
};

// Floods this blind's own current position to the whole fleet, encrypted
// with NetKey (not an AppKey) so ANY enrolled phone can read it -- unlike
// send_status_() above, which only the specific phone that sent the
// triggering CMD can decrypt. Called from run_periodic_broadcast_tick_()
// (every PERIODIC_BROADCAST_INTERVAL_MS) and immediately from
// BleRelay::notify_movement_stopped() right after movement genuinely ends,
// so a phone watching several blinds sees a fresh position without having
// to individually command each one first.
// plain[1]'s STATUS_FLAG_CALIBRATED bit carries whether this blind is
// actually calibrated -- an uncalibrated blind's `position` is never a
// real measurement, just whatever it defaulted or was last saved to, and
// must not be averaged in with real readings from calibrated blinds (see
// main.dart's _aggregatePosition(), which only includes entries with the
// flag set). Still broadcasting *something* rather than staying silent
// lets the app tell "this blind exists but needs calibrating" apart from
// "out of range/never heard of it".
static void broadcast_own_status_() {
  uint8_t status = owner_ != nullptr ? static_cast<uint8_t>(lroundf(owner_->position * 100.0f)) : 0;
  uint8_t flags = 0;
  if (owner_ != nullptr) {
    if (owner_->is_calibrated()) flags |= STATUS_FLAG_CALIBRATED;
    if (owner_->is_moving()) flags |= STATUS_FLAG_MOVING;
    if (owner_->is_fault_active()) flags |= STATUS_FLAG_FAULT;
  }
  uint8_t plain[2] = {status, flags};
  send_encrypted_(MSG_STATUS_BROADCAST, ADDR_BROADCAST, 0, NET_KEY, plain);
}

// Floods "app_idx is now authorized" (MSG_KEY_ANNOUNCE) or "...no longer
// authorized" (MSG_KEY_REVOKE) to the fleet, authenticated with NetKey.
static void flood_key_event_(uint8_t msg_type, uint8_t app_idx) {
  uint8_t plain[2] = {app_idx, 0};
  send_encrypted_(msg_type, ADDR_BROADCAST, 0, NET_KEY, plain);
}

// Defined further down alongside recent_announces_/run_periodic_broadcast_tick_.
static void remember_recent_announce_(uint8_t app_idx);

struct EnrollResult {
  uint8_t app_key[16];
  uint8_t app_idx;
  uint16_t addr;
};

// Mints enrollment material for a newly-connecting phone: a fresh app_idx
// (checked against our local validity bitmap to avoid immediately colliding
// with an already-enrolled phone -- collisions are otherwise merely
// possible, not catastrophic on their own, but here they'd mean two phones
// silently share one identity/AppKey, so it's worth a few extra tries), its
// derived AppKey, and a relay address of its own (top bit set, see
// own_addr_'s comment, so it can never collide with a blind's address).
// Marks the app_idx valid locally and floods KEY_ANNOUNCE so the rest of the
// fleet accepts CMD packets using it.
static bool enroll_new_phone_(EnrollResult *out) {
  uint8_t app_idx = 0;
  bool found = false;
  for (int attempt = 0; attempt < 16; attempt++) {
    uint8_t candidate = (uint8_t) esphome::random_uint32();
    if (!app_idx_valid_(candidate)) {
      app_idx = candidate;
      found = true;
      break;
    }
  }
  if (!found) {
    app_idx = (uint8_t) esphome::random_uint32();
    ESP_LOGW(TAG, "enroll: app_idx collision after 16 tries, reusing 0x%02x anyway", app_idx);
  }

  uint16_t addr = 0x8000 | (esphome::random_uint32() & 0x7FFF);
  if (addr == ADDR_BROADCAST) addr = 0x8000;

  if (!derive_app_key_(app_idx, out->app_key)) {
    ESP_LOGE(TAG, "enroll: AppKey derivation failed");
    return false;
  }
  out->app_idx = app_idx;
  out->addr = addr;

  app_idx_set_valid_(app_idx, true);
  save_key_validity_();
  flood_key_event_(MSG_KEY_ANNOUNCE, app_idx);
  remember_recent_announce_(app_idx);
  ESP_LOGI(TAG, "Enrolled phone: app_idx=0x%02x addr=0x%04x", app_idx, addr);
  return true;
}

// Marks app_idx unauthorized locally and floods KEY_REVOKE so the rest of
// the fleet stops accepting CMD packets under it. No-op (still floods, in
// case a peer's local state disagrees) if we already considered it invalid.
static void remember_recent_revoke_(uint8_t app_idx);

static void revoke_app_idx_(uint8_t app_idx) {
  app_idx_set_valid_(app_idx, false);
  save_key_validity_();
  flood_key_event_(MSG_KEY_REVOKE, app_idx);
  remember_recent_revoke_(app_idx);
  ESP_LOGI(TAG, "Revoked app_idx=0x%02x", app_idx);
}

// --- Gossip: periodic re-announcement of KEY_ANNOUNCE/KEY_REVOKE -----------
// KEY_ANNOUNCE/KEY_REVOKE were originally one-shot floods -- fine as long as
// every blind is online and in range at the exact moment one goes out, but
// that's not guaranteed (a blind mid-reboot, out of range, or added to the
// fleet *after* a phone already enrolled elsewhere never catches up on its
// own -- confirmed as a real issue during hardware testing 2026-07-25, see
// docs/plans/vivid-noodling-gray.md). Fix: every blind also periodically
// re-floods what it currently believes, so a straggler eventually hears a
// repeat and self-heals -- no new message type, no new GATT role, no
// fragmentation. Safe to replay any number of times in any order because
// each event ("app_idx X is valid" / "app_idx X is not valid") is
// independently idempotent -- unlike periodically re-broadcasting the whole
// key_validity_ bitmap at once, which would need a way to tell "whose
// snapshot is newer" to merge safely (a stale bitmap could accidentally
// resurrect something another blind already revoked).
// Was 60000 (1 minute), then 10000 -- lowered again 2026-07-26 to unify
// with the position-broadcast interval the user asked for (5s) rather than
// run two separate periodic timers for what's conceptually the same thing
// (a blind periodically telling the fleet something about itself, always
// NetKey-authenticated, always over the flood channel). One shared tick
// means recent_announces_/recent_revokes_ reminders and the round-robin
// KEY_ANNOUNCE catch-up now also cycle faster as a side effect -- a pure
// improvement, not a tradeoff, since there was never a reason for those to
// be slower than the position broadcast specifically.
static constexpr uint32_t PERIODIC_BROADCAST_INTERVAL_MS = 5000;
static uint32_t last_periodic_broadcast_ms_ = 0;
static uint16_t gossip_next_app_idx_ = 0;  // round-robin cursor over valid_bitmap

// Revocations get the same periodic-reminder treatment, but only for a
// limited retention window -- unlike currently-valid app_idx (which we can
// always re-derive/re-announce for as long as they stay valid), "recently
// revoked" is transient by nature and intentionally NOT persisted to flash:
// losing this list on a reboot within the retention window just means a few
// fewer reminder repeats go out, not a correctness problem (the original
// revoke already flooded once and updated key_validity_ persistently).
static constexpr uint32_t REVOKE_REMIND_RETENTION_MS = 3600000;  // 1 hour
static constexpr uint8_t RECENT_REVOKE_SIZE = 8;
struct RecentRevoke {
  uint8_t app_idx;
  uint32_t revoked_at_ms;
  bool used;
};
static RecentRevoke recent_revokes_[RECENT_REVOKE_SIZE] = {};

static void remember_recent_revoke_(uint8_t app_idx) {
  uint32_t now = millis();
  for (auto &r : recent_revokes_) {
    if (r.used && r.app_idx == app_idx) {
      r.revoked_at_ms = now;
      return;
    }
  }
  for (auto &r : recent_revokes_) {
    if (!r.used) {
      r = {app_idx, now, true};
      return;
    }
  }
  // All slots full (8 revocations within the last hour -- unlikely for a
  // home-sized fleet): evict the oldest reminder rather than drop the new
  // one silently.
  RecentRevoke *oldest = &recent_revokes_[0];
  for (auto &r : recent_revokes_) {
    if (r.revoked_at_ms < oldest->revoked_at_ms) oldest = &r;
  }
  *oldest = {app_idx, now, true};
}

// Same "recent event gets a reminder on every tick" treatment as revokes
// above, but for freshly-enrolled app_idx -- added 2026-07-25 after finding
// the plain round-robin below can't be trusted to catch up promptly on its
// own. gossip_next_app_idx_ resets to 0 on every reboot and only advances
// by one valid entry per tick; a fleet with any accumulated history of
// past enrollments (this test rig alone had a day's worth) can leave a
// brand-new app_idx waiting behind all of them, indistinguishable from
// "years-old and never touched" to that plain scan. This list makes
// recently-enrolled app_idx skip that queue: they ride along on every
// gossip tick for ANNOUNCE_REMIND_RETENTION_MS, same as a fresh revoke
// does, instead of taking their turn.
static constexpr uint32_t ANNOUNCE_REMIND_RETENTION_MS = 600000;  // 10 minutes
static constexpr uint8_t RECENT_ANNOUNCE_SIZE = 8;
struct RecentAnnounce {
  uint8_t app_idx;
  uint32_t announced_at_ms;
  bool used;
};
static RecentAnnounce recent_announces_[RECENT_ANNOUNCE_SIZE] = {};

static void remember_recent_announce_(uint8_t app_idx) {
  uint32_t now = millis();
  for (auto &r : recent_announces_) {
    if (r.used && r.app_idx == app_idx) {
      r.announced_at_ms = now;
      return;
    }
  }
  for (auto &r : recent_announces_) {
    if (!r.used) {
      r = {app_idx, now, true};
      return;
    }
  }
  RecentAnnounce *oldest = &recent_announces_[0];
  for (auto &r : recent_announces_) {
    if (r.announced_at_ms < oldest->announced_at_ms) oldest = &r;
  }
  *oldest = {app_idx, now, true};
}

// Called once per PERIODIC_BROADCAST_INTERVAL_MS from BleRelay::loop().
// Reminds every not-yet-expired recent announce and recent revoke (both get
// priority -- see remember_recent_announce_()'s comment for why the plain
// round-robin alone can't be trusted to catch these up promptly), re-
// announces one OTHER currently-valid app_idx via round-robin as slow
// background upkeep for anything old enough to have fallen out of both
// recent lists, and broadcasts this blind's own current position (see
// broadcast_own_status_()) -- one shared tick for everything this blind
// periodically tells the fleet about itself, rather than separate timers
// for authorization gossip vs. position.
static void run_periodic_broadcast_tick_() {
  uint32_t now = millis();

  // Recent announces/revokes deliberately stay single-shot here (not
  // repeated like the immediate at-enroll/at-revoke flood) -- with up to
  // RECENT_ANNOUNCE_SIZE + RECENT_REVOKE_SIZE reminders active at once,
  // tripling each would risk overflowing tx_queue_'s 8 slots in one tick.
  // They already get a fresh independent chance every tick for their whole
  // retention window, which is its own form of repetition spread out over
  // time instead of bunched up here.
  for (auto &r : recent_announces_) {
    if (!r.used) continue;
    if (now - r.announced_at_ms >= ANNOUNCE_REMIND_RETENTION_MS) {
      r.used = false;
      continue;
    }
    flood_key_event_(MSG_KEY_ANNOUNCE, r.app_idx);
  }
  for (auto &r : recent_revokes_) {
    if (!r.used) continue;
    if (now - r.revoked_at_ms >= REVOKE_REMIND_RETENTION_MS) {
      r.used = false;
      continue;
    }
    flood_key_event_(MSG_KEY_REVOKE, r.app_idx);
  }

  for (uint16_t i = 0; i < 256; i++) {
    uint8_t candidate = (uint8_t) ((gossip_next_app_idx_ + i) % 256);
    if (app_idx_valid_(candidate)) {
      flood_key_event_(MSG_KEY_ANNOUNCE, candidate);
      gossip_next_app_idx_ = (uint8_t) (candidate + 1);
      break;
    }
  }

  broadcast_own_status_();
}

// Shared by handle_adv_report_() (payload extracted from an overheard
// advertisement) and the GATT cmd-inject characteristic (payload written
// directly by a connected phone, already in this exact shape -- see
// RelayPacket.toBytes() in vendor_command.dart): dedup, decrypt+dispatch if
// it's a CMD/KEY_ANNOUNCE/KEY_REVOKE addressed to or relevant to us, and
// relay onward (flooding) regardless of who it's addressed to. A phone can
// never broadcast a flood packet itself (iOS CoreBluetooth restriction, see
// the plan), so this is also the only path by which a phone's CMD ever
// enters the mesh.
static void process_relay_payload_(const uint8_t *payload, uint8_t payload_len) {
  if (payload_len < PAYLOAD_HEADER_LEN + MIC_LEN) return;

  uint8_t msg_type = payload[0];
  uint8_t ttl = payload[1];
  uint32_t msg_id = ((uint32_t) payload[2] << 24) | ((uint32_t) payload[3] << 16) | ((uint32_t) payload[4] << 8) |
                     payload[5];
  uint16_t src = (payload[6] << 8) | payload[7];
  uint16_t dst = (payload[8] << 8) | payload[9];
  uint8_t app_idx = payload[10];
  const uint8_t *cipher_and_mic = payload + PAYLOAD_HEADER_LEN;
  uint8_t cipher_len = payload_len - PAYLOAD_HEADER_LEN - MIC_LEN;

  if (src == own_addr_) return;  // our own packet, e.g. echoed by a relay
  if (dedup_seen_(src, msg_id)) return;
  dedup_remember_(src, msg_id);

  if (cipher_len == 2) {
    uint8_t nonce[13];
    build_nonce_(msg_type, src, msg_id, nonce);
    uint8_t plain[2];

    if (msg_type == MSG_CMD && (dst == own_addr_ || dst == ADDR_BROADCAST)) {
      if (!app_idx_valid_(app_idx)) {
        ESP_LOGW(TAG, "CMD with unauthorized app_idx=0x%02x, dropping", app_idx);
      } else {
        uint8_t app_key[16];
        if (derive_app_key_(app_idx, app_key) &&
            ccm_decrypt_(app_key, nonce, cipher_and_mic, cipher_len, cipher_and_mic + cipher_len, plain)) {
          ESP_LOGI(TAG, "CMD from 0x%04x: cmd=%u param=%u", src, plain[0], plain[1]);
          dispatch_command_(plain[0], plain[1]);
          send_status_(app_idx);
        } else {
          ESP_LOGW(TAG, "CMD from 0x%04x failed to authenticate, dropping", src);
        }
      }
    } else if (msg_type == MSG_KEY_ANNOUNCE || msg_type == MSG_KEY_REVOKE) {
      if (ccm_decrypt_(NET_KEY, nonce, cipher_and_mic, cipher_len, cipher_and_mic + cipher_len, plain)) {
        uint8_t announced_app_idx = plain[0];
        bool now_valid = (msg_type == MSG_KEY_ANNOUNCE);
        if (app_idx_valid_(announced_app_idx) != now_valid) {
          app_idx_set_valid_(announced_app_idx, now_valid);
          save_key_validity_();
          ESP_LOGI(TAG, "%s app_idx=0x%02x (from 0x%04x)", now_valid ? "Authorized" : "Revoked", announced_app_idx,
                    src);
        }
      } else {
        ESP_LOGW(TAG, "KEY_ANNOUNCE/REVOKE from 0x%04x failed to authenticate, dropping", src);
      }
    }
  } else if (msg_type == MSG_CMD || msg_type == MSG_KEY_ANNOUNCE || msg_type == MSG_KEY_REVOKE) {
    ESP_LOGW(TAG, "msg_type=%u payload wrong size (%u)", msg_type, cipher_len);
  }

  if (ttl == 0) return;
  // Relay onward regardless of who it's addressed to -- flooding.
  uint8_t adv[MAX_ADV_LEN];
  uint8_t len = build_adv_(msg_type, ttl - 1, msg_id, src, dst, app_idx, cipher_and_mic, cipher_len + MIC_LEN, adv);
  if (len == 0) return;
  // MSG_CMD gets extra redundancy at this hop specifically -- see
  // CMD_RELAY_REPEAT_COUNT's comment. Deliberately the SAME msg_id for
  // every repeat here (unlike flood_key_event_repeated_(), which uses a
  // fresh one per call): dedup_seen_() above already guarantees this
  // function only reaches this point once per (src, msg_id) no matter how
  // many of a previous hop's own repeated copies actually arrived, so a
  // neighbour that catches any one of these three copies processes it
  // once and relays its own three onward -- reusing the id doesn't cause
  // re-processing, and a fresh one isn't needed to get past this node's
  // own dedup a second time.
  uint8_t repeats = (msg_type == MSG_CMD) ? CMD_RELAY_REPEAT_COUNT : 1;
  for (uint8_t i = 0; i < repeats; i++) {
    enqueue_tx_(adv, len, 150);
  }
}

// Parses one received advertising report's raw AD bytes; if it's one of
// ours (matching Company ID), hands the payload to process_relay_payload_().
static void handle_adv_report_(const uint8_t *data, uint8_t length) {
  // Walk AD structures looking for Manufacturer Specific Data (0xFF).
  uint8_t i = 0;
  const uint8_t *mfg = nullptr;
  uint8_t mfg_len = 0;
  while (i < length) {
    uint8_t struct_len = data[i];
    if (struct_len == 0 || i + 1 + struct_len > length) break;
    uint8_t ad_type = data[i + 1];
    if (ad_type == 0xFF && struct_len >= 3) {
      mfg = data + i + 2;
      mfg_len = struct_len - 1;
      break;
    }
    i += 1 + struct_len;
  }
  if (mfg == nullptr) return;

  uint16_t cid = mfg[0] | (mfg[1] << 8);
  if (cid != COMPANY_ID) return;
  process_relay_payload_(mfg + 2, mfg_len - 2);
}

// --- GATT access callback ----------------------------------------------
static int gatt_access_cb_(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->chr->uuid == &enroll_chr_uuid_.u) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    if (!provisioned_) {
      // Unprovisioned: return a single zero byte instead of the usual 35 --
      // response *length* is how EnrollConnection.readEnrollment() tells
      // "this blind needs provisioning first" apart from a normal
      // enrollment, without having to sniff platform-specific ATT error
      // codes across iOS/Android/bleak.
      uint8_t sentinel = 0;
      return os_mbuf_append(ctxt->om, &sentinel, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    EnrollResult res;
    if (!enroll_new_phone_(&res)) return BLE_ATT_ERR_UNLIKELY;
    // Response layout (35 bytes, must match EnrollConnection.readEnrollment()
    // in ble_transport.dart): netKey(16) + appKey(16) + appIdx(1) + ownAddr(2, BE).
    uint8_t resp[35];
    memcpy(resp, NET_KEY, 16);
    memcpy(resp + 16, res.app_key, 16);
    resp[32] = res.app_idx;
    resp[33] = (uint8_t) (res.addr >> 8);
    resp[34] = (uint8_t) res.addr;
    return os_mbuf_append(ctxt->om, resp, sizeof(resp)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  if (ctxt->chr->uuid == &cmd_inject_chr_uuid_.u) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    uint8_t buf[MAX_ADV_LEN];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0) return BLE_ATT_ERR_UNLIKELY;
    process_relay_payload_(buf, (uint8_t) len);
    return 0;
  }

  if (ctxt->chr->uuid == &revoke_chr_uuid_.u) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    uint8_t app_idx = 0;
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, &app_idx, sizeof(app_idx), &len) != 0 || len != 1) {
      return BLE_ATT_ERR_UNLIKELY;
    }
    revoke_app_idx_(app_idx);
    return 0;
  }

  if (ctxt->chr->uuid == &provision_chr_uuid_.u) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    // The one real access check anywhere in this GATT server: refuse to
    // overwrite an already-provisioned blind's NetKey over BLE, compile-time
    // or not, or anyone in range could hijack a working blind.
    if (provisioned_) return BLE_ATT_ERR_INSUFFICIENT_AUTHOR;
    uint8_t key[16];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, key, sizeof(key), &len) != 0 || len != sizeof(key)) {
      return BLE_ATT_ERR_UNLIKELY;
    }
    return provision_net_key_(key) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  if (ctxt->chr->uuid == &own_addr_chr_uuid_.u) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    uint8_t resp[2] = {(uint8_t) (own_addr_ >> 8), (uint8_t) own_addr_};
    return os_mbuf_append(ctxt->om, resp, sizeof(resp)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

static int gap_event_handler_(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_DISC:
      handle_adv_report_(event->disc.data, event->disc.length_data);
      return 0;
    case BLE_GAP_EVENT_CONNECT: {
      AdvLock lock;
      if (event->connect.status == 0) {
        phone_connected_ = true;
        // The connectable idle beacon that led to this connection was
        // consumed by it -- NimBLE stops it automatically (no separate
        // ADV_COMPLETE fires for that case, CONNECT replaces it), but
        // adv_state_ was never updated to match. Left stale, pump_tx_queue_()
        // would later call ble_gap_adv_stop() on an already-stopped
        // advertisement before its first flood burst -- harmless (the
        // return code isn't checked) but worth keeping consistent now that
        // flood bursts run concurrently with an active connection.
        adv_state_ = ADV_STATE_NONE;
        ESP_LOGI(TAG, "Phone connected, conn_handle=%u", event->connect.conn_handle);
      }
      return 0;
    }
    case BLE_GAP_EVENT_DISCONNECT: {
      AdvLock lock;
      phone_connected_ = false;
      ESP_LOGI(TAG, "Phone disconnected, reason=%d", event->disconnect.reason);
      start_idle_adv_();
      return 0;
    }
    case BLE_GAP_EVENT_ADV_COMPLETE: {
      AdvLock lock;
      adv_state_ = ADV_STATE_NONE;
      start_idle_adv_();  // no-op if a phone is now connected
      return 0;
    }
    default:
      return 0;
  }
}

static void start_scan_() {
  struct ble_gap_disc_params params = {};
  params.passive = 1;
  params.filter_duplicates = 0;
  params.itvl = 160;    // 100ms in 0.625ms units
  params.window = 160;  // continuous (window == interval)
  int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event_handler_, nullptr);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
  }
}

void BleRelay::set_net_key(std::array<uint8_t, 16> key) {
  memcpy(NET_KEY, key.data(), 16);
  provisioned_ = true;
}

void BleRelay::setup(JalouzeeBlinds *owner) {
  owner_ = owner;

  if (!nimble_bringup()) {
    ESP_LOGE(TAG, "BLE bring-up failed - relay will not work");
    return;
  }

  key_validity_pref_ = global_preferences->make_preference<AppKeyValidityStore>(fnv1_hash("jalouzee_ble_relay_appidx"));
  if (!key_validity_pref_.load(&key_validity_)) {
    key_validity_ = AppKeyValidityStore{};  // fresh device: nothing authorized yet
  }

  net_key_pref_ = global_preferences->make_preference<NetKeyStore>(fnv1_hash("jalouzee_ble_relay_netkey"));
  if (!provisioned_) {
    // Not set at compile time (set_net_key() wasn't called from codegen) --
    // fall back to whatever was provisioned over BLE on a previous boot, if
    // any. A load failure here just means "never provisioned yet", which is
    // the normal state for fresh factory firmware.
    if (net_key_pref_.load(&net_key_store_)) {
      memcpy(NET_KEY, net_key_store_.net_key, 16);
      provisioned_ = true;
    }
  }

  // Armed unconditionally -- originally this was skipped for boards with a
  // compile-time NetKey (their owner already has ESPHome access, so a flaky
  // power supply triggering an accidental reset was pure downside for
  // them). Deliberately removed at the user's request, accepting that
  // trade-off, so this maintainer's own !secret-configured boards can also
  // be reset by power-cycling for testing -- see docs/plans/
  // vivid-noodling-gray.md.
  boot_counter_pref_ = global_preferences->make_preference<BootCounterStore>(fnv1_hash("jalouzee_ble_relay_bootcount"));
  if (!boot_counter_pref_.load(&boot_counter_)) {
    boot_counter_ = BootCounterStore{};
  }
  boot_counter_.count++;
  boot_counter_pref_.save(&boot_counter_);
  ESP_LOGI(TAG, "Power-cycle count: %u/%u (%u quick cycles in a row triggers a factory reset)", boot_counter_.count,
           FACTORY_RESET_BOOT_COUNT, FACTORY_RESET_BOOT_COUNT);
  if (boot_counter_.count >= FACTORY_RESET_BOOT_COUNT) {
    factory_reset_();
  }
  setup_millis_ = millis();

  ESP_LOGI(TAG, "BLE relay up, own_addr=0x%04x, provisioned=%d, scanning...", own_addr_, provisioned_);
  start_scan_();
  start_idle_adv_();
}

// --- loop(): pump the outgoing relay queue + periodic status dump ---------
// Runs even while a phone is GATT-connected (phone_connected_) -- flood
// bursts are non-connectable advertising, a separate radio activity from
// the peripheral connection, and ESP32's NimBLE controller multiplexes the
// two concurrently. Earlier this queue was gated on !phone_connected_,
// which meant relaying anything (including a broadcast CMD the phone just
// injected into *this* node) silently stalled for as long as the phone
// stayed connected -- every other blind in the fleet would only catch up
// once this node's connection finally dropped. Found 2026-07-25 diagnosing
// exactly that: board1 executed a backlog of commands in one burst, several
// minutes after the taps that produced them, right as the connected node's
// GATT session ended.
static void pump_tx_queue_() {
  AdvLock lock;
  if (adv_state_ == ADV_STATE_FLOOD_BURST) return;
  uint32_t now = millis();
  for (auto &slot : tx_queue_) {
    if (!slot.valid || slot.send_at_ms > now) continue;

    if (adv_state_ == ADV_STATE_IDLE_BEACON) {
      ble_gap_adv_stop();
      adv_state_ = ADV_STATE_NONE;
    }

    struct ble_gap_adv_params adv_params = {};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_NON;
    adv_params.itvl_min = 160;  // 100ms
    adv_params.itvl_max = 160;

    if (ble_gap_adv_set_data(slot.adv_bytes, slot.adv_len) != 0) {
      ESP_LOGW(TAG, "ble_gap_adv_set_data failed");
      slot.valid = false;
      return;
    }
    // Short burst (a few repeats at the 100ms interval above), then
    // BLE_GAP_EVENT_ADV_COMPLETE fires and resumes the idle beacon.
    if (ble_gap_adv_start(own_addr_type, nullptr, 350, &adv_params, gap_event_handler_, nullptr) != 0) {
      ESP_LOGW(TAG, "ble_gap_adv_start failed");
    } else {
      adv_state_ = ADV_STATE_FLOOD_BURST;
    }
    slot.valid = false;
    return;  // one at a time
  }
}

static constexpr uint32_t STATUS_LOG_INTERVAL_MS = 30000;
static uint32_t last_status_log_ms_ = 0;

void BleRelay::loop() {
  pump_tx_queue_();

  // Independent of the throttled status log below -- must run every tick so
  // FACTORY_RESET_GRACE_MS stays accurate to within one loop() period,
  // rather than being coupled to STATUS_LOG_INTERVAL_MS's much longer 30s
  // window (which would make a stable boot's counter-clear arrive late,
  // undermining what makes "N quick cycles" specific -- see this section's
  // comment further up).
  const uint32_t now = millis();

  if (!boot_count_cleared_ && now - setup_millis_ >= FACTORY_RESET_GRACE_MS) {
    boot_count_cleared_ = true;
    if (boot_counter_.count != 0) {
      boot_counter_ = BootCounterStore{};
      boot_counter_pref_.save(&boot_counter_);
      ESP_LOGI(TAG, "Boot confirmed stable, power-cycle counter reset to 0");
    }
  }

  // Same reasoning as the block above -- must run every tick, not just when
  // the throttled status log below happens to fire, or
  // PERIODIC_BROADCAST_INTERVAL_MS would effectively stretch out to
  // STATUS_LOG_INTERVAL_MS.
  if (provisioned_ && now - last_periodic_broadcast_ms_ >= PERIODIC_BROADCAST_INTERVAL_MS) {
    last_periodic_broadcast_ms_ = now;
    run_periodic_broadcast_tick_();
  }

  if (now - last_status_log_ms_ < STATUS_LOG_INTERVAL_MS) return;
  last_status_log_ms_ = now;
  ESP_LOGI(TAG, "status: own_addr=0x%04x adv_state=%d phone_connected=%d", own_addr_, adv_state_, phone_connected_);
}

void BleRelay::notify_movement_stopped() {
  if (!provisioned_) return;
  broadcast_own_status_();
  // Avoid a near-duplicate send from the periodic tick landing right after
  // this one -- not incorrect either way (broadcasts are idempotent, each
  // just reports "current position now"), just wasted radio time.
  last_periodic_broadcast_ms_ = millis();
}

}  // namespace jalouzee_blinds
}  // namespace esphome

#endif  // USE_ESP32
