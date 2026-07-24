#include "ble_mesh_test.h"

#ifdef USE_ESP32

#include <cstring>
#include <inttypes.h>

#include "esphome/core/log.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace esphome {
namespace ble_mesh_cover {

static const char *const TAG = "ble_mesh_cover";

// Placeholder — NOT a real Bluetooth SIG-assigned Company ID. Fine for
// private/development use; a commercial product needs to register a real
// one (or use a different vendor-model identification scheme) before
// distribution.
static constexpr uint16_t BLE_MESH_CID = 0xFFFF;

static constexpr uint16_t VND_MODEL_ID_COVER = 0x0000;
static constexpr uint32_t VND_OP_COVER_CMD = ESP_BLE_MESH_MODEL_OP_3(0x00, BLE_MESH_CID);
static constexpr uint32_t VND_OP_COVER_STATUS = ESP_BLE_MESH_MODEL_OP_3(0x01, BLE_MESH_CID);

// "JB" for jalouzee_blinds — bytes [2..15] get overwritten with the node's
// own BLE address once known (see BleMeshTest::setup()), same convention as
// ESP-IDF's own ble_mesh_example_init.c: keeps distinct unprovisioned
// devices from colliding on the same UUID.
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x4A, 0x42};

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_COVER_CMD, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(BLE_MESH_CID, VND_MODEL_ID_COVER, vnd_op, NULL, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = BLE_MESH_CID,
    .element_count = 1,
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

static void prov_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param) {
  switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
      ESP_LOGI(TAG, "Mesh provisioning callback registered, err=%d", param->prov_register_comp.err_code);
      break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
      ESP_LOGI(TAG, "Provisioning enabled, err=%d", param->node_prov_enable_comp.err_code);
      break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
      ESP_LOGI(TAG, "Provisioning link open (%s)",
               param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
      break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
      ESP_LOGI(TAG, "Provisioning link closed");
      break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
      ESP_LOGI(TAG, "Provisioning complete - net_idx=0x%03x addr=0x%04x", param->node_prov_complete.net_idx,
               param->node_prov_complete.addr);
      break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
      ESP_LOGW(TAG, "Node reset (unprovisioned)");
      break;
    default:
      break;
  }
}

static void config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param) {
  if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT &&
      param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD) {
    ESP_LOGI(TAG, "AppKey added - net_idx=0x%04x app_idx=0x%04x", param->value.state_change.appkey_add.net_idx,
             param->value.state_change.appkey_add.app_idx);
  }
}

static void custom_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param) {
  switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
      if (param->model_operation.opcode == VND_OP_COVER_CMD && param->model_operation.length >= 2) {
        uint8_t cmd = param->model_operation.msg[0];
        uint8_t cmd_param = param->model_operation.msg[1];
        ESP_LOGI(TAG, "Received cover command: cmd=%u param=%u", cmd, cmd_param);
        // TODO(step 2): dispatch into JalouzeeBlinds::control() instead of
        // just acking with the command byte we received.
        esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0], param->model_operation.ctx,
                                                            VND_OP_COVER_STATUS, sizeof(cmd), &cmd);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "Failed to send status ack: %d", err);
        }
      }
      break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
      if (param->model_send_comp.err_code != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
      }
      break;
    default:
      break;
  }
}

// --- NimBLE host bring-up -------------------------------------------------
// Ported near-verbatim from ESP-IDF's examples/bluetooth/esp_ble_mesh/
// vendor_models/vendor_server (NimBLE branch of ble_mesh_example_init.c) —
// this is proven bring-up sequencing, not something to improvise on.
static SemaphoreHandle_t mesh_sem;
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};

static void mesh_on_reset(int reason) { ESP_LOGI(TAG, "NimBLE host reset, reason=%d", reason); }

static void mesh_on_sync() {
  ble_hs_util_ensure_addr(0);
  if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
    ESP_LOGE(TAG, "Failed to infer own BLE address type");
    return;
  }
  ble_hs_id_copy_addr(own_addr_type, addr_val, nullptr);
  xSemaphoreGive(mesh_sem);
}

static void mesh_host_task(void *) {
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run();  // returns only after nimble_port_stop()
  nimble_port_freertos_deinit();
}

// From the NimBLE host's "store/config" component — persists bonding data
// to NVS. No public header in this ESP-IDF layout (same as the upstream
// example), hence the forward declaration instead of an #include.
extern "C" void ble_store_config_init(void);

static bool nimble_bringup() {
  mesh_sem = xSemaphoreCreateBinary();
  if (mesh_sem == nullptr) {
    ESP_LOGE(TAG, "Failed to create bring-up semaphore");
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
  nimble_port_freertos_init(mesh_host_task);
  xSemaphoreTake(mesh_sem, portMAX_DELAY);
  return true;
}

void BleMeshTest::setup() {
  if (!nimble_bringup()) {
    ESP_LOGE(TAG, "BLE bring-up failed - mesh will not start");
    this->mark_failed();
    return;
  }

  memcpy(dev_uuid + 2, addr_val, sizeof(addr_val));

  esp_ble_mesh_register_prov_callback(prov_cb);
  esp_ble_mesh_register_config_server_callback(config_server_cb);
  esp_ble_mesh_register_custom_model_callback(custom_model_cb);

  if (esp_ble_mesh_init(&provision, &composition) != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_mesh_init failed");
    this->mark_failed();
    return;
  }
  if (esp_ble_mesh_node_prov_enable(
          (esp_ble_mesh_prov_bearer_t) (ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT)) != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_mesh_node_prov_enable failed");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "BLE Mesh node initialized, waiting for provisioning");
}

void BleMeshTest::dump_config() { ESP_LOGCONFIG(TAG, "BLE Mesh Cover (bring-up test)"); }

}  // namespace ble_mesh_cover
}  // namespace esphome

#endif  // USE_ESP32
