import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.esp32 import add_idf_sdkconfig_option
from esphome.const import CONF_ID

# Step 1 bring-up for BLE Mesh integration (see docs/ble-remote-design.md):
# just gets a node provisioned and able to receive/ack a custom vendor-model
# message. Not wired into jalouzee_blinds' cover control yet.
CODEOWNERS = ["@your-github-handle"]
DEPENDENCIES = ["esp32"]

ble_mesh_cover_ns = cg.esphome_ns.namespace("ble_mesh_cover")
BleMeshTest = ble_mesh_cover_ns.class_("BleMeshTest", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BleMeshTest),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on(["esp32"]),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # NimBLE host — lighter-weight than Bluedroid, and what esp_ble_mesh's
    # NimBLE integration (used below) expects. Mutually exclusive with
    # ESPHome's own esp32_ble component (which hardcodes Bluedroid) — fine
    # here since this project doesn't use esp32_ble.
    add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", False)

    # BLE Mesh: node role, GATT provisioning bearer (so a phone can
    # provision/reach this device without PB-ADV support), settings
    # persistence (NetKey/AppKey/addresses survive reboot), GATT proxy so
    # phones can reach the mesh without being relay nodes themselves.
    add_idf_sdkconfig_option("CONFIG_BLE_MESH", True)
    add_idf_sdkconfig_option("CONFIG_BLE_MESH_NODE", True)
    add_idf_sdkconfig_option("CONFIG_BLE_MESH_PB_GATT", True)
    add_idf_sdkconfig_option("CONFIG_BLE_MESH_SETTINGS", True)
    add_idf_sdkconfig_option("CONFIG_BLE_MESH_GATT_PROXY_SERVER", True)
