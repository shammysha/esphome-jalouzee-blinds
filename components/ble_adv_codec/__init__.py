"""
ble_adv_codec — shared library component for the jalouzee_blinds BLE
broadcast remote-control protocol (see docs/ble-remote-design.md).

Not user-facing: it has no YAML platform of its own and is never referenced
directly in a config. ble_adv_remote and ble_adv_cover pull it in via
DEPENDENCIES = ["ble_adv_codec"] and use its C++ API (packet.h/cmac.h/
codec.h) directly.

ESP32-only, like the rest of the BLE protocol — cmac.cpp itself is guarded
with #ifdef USE_ESP32 (compiles to a false-returning stub otherwise), but
components that actually use this one for real should still gate their own
config with cv.only_on(["esp32"]).
"""

CODEOWNERS = ["@your-github-handle"]
