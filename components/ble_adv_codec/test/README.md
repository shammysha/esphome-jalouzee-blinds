Standalone tests for `ble_adv_codec`, run manually — ESPHome external
components have no built-in unit test runner, and these aren't part of the
compiled firmware.

## `test_packet.cpp`

Pure logic (packet pack/unpack, replay-counter comparison) — no crypto
backend, so it builds with any C++17 compiler, host or target:

```sh
g++ -std=c++17 -Wall -Wextra -I.. -o test_packet test_packet.cpp ../packet.cpp
./test_packet
```

## CMAC (`cmac.cpp`) — not yet covered by an automated test here

`cmac.cpp` links against mbedtls, which on this project only exists inside
ESP-IDF's bundled copy (pulled in automatically by `esphome compile`, not
the system's `libmbedtls-dev`) — so it isn't buildable as a standalone host
binary the way `test_packet.cpp` is. Instead, the algorithm and wire format
were cross-checked with Python's `cryptography` library against the RFC
4493 CMAC test vectors, plus a worked example matching this component's
exact header layout:

```python
from cryptography.hazmat.primitives import cmac
from cryptography.hazmat.primitives.ciphers import algorithms

c = cmac.CMAC(algorithms.AES(bytes.fromhex("000102030405060708090a0b0c0d0e0f")))
c.update(bytes.fromhex("4a11066401020302"))  # magic, device_id=0x11, cmd=SET_POSITION,
                                              # param=0x64, counter=0x010203, slot_id=0x02
c.finalize()[:4].hex()  # -> "edeb8840"
```

Once `ble_adv_remote` or `ble_adv_cover` exists as a real, buildable YAML
target, add an ESPHome-side check (e.g. a boot-time log line comparing
`compute_cmac()`'s output for this exact vector against `edeb8840`) to
confirm the mbedtls-linked code actually produces it.
