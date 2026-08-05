# Blue V1 — QEMU simulator

Run Blue V1 firmware in **ESP-IDF QEMU** (no hardware). Useful for WebSocket / MCP integration testing against [esp32-server-blue](../../../../esp32-server-blue/BLUE.md).

Requires **ESP-IDF 6.x** (QEMU target for ESP32-S3).

## Stop stale QEMU processes

If a previous run hung or Ctrl+C left QEMU running:

```bash
pkill -9 qemu-system-xtensa 2>/dev/null
pkill -f "idf.py qemu" 2>/dev/null
```

## Build (first time or after code changes)

```bash
export IDF_PYTHON_ENV_PATH=~/.espressif/python_env/idf6.0_py3.12_env
source ~/esp/esp-idf/export.sh
cd ~/work/esp32-blue
python scripts/build.py blue-v1
```

If your checkout is still named `xiaozhi-esp32`, use `cd ~/work/xiaozhi-esp32` instead — same board profile.

## Run QEMU

```bash
export IDF_PYTHON_ENV_PATH=~/.espressif/python_env/idf6.0_py3.12_env
source ~/esp/esp-idf/export.sh
cd ~/work/xiaozhi-esp32
idf.py qemu
```

One-liner after build:

```bash
pkill -9 qemu-system-xtensa 2>/dev/null; pkill -f "idf.py qemu" 2>/dev/null; \
export IDF_PYTHON_ENV_PATH=~/.espressif/python_env/idf6.0_py3.12_env && \
source ~/esp/esp-idf/export.sh && cd ~/work/xiaozhi-esp32 && idf.py qemu
```

## Server pairing

1. Start backend: `esp32-server-blue/run.sh` (WebSocket default `ws://<lan-ip>:8000/xiaozhi/v1/`).
2. Configure QEMU device WiFi / OTA URL to that WebSocket (same as real hardware).
3. Optional browser test without QEMU: [digital-human](../../../../esp32-server-blue/BLUE.md#digital-human-browser) at `http://127.0.0.1:8006/index.html`.

## Notes

- Motor / display / audio behavior in QEMU is limited vs real Blue V1 hardware; use digital-human for full MCP motor simulation.
- Adjust `IDF_PYTHON_ENV_PATH` if your ESP-IDF Python env name differs (`ls ~/.espressif/python_env/`).
