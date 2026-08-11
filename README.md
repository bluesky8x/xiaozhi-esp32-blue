# esp32-blue

Firmware for **Blue V1** — ESP32-S3 voice robot (Xiaozhi / 小智).

| Spec | Value |
|------|--------|
| MCU | ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB Octal PSRAM) |
| Display | ST7789 1.54" 240×240, Otto GIF face |
| Motor | MX1508 (PWM on GPIO 11–14) |
| Audio | INMP441 + MAX98357 |

## Setup (full stack on Mac)

Two repos — do **not** mix them:

| Repo | Folder | Command |
|------|--------|---------|
| **Server** (Python) | `esp32-server-blue` | `./run.sh` |
| **Firmware** (ESP32) | `esp32-blue` | `idf.py flash monitor` |

Detailed Intel MacBook Pro 2019 guide: [esp32-server-blue/docs/macos-intel-build.md](../esp32-server-blue/docs/macos-intel-build.md)

### Phase 1 — One-time tools (~1 hour)

```bash
xcode-select --install
brew install git pyenv ffmpeg opus cmake ninja dfu-util ccache openssl@3 readline xz zlib

# pyenv in ~/.zshrc, then:
pyenv install 3.10.19

# ESP-IDF 6.0.2
mkdir -p ~/esp && cd ~/esp
git clone -b v6.0.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3
```

Add to `~/.zshrc`:

```bash
get_idf() {
  export IDF_PYTHON_ENV_PATH="$HOME/.espressif/python_env/idf6.0_py3.12_env"
  . "$HOME/esp/esp-idf/export.sh"
}
```

Run `source ~/.zshrc`. Always use **`get_idf`** (not bare `source export.sh`) if your system Python is 3.14.

### Phase 2 — Server

```bash
cd ~/work/esp32-server-blue/main/xiaozhi-server
pyenv local 3.10.19
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

cp data/.config.yaml.example data/.config.yaml
# Edit data/.config.yaml — API keys + LAN IP (see below)

cd ~/work/esp32-server-blue
./run.sh
```

Get Mac LAN IP: `ipconfig getifaddr en0`

In `data/.config.yaml`:

```yaml
server:
  websocket: ws://YOUR_LAN_IP:8000/xiaozhi/v1/
  vision_explain: http://YOUR_LAN_IP:8003/mcp/vision/explain
LLM:
  GeminiLLM:
    api_key: ...
ASR:
  OpenaiASR:
    api_key: ...
```

**Firewall:** System Settings → Firewall → allow **python3.10** (pyenv) incoming connections.

Test: `curl http://YOUR_LAN_IP:8003/xiaozhi/ota/`

### Phase 3 — Firmware (in `esp32-blue`, not esp32-server-blue)

```bash
deactivate                    # exit server .venv if active
get_idf

cd ~/work/esp32-blue
python scripts/build.py blue-v2    # first time: 20–45 min on Intel Mac

ls /dev/cu.usbmodem*
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

Replace the port with yours from `ls`.

### Phase 4 — Robot WiFi / OTA

#### How to open the WiFi setup page

The robot creates a hotspot **`Xiaozhi-XXXX`** (XXXX = last 2 bytes of MAC, e.g. `Xiaozhi-FB01`). The setup page is **`http://192.168.4.1`**.

**Automatic (first boot or no WiFi saved)**  
Power on → wait ~2 s → robot enters配网 mode (screen shows hotspot name + URL).

**During boot**  
While the face is still starting up, **short-press the BOOT button once** (GPIO 0).

**WiFi wrong / can’t connect**  
After **60 seconds** of failed connect, it enters config mode automatically.

**Change WiFi or OTA later**  
**Hold BOOT for 5 seconds** → factory reset → reboot → config AP comes back.  
*(Clears saved WiFi and settings.)*

#### On your phone or Mac

1. Open WiFi settings and join **`Xiaozhi-XXXX`** (no password on most builds).
2. Open a browser → **`http://192.168.4.1`**
3. Choose your home WiFi, enter password, **Save**.
4. Tap **Advanced** at the top → set **OTA URL**:
   ```text
   http://YOUR_LAN_IP:8003/xiaozhi/ota/
   ```
5. Save → robot reboots and joins your WiFi.

Serial monitor shows the exact SSID, e.g. `Wifi config mode entered` and `Xiaozhi-FB01`.

After activation, the robot uses your server WebSocket on port **8000**.

### Daily use

```bash
# Terminal A — server
cd ~/work/esp32-server-blue && ./run.sh

# Terminal B — serial log (optional)
get_idf && cd ~/work/esp32-blue && idf.py -p /dev/cu.usbmodem1101 monitor
```

---

## Install ESP-IDF (macOS / Linux)

Blue builds with **ESP-IDF v6.0.2** (default in `scripts/build.py`). Target: **esp32s3**.

Full stack setup (server + firmware): [esp32-server-blue/BLUE.md](../esp32-server-blue/BLUE.md#macos--install-dependencies-local-build)

**MacBook Pro 2019 (Intel):** [esp32-server-blue/docs/macos-intel-build.md](../esp32-server-blue/docs/macos-intel-build.md)

### macOS prerequisites

```bash
xcode-select --install
brew install cmake ninja dfu-util ccache git
```

### Install ESP-IDF

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v6.0.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
```

Add to `~/.zshrc` (optional):

```bash
get_idf() {
  export IDF_PYTHON_ENV_PATH="$HOME/.espressif/python_env/idf6.0_py3.12_env"
  . "$HOME/esp/esp-idf/export.sh"
}
```

Verify:

```bash
source ~/esp/esp-idf/export.sh
idf.py --version
```

Official docs: [ESP-IDF get started — macOS](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32/get-started/macos-setup.html)

> ESP-IDF uses its own Python 3.12 under `~/.espressif/`. This is separate from the **pyenv 3.10** used by esp32-server-blue.

## Build

```bash
cd esp32-blue
source ~/esp/esp-idf/export.sh   # or: get_idf

python scripts/build.py blue-v2   # new PCB (recommended)
# python scripts/build.py blue-v1 # legacy PCB
```

## Flash

```bash
ls /dev/cu.usbmodem*              # Mac — pick your port
source ~/esp/esp-idf/export.sh
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

Set OTA URL on device (WiFi portal → Advanced) to your server, e.g. `http://192.168.x.x:8003/xiaozhi/ota/` — see [esp32-server-blue/BLUE.md](../esp32-server-blue/BLUE.md).

## QEMU (no hardware)

Run in ESP-IDF QEMU — [main/boards/blue-v2/QEMU.md](main/boards/blue-v2/QEMU.md).

```bash
export IDF_PYTHON_ENV_PATH=~/.espressif/python_env/idf6.0_py3.12_env
source ~/esp/esp-idf/export.sh
cd esp32-blue
python scripts/build.py blue-v2
idf.py qemu
```

Pair with [esp32-server-blue](../esp32-server-blue/BLUE.md) WebSocket on your LAN.

## Docs

- [Blue V2 board](main/boards/blue-v2/README.md) — **new PCB (recommended)**
- [Blue V2 wiring / pin map](main/boards/blue-v2/WIRING.md)
- [Blue V1 board](main/boards/blue-v1/README.md) — legacy pinout
- [QEMU simulator](main/boards/blue-v2/QEMU.md)
- [Wiring / GPIO (V1)](main/boards/blue-v1/WIRING.md)
- [Backend server](../esp32-server-blue/BLUE.md) — Xiaozhi + Kira character (`esp32-server-blue`)

## Stack

| Component | Path |
|-----------|------|
| Firmware (this repo) | `esp32-blue` — boards **`blue-v1`** (legacy PCB) · **`blue-v2`** (pin-optimized, recommended for new PCB) |
| Python server | `esp32-server-blue` — run `./run.sh` after config |

## Ported from

Board profile migrated from `xiaozhi-esp32/main/boards/blue-v1`.
