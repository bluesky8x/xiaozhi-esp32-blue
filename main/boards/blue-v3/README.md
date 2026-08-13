# Blue V3 — LCD + audio (chat stability)

Same **ST7789 1.54" + Otto GIF face + INMP441/MAX98357** as Blue V2, without motor, ToF, or robot MCP tools.

Use this profile to test **chat stability** without SPI/motor/I2C interference.

## vs Blue V2

| | V2 (full) | V3 (chat test) |
|---|-----------|----------------|
| ST7789 + Otto | Yes | Yes |
| Audio I2S | Yes | Yes |
| Motor / ToF | Optional | **No** |
| Deferred I2S init | Yes | Yes |

## Wiring

Identical to [Blue V2](../blue-v2/WIRING.md) for MCU, LCD, and audio.

## Build (macOS / Linux)

```bash
get_idf
cd esp32-blue
python scripts/build.py blue-v3
idf.py -p /dev/cu.usbmodem* flash monitor
```

Pair with `esp32-server-blue` on your LAN.
