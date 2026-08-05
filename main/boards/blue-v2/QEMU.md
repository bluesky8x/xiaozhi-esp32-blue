# Blue V2 — QEMU simulator

Same as [Blue V1 QEMU](../blue-v1/QEMU.md), but build target **`blue-v2`**.

```bash
export IDF_PYTHON_ENV_PATH=~/.espressif/python_env/idf6.0_py3.12_env
source ~/esp/esp-idf/export.sh
cd ~/work/esp32-blue
python scripts/build.py blue-v2

pkill -9 qemu-system-xtensa 2>/dev/null
pkill -f "idf.py qemu" 2>/dev/null
idf.py qemu
```

Backend: [esp32-server-blue/BLUE.md](../../../../esp32-server-blue/BLUE.md)
