#!/bin/bash
# Скрипт ТОЛЬКО для прошивки SPIFFS
set -e
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
fi
PORT=$(ls /dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART /dev/cu.ch341* 2>/dev/null | head -n 1)
if [ -z "$PORT" ]; then
    echo "ERROR: Serial port not found!"
    exit 1
fi
echo "--- Flashing SPIFFS to 0xc20000 on $PORT ---"
esptool.py --chip esp32 --port "$PORT" --baud 921600 write_flash 0xc20000 spiffs.bin