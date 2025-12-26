Import("env")
import os

# В PlatformIO SCons __file__ не определен.
# Используем путь от корня проекта.
project_root = "/Users/andreibodrov/work/codec"
monolith = os.path.join(
    project_root,
    "esphome_src",
    "esphome",
    "components",
    "cx_audio",
    "lib",
    "libva_sdk_monolith.a",
)

print(f"CX_AUDIO [LINKER FIX]: Applying isolation. Monolith: {monolith}")

# Добавляем флаги ТОЛЬКО для приложения
# env.Append(LINKFLAGS=...) в PlatformIO ESP-IDF сборке обычно
# нацелен на финальный firmware.elf
env.Append(
    LINKFLAGS=[
        "-Wl,-u,va_board_init",
        "-Wl,-u,media_hal_get_handle",
        "-Wl,-u,va_dsp_init",
        "-Wl,--whole-archive",
        monolith,
        "-Wl,--no-whole-archive",
    ]
)

# Включаем стандартную библиотеку C++
env.Append(LIBS=["stdc++"])
