Import("env")
import os

# В PlatformIO PROJECT_DIR указывает на директорию проекта (где yaml файл)
# Но компонент находится в esphome_src/esphome/components/cx_audio/
# Нужно найти esphome_src относительно PROJECT_DIR

# Сначала пробуем найти относительно скрипта (если __file__ определен)
monolith = None
try:
    script_path = os.path.dirname(os.path.abspath(__file__))
    # script_path = .../esphome_src/esphome/components/cx_audio/
    monolith = os.path.join(script_path, "lib", "libva_sdk_monolith.a")
    if not os.path.exists(monolith):
        monolith = None
except:
    pass

# Если не нашли, пробуем через PROJECT_DIR
if monolith is None or not os.path.exists(monolith):
    # PROJECT_DIR может быть esphome_src или esphome_src/.esphome/build/...
    project_dir = env["PROJECT_DIR"]
    
    # Пробуем разные варианты путей
    paths_to_try = [
        os.path.join(project_dir, "esphome", "components", "cx_audio", "lib", "libva_sdk_monolith.a"),
        os.path.join(os.path.dirname(project_dir), "esphome", "components", "cx_audio", "lib", "libva_sdk_monolith.a"),
    ]
    
    # Если PROJECT_DIR содержит .esphome, поднимаемся выше
    if ".esphome" in project_dir:
        base_dir = project_dir.split(".esphome")[0]
        paths_to_try.append(os.path.join(base_dir, "esphome", "components", "cx_audio", "lib", "libva_sdk_monolith.a"))
    
    for path in paths_to_try:
        if os.path.exists(path):
            monolith = path
            break

# Если все еще не нашли - ошибка
if monolith is None or not os.path.exists(monolith):
    print(f"ERROR: Cannot find libva_sdk_monolith.a")
    print(f"  PROJECT_DIR: {env['PROJECT_DIR']}")
    if 'script_path' in locals():
        print(f"  Script path: {script_path}")
    raise FileNotFoundError(f"libva_sdk_monolith.a not found")

print(f"CX_AUDIO [LINKER FIX]: Applying isolation. Monolith: {monolith}")
print(f"CX_AUDIO [LINKER FIX]: libcnx-ipc.a already included in monolith")

# Добавляем флаги ТОЛЬКО для приложения
# env.Append(LINKFLAGS=...) в PlatformIO ESP-IDF сборке обычно
# нацелен на финальный firmware.elf
env.Append(
    LINKFLAGS=[
        "-Wl,-u,va_board_init",
        "-Wl,-u,media_hal_get_handle",
        "-Wl,-u,va_dsp_init",
        "-Wl,-u,cnx20921_init",
        "-Wl,-u,cnx20921_stream_audio",
        "-Wl,-u,va_dsp_hal_init",
        "-Wl,--whole-archive",
        monolith,
        "-Wl,--no-whole-archive",
    ]
)

# Включаем стандартную библиотеку C++
env.Append(LIBS=["stdc++"])
