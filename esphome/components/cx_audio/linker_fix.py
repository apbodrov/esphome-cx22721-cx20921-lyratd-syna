Import("env")
import os

# В PlatformIO PROJECT_DIR указывает на директорию проекта (где yaml файл)
# Но компонент находится в esphome_src/esphome/components/cx_audio/
# Нужно найти esphome_src относительно PROJECT_DIR

# Поиск библиотеки libcnx-ipc.a
import os

project_dir = env.subst("$PROJECT_DIR")
# Если мы внутри .esphome/build/..., поднимаемся выше
if ".esphome" in project_dir:
    project_root = project_dir.split(".esphome")[0]
else:
    project_root = project_dir

# Варианты путей к библиотеке
search_paths = [
    os.path.join(project_root, "esphome_src", "esphome", "components", "cx_audio", "lib", "libcnx-ipc.a"),
    os.path.join(project_root, "esphome", "components", "cx_audio", "lib", "libcnx-ipc.a"),
    os.path.join(os.getcwd(), "esphome", "components", "cx_audio", "lib", "libcnx-ipc.a"),
    os.path.join(os.getcwd(), "lib", "libcnx-ipc.a"),
]

libcnx_ipc = None
for path in search_paths:
    if os.path.exists(path):
        libcnx_ipc = path
        break

if libcnx_ipc is None:
    print(f"ERROR: Cannot find libcnx-ipc.a. Checked paths:")
    for path in search_paths:
        print(f"  - {path}")
    print(f"PROJECT_DIR: {project_dir}")
    print(f"Current Dir: {os.getcwd()}")
    raise FileNotFoundError("libcnx-ipc.a not found")

component_dir = os.path.dirname(os.path.dirname(libcnx_ipc))

print(f"CX_AUDIO [LINKER FIX]: Using libcnx-ipc.a instead of monolith")
print(f"CX_AUDIO [LINKER FIX]: Lib path: {libcnx_ipc}")
print(f"CX_AUDIO [LINKER FIX]: Component dir: {component_dir}")

# Явно добавляем файлы для компиляции через PlatformIO
# ESPHome автоматически компилирует .c и .cpp файлы из корня компонента
# Но мы можем добавить их в SRC_FILTER если нужно. По умолчанию они подхватываются.

# Добавляем флаги -u для функций из библиотеки, чтобы они линковались
env.Append(
    LINKFLAGS=[
        "-Wl,-u,cnx20921_init",
        "-Wl,-u,cx20921SetMicGain",
        "-Wl,-u,cnx20921_start_speech",
        "-Wl,-u,cnx20921_stop_capture",
        "-Wl,-u,cnx20921_stream_audio",
            "-Wl,--wrap=i2c_master_cmd_begin",
            "-Wl,--whole-archive",
            libcnx_ipc,
            "-Wl,--no-whole-archive",    ]
)

# Включаем стандартную библиотеку C++
env.Append(LIBS=["stdc++"])
