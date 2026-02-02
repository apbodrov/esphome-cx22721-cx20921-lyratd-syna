Import("env")
import os

project_dir = env.subst("$PROJECT_DIR")
if ".esphome" in project_dir:
    project_root = project_dir.split(".esphome")[0]
else:
    project_root = project_dir

# Пути к оригинальным библиотекам SDK
lib_dir = os.path.join(project_root, "esphome", "components", "cx_audio", "lib")
if not os.path.exists(lib_dir):
    lib_dir = os.path.join(
        project_root, "esphome_src", "esphome", "components", "cx_audio", "lib"
    )

libva_dsp_path = os.path.join(lib_dir, "libva_dsp.a")
libcnx_path = os.path.join(lib_dir, "libcnx-ipc.a")

for p in [libva_dsp_path, libcnx_path]:
    if not os.path.exists(p):
        print(f"ERROR: {p} not found!")
        raise FileNotFoundError(p)

print("CX_AUDIO [LINKER]: Using Original SDK Libs.")
print(f"  VA_DSP: {libva_dsp_path}")
print(f"  CNX_IPC: {libcnx_path}")

env.Append(
    LINKFLAGS=[
        "-Wl,-u,va_board_init",
        "-Wl,-u,va_dsp_init",
        "-Wl,-u,cnx20921_init",
        "-Wl,-u,cnx20921_stream_audio",
        "-Wl,-u,cx20921SetMicGain",
                "-Wl,-u,va_boot_dsp_signal", # Наш Glue символ
                "-Wl,-u,esp_vfs_spiffs_register",
                "-Wl,--wrap=i2c_master_cmd_begin",
                "-Wl,--wrap=cnx20921_init",
                # Подключаем обе библиотеки целиком
        
        "-Wl,--whole-archive",
        libcnx_path,
        libva_dsp_path,
        "-Wl,--no-whole-archive",
    ]
)

# Включаем стандартную библиотеку C++ для SDK
env.Append(LIBS=["stdc++"])
