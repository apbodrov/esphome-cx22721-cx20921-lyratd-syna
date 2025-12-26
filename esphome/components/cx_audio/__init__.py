import os

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@andreibodrov"]
DEPENDENCIES = ["esp32", "microphone", "speaker"]

cx_audio_ns = cg.esphome_ns.namespace("cx_audio")
CXAudio = cx_audio_ns.class_("CXAudio", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CXAudio),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    this_dir = os.path.dirname(os.path.abspath(__file__))
    include_dir = os.path.join(this_dir, "include")

    # Инклюды безопасны для всех стадий
    cg.add_build_flag(f"-I{include_dir}")
    cg.add_build_flag("-DCONFIG_SYNA_V1_2_BOARD")
    cg.add_build_flag("-DVOICE_ASSISTANT_AVS")

    # Используем скрипт для изоляции линковки монолита от загрузчика
    cg.add_platformio_option("extra_scripts", [os.path.join(this_dir, "linker_fix.py")])
