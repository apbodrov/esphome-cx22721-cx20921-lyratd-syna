import esphome.codegen as cg
from esphome.components import audio, cx_audio, microphone
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MIC_GAIN

DEPENDENCIES = ["cx_audio"]
CODEOWNERS = ["@andreibodrov"]

CONF_CX_AUDIO_ID = "cx_audio_id"

cx_i2s_ns = cg.esphome_ns.namespace("cx_i2s")
CXI2SMicrophone = cx_i2s_ns.class_(
    "CXI2SMicrophone", microphone.Microphone, cg.Component
)
CXI2SMicrophone.header = "esphome/components/cx_audio/cx_i2s.h"


def _set_stream_limits(config):
    audio.set_stream_limits(
        min_bits_per_sample=16,
        max_bits_per_sample=16,
        min_channels=1,
        max_channels=1,
        min_sample_rate=16000,
        max_sample_rate=16000,
    )(config)
    return config


CONFIG_SCHEMA = cv.All(
    microphone.MICROPHONE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(CXI2SMicrophone),
            cv.GenerateID(CONF_CX_AUDIO_ID): cv.use_id(cx_audio.CXAudio),
            cv.Optional(CONF_MIC_GAIN, default="24db"): cv.All(
                cv.decibel, cv.float_range(min=-50, max=50)
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _set_stream_limits,
)


async def to_code(config):
    cg.add_global(cx_i2s_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await microphone.register_microphone(var, config)

    parent = await cg.get_variable(config[CONF_CX_AUDIO_ID])
    cg.add(var.set_cx_audio(parent))

    if CONF_MIC_GAIN in config:
        cg.add(var.set_mic_gain(config[CONF_MIC_GAIN]))
