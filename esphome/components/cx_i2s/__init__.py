import esphome.codegen as cg
import esphome.config_validation as cv

CODEOWNERS = ["@andreibodrov"]

cx_i2s_ns = cg.esphome_ns.namespace("cx_i2s")

CONFIG_SCHEMA = cv.Schema({})


async def to_code(config):
    pass
