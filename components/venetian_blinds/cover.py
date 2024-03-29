import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import cover
from esphome.const import (
    CONF_ID,
    CONF_CLOSE_ACTION,
    CONF_CLOSE_DURATION,
    CONF_OPEN_ACTION,
    CONF_OPEN_DURATION,
    CONF_STOP_ACTION,
    CONF_ASSUMED_STATE,
)

CONF_TILT_DURATION = "tilt_duration"
CONF_ACTUATOR_ACTIVATION_DURATION = "actuator_activation_duration"

venetian_blinds_ns = cg.esphome_ns.namespace('venetian_blinds')
VenetianBlinds = venetian_blinds_ns.class_('VenetianBlinds', cover.Cover, cg.Component)

CONFIG_SCHEMA = cover.COVER_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(VenetianBlinds),
    cv.Required(CONF_OPEN_ACTION): automation.validate_automation(single=True),
    cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
    cv.Required(CONF_CLOSE_ACTION): automation.validate_automation(single=True),
    cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
    cv.Required(CONF_STOP_ACTION): automation.validate_automation(single=True),
    cv.Required(CONF_TILT_DURATION): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_ACTUATOR_ACTIVATION_DURATION, default="0s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_ASSUMED_STATE, default=True): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)
    await automation.build_automation(
        var.get_stop_trigger(), [], config[CONF_STOP_ACTION]
    )
    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    await automation.build_automation(
        var.get_open_trigger(), [], config[CONF_OPEN_ACTION]
    )
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    await automation.build_automation(
        var.get_close_trigger(), [], config[CONF_CLOSE_ACTION]
    )
    cg.add(var.set_tilt_duration(config[CONF_TILT_DURATION]))
    cg.add(var.set_actuator_activation_duration(config[CONF_ACTUATOR_ACTIVATION_DURATION]))
    cg.add(var.set_assumed_state(config[CONF_ASSUMED_STATE]))

def venetian_blinds_to_code(config):  
  # existing code...  
  
  cg.add(cg.App.register_component(var, config))  
  
  if CONF_AUTO_DETECT_ENDSTOPS in config:  
    detect_action = automation.build_automation(var.get_auto_detect_endstops_trigger(), [],  
                                                config[CONF_AUTO_DETECT_ENDSTOPS])  
    cg.add(detect_action)  
