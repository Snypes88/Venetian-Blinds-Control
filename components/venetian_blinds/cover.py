import esphome.codegen as cg  
import esphome.config_validation as cv  
from esphome import automation  
from esphome.components import cover, sensor  
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
CONF_POWER_UP = "power_up"  
CONF_POWER_DOWN = "power_down"  
CONF_POWER_THRESHOLD = "power_threshold"  
CONF_IS_AT_OPEN_ENDSTOP = "is_at_open_endstop"  
CONF_IS_AT_CLOSED_ENDSTOP = "is_at_closed_endstop"  

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
    cv.Required(CONF_POWER_UP): cv.use_id(sensor.Sensor),  
    cv.Required(CONF_POWER_DOWN): cv.use_id(sensor.Sensor),  
    cv.Optional(CONF_POWER_THRESHOLD, default=10.0): cv.float_,  
    cv.Required(CONF_IS_AT_OPEN_ENDSTOP): cv.use_id(binary_sensor.BinarySensor),  
    cv.Required(CONF_IS_AT_CLOSED_ENDSTOP): cv.use_id(binary_sensor.BinarySensor),  
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
    power_up = await cg.get_variable(config[CONF_POWER_UP])  
    power_down = await cg.get_variable(config[CONF_POWER_DOWN])  
    cg.add(var.set_power_sensors(power_up, power_down))  
    cg.add(var.set_power_threshold(config[CONF_POWER_THRESHOLD]))  
    is_at_open_endstop = await cg.get_variable(config[CONF_IS_AT_OPEN_ENDSTOP])  
    cg.add(var.set_is_at_open_endstop_sensor(is_at_open_endstop))  
    is_at_closed_endstop = await cg.get_variable(config[CONF_IS_AT_CLOSED_ENDSTOP])  
    cg.add(var.set_is_at_closed_endstop_sensor(is_at_closed_endstop))  

