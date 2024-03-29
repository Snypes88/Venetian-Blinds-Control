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
CONF_OPEN_DURATION_SENSOR = 'open_duration_sensor'  
CONF_CLOSE_DURATION_SENSOR = 'close_duration_sensor'  
CONF_POWER_UP_SENSOR = 'power_up_sensor'  
CONF_POWER_DOWN_SENSOR = 'power_down_sensor'  
  
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
    cv.Optional(CONF_OPEN_DURATION_SENSOR): cv.use_id(sensor.Sensor),  
    cv.Optional(CONF_CLOSE_DURATION_SENSOR): cv.use_id(sensor.Sensor),  
    cv.Optional(CONF_POWER_UP_SENSOR): cv.use_id(sensor.Sensor),  
    cv.Optional(CONF_POWER_DOWN_SENSOR): cv.use_id(sensor.Sensor),  
}).extend(cv.COMPONENT_SCHEMA)  
  
async def to_code(config):  
    var = cg.new_Pvariable(config[CONF_ID])  
    await cg.register_component(var, config)  
    await cover.register_cover(var, config)  
      
    # existing code...  
  
    if CONF_OPEN_DURATION_SENSOR in config:  
        sensor_ = await cg.get_variable(config[CONF_OPEN_DURATION_SENSOR])  
        cg.add(var.set_open_duration_sensor(sensor_))  
    if CONF_CLOSE_DURATION_SENSOR in config:  
        sensor_ = await cg.get_variable(config[CONF_CLOSE_DURATION_SENSOR])  
        cg.add(var.set_close_duration_sensor(sensor_))  
    if CONF_POWER_UP_SENSOR in config:  
        sensor_ = await cg.get_variable(config[CONF_POWER_UP_SENSOR])  
        cg.add(var.set_power_up_sensor(sensor_))  
    if CONF_POWER_DOWN_SENSOR in config:  
        sensor_ = await cg.get_variable(config[CONF_POWER_DOWN_SENSOR])  
        cg.add(var.set_power_down_sensor(sensor_))  
