import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button, cover, number, output, sensor, switch, text_sensor
from esphome.const import (
    CONF_ASSUMED_STATE,
    CONF_CLOSE_DURATION,
    CONF_MODE,
    CONF_OPEN_DURATION,
    CONF_THRESHOLD,
    DEVICE_CLASS_DURATION,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_SECOND,
)

AUTO_LOAD = ["button", "number", "sensor", "switch", "text_sensor"]
DEPENDENCIES = ["output"]

CONF_TILT_DURATION = "tilt_duration"
CONF_ACTUATOR_ACTIVATION_DURATION = "actuator_activation_duration"
CONF_UP_OUTPUT = "up_output"
CONF_DOWN_OUTPUT = "down_output"
CONF_POWER_UP = "power_up"
CONF_POWER_DOWN = "power_down"
CONF_CUT_THRESHOLD = "cut_threshold"
CONF_OPEN_NUMBER = "open_number"
CONF_CLOSE_NUMBER = "close_number"
CONF_TILT_NUMBER = "tilt_number"
CONF_LAST_RUN_TIME = "last_run_time"
CONF_LAST_CUT_TIME = "last_cut_time"
CONF_HOME_ERROR = "home_error"
CONF_POWER_METER = "power_meter"
CONF_STATUS = "status"
CONF_SERVICE_MODE = "service_mode"

venetian_blinds_ns = cg.esphome_ns.namespace("venetian_blinds")
VenetianBlinds = venetian_blinds_ns.class_("VenetianBlinds", cover.Cover, cg.Component)
DurationNumber = venetian_blinds_ns.class_("DurationNumber", number.Number)
DurationKind = venetian_blinds_ns.enum("DurationKind", is_class=True)
ServiceButton = venetian_blinds_ns.class_("ServiceButton", button.Button)
ServiceModeSwitch = venetian_blinds_ns.class_("ServiceModeSwitch", switch.Switch)
ServiceAction = venetian_blinds_ns.enum("ServiceAction", is_class=True)

SERVICE_BUTTONS = {
    "calibrate_button": (ServiceAction.CALIBRATE, "mdi:timer-sync-outline"),
    "set_upper_button": (ServiceAction.SET_UPPER, "mdi:arrow-collapse-up"),
    "set_lower_button": (ServiceAction.SET_LOWER, "mdi:arrow-collapse-down"),
    "limit_run_button": (ServiceAction.RUN, "mdi:play"),
    "limit_nudge_button": (ServiceAction.NUDGE, "mdi:debug-step-over"),
    "limit_store_button": (ServiceAction.STORE, "mdi:content-save"),
    "limit_cancel_button": (ServiceAction.CANCEL, "mdi:cancel"),
}

_DURATION_NUMBER_SCHEMA = number.number_schema(
    DurationNumber,
    entity_category=ENTITY_CATEGORY_CONFIG,
    device_class=DEVICE_CLASS_DURATION,
    unit_of_measurement=UNIT_SECOND,
    icon="mdi:timer-outline",
).extend({cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True)})


def _diagnostic_seconds():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_SECOND,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_DURATION,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )


def _validate_timing(config):
    overhead = config[CONF_TILT_DURATION].total_milliseconds + (
        config[CONF_ACTUATOR_ACTIVATION_DURATION].total_milliseconds
    )
    for key in (CONF_OPEN_DURATION, CONF_CLOSE_DURATION):
        if config[key].total_milliseconds < overhead + 5000:
            raise cv.Invalid(f"{key} must allow at least 5 s of travel after tilt and activation")
    return config


CONFIG_SCHEMA = cv.All(
    cover.cover_schema(VenetianBlinds)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(VenetianBlinds),
            cv.Required(CONF_UP_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_DOWN_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_TILT_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ACTUATOR_ACTIVATION_DURATION, default="200ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ASSUMED_STATE, default=True): cv.boolean,
            cv.Optional(CONF_POWER_UP): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_POWER_DOWN): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_POWER_METER): cv.use_id(cg.PollingComponent),
            cv.Optional(CONF_CUT_THRESHOLD, default=30.0): cv.positive_float,
            cv.Optional(CONF_OPEN_NUMBER): _DURATION_NUMBER_SCHEMA,
            cv.Optional(CONF_CLOSE_NUMBER): _DURATION_NUMBER_SCHEMA,
            cv.Optional(CONF_TILT_NUMBER): _DURATION_NUMBER_SCHEMA,
            cv.Optional(CONF_LAST_RUN_TIME): _diagnostic_seconds(),
            cv.Optional(CONF_LAST_CUT_TIME): _diagnostic_seconds(),
            cv.Optional(CONF_HOME_ERROR): sensor.sensor_schema(
                unit_of_measurement=UNIT_SECOND,
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:information-outline",
            ),
            cv.Optional(CONF_SERVICE_MODE): switch.switch_schema(
                ServiceModeSwitch,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:wrench-clock",
                default_restore_mode="ALWAYS_OFF",
            ),
            **{
                cv.Optional(key): button.button_schema(
                    ServiceButton, entity_category=ENTITY_CATEGORY_CONFIG, icon=icon
                )
                for key, (_, icon) in SERVICE_BUTTONS.items()
            },
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _validate_timing,
)


async def _add_number(var, config, key, setter, kind, min_value, max_value, step):
    if conf := config.get(key):
        num = await number.new_number(conf, min_value=min_value, max_value=max_value, step=step)
        cg.add(num.set_parent(var))
        cg.add(num.set_kind(kind))
        cg.add(setter(num))


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    cg.add(var.set_up_output(await cg.get_variable(config[CONF_UP_OUTPUT])))
    cg.add(var.set_down_output(await cg.get_variable(config[CONF_DOWN_OUTPUT])))
    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_tilt_duration(config[CONF_TILT_DURATION]))
    cg.add(var.set_actuator_activation_duration(config[CONF_ACTUATOR_ACTIVATION_DURATION]))
    cg.add(var.set_assumed_state(config[CONF_ASSUMED_STATE]))
    cg.add(var.set_cut_threshold(config[CONF_CUT_THRESHOLD]))

    if CONF_POWER_METER in config:
        cg.add(var.set_power_meter(await cg.get_variable(config[CONF_POWER_METER])))
    if CONF_POWER_UP in config:
        cg.add(var.set_power_up_sensor(await cg.get_variable(config[CONF_POWER_UP])))
    if CONF_POWER_DOWN in config:
        cg.add(var.set_power_down_sensor(await cg.get_variable(config[CONF_POWER_DOWN])))

    await _add_number(var, config, CONF_OPEN_NUMBER, var.set_open_number, DurationKind.OPEN, 6, 180, 0.1)
    await _add_number(var, config, CONF_CLOSE_NUMBER, var.set_close_number, DurationKind.CLOSE, 6, 180, 0.1)
    await _add_number(var, config, CONF_TILT_NUMBER, var.set_tilt_number, DurationKind.TILT, 0.2, 20, 0.01)

    if conf := config.get(CONF_LAST_RUN_TIME):
        cg.add(var.set_last_run_sensor(await sensor.new_sensor(conf)))
    if conf := config.get(CONF_LAST_CUT_TIME):
        cg.add(var.set_last_cut_sensor(await sensor.new_sensor(conf)))
    if conf := config.get(CONF_HOME_ERROR):
        cg.add(var.set_home_error_sensor(await sensor.new_sensor(conf)))
    if conf := config.get(CONF_STATUS):
        cg.add(var.set_status_sensor(await text_sensor.new_text_sensor(conf)))
    if conf := config.get(CONF_SERVICE_MODE):
        sw = await switch.new_switch(conf)
        cg.add(sw.set_parent(var))
        cg.add(var.set_service_mode_switch(sw))
    for key, (action, _) in SERVICE_BUTTONS.items():
        if conf := config.get(key):
            btn = await button.new_button(conf)
            cg.add(btn.set_parent(var))
            cg.add(btn.set_action(action))
