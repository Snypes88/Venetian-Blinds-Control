# Venetian-Blinds-Control

An ESPHome cover for venetian blinds, controlling the action (open/close/stop), the position (0-100) and the tilt (0-100).

This fork of [bruxy70/Venetian-Blinds-Control](https://github.com/bruxy70/Venetian-Blinds-Control) adds what a time-based cover can't do by itself:

- **Homing from motor power.** A full open or close runs until the motor cuts itself off at its limit, then re-anchors the position there, so errors don't build up.
- **Runtime times.** Open, close and tilt times are number entities saved on the device, and `Calibrate Travel` measures open and close for you.
- **End stops from a dashboard.** The motor's upper and lower limits can be set from Home Assistant, with no installer tool.
- **Direct relay control.** The cover drives the two relays itself, so it can enforce the interlock and the reversal gap.

For the position, 0 is closed and 100 is open, as with other covers in Home Assistant.

**Hardware.** Tested on Sonoff Dual R3 boards with a BL0939 power meter and Somfy J4 WT motors, on ESPHome 2026.10. The limit procedure follows Somfy guide 5133945A and is specific to those motors. Other motors need their own check before you use it.

## Installation

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Snypes88/Venetian-Blinds-Control
      ref: v2.0.0
    components: [venetian_blinds]
```

## Configuration

`up_output` and `down_output` replace the `open_action`, `close_action` and `stop_action` of v1, so **v1 configs need changing**. Saved times from v1 are not carried over, and the configured times are used until new ones are saved.

```yaml
uart:
  tx_pin: GPIO25
  rx_pin: GPIO26
  baud_rate: 4800
  parity: NONE
  stop_bits: 2

sensor:
  - platform: bl0939
    id: power_meter
    update_interval: 120s  # idle rate; the cover polls faster by itself while moving
    active_power_1:
      name: "Active Power Up"
      id: power_up
    active_power_2:
      name: "Active Power Down"
      id: power_down

# No HA entity on the relays: only the cover can close them.
output:
  - platform: gpio
    pin: 27
    id: relay_up
  - platform: gpio
    pin: 14
    id: relay_down

cover:
  - platform: venetian_blinds
    name: "Venetian"
    up_output: relay_up
    down_output: relay_down
    open_duration: 40000ms
    close_duration: 38000ms
    tilt_duration: 1400ms
    power_up: power_up
    power_down: power_down
    power_meter: power_meter
    open_number:
      name: "Open Duration"
    close_number:
      name: "Close Duration"
    tilt_number:
      name: "Tilt Duration"
    last_run_time:
      name: "Last Run Time"
    last_cut_time:
      name: "Last Cut Time"
    home_error:
      name: "Home Error"
    status:
      name: "Status"
    calibrate_button:
      name: "Calibrate Travel"
    service_mode:
      name: "Service Mode"
    set_upper_button:
      name: "Set Upper Limit"
    set_lower_button:
      name: "Set Lower Limit"
    limit_run_button:
      name: "Limit Run"
    limit_nudge_button:
      name: "Limit Nudge"
    limit_store_button:
      name: "Limit Store"
    limit_cancel_button:
      name: "Limit Cancel"
```

| Option | Default | Purpose |
|---|---|---|
| `open_duration`, `close_duration`, `tilt_duration` | required | Starting times, used only when the device has none saved. |
| `actuator_activation_duration` | `200ms` | Time for the motor to start. Somfy asks for commands of at least 200 ms. |
| `cut_threshold` | `30` | Watts below which the motor counts as having cut off. |
| `power_up`, `power_down`, `power_meter` | optional | The active power sensors and the meter. Homing, calibration and limit setting were only tested with these fitted. |

All entities are optional, so leave out any you don't want.

## Use

**Calibrating.** Press `Calibrate Travel`. The blind runs to both limits, then saves the measured open and close times. Set `Tilt Duration` by eye: the rail still moving at the end of a tilt means it's too long, and slats that don't fully turn mean it's too short.

**Faults.** `Status` reports problems. If the motor stops away from a limit, the position is marked untrusted, and the next full open or close puts it right. `Home Error` shows how far the model was from the real limit on the last full run.

**Setting a motor limit**

1. Turn on `Service Mode`. The limit buttons do nothing without it.
2. Press `Set Upper Limit` or `Set Lower Limit`. The blind parks away from the limit, then holds both relays for 8 s. Wait for `Status` to say `In setting mode`.
3. Press `Limit Run`, then Stop on the cover when it's near the target. Use `Limit Nudge` for the last part.
4. Press `Limit Store` once. Service Mode turns itself off.
5. Press `Calibrate Travel`. Only the full-run times show that a limit really moved. Power during a store press proves nothing.

Keep at least 15 cm between the two limits, as the Somfy guide asks.

**Limit Cancel can leave the motor in setting mode.** The firmware can't tell, and normal moves could then store a wrong limit. If you cancel, run the procedure again and finish it with `Limit Store`.

**Both relays together are a motor command.** Holding them for 5 s enters setting mode and 15 s is a factory reset, so don't wire anything else to them. The firmware stops at 8 s and doesn't offer a factory reset.

## Wall switches

Wall switches work through the normal cover actions:

```yaml
binary_sensor:
  - id: key1
    platform: gpio
    pin:
      number: ${key1_gpio}
      mode: INPUT_PULLUP
      inverted: True
    on_press:
      then:
        - cover.open: cover1
    on_release:
      then:
        - cover.stop: cover1
```

Use the same pattern with `cover.close` for the down key, and give the cover `id: cover1`. This example comes from the v1 README and hasn't been tested with v2.

## Development

The motion logic is in `components/venetian_blinds/blind_core.h`, with no ESPHome dependencies, so it can be tested on a PC:

```bash
cd tests
c++ -std=c++17 -O1 -w host_test.cpp -o host_test
./host_test
```

## Legacy - Arduino code (not maintained)

Control Shutters or Venetian blinds via buttons and MQTT. Use 2 relays (up and down) and 2 binary inputs to independently control position and tilt. The blinds tilt down when going down and tilt up going up. Using timers to calculate the position/tilt independently without the need for end stops. For shutters recognize "vent position", when shutters are closed but leave open slits. Designed to work with Sonoff 4ch or Sonoff 4ch Pro to control two units, or Sonoff Dual for a single unit. (Will work with any ESP 8266 board).

<img src="https://github.com/bruxy70/Venetian-Blinds-Control/blob/master/images/Blinds.gif">

I have been using this for a long time and it works reliably.

## Getting Started

## Built With

- [Arduino](https://www.arduino.cc/en/Main/Software)

### Prerequisites

- Download source from the Sonoff_rolety directory
- Add http://arduino.esp8266.com/stable/package_esp8266com_index.json to Preferences/Additional board manager
- Add ESP8266 board
- Add libraries

### Building

- Check instructions in sonoff.ino to configure project
- Set paramaters in config.h
- Build

### Using

- Developed for shuters controled by 2 switches - one moves shutter up, the second for down. They switch on an L-wire.
- The project involves installing an ESP 8266 board with two relays, and connecting each L-wire to one relay
- It supports either 1 shutter (2 relays) or 2 shutters (4 relays)
- To use with Sonoff 4ch (or Sonoff 4ch pro):
  - You need to have an N wire - connect it to one of the Sonoff N sockets
  - Connect the common wire to the Sonoff In L socket (and to the C sockets if using Pro version)
  - Connect the wire for shutter 1 up direction to Out 1 L socket, shutter 1 down to Out 2, shutter 2 up to Out 3 and shutter 2 down to Out 4
  - If you want to use the original wall switch, solder wires to the 4 buttons on Sonoff 4ch, and connect 1st switch between button 1 and GND and so on
  - The program will make sure that it will not connect relay 1&2 and relay 3&4 at the same time. If relay 1 is on and you want to activate relay 2, it will automatically disconnect relay 1 (connecting both wires for up and down simultaneously is often use to enter the shutter programming mode and coudl mess-up setting of the end positions).

## Author

- **Václav Chaloupka** - _Initial work_ - [bruxy70](https://github.com/bruxy70)
- **Snypes88** - _v2: power homing, runtime times and limit setting_

MIT licence, see `LICENSE`. The v1 videos on the original author's YouTube channel explain the time-based cover this grew from: https://youtu.be/tg3nOoBFJLU and https://youtu.be/dk5yDBRHhbQ
