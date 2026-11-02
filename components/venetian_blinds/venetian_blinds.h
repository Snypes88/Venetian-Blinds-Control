#pragma once
#include "esphome/components/button/button.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/number/number.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "blind_core.h"

namespace esphome {
namespace venetian_blinds {

class VenetianBlinds;

enum class DurationKind : uint8_t { OPEN = 0, CLOSE = 1, TILT = 2 };

class DurationNumber : public number::Number {
 public:
  void set_parent(VenetianBlinds *parent) { this->parent_ = parent; }
  void set_kind(DurationKind kind) { this->kind_ = kind; }

 protected:
  void control(float value) override;

  VenetianBlinds *parent_{nullptr};
  DurationKind kind_{DurationKind::OPEN};
};

enum class ServiceAction : uint8_t { CALIBRATE, SET_UPPER, SET_LOWER, RUN, NUDGE, STORE, CANCEL };

class ServiceButton : public button::Button {
 public:
  void set_parent(VenetianBlinds *parent) { this->parent_ = parent; }
  void set_action(ServiceAction action) { this->action_ = action; }

 protected:
  void press_action() override;

  VenetianBlinds *parent_{nullptr};
  ServiceAction action_{ServiceAction::CANCEL};
};

class ServiceModeSwitch : public switch_::Switch {
 public:
  void set_parent(VenetianBlinds *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;

  VenetianBlinds *parent_{nullptr};
};

class VenetianBlinds : public Component, public cover::Cover {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;

  void set_up_output(output::BinaryOutput *output) { this->up_output_ = output; }
  void set_down_output(output::BinaryOutput *output) { this->down_output_ = output; }
  void set_open_duration(uint32_t ms) { this->seed_.open_ms = ms; }
  void set_close_duration(uint32_t ms) { this->seed_.close_ms = ms; }
  void set_tilt_duration(uint32_t ms) { this->seed_.tilt_ms = ms; }
  void set_actuator_activation_duration(uint32_t ms) { this->seed_.activation_ms = ms; }
  void set_assumed_state(bool value) { this->assumed_state_ = value; }
  void set_power_up_sensor(sensor::Sensor *sensor) { this->power_up_ = sensor; }
  void set_power_down_sensor(sensor::Sensor *sensor) { this->power_down_ = sensor; }
  void set_power_meter(PollingComponent *meter) { this->meter_ = meter; }
  void set_cut_threshold(float watts) { this->ctrl_.detector.threshold_w = watts; }
  void set_open_number(DurationNumber *number) { this->numbers_[0] = number; }
  void set_close_number(DurationNumber *number) { this->numbers_[1] = number; }
  void set_tilt_number(DurationNumber *number) { this->numbers_[2] = number; }
  void set_last_run_sensor(sensor::Sensor *sensor) { this->last_run_sensor_ = sensor; }
  void set_last_cut_sensor(sensor::Sensor *sensor) { this->last_cut_sensor_ = sensor; }
  void set_home_error_sensor(sensor::Sensor *sensor) { this->home_error_sensor_ = sensor; }
  void set_status_sensor(text_sensor::TextSensor *sensor) { this->status_sensor_ = sensor; }
  void set_service_mode_switch(ServiceModeSwitch *service_switch) { this->service_switch_ = service_switch; }

  // Applies a new duration if the resulting timing is valid, else keeps the old one.
  void set_duration(DurationKind kind, uint32_t ms);
  void service_action(ServiceAction action);
  void set_service_mode(bool on);

 protected:
  void load_timing_();
  void publish_number_(DurationKind kind);
  void apply_outputs_();
  void sync_state_();
  void on_power_(Direction channel, float watts);
  void on_run_end_(uint32_t now, RunEnd end);
  void refresh_();
  void update_meter_interval_();
  void pump_messages_();
  void set_status_(const char *status);
  void save_measured_timing_();

  MotionController ctrl_;
  TravelCalibration calibration_;
  LimitSequence limit_;
  Timing seed_;
  bool assumed_state_{false};
  bool was_idle_{true};
  bool service_mode_{false};
  uint32_t last_publish_{0};
  uint32_t service_from_{0};
  uint32_t meter_idle_ms_{0};
  uint32_t meter_interval_ms_{0};
  uint32_t meter_fast_until_{0};
  const char *status_{nullptr};

  output::BinaryOutput *up_output_{nullptr};
  output::BinaryOutput *down_output_{nullptr};
  sensor::Sensor *power_up_{nullptr};
  sensor::Sensor *power_down_{nullptr};
  sensor::Sensor *last_run_sensor_{nullptr};
  sensor::Sensor *last_cut_sensor_{nullptr};
  sensor::Sensor *home_error_sensor_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};
  ServiceModeSwitch *service_switch_{nullptr};
  PollingComponent *meter_{nullptr};
  DurationNumber *numbers_[3]{nullptr, nullptr, nullptr};
  ESPPreferenceObject prefs_[3];
};

}  // namespace venetian_blinds
}  // namespace esphome
