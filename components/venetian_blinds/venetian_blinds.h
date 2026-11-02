#pragma once
#include "esphome/components/cover/cover.h"
#include "esphome/components/number/number.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/sensor/sensor.h"
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
  void set_cut_threshold(float watts) { this->ctrl_.detector.threshold_w = watts; }
  void set_open_number(DurationNumber *number) { this->numbers_[0] = number; }
  void set_close_number(DurationNumber *number) { this->numbers_[1] = number; }
  void set_tilt_number(DurationNumber *number) { this->numbers_[2] = number; }
  void set_last_run_sensor(sensor::Sensor *sensor) { this->last_run_sensor_ = sensor; }
  void set_last_cut_sensor(sensor::Sensor *sensor) { this->last_cut_sensor_ = sensor; }
  void set_cut_position_sensor(sensor::Sensor *sensor) { this->cut_position_sensor_ = sensor; }

  // Applies a new duration if the resulting timing is valid, else keeps the old one.
  void set_duration(DurationKind kind, uint32_t ms);

 protected:
  void load_timing_();
  void publish_number_(DurationKind kind);
  void apply_outputs_();
  void sync_state_();
  void on_power_(Direction channel, float watts);

  MotionController ctrl_;
  Timing seed_;
  bool assumed_state_{false};
  bool was_idle_{true};
  uint32_t last_publish_{0};

  output::BinaryOutput *up_output_{nullptr};
  output::BinaryOutput *down_output_{nullptr};
  sensor::Sensor *power_up_{nullptr};
  sensor::Sensor *power_down_{nullptr};
  sensor::Sensor *last_run_sensor_{nullptr};
  sensor::Sensor *last_cut_sensor_{nullptr};
  sensor::Sensor *cut_position_sensor_{nullptr};
  DurationNumber *numbers_[3]{nullptr, nullptr, nullptr};
  ESPPreferenceObject prefs_[3];
};

}  // namespace venetian_blinds
}  // namespace esphome
