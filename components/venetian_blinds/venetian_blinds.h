#pragma once
#include "esphome/components/cover/cover.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome {
namespace venetian_blinds {

class VenetianBlinds : public Component, public cover::Cover {
public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;
  Trigger<> *get_open_trigger() const { return this->open_trigger; }
  Trigger<> *get_close_trigger() const { return this->close_trigger; }
  Trigger<> *get_stop_trigger() const { return this->stop_trigger; }
  void set_open_duration(uint32_t open) { this->open_duration = open; }
  void set_close_duration(uint32_t close) { this->close_duration = close; }
  void set_tilt_duration(uint32_t tilt) { this->tilt_duration = tilt; }
  void set_actuator_activation_duration(uint32_t actuator_activation) { this->actuator_activation_duration = actuator_activation; }
  void set_assumed_state(bool value) { this->assumed_state = value; }
  // Runtime calibration (v6): apply open/close/tilt durations in ms and recompute
  // the net-travel budgets + rescale the internal counters so the published
  // position/tilt stay continuous. Safe to call any time; takes effect on the
  // next cover operation (a running operation keeps its original time budget).
  void set_all_durations(uint32_t open, uint32_t close, uint32_t tilt) {
    this->open_duration = open;
    this->close_duration = close;
    this->tilt_duration = tilt;
    this->recompute_net_durations_();
  }

protected:
  Trigger<> *open_trigger{new Trigger<>()};
  Trigger<> *close_trigger{new Trigger<>()};
  Trigger<> *stop_trigger{new Trigger<>()};
  uint32_t open_duration;
  uint32_t close_duration;
  uint32_t tilt_duration;
  uint32_t actuator_activation_duration;
  bool assumed_state{false};

private:
  uint32_t start_dir_time_{0};
  uint32_t last_recompute_time_{0};
  uint32_t last_publish_time_{0};
  uint32_t open_net_duration_;
  uint32_t close_net_duration_;
  uint32_t target_position_{0};
  uint32_t target_tilt_{0};
  int exact_position_{0};
  int exact_tilt_{0};

  void recompute_net_durations_() {
    const uint32_t tilt_and_activation_duration =
        this->tilt_duration + this->actuator_activation_duration;
    this->open_net_duration_ = this->open_duration - tilt_and_activation_duration;
    this->close_net_duration_ = this->close_duration - tilt_and_activation_duration;
    // Keep position/tilt continuous across a duration change (only when idle -
    // a running operation keeps its original budget and normalisation).
    if (this->current_operation == COVER_OPERATION_IDLE) {
      this->exact_position_ =
          (int) (this->close_net_duration_ * this->position);
      this->exact_tilt_ = (int) (this->tilt_duration * this->tilt);
    }
  }

  void stop_prev_trigger_();
  bool is_at_target_() const;
  void start_direction_(cover::CoverOperation dir);
  void recompute_position_();

  Trigger<> *prev_command_trigger_{nullptr};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
};

} // namespace venetian_blinds
} // namespace esphome
