#pragma once  
#include "esphome/components/cover/cover.h"  
#include "esphome/core/automation.h"  
#include "esphome/core/component.h"  
#include "esphome/components/sensor/sensor.h"  
#include "esphome/components/binary_sensor/binary_sensor.h" // add this  
  
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
  void set_power_sensors(sensor::Sensor *power_up_sensor, sensor::Sensor *power_down_sensor) {  
    this->power_up_sensor_ = power_up_sensor;  
    this->power_down_sensor_ = power_down_sensor;  
  }  
  void set_power_threshold(float power_threshold) {  
    this->power_threshold_ = power_threshold;  
  }  
  void set_is_at_open_endstop_sensor(binary_sensor::BinarySensor *sensor) {  
    this->is_at_open_endstop_sensor_ = sensor;  
  }  
  void set_is_at_closed_endstop_sensor(binary_sensor::BinarySensor *sensor) {  
    this->is_at_closed_endstop_sensor_ = sensor;  
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
  sensor::Sensor *power_up_sensor_;  
  sensor::Sensor *power_down_sensor_;  
  float power_threshold_;  
  binary_sensor::BinarySensor *is_at_open_endstop_sensor_; // add this  
  binary_sensor::BinarySensor *is_at_closed_endstop_sensor_; // add this  
  
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
  
  void stop_prev_trigger_();  
  bool is_at_target_() const;  
  void start_direction_(cover::CoverOperation dir);  
  void recompute_position_();  
  
  Trigger<> *prev_command_trigger_{nullptr};  
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};  
};  
  
} // namespace venetian_blinds  
} // namespace esphome  
