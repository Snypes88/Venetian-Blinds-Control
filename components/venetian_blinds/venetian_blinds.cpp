#include "venetian_blinds.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace venetian_blinds {

static const char *const TAG = "venetian_blinds.cover";

// Service Mode turns itself off this long after the last service button.
static const uint32_t SERVICE_MODE_MS = 300000;
static const uint32_t METER_MOVING_MS = 500;
static const uint32_t METER_ENTRY_MS = 250;
// Longer than the sensors' throttle, so HA gets a reading from after the motor has stopped.
static const uint32_t METER_LINGER_MS = 6000;

using namespace esphome::cover;

void DurationNumber::control(float value) {
  this->parent_->set_duration(this->kind_, (uint32_t) lroundf(value * 1000.0f));
}

void ServiceButton::press_action() { this->parent_->service_action(this->action_); }

void ServiceModeSwitch::write_state(bool state) {
  this->parent_->set_service_mode(state);
  this->publish_state(state);
}

void VenetianBlinds::dump_config() {
  LOG_COVER("", "Venetian Blinds", this);
  const Timing &t = this->ctrl_.timing;
  ESP_LOGCONFIG(TAG, "  Open Duration: %.2fs", t.open_ms / 1e3f);
  ESP_LOGCONFIG(TAG, "  Close Duration: %.2fs", t.close_ms / 1e3f);
  ESP_LOGCONFIG(TAG, "  Tilt Duration: %.2fs", t.tilt_ms / 1e3f);
  ESP_LOGCONFIG(TAG, "  Actuator Activation Duration: %.2fs", t.activation_ms / 1e3f);
  ESP_LOGCONFIG(TAG, "  Cut Threshold: %.0fW", this->ctrl_.detector.threshold_w);
}

void VenetianBlinds::load_timing_() {
  static const char *const KEYS[3] = {"venetian_blinds_open_ms", "venetian_blinds_close_ms",
                                      "venetian_blinds_tilt_ms"};
  Timing timing = this->seed_;
  uint32_t *fields[3] = {&timing.open_ms, &timing.close_ms, &timing.tilt_ms};
  for (int i = 0; i < 3; i++) {
    this->prefs_[i] = global_preferences->make_preference<uint32_t>(fnv1_hash(KEYS[i]));
    uint32_t saved;
    if (this->prefs_[i].load(&saved))
      *fields[i] = saved;
  }
  if (!timing.valid()) {
    ESP_LOGW(TAG, "Saved durations are invalid, using the configured seeds");
    timing = this->seed_;
  }
  this->ctrl_.timing = timing;
}

void VenetianBlinds::setup() {
  this->load_timing_();

  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = 0.0f;
    this->tilt = 0.0f;
  }
  this->ctrl_.model.position = clamp(this->position, 0.0f, 1.0f);
  this->ctrl_.model.tilt = clamp(this->tilt, 0.0f, 1.0f);

  if (this->power_up_ != nullptr) {
    this->power_up_->add_on_raw_state_callback([this](float watts) { this->on_power_(Direction::UP, watts); });
  }
  if (this->power_down_ != nullptr) {
    this->power_down_->add_on_raw_state_callback([this](float watts) { this->on_power_(Direction::DOWN, watts); });
  }

  this->publish_number_(DurationKind::OPEN);
  this->publish_number_(DurationKind::CLOSE);
  this->publish_number_(DurationKind::TILT);

  if (this->meter_ != nullptr) {
    this->meter_idle_ms_ = this->meter_->get_update_interval();
    this->meter_interval_ms_ = this->meter_idle_ms_;
  }
  this->set_status_("OK");
}

void VenetianBlinds::set_status_(const char *status) {
  if (status == this->status_)
    return;
  this->status_ = status;
  ESP_LOGI(TAG, "Status: %s", status);
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(status);
}

void VenetianBlinds::pump_messages_() {
  if (const char *message = this->calibration_.take_message())
    this->set_status_(message);
  if (const char *message = this->limit_.take_message())
    this->set_status_(message);
}

void VenetianBlinds::save_measured_timing_() {
  const Timing &t = this->ctrl_.timing;
  uint32_t open_ms = t.open_ms;
  uint32_t close_ms = t.close_ms;
  this->prefs_[(uint8_t) DurationKind::OPEN].save(&open_ms);
  this->prefs_[(uint8_t) DurationKind::CLOSE].save(&close_ms);
  this->publish_number_(DurationKind::OPEN);
  this->publish_number_(DurationKind::CLOSE);
  ESP_LOGI(TAG, "Calibrated: open %.2fs, close %.2fs", open_ms / 1e3f, close_ms / 1e3f);
}

// The meter is only polled fast while something could need it, so HA telemetry stays sparse.
void VenetianBlinds::update_meter_interval_() {
  if (this->meter_ == nullptr)
    return;
  const uint32_t now = millis();
  uint32_t wanted = this->meter_idle_ms_;
  if (this->limit_.fast_polling()) {
    wanted = METER_ENTRY_MS;
    this->meter_fast_until_ = now + METER_LINGER_MS;
  } else if (!this->ctrl_.idle() || this->limit_.relay_up() || this->limit_.relay_down() ||
             this->calibration_.active()) {
    wanted = METER_MOVING_MS;
    this->meter_fast_until_ = now + METER_LINGER_MS;
  } else if ((int32_t) (this->meter_fast_until_ - now) > 0) {
    wanted = METER_MOVING_MS;
  }
  if (wanted == this->meter_interval_ms_)
    return;
  this->meter_interval_ms_ = wanted;
  this->meter_->set_update_interval(wanted);
  this->meter_->start_poller();
}

void VenetianBlinds::refresh_() {
  this->apply_outputs_();
  this->sync_state_();
  this->update_meter_interval_();
  this->pump_messages_();
}

void VenetianBlinds::set_service_mode(bool on) {
  const uint32_t now = millis();
  if (!on && this->limit_.active())
    this->limit_.cancel(now, this->ctrl_);
  this->service_mode_ = on;
  this->service_from_ = now;
  this->refresh_();
  this->was_idle_ = false;
}

void VenetianBlinds::service_action(ServiceAction action) {
  const uint32_t now = millis();
  const bool free = this->ctrl_.idle() && !this->limit_.active() && !this->calibration_.active();
  bool accepted = true;
  const char *refused = "Blind is busy. Wait for it to finish, or press Limit Cancel";
  switch (action) {
    case ServiceAction::CALIBRATE:
      accepted = free;
      if (free)
        this->calibration_.start(now, this->ctrl_);
      break;
    case ServiceAction::SET_UPPER:
    case ServiceAction::SET_LOWER:
      if (!this->service_mode_) {
        this->set_status_("Turn on Service Mode first");
        return;
      }
      accepted = free;
      if (free)
        this->limit_.start(now, action == ServiceAction::SET_UPPER ? LimitKind::UPPER : LimitKind::LOWER, this->ctrl_);
      break;
    case ServiceAction::RUN:
    case ServiceAction::NUDGE:
    case ServiceAction::STORE:
      if (action == ServiceAction::RUN) {
        accepted = this->limit_.run(now);
      } else if (action == ServiceAction::NUDGE) {
        accepted = this->limit_.nudge(now);
      } else {
        accepted = this->limit_.store(now);
      }
      if (!this->limit_.active()) {
        refused = "No limit is being set. Press Set Upper Limit or Set Lower Limit first";
      } else if (action == ServiceAction::STORE && this->limit_.ready() && !this->limit_.can_store()) {
        refused = "Move the blind with Limit Run or Limit Nudge before storing";
      } else {
        refused = "Wait: not ready yet. These buttons work once Status says In setting mode";
      }
      break;
    case ServiceAction::CANCEL:
      if (this->limit_.active()) {
        this->limit_.cancel(now, this->ctrl_);
      } else if (this->calibration_.active()) {
        this->calibration_.abort();
        this->ctrl_.request_stop(now);
      }
      break;
  }
  if (!accepted)
    this->set_status_(refused);
  this->service_from_ = now;
  this->refresh_();
  this->was_idle_ = false;
}

void VenetianBlinds::publish_number_(DurationKind kind) {
  DurationNumber *number = this->numbers_[(uint8_t) kind];
  if (number == nullptr)
    return;
  const Timing &t = this->ctrl_.timing;
  const uint32_t ms = kind == DurationKind::OPEN ? t.open_ms : (kind == DurationKind::CLOSE ? t.close_ms : t.tilt_ms);
  number->publish_state(ms / 1000.0f);
}

void VenetianBlinds::set_duration(DurationKind kind, uint32_t ms) {
  Timing candidate = this->ctrl_.timing;
  uint32_t *field = kind == DurationKind::OPEN ? &candidate.open_ms
                                               : (kind == DurationKind::CLOSE ? &candidate.close_ms : &candidate.tilt_ms);
  *field = ms;
  if (candidate.valid()) {
    this->ctrl_.timing = candidate;
    this->prefs_[(uint8_t) kind].save(&ms);
  } else {
    ESP_LOGW(TAG, "Rejected duration %.2fs: needs at least %.1fs of net travel", ms / 1e3f, MIN_NET_TRAVEL_MS / 1e3f);
  }
  // Republishing on rejection too puts the old value back in the UI.
  this->publish_number_(kind);
}

CoverTraits VenetianBlinds::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_tilt(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(this->assumed_state_);
  return traits;
}

void VenetianBlinds::control(const CoverCall &call) {
  const uint32_t now = millis();
  // In setting mode an ordinary up or down press could store a limit, so only Stop gets through.
  if (this->limit_.active()) {
    if (call.get_stop()) {
      this->limit_.stop(now, this->ctrl_);
    } else {
      ESP_LOGW(TAG, "Ignored a cover command while setting a limit");
    }
    this->refresh_();
    this->publish_state(false);
    this->was_idle_ = false;
    return;
  }
  if (this->calibration_.active())
    this->calibration_.abort();
  if (call.get_stop()) {
    this->ctrl_.request_stop(now);
  }
  if (call.get_position().has_value()) {
    if (!this->ctrl_.request_position(now, *call.get_position())) {
      ESP_LOGD(TAG, "Position request needs no movement");
    }
  } else if (call.get_tilt().has_value()) {
    if (!this->ctrl_.request_tilt(now, *call.get_tilt())) {
      ESP_LOGD(TAG, "Tilt request needs no movement");
    }
  }
  this->refresh_();
  this->publish_state(false);
  this->last_publish_ = now;
  this->was_idle_ = false;
}

void VenetianBlinds::apply_outputs_() {
  // Open before close, so the relays never overlap. Only the limit sequence's entry step asks
  // for both together, and it caps that well short of the 15 s factory reset.
  const bool up = this->ctrl_.relay_up() || this->limit_.relay_up();
  const bool down = this->ctrl_.relay_down() || this->limit_.relay_down();
  if (!up)
    this->up_output_->turn_off();
  if (!down)
    this->down_output_->turn_off();
  if (up)
    this->up_output_->turn_on();
  if (down)
    this->down_output_->turn_on();
}

void VenetianBlinds::sync_state_() {
  this->position = this->ctrl_.model.position;
  this->tilt = this->ctrl_.model.tilt;
  if (this->ctrl_.busy_up()) {
    this->current_operation = COVER_OPERATION_OPENING;
  } else if (this->ctrl_.busy_down()) {
    this->current_operation = COVER_OPERATION_CLOSING;
  } else {
    this->current_operation = COVER_OPERATION_IDLE;
  }
}

void VenetianBlinds::on_power_(Direction channel, float watts) {
  const uint32_t now = millis();
  if (this->limit_.relay_up() || this->limit_.relay_down()) {
    ESP_LOGD(TAG, "limit channel=%s p=%.1f", channel == Direction::UP ? "up" : "down", watts);
    this->limit_.on_power(now, watts);
    this->apply_outputs_();
    return;
  }
  const PowerEvent event = this->ctrl_.on_power(now, channel, watts);
  if (this->ctrl_.relay_up() || this->ctrl_.relay_down()) {
    if (channel == (this->ctrl_.relay_up() ? Direction::UP : Direction::DOWN)) {
      ESP_LOGD(TAG, "trace t=%u p=%.1f pos=%.3f tilt=%.3f", (unsigned) this->ctrl_.run_elapsed(now), watts,
               this->ctrl_.model.position, this->ctrl_.model.tilt);
    }
  }
  const char *dir = channel == Direction::UP ? "up" : "down";
  if (event == PowerEvent::CUT) {
    const float cut_s = this->ctrl_.last_cut_ms() / 1000.0f;
    ESP_LOGI(TAG, "Cut going %s at %.2fs", dir, cut_s);
    if (this->last_cut_sensor_ != nullptr)
      this->last_cut_sensor_->publish_state(cut_s);
    if (this->home_error_sensor_ != nullptr && this->ctrl_.home_error_valid())
      this->home_error_sensor_->publish_state(this->ctrl_.home_error_ms() / 1000.0f);
  } else if (event == PowerEvent::NEVER_RAN) {
    ESP_LOGI(TAG, "Motor never ran going %s", dir);
    if (this->last_cut_sensor_ != nullptr)
      this->last_cut_sensor_->publish_state(0.0f);
  }
  if (event != PowerEvent::NONE)
    this->apply_outputs_();
}

void VenetianBlinds::on_run_end_(uint32_t now, RunEnd end) {
  const float run_s = this->ctrl_.last_run_ms() / 1000.0f;
  ESP_LOGD(TAG, "Run ended after %.2fs", run_s);
  if (this->last_run_sensor_ != nullptr)
    this->last_run_sensor_->publish_state(run_s);

  if (this->calibration_.active()) {
    this->calibration_.on_run_end(now, end, this->ctrl_);
    if (this->calibration_.take_timing_changed())
      this->save_measured_timing_();
    return;
  }
  if (this->limit_.active()) {
    this->limit_.on_run_end(now, end, this->ctrl_);
    return;
  }
  switch (end) {
    case RunEnd::AT_LIMIT:
      this->set_status_("OK");
      break;
    case RunEnd::TIME_LIMIT:
      this->set_status_("Fault: no limit seen before the time limit");
      break;
    case RunEnd::MID_TRAVEL_CUT:
      this->set_status_("Fault: motor stopped mid-travel, position untrusted. Run a full open or close");
      break;
    case RunEnd::NO_POWER:
      this->set_status_("Fault: motor drew no power away from a limit, position untrusted");
      break;
    default:
      break;
  }
}

void VenetianBlinds::loop() {
  if (this->was_idle_ && this->ctrl_.idle() && !this->calibration_.active() && !this->limit_.active() &&
      !this->service_mode_ && this->meter_interval_ms_ == this->meter_idle_ms_)
    return;

  const uint32_t now = millis();
  if (this->service_mode_ && now - this->service_from_ >= SERVICE_MODE_MS) {
    ESP_LOGI(TAG, "Service Mode timed out");
    if (this->service_switch_ != nullptr) {
      this->service_switch_->turn_off();
    } else {
      this->set_service_mode(false);
    }
  }

  this->ctrl_.tick(now);
  const RunEnd end = this->ctrl_.take_run_end();
  if (end != RunEnd::NONE)
    this->on_run_end_(now, end);
  this->calibration_.tick(now, this->ctrl_);
  this->limit_.tick(now, this->ctrl_);
  if (this->limit_.take_store_finished() && this->service_mode_) {
    if (this->service_switch_ != nullptr) {
      this->service_switch_->turn_off();
    } else {
      this->set_service_mode(false);
    }
  }
  this->refresh_();

  if (this->ctrl_.idle()) {
    if (!this->was_idle_) {
      this->publish_state();
      this->was_idle_ = true;
    }
  } else {
    this->was_idle_ = false;
    if (now - this->last_publish_ > 1000) {
      this->publish_state(false);
      this->last_publish_ = now;
    }
  }
}

}  // namespace venetian_blinds
}  // namespace esphome
