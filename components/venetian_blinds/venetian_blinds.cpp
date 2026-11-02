#include "venetian_blinds.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace venetian_blinds {

static const char *const TAG = "venetian_blinds.cover";

using namespace esphome::cover;

void DurationNumber::control(float value) {
  this->parent_->set_duration(this->kind_, (uint32_t) lroundf(value * 1000.0f));
}

void VenetianBlinds::dump_config() {
  LOG_COVER("", "Venetian Blinds", this);
  const Timing &t = this->ctrl_.timing;
  ESP_LOGCONFIG(TAG, "  Open Duration: %.2fs", t.open_ms / 1e3f);
  ESP_LOGCONFIG(TAG, "  Close Duration: %.2fs", t.close_ms / 1e3f);
  ESP_LOGCONFIG(TAG, "  Tilt Duration: %.2fs", t.tilt_ms / 1e3f);
  ESP_LOGCONFIG(TAG, "  Actuator Activation Duration: %.2fs", t.activation_ms / 1e3f);
  ESP_LOGCONFIG(TAG, "  Cut Threshold: %.0fW (log only, homing not enabled)", this->ctrl_.detector.threshold_w);
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
  this->apply_outputs_();
  this->sync_state_();
  this->publish_state(false);
  this->last_publish_ = now;
  this->was_idle_ = false;
}

void VenetianBlinds::apply_outputs_() {
  // Open before close, so the two relays are never on together.
  if (!this->ctrl_.relay_up())
    this->up_output_->turn_off();
  if (!this->ctrl_.relay_down())
    this->down_output_->turn_off();
  if (this->ctrl_.relay_up())
    this->up_output_->turn_on();
  if (this->ctrl_.relay_down())
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
  const PowerEvent event = this->ctrl_.on_power(now, channel, watts);
  if (this->ctrl_.relay_up() || this->ctrl_.relay_down()) {
    if (channel == (this->ctrl_.relay_up() ? Direction::UP : Direction::DOWN)) {
      ESP_LOGD(TAG, "trace t=%u p=%.1f pos=%.3f tilt=%.3f", (unsigned) this->ctrl_.run_elapsed(now), watts,
               this->ctrl_.model.position, this->ctrl_.model.tilt);
    }
  }
  const char *dir = channel == Direction::UP ? "up" : "down";
  if (event == PowerEvent::CUT) {
    const float cut_s = this->ctrl_.detector.cut_ms() / 1000.0f;
    ESP_LOGI(TAG, "Cut going %s at %.2fs, model position %.3f", dir, cut_s, this->ctrl_.event_position());
    if (this->last_cut_sensor_ != nullptr)
      this->last_cut_sensor_->publish_state(cut_s);
    if (this->cut_position_sensor_ != nullptr)
      this->cut_position_sensor_->publish_state(this->ctrl_.event_position() * 100.0f);
  } else if (event == PowerEvent::NEVER_RAN) {
    ESP_LOGI(TAG, "Motor never ran going %s: already at its limit", dir);
    if (this->last_cut_sensor_ != nullptr)
      this->last_cut_sensor_->publish_state(0.0f);
  }
}

void VenetianBlinds::loop() {
  if (this->was_idle_ && this->ctrl_.idle())
    return;

  const uint32_t now = millis();
  this->ctrl_.tick(now);
  this->apply_outputs_();
  this->sync_state_();

  const RunEnd end = this->ctrl_.take_run_end();
  if (end != RunEnd::NONE) {
    const float run_s = this->ctrl_.last_run_ms() / 1000.0f;
    if (end == RunEnd::TIME_LIMIT) {
      ESP_LOGW(TAG, "Run hit its time limit after %.1fs", run_s);
    } else {
      ESP_LOGD(TAG, "Run ended after %.2fs", run_s);
    }
    if (this->last_run_sensor_ != nullptr)
      this->last_run_sensor_->publish_state(run_s);
  }

  if (this->ctrl_.idle()) {
    this->publish_state();
    this->was_idle_ = true;
  } else if (now - this->last_publish_ > 1000) {
    this->publish_state(false);
    this->last_publish_ = now;
  }
}

}  // namespace venetian_blinds
}  // namespace esphome
