#pragma once
// Motion logic with no ESPHome dependency, so it can be tested on a PC
// (tests/host_test.cpp). The component only wires it to relays and sensors.
#include <cmath>
#include <cstdint>

namespace esphome {
namespace venetian_blinds {

// From Somfy J4 WT Protect guide 5133945A.
static const uint32_t MIN_COMMAND_MS = 200;
static const uint32_t REVERSAL_DEAD_TIME_MS = 500;

static const uint32_t MIN_NET_TRAVEL_MS = 5000;
static const uint32_t MIN_TILT_MS = 200;
static const uint32_t MAX_TILT_MS = 20000;
static const uint32_t MAX_TRAVEL_MS = 180000;

// Independent of power: a relay never stays on longer than the model expects plus this.
static const uint32_t RUN_MARGIN_MS = 15000;
static const uint32_t MAX_RUN_MS = 200000;
static const uint32_t TOP_PROBE_MS = 3000;

enum class Direction : int8_t { UP = 1, DOWN = -1 };

struct Timing {
  uint32_t open_ms{0};
  uint32_t close_ms{0};
  uint32_t tilt_ms{0};
  uint32_t activation_ms{0};

  bool valid() const {
    const uint32_t overhead = tilt_ms + activation_ms;
    return tilt_ms >= MIN_TILT_MS && tilt_ms <= MAX_TILT_MS && open_ms <= MAX_TRAVEL_MS &&
           close_ms <= MAX_TRAVEL_MS && open_ms >= overhead + MIN_NET_TRAVEL_MS &&
           close_ms >= overhead + MIN_NET_TRAVEL_MS;
  }
  uint32_t net_ms(Direction dir) const {
    return (dir == Direction::UP ? open_ms : close_ms) - tilt_ms - activation_ms;
  }
};

// Position and tilt are both 0-1 and share one scale whatever the direction.
// Slats rotate first; only the time left after that moves the rail.
class TravelModel {
 public:
  float position{0.0f};
  float tilt{0.0f};

  void advance(Direction dir, uint32_t moving_ms, const Timing &timing) {
    const float sign = dir == Direction::UP ? 1.0f : -1.0f;
    const float tilt_room_ms = (dir == Direction::UP ? 1.0f - tilt : tilt) * timing.tilt_ms;
    const float tilt_used_ms = std::fmin((float) moving_ms, tilt_room_ms);
    tilt = clamp01_(tilt + sign * tilt_used_ms / timing.tilt_ms);
    const float travel_ms = moving_ms - tilt_used_ms;
    if (travel_ms > 0.0f)
      position = clamp01_(position + sign * travel_ms / timing.net_ms(dir));
  }
  uint32_t ms_to_position(Direction dir, float target, const Timing &timing) const {
    const float tilt_room = dir == Direction::UP ? 1.0f - tilt : tilt;
    return (uint32_t) (tilt_room * timing.tilt_ms + std::fabs(target - position) * timing.net_ms(dir));
  }
  uint32_t ms_to_tilt(float target, const Timing &timing) const {
    return (uint32_t) (std::fabs(target - tilt) * timing.tilt_ms);
  }

 private:
  static float clamp01_(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
};

enum class PowerEvent { NONE, CUT, NEVER_RAN };

// Spots the motor stopping itself: it ran, then power fell while the relay stayed on.
class CutDetector {
 public:
  float threshold_w{30.0f};
  uint32_t blank_ms{2000};
  uint32_t start_window_ms{4000};
  uint8_t readings_below{3};

  void reset() {
    seen_running_ = false;
    below_ = 0;
    done_ = false;
    cut_ms_ = 0;
  }
  // t_ms is the time since the relay closed. Returns an event once per run at most.
  PowerEvent feed(uint32_t t_ms, float watts) {
    if (done_ || std::isnan(watts))
      return PowerEvent::NONE;
    if (watts >= threshold_w) {
      seen_running_ = true;
      below_ = 0;
      return PowerEvent::NONE;
    }
    if (t_ms < blank_ms)
      return PowerEvent::NONE;
    if (!seen_running_) {
      if (t_ms < start_window_ms)
        return PowerEvent::NONE;
      done_ = true;
      return PowerEvent::NEVER_RAN;
    }
    if (below_ == 0)
      cut_ms_ = t_ms;
    if (++below_ < readings_below)
      return PowerEvent::NONE;
    done_ = true;
    return PowerEvent::CUT;
  }
  uint32_t cut_ms() const { return cut_ms_; }

 private:
  bool seen_running_{false};
  bool done_{false};
  uint8_t below_{0};
  uint32_t cut_ms_{0};
};

enum class RunEnd { NONE, ARRIVED, STOPPED, TIME_LIMIT };

class MotionController {
 public:
  Timing timing;
  TravelModel model;
  CutDetector detector;

  bool relay_up() const { return relay_on_ && active_dir_ == Direction::UP; }
  bool relay_down() const { return relay_on_ && active_dir_ == Direction::DOWN; }
  bool idle() const { return state_ == State::IDLE; }
  // The direction being driven or waited for, for reporting the cover operation.
  bool busy_up() const { return !idle() && !stopping_ && heading_ == Direction::UP; }
  bool busy_down() const { return !idle() && !stopping_ && heading_ == Direction::DOWN; }

  void request_stop(uint32_t now) {
    advance_(now);
    if (state_ == State::RUNNING) {
      end_run_(now, RunEnd::STOPPED);
    } else if (state_ == State::DEAD_TIME) {
      state_ = State::IDLE;
    } else if (state_ == State::HOLD_MIN) {
      pending_ = false;
      stopping_ = true;
    }
  }
  // Returns false if the request needs no movement.
  bool request_position(uint32_t now, float requested) {
    advance_(now);
    if (requested <= 0.0f) {
      if (model.position <= 0.0f && model.tilt <= 0.0f)
        return false;
      move_(now, Direction::DOWN, Kind::POSITION, 0.0f);
    } else if (requested >= 1.0f) {
      if (model.position >= 1.0f && model.tilt >= 1.0f) {
        move_(now, Direction::UP, Kind::TOP_PROBE, 1.0f);
      } else {
        move_(now, Direction::UP, Kind::POSITION, 1.0f);
      }
    } else {
      if (std::fabs(requested - model.position) < 0.001f)
        return false;
      move_(now, requested > model.position ? Direction::UP : Direction::DOWN, Kind::POSITION, requested);
    }
    return true;
  }
  bool request_tilt(uint32_t now, float requested) {
    advance_(now);
    if (std::fabs(requested - model.tilt) < 0.001f)
      return false;
    move_(now, requested > model.tilt ? Direction::UP : Direction::DOWN, Kind::TILT, requested);
    return true;
  }

  void tick(uint32_t now) {
    switch (state_) {
      case State::IDLE:
        return;
      case State::DEAD_TIME:
        if (now - last_off_ >= REVERSAL_DEAD_TIME_MS)
          start_(now);
        return;
      case State::HOLD_MIN:
        if (now - run_start_ >= MIN_COMMAND_MS)
          relay_off_(now);
        return;
      case State::RUNNING:
        advance_(now);
        if (at_target_(now)) {
          snap_to_target_();
          end_run_(now, RunEnd::ARRIVED);
        } else if (now - run_start_ >= run_limit_ms_) {
          end_run_(now, RunEnd::TIME_LIMIT);
        }
        return;
    }
  }

  // Feed each power reading from the active channel's sensor.
  PowerEvent on_power(uint32_t now, Direction channel, float watts) {
    if (state_ != State::RUNNING || channel != active_dir_)
      return PowerEvent::NONE;
    advance_(now);
    const PowerEvent event = detector.feed(now - run_start_, watts);
    if (event != PowerEvent::NONE)
      event_position_ = model.position;
    return event;
  }
  uint32_t run_elapsed(uint32_t now) const { return now - run_start_; }
  float event_position() const { return event_position_; }
  bool active_is_up() const { return active_dir_ == Direction::UP; }

  // Each reports once and then clears.
  RunEnd take_run_end() {
    const RunEnd end = run_end_;
    run_end_ = RunEnd::NONE;
    return end;
  }
  uint32_t last_run_ms() const { return last_run_ms_; }

 private:
  enum class State { IDLE, RUNNING, HOLD_MIN, DEAD_TIME };
  enum class Kind { POSITION, TILT, TOP_PROBE };

  void move_(uint32_t now, Direction dir, Kind kind, float target) {
    heading_ = dir;
    kind_ = kind;
    target_ = target;
    stopping_ = false;
    pending_ = true;
    const bool energised = state_ == State::RUNNING || state_ == State::HOLD_MIN;
    if (energised && dir == active_dir_) {
      state_ = State::RUNNING;
      pending_ = false;
      set_run_limit_(now);
    } else if (energised) {
      release_(now);
    } else if (state_ == State::IDLE) {
      if (has_run_ && dir != active_dir_ && now - last_off_ < REVERSAL_DEAD_TIME_MS) {
        state_ = State::DEAD_TIME;
      } else {
        start_(now);
      }
    }
  }
  void start_(uint32_t now) {
    pending_ = false;
    state_ = State::RUNNING;
    active_dir_ = heading_;
    relay_on_ = true;
    has_run_ = true;
    run_start_ = now;
    last_tick_ = now;
    detector.reset();
    set_run_limit_(now);
  }
  void set_run_limit_(uint32_t now) {
    uint32_t remaining;
    switch (kind_) {
      case Kind::TILT:
        remaining = model.ms_to_tilt(target_, timing);
        break;
      case Kind::TOP_PROBE:
        remaining = TOP_PROBE_MS;
        break;
      default:
        remaining = model.ms_to_position(heading_, target_, timing);
        break;
    }
    const uint32_t limit = (now - run_start_) + timing.activation_ms + remaining + RUN_MARGIN_MS;
    run_limit_ms_ = limit < MAX_RUN_MS ? limit : MAX_RUN_MS;
  }
  void advance_(uint32_t now) {
    if (state_ != State::RUNNING)
      return;
    const uint32_t moving_from = run_start_ + timing.activation_ms;
    if ((int32_t) (now - moving_from) <= 0) {
      last_tick_ = now;
      return;
    }
    const uint32_t from = (int32_t) (last_tick_ - moving_from) < 0 ? moving_from : last_tick_;
    model.advance(active_dir_, now - from, timing);
    last_tick_ = now;
  }
  bool at_target_(uint32_t now) const {
    const bool up = active_dir_ == Direction::UP;
    switch (kind_) {
      case Kind::TILT:
        return up ? model.tilt >= target_ : model.tilt <= target_;
      case Kind::TOP_PROBE:
        return now - run_start_ >= TOP_PROBE_MS;
      default:
        return up ? (model.tilt >= 1.0f && model.position >= target_)
                  : (model.tilt <= 0.0f && model.position <= target_);
    }
  }
  void snap_to_target_() {
    if (kind_ == Kind::TILT) {
      model.tilt = target_;
    } else if (kind_ == Kind::POSITION) {
      model.position = target_;
    }
  }
  void end_run_(uint32_t now, RunEnd end) {
    run_end_ = end;
    stopping_ = true;
    release_(now);
  }
  // Relay off now, or as soon as the motor's minimum command length has passed.
  void release_(uint32_t now) {
    if (now - run_start_ >= MIN_COMMAND_MS) {
      relay_off_(now);
    } else {
      state_ = State::HOLD_MIN;
    }
  }
  void relay_off_(uint32_t now) {
    relay_on_ = false;
    last_off_ = now;
    last_run_ms_ = now - run_start_;
    state_ = pending_ ? State::DEAD_TIME : State::IDLE;
    if (state_ == State::DEAD_TIME && heading_ == active_dir_)
      start_(now);
    stopping_ = false;
  }

  State state_{State::IDLE};
  Kind kind_{Kind::POSITION};
  Direction active_dir_{Direction::UP};
  Direction heading_{Direction::UP};
  RunEnd run_end_{RunEnd::NONE};
  bool relay_on_{false};
  bool has_run_{false};
  bool pending_{false};
  bool stopping_{false};
  float target_{0.0f};
  float event_position_{0.0f};
  uint32_t run_start_{0};
  uint32_t last_tick_{0};
  uint32_t last_off_{0};
  uint32_t run_limit_ms_{0};
  uint32_t last_run_ms_{0};
};

}  // namespace venetian_blinds
}  // namespace esphome
