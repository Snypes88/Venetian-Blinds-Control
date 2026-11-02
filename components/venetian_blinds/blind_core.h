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
// A motor stop inside this much of the end being approached counts as that end. Anywhere else
// it may be an obstacle, so it never moves the anchor.
static const float END_ZONE = 0.15f;
// Measured on Master Side at 500 ms polling: the first low reading comes this long after the stop.
static const uint32_t CUT_LAG_MS = 800;

static const uint32_t CAL_MIN_RUN_MS = 5000;
static const uint32_t CAL_MAX_RUN_MS = 150000;
static const uint32_t CAL_PAUSE_MS = 1000;
static const uint32_t CAL_MAX_DIFF_PERCENT = 25;

// Guide 5133945A section 4. Both relays for 5 s enters setting mode; 15 s is a factory reset.
// The motor answers from 5 s with a short up and down jog. Releasing at the first sign of it cut
// the jog short and the motor never took the limit, so the hold is a fixed time past the jog.
static const uint32_t LIMIT_ENTRY_ACK_FROM_MS = 5000;
static const uint32_t LIMIT_ENTRY_HOLD_MS = 8000;
static const uint32_t LIMIT_PARK_PAUSE_MS = 10000;
static const uint32_t LIMIT_DRIVE_CAP_MS = 60000;
static const uint32_t LIMIT_NUDGE_MS = 500;
static const uint32_t LIMIT_STORE_PRESS_MS = 1000;
static const uint32_t LIMIT_NO_MOVE_MS = 4000;
static const float LIMIT_MOVE_W = 10.0f;
static const float PARK_LOW = 0.3f;
static const float PARK_HIGH = 0.5f;

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

enum class RunEnd { NONE, ARRIVED, STOPPED, TIME_LIMIT, AT_LIMIT, MID_TRAVEL_CUT, NO_POWER };

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
    any_cut_is_limit_ = false;
    // A full run always goes to the motor's own stop, even if the model says it is there already.
    if (requested <= 0.0f) {
      move_(now, Direction::DOWN, Kind::FULL, 0.0f);
    } else if (requested >= 1.0f) {
      move_(now, Direction::UP, Kind::FULL, 1.0f);
    } else {
      if (std::fabs(requested - model.position) < 0.001f)
        return false;
      move_(now, requested > model.position ? Direction::UP : Direction::DOWN, Kind::POSITION, requested);
    }
    return true;
  }
  bool request_tilt(uint32_t now, float requested) {
    advance_(now);
    any_cut_is_limit_ = false;
    if (std::fabs(requested - model.tilt) < 0.001f)
      return false;
    move_(now, requested > model.tilt ? Direction::UP : Direction::DOWN, Kind::TILT, requested);
    return true;
  }

  // Both limits are motor stops, so during calibration a cut anywhere counts, however wrong the
  // old times are. The run limit is the longest calibration accepts, not the model's time.
  void start_calibration_run(uint32_t now, Direction dir) {
    advance_(now);
    any_cut_is_limit_ = true;
    move_(now, dir, Kind::FULL, dir == Direction::UP ? 1.0f : 0.0f);
  }
  // The model can't say where the blind is after the limit sequence moved it, so the next stop
  // counts wherever it comes.
  void mark_position_unknown() { position_unknown_ = true; }
  // The limit sequence drove the relays itself, so the reversal gap has to count from then.
  void note_external_release(uint32_t now) {
    has_run_ = true;
    external_release_ = true;
    last_off_ = now;
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
    // Tilt is a time model only, and is too short for the detector on a normal blind anyway.
    if (event == PowerEvent::NONE || kind_ == Kind::TILT)
      return PowerEvent::NONE;
    const bool up = active_dir_ == Direction::UP;
    const bool in_end_zone = up ? model.position >= 1.0f - END_ZONE : model.position <= END_ZONE;
    ended_on_cut_ = event == PowerEvent::CUT;
    home_error_valid_ = ended_on_cut_ && kind_ == Kind::FULL && !any_cut_is_limit_ && !position_unknown_ && in_end_zone;
    if (home_error_valid_)
      home_error_ms_ = (int32_t) detector.cut_ms() - (int32_t) CUT_LAG_MS - (int32_t) expected_run_ms_;
    if (in_end_zone || any_cut_is_limit_ || position_unknown_) {
      position_unknown_ = false;
      model.position = up ? 1.0f : 0.0f;
      model.tilt = model.position;
      end_run_(now, RunEnd::AT_LIMIT);
    } else {
      end_run_(now, ended_on_cut_ ? RunEnd::MID_TRAVEL_CUT : RunEnd::NO_POWER);
    }
    return event;
  }
  uint32_t run_elapsed(uint32_t now) const { return now - run_start_; }
  bool ended_on_cut() const { return ended_on_cut_; }
  uint32_t last_cut_ms() const { return detector.cut_ms(); }
  // How far the model's time was from the motor's stop. Positive: the run took longer than modelled.
  bool home_error_valid() const { return home_error_valid_; }
  int32_t home_error_ms() const { return home_error_ms_; }
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
  enum class Kind { POSITION, TILT, FULL };

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
      if (has_run_ && (dir != active_dir_ || external_release_) && now - last_off_ < REVERSAL_DEAD_TIME_MS) {
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
    external_release_ = false;
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
      case Kind::FULL:
        // The model may be wrong about where the blind is, so allow a whole run from anywhere.
        remaining = any_cut_is_limit_ ? CAL_MAX_RUN_MS
                                      : (heading_ == Direction::UP ? timing.open_ms : timing.close_ms);
        expected_run_ms_ = (now - run_start_) + timing.activation_ms + model.ms_to_position(heading_, target_, timing);
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
    // A tilt that overshoots by a loop iteration mustn't creep the height.
    const float position = model.position;
    model.advance(active_dir_, now - from, timing);
    if (kind_ == Kind::TILT)
      model.position = position;
    last_tick_ = now;
  }
  bool at_target_(uint32_t now) const {
    const bool up = active_dir_ == Direction::UP;
    switch (kind_) {
      case Kind::TILT:
        return up ? model.tilt >= target_ : model.tilt <= target_;
      case Kind::FULL:
        return false;  // only the motor's stop or the time limit ends a full run
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
  bool any_cut_is_limit_{false};
  bool position_unknown_{false};
  bool external_release_{false};
  bool ended_on_cut_{false};
  bool home_error_valid_{false};
  float target_{0.0f};
  int32_t home_error_ms_{0};
  uint32_t expected_run_ms_{0};
  uint32_t run_start_{0};
  uint32_t last_tick_{0};
  uint32_t last_off_{0};
  uint32_t run_limit_ms_{0};
  uint32_t last_run_ms_{0};
};

// Up to the top, down timed, up timed, then back to where the blind was.
class TravelCalibration {
 public:
  bool active() const { return step_ != Step::IDLE; }

  void start(uint32_t now, MotionController &ctrl) {
    ctrl.take_run_end();  // an earlier run's end isn't ours
    saved_position_ = ctrl.model.position;
    saved_tilt_ = ctrl.model.tilt;
    step_ = Step::TO_TOP;
    message_ = "Calibrating: going to the top";
    ctrl.start_calibration_run(now, Direction::UP);
  }
  void abort() {
    step_ = Step::IDLE;
    message_ = "Calibration aborted, times unchanged";
  }
  void on_run_end(uint32_t now, RunEnd end, MotionController &ctrl) {
    switch (step_) {
      case Step::TO_TOP:
        if (end != RunEnd::AT_LIMIT)
          return fail_("Calibration failed: no stop seen going up, times unchanged");
        pause_(now, Step::DOWN);
        break;
      case Step::DOWN:
        if (!measure_(end, ctrl, &close_ms_))
          return fail_("Calibration failed: no valid stop going down, times unchanged");
        pause_(now, Step::UP);
        break;
      case Step::UP:
        if (!measure_(end, ctrl, &open_ms_))
          return fail_("Calibration failed: no valid stop going up, times unchanged");
        finish_measuring_(now, ctrl);
        break;
      case Step::RETURN_POSITION:
        if (end != RunEnd::ARRIVED && end != RunEnd::AT_LIMIT)
          return fail_("Times saved, but the return move failed");
        return_tilt_(now, ctrl);
        break;
      case Step::RETURN_TILT:
        step_ = Step::IDLE;
        message_ = "Calibration done";
        break;
      default:
        break;
    }
  }
  void tick(uint32_t now, MotionController &ctrl) {
    if (step_ != Step::PAUSE || now - pause_from_ < CAL_PAUSE_MS)
      return;
    step_ = next_;
    message_ = step_ == Step::DOWN ? "Calibrating: timing down" : "Calibrating: timing up";
    ctrl.start_calibration_run(now, step_ == Step::DOWN ? Direction::DOWN : Direction::UP);
  }
  // Each reports once and then clears.
  bool take_timing_changed() {
    const bool changed = timing_changed_;
    timing_changed_ = false;
    return changed;
  }
  const char *take_message() {
    const char *message = message_;
    message_ = nullptr;
    return message;
  }

 private:
  enum class Step { IDLE, TO_TOP, PAUSE, DOWN, UP, RETURN_POSITION, RETURN_TILT };

  void pause_(uint32_t now, Step next) {
    step_ = Step::PAUSE;
    next_ = next;
    pause_from_ = now;
  }
  void fail_(const char *message) {
    step_ = Step::IDLE;
    message_ = message;
  }
  static bool measure_(RunEnd end, const MotionController &ctrl, uint32_t *ms) {
    if (end != RunEnd::AT_LIMIT || !ctrl.ended_on_cut() || ctrl.last_cut_ms() <= CUT_LAG_MS)
      return false;
    *ms = ctrl.last_cut_ms() - CUT_LAG_MS;
    return *ms >= CAL_MIN_RUN_MS && *ms <= CAL_MAX_RUN_MS;
  }
  void finish_measuring_(uint32_t now, MotionController &ctrl) {
    const uint32_t high = open_ms_ > close_ms_ ? open_ms_ : close_ms_;
    const uint32_t low = open_ms_ > close_ms_ ? close_ms_ : open_ms_;
    Timing candidate = ctrl.timing;
    candidate.open_ms = open_ms_;
    candidate.close_ms = close_ms_;
    if ((uint64_t) (high - low) * 100 > (uint64_t) high * CAL_MAX_DIFF_PERCENT || !candidate.valid())
      return fail_("Calibration rejected: measured times are not plausible, times unchanged");
    ctrl.timing = candidate;
    timing_changed_ = true;
    if (std::fabs(saved_position_ - ctrl.model.position) < 0.001f)
      return return_tilt_(now, ctrl);
    step_ = Step::RETURN_POSITION;
    message_ = "Times saved, returning";
    ctrl.request_position(now, saved_position_);
  }
  void return_tilt_(uint32_t now, MotionController &ctrl) {
    if (ctrl.request_tilt(now, saved_tilt_)) {
      step_ = Step::RETURN_TILT;
      message_ = "Times saved, returning";
    } else {
      step_ = Step::IDLE;
      message_ = "Calibration done";
    }
  }

  Step step_{Step::IDLE};
  Step next_{Step::IDLE};
  const char *message_{nullptr};
  bool timing_changed_{false};
  float saved_position_{0.0f};
  float saved_tilt_{0.0f};
  uint32_t open_ms_{0};
  uint32_t close_ms_{0};
  uint32_t pause_from_{0};
};

enum class LimitKind : uint8_t { UPPER, LOWER };

// Sets a motor limit by guide 5133945A section 4. Someone watches the blind throughout.
// Parking always ends on an up move with a long pause: a brief down press before the 5 s press
// would change a motor function instead of the limits.
class LimitSequence {
 public:
  bool active() const { return state_ != State::IDLE; }
  bool relay_up() const { return state_ == State::ENTERING || (pressing_() && press_dir_ == Direction::UP); }
  bool relay_down() const { return state_ == State::ENTERING || (pressing_() && press_dir_ == Direction::DOWN); }
  bool fast_polling() const { return state_ == State::ENTERING || state_ == State::STORING; }

  void start(uint32_t now, LimitKind kind, MotionController &ctrl) {
    ctrl.take_run_end();  // an earlier run's end isn't ours
    kind_ = kind;
    entry_attempted_ = false;
    ack_seen_ = false;
    driven_ = false;
    message_ = "Parking the blind";
    if (ctrl.model.position >= PARK_LOW + 0.1f) {
      state_ = State::PARK_LOW;
      ctrl.request_position(now, PARK_LOW);
    } else {
      state_ = State::PARK_HIGH;
      ctrl.request_position(now, PARK_HIGH);
    }
  }
  void on_run_end(uint32_t now, RunEnd end, MotionController &ctrl) {
    if (state_ != State::PARK_LOW && state_ != State::PARK_HIGH)
      return;
    if (end != RunEnd::ARRIVED) {
      state_ = State::IDLE;
      message_ = "Parking failed, motor limits unchanged";
    } else if (state_ == State::PARK_LOW) {
      state_ = State::PARK_HIGH;
      ctrl.request_position(now, PARK_HIGH);
    } else {
      state_ = State::PARK_PAUSE;
      state_from_ = now;
      message_ = "Parked, waiting before entering setting mode";
    }
  }
  void tick(uint32_t now, MotionController &ctrl) {
    switch (state_) {
      case State::PARK_PAUSE:
        if (now - state_from_ >= LIMIT_PARK_PAUSE_MS) {
          state_ = State::ENTERING;
          state_from_ = now;
          entry_attempted_ = true;
          message_ = "Entering setting mode: both relays on";
        }
        break;
      case State::ENTERING:
        if (now - state_from_ >= LIMIT_ENTRY_HOLD_MS)
          end_entry_(now, ack_seen_ ? "In setting mode. Limit Run or Limit Nudge moves the blind"
                                    : "No movement seen on power. If the blind jogged press Limit Run, otherwise "
                                      "Limit Cancel");
        break;
      case State::DRIVE_WAIT:
        if (now - last_off_ >= REVERSAL_DEAD_TIME_MS)
          begin_press_(now, State::DRIVING);
        break;
      case State::DRIVING:
        if (now - state_from_ >= press_limit_ms_) {
          const bool timed_out = press_limit_ms_ == LIMIT_DRIVE_CAP_MS;
          release_press_(now, State::READY);
          message_ = timed_out ? "Drive time cap reached. Press Limit Store or Limit Cancel"
                               : "In setting mode. Limit Run, Limit Nudge, Limit Store or Limit Cancel";
        } else if (press_limit_ms_ != LIMIT_NUDGE_MS && !press_moved_ && now - state_from_ >= LIMIT_NO_MOVE_MS) {
          release_press_(now, State::READY);
          message_ = kind_ == LimitKind::LOWER ? "Blind did not move: this lower limit may be factory locked"
                                               : "Blind did not move. Press Limit Cancel";
        }
        break;
      case State::STORE_WAIT:
        if (now - last_off_ >= REVERSAL_DEAD_TIME_MS)
          begin_press_(now, State::STORING);
        break;
      case State::STORING:
        if (now - state_from_ >= press_limit_ms_)
          finish_store_(now, ctrl);
        break;
      default:
        break;
    }
  }
  void on_power(uint32_t now, float watts) {
    if (!(std::fabs(watts) >= LIMIT_MOVE_W))  // written this way so NaN is ignored too
      return;
    if (state_ == State::ENTERING && now - state_from_ >= LIMIT_ENTRY_ACK_FROM_MS) {
      ack_seen_ = true;
    } else if (pressing_()) {
      press_moved_ = true;
    }
  }
  // Each returns false if the sequence isn't at a point where the button means anything.
  bool run(uint32_t now) { return press_(now, LIMIT_DRIVE_CAP_MS); }
  bool nudge(uint32_t now) { return press_(now, LIMIT_NUDGE_MS); }
  bool store(uint32_t now) {
    // With no move yet, a store press would make the parked height the new limit.
    if (state_ != State::READY || !driven_)
      return false;
    press_dir_ = kind_ == LimitKind::UPPER ? Direction::DOWN : Direction::UP;
    press_limit_ms_ = LIMIT_STORE_PRESS_MS;
    press_moved_ = false;
    state_ = State::STORE_WAIT;
    message_ = "Storing";
    if (now - last_off_ >= REVERSAL_DEAD_TIME_MS)
      begin_press_(now, State::STORING);
    return true;
  }
  void stop(uint32_t now, MotionController &ctrl) {
    if (parking_()) {
      cancel(now, ctrl);
    } else if (state_ == State::DRIVE_WAIT) {
      state_ = State::READY;
    } else if (state_ == State::DRIVING && now - state_from_ >= MIN_COMMAND_MS) {
      release_press_(now, State::READY);
      message_ = "In setting mode. Limit Run, Limit Nudge, Limit Store or Limit Cancel";
    } else if (state_ == State::DRIVING) {
      press_limit_ms_ = MIN_COMMAND_MS;
    } else if (state_ == State::ENTERING || state_ == State::STORE_WAIT || state_ == State::STORING) {
      cancel(now, ctrl);
    }
  }
  void cancel(uint32_t now, MotionController &ctrl) {
    if (parking_()) {
      ctrl.request_stop(now);
      message_ = "Cancelled while parking, motor limits unchanged";
    } else {
      message_ = entry_attempted_ ? "Cancelled. The motor may still be in setting mode: check before normal use"
                                  : "Cancelled, motor limits unchanged";
      ctrl.mark_position_unknown();
      ctrl.note_external_release(now);
    }
    state_ = State::IDLE;
  }
  const char *take_message() {
    const char *message = message_;
    message_ = nullptr;
    return message;
  }

 private:
  enum class State { IDLE, PARK_LOW, PARK_HIGH, PARK_PAUSE, ENTERING, READY, DRIVE_WAIT, DRIVING, STORE_WAIT, STORING };

  bool parking_() const {
    return state_ == State::PARK_LOW || state_ == State::PARK_HIGH || state_ == State::PARK_PAUSE;
  }
  bool pressing_() const { return state_ == State::DRIVING || state_ == State::STORING; }
  void end_entry_(uint32_t now, const char *message) {
    state_ = State::READY;
    last_off_ = now;
    message_ = message;
  }
  bool press_(uint32_t now, uint32_t limit_ms) {
    if (state_ != State::READY)
      return false;
    press_dir_ = kind_ == LimitKind::UPPER ? Direction::UP : Direction::DOWN;
    press_limit_ms_ = limit_ms;
    press_moved_ = false;
    driven_ = true;
    state_ = State::DRIVE_WAIT;
    message_ = "Moving";
    if (now - last_off_ >= REVERSAL_DEAD_TIME_MS)
      begin_press_(now, State::DRIVING);
    return true;
  }
  void begin_press_(uint32_t now, State state) {
    state_ = state;
    state_from_ = now;
  }
  void release_press_(uint32_t now, State state) {
    state_ = state;
    last_off_ = now;
  }
  void finish_store_(uint32_t now, MotionController &ctrl) {
    const bool confirmed = ack_seen_ && press_moved_;
    state_ = State::IDLE;
    ctrl.mark_position_unknown();
    ctrl.note_external_release(now);
    // Power during the press shows the blind moved, which an ordinary move would do too. Only the
    // time of a full run can prove the limit changed.
    message_ = confirmed ? "Store press done. Run a full open and close: the close time shows if the limit moved"
                         : "Store press sent, no movement seen on power. Run a full open and close to check";
  }

  State state_{State::IDLE};
  LimitKind kind_{LimitKind::LOWER};
  Direction press_dir_{Direction::DOWN};
  const char *message_{nullptr};
  bool entry_attempted_{false};
  bool ack_seen_{false};
  bool driven_{false};
  bool press_moved_{false};
  uint32_t state_from_{0};
  uint32_t last_off_{0};
  uint32_t press_limit_ms_{0};
};

}  // namespace venetian_blinds
}  // namespace esphome
