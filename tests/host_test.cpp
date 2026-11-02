// Host tests for blind_core.h. No ESPHome or hardware involved.
#include <cstdio>
#include <cstring>

#include "../components/venetian_blinds/blind_core.h"
#include "t1_traces.h"

using namespace esphome::venetian_blinds;

static int failures = 0;
static int checks = 0;

static void check(bool ok, const char *what) {
  checks++;
  if (!ok) {
    failures++;
    printf("FAIL: %s\n", what);
  }
}
static void near(float a, float b, float tol, const char *what) {
  checks++;
  if (std::fabs(a - b) > tol) {
    failures++;
    printf("FAIL: %s: got %.4f, expected %.4f\n", what, a, b);
  }
}

static Timing make_timing(uint32_t open_ms, uint32_t close_ms) {
  Timing t;
  t.open_ms = open_ms;
  t.close_ms = close_ms;
  t.tilt_ms = 1750;
  t.activation_ms = 200;
  return t;
}

// Runs the controller in 10 ms steps until idle or out of time, returning the time reached.
static uint32_t run_until_idle(MotionController &c, uint32_t now, uint32_t max_ms = 300000) {
  const uint32_t end = now + max_ms;
  while (!c.idle() && now < end) {
    now += 10;
    c.tick(now);
  }
  return now;
}
static uint32_t relay_time(MotionController &c, uint32_t now) {
  const uint32_t start = now;
  uint32_t on_ms = 0;
  while (!c.idle() && now < start + 300000) {
    if (c.relay_up() || c.relay_down())
      on_ms += 10;
    now += 10;
    c.tick(now);
  }
  return on_ms;
}

// Drives a run with a motor that stops itself after motor_ms. motor_ms 0 means it never starts.
// Returns the time reached. The meter reads every 500 ms unless the meter is dead.
static uint32_t run_motor(MotionController &c, uint32_t now, uint32_t motor_ms, bool meter = true) {
  const uint32_t start = now;
  uint32_t relay_from = now;
  bool on = false;
  while (!c.idle() && now < start + 300000) {
    now += 10;
    c.tick(now);
    const bool relay = c.relay_up() || c.relay_down();
    if (relay && !on)
      relay_from = now;
    on = relay;
    if (meter && relay && (now - relay_from) % 500 == 0)
      c.on_power(now, c.active_is_up() ? Direction::UP : Direction::DOWN,
                 motor_ms > 0 && now - relay_from < motor_ms + CUT_LAG_MS ? 100.0f : 0.5f);
  }
  return now;
}

static void test_position_scale() {
  // Living: the upstream bug showed 61% after this and took 19.3 s to reach 25%.
  MotionController c;
  c.timing = make_timing(67000, 55000);
  uint32_t now = 1000;
  c.request_position(now, 0.5f);
  now = run_until_idle(c, now);
  near(c.model.position, 0.5f, 0.0001f, "open to 50%");
  now += 1000;
  c.tick(now);
  c.request_position(now, 0.25f);
  now += 200 + 900;
  c.tick(now);
  check(c.model.position >= 0.49f, "no jump when reversing");
  c.request_stop(now);
  now = run_until_idle(c, now);
  c.model.position = 0.5f;
  c.model.tilt = 0.0f;
  now += 1000;
  c.request_position(now, 0.25f);
  const uint32_t on = relay_time(c, now);
  near(on / 1000.0f, 0.2f + 0.25f * 53.05f, 0.1f, "50 to 25% takes 13.5 s with slats already closed");
}

static void test_close_longer_than_open() {
  // Upstream stopped this close after open_net, leaving the blind short.
  MotionController c;
  c.timing = make_timing(40000, 50000);
  uint32_t now = 1000;
  c.request_position(now, 1.0f);
  now = run_motor(c, now, 40000);
  now += 1000;
  c.request_position(now, 0.0f);
  const uint32_t end = run_motor(c, now, 50000);
  near(c.last_run_ms() / 1000.0f, 52.0f, 0.6f, "full close runs the whole close time and on to the cut");
  check(end > now, "ran");
  check(c.take_run_end() == RunEnd::AT_LIMIT, "ended at the limit");
}

static void test_duration_change_mid_move() {
  // Upstream left the relay on forever here.
  MotionController c;
  c.timing = make_timing(60000, 60000);
  uint32_t now = 1000;
  c.request_position(now, 0.9f);
  now += 10000;
  c.tick(now);
  const float before = c.model.position;
  c.timing.open_ms = 30000;
  now += 10;
  c.tick(now);
  near(c.model.position, before, 0.001f, "position continuous across a change");
  now = run_until_idle(c, now, 40000);
  check(c.idle(), "relay released after shortening open time");
  check(!c.relay_up(), "relay off");
  near(c.model.position, 0.9f, 0.0001f, "arrived at the target");

  c.timing = make_timing(60000, 60000);
  c.model.position = 0.5f;
  c.model.tilt = 1.0f;
  now += 1000;
  c.request_tilt(now, 0.5f);
  c.timing.tilt_ms = 400;
  now = run_until_idle(c, now, 40000);
  check(c.idle(), "relay released after shortening tilt time");
}

static void test_reversal_dead_time() {
  MotionController c;
  c.timing = make_timing(60000, 60000);
  uint32_t now = 1000;
  c.request_position(now, 1.0f);
  now += 5000;
  c.tick(now);
  check(c.relay_up(), "running up");
  c.request_position(now, 0.0f);
  check(!c.relay_up() && !c.relay_down(), "reversal opens the relay first");
  now += 490;
  c.tick(now);
  check(!c.relay_down(), "down still off at 490 ms");
  now += 10;
  c.tick(now);
  check(c.relay_down(), "down on at 500 ms");
  c.request_stop(now + 5000);
  now += 5000;

  // Stop then reverse, which upstream did with no delay at all.
  c.tick(now);
  check(c.idle(), "stopped");
  now += 100;
  c.request_position(now, 1.0f);
  check(!c.relay_up(), "stop then reverse waits");
  now += 390;
  c.tick(now);
  check(!c.relay_up(), "still waiting at 490 ms after stop");
  now += 10;
  c.tick(now);
  check(c.relay_up(), "up on 500 ms after stop");

  // Same direction after a stop doesn't need to wait.
  c.request_stop(now + 1000);
  now += 1100;
  c.tick(now);
  c.request_position(now, 1.0f);
  check(c.relay_up(), "same direction restarts at once");
}

static void test_min_command_length() {
  MotionController c;
  c.timing = make_timing(60000, 60000);
  uint32_t now = 1000;
  c.request_position(now, 1.0f);
  now += 50;
  c.request_stop(now);
  check(c.relay_up(), "relay held for the minimum command");
  now += 140;
  c.tick(now);
  check(c.relay_up(), "still held at 190 ms");
  now += 10;
  c.tick(now);
  check(!c.relay_up() && c.idle(), "released at 200 ms");
}

static void test_requests_at_ends() {
  MotionController c;
  c.timing = make_timing(60000, 60000);
  check(c.request_position(1000, 0.0f), "close at the model's bottom still runs");
  uint32_t now = run_motor(c, 1000, 0);
  near(c.last_run_ms() / 1000.0f, 4.0f, 0.1f, "stops when the motor never starts");
  check(c.take_run_end() == RunEnd::AT_LIMIT, "already at the lower limit");

  c.model.position = 1.0f;
  c.model.tilt = 0.35f;
  now += 1000;
  check(c.request_position(now, 1.0f), "open at top with slats part open moves");
  now = run_motor(c, now, 1000);
  near(c.model.tilt, 1.0f, 0.0001f, "a cut at the top anchors the slats");
  c.take_run_end();

  now += 1000;
  check(c.request_position(now, 1.0f), "open when fully open still runs");
  now = run_motor(c, now, 0);
  near(c.last_run_ms() / 1000.0f, 4.0f, 0.1f, "top check reports inside its own run");
  near(c.model.position, 1.0f, 0.0001f, "top check leaves the position");
}

static void test_tilt_keeps_position() {
  MotionController c;
  c.timing = make_timing(40000, 40000);
  c.model.position = 0.5f;
  c.model.tilt = 0.0f;
  uint32_t now = 1000;
  c.request_tilt(now, 0.42f);
  now = run_until_idle(c, now);
  near(c.model.tilt, 0.42f, 0.0001f, "tilt lands exactly");
  near(c.model.position, 0.5f, 0.0001f, "tilt doesn't creep position");
  for (int i = 0; i < 50; i++) {
    now += 1000;
    c.request_tilt(now, i % 2 ? 0.42f : 0.8f);
    now = run_until_idle(c, now);
  }
  near(c.model.position, 0.5f, 0.0001f, "no creep after 50 flips");
}

static void test_time_limit() {
  MotionController c;
  c.timing = make_timing(40000, 40000);
  uint32_t now = 1000;
  c.request_position(now, 1.0f);
  const uint32_t end = run_motor(c, now, 0, false);
  check(c.idle(), "time limit releases the relay with a dead meter");
  near((end - now) / 1000.0f, 40.0f + 0.2f + 15.0f, 0.5f, "stops at a whole run plus margin");
  check(c.take_run_end() == RunEnd::TIME_LIMIT, "reported as a time limit");
}

static void test_homing() {
  MotionController c;
  c.timing = make_timing(40000, 40000);
  uint32_t now = 1000;

  // The time model is 3 s short: the relay stays on until the motor stops, then snaps.
  c.request_position(now, 1.0f);
  now = run_motor(c, now, 43000);
  check(c.take_run_end() == RunEnd::AT_LIMIT, "full open ends at the limit");
  near(c.model.position, 1.0f, 0.0001f, "position anchored");
  near(c.model.tilt, 1.0f, 0.0001f, "tilt anchored");
  check(c.home_error_valid(), "home error reported");
  near(c.home_error_ms() / 1000.0f, 3.0f, 0.6f, "home error is the time the model was short");

  // A stop halfway up may be an obstacle, so it must not become the top.
  now += 1000;
  c.model.position = 0.0f;
  c.model.tilt = 0.0f;
  c.request_position(now, 1.0f);
  now = run_motor(c, now, 15000);
  check(c.take_run_end() == RunEnd::MID_TRAVEL_CUT, "mid-travel stop is a fault");
  check(c.model.position < 0.5f, "mid-travel stop isn't anchored");
  check(c.idle() && !c.relay_up(), "mid-travel stop releases the relay");

  // No power from far away from an end: motor protection, not the limit.
  now += 1000;
  c.request_position(now, 1.0f);
  now = run_motor(c, now, 0);
  check(c.take_run_end() == RunEnd::NO_POWER, "no power mid-travel is a fault");
  check(c.model.position < 0.85f, "no power mid-travel isn't anchored");

  // A partial move that meets the stop near the end anchors there.
  now += 1000;
  c.model.position = 0.9f;
  c.model.tilt = 1.0f;
  c.request_position(now, 0.99f);
  c.timing.open_ms = 170000;  // too slow to arrive first
  now = run_motor(c, now, 3000);
  check(c.take_run_end() == RunEnd::AT_LIMIT, "partial move anchors in the end zone");
  near(c.model.position, 1.0f, 0.0001f, "at the top");
  c.timing = make_timing(40000, 40000);

  // A tilt is timed only, whatever the meter says.
  now += 1000;
  c.model.position = 0.5f;
  c.timing.tilt_ms = 6000;
  c.model.tilt = 0.0f;
  c.request_tilt(now, 1.0f);
  now = run_motor(c, now, 0);
  check(c.take_run_end() == RunEnd::ARRIVED, "tilt ignores the detector");
  near(c.model.position, 0.5f, 0.0001f, "tilt didn't anchor");
}

static void calibrate(MotionController &c, TravelCalibration &cal, uint32_t &now, uint32_t down_ms, uint32_t up_ms,
                      uint32_t top_ms = 2000) {
  cal.start(now, c);
  int guard = 0;
  while (cal.active() && guard++ < 20) {
    // Each phase runs until the controller goes idle, then the sequence starts the next.
    const bool timing_down = c.busy_down();
    const uint32_t motor = guard == 1 ? top_ms : (timing_down ? down_ms : up_ms);
    now = run_motor(c, now, motor);
    const RunEnd end = c.take_run_end();
    if (end != RunEnd::NONE)
      cal.on_run_end(now, end, c);
    for (int i = 0; i < 120 && cal.active() && c.idle(); i++) {
      now += 10;
      c.tick(now);
      cal.tick(now, c);
    }
  }
}

static void test_calibration() {
  MotionController c;
  c.timing = make_timing(54000, 54000);
  TravelCalibration cal;
  uint32_t now = 1000;
  c.model.position = 0.5f;
  c.model.tilt = 0.4f;
  calibrate(c, cal, now, 33400, 33000);
  check(!cal.active(), "calibration finished");
  check(cal.take_timing_changed(), "times changed");
  near(c.timing.open_ms / 1000.0f, 33.0f, 0.6f, "open measured with far too long old times");
  near(c.timing.close_ms / 1000.0f, 33.4f, 0.6f, "close measured");
  near(c.model.position, 0.5f, 0.001f, "position returned");
  near(c.model.tilt, 0.4f, 0.001f, "tilt returned");

  // Old times far too short also work: the model reaches its end long before the cut.
  c.timing = make_timing(20000, 20000);
  c.model.position = 1.0f;
  c.model.tilt = 1.0f;
  now += 1000;
  calibrate(c, cal, now, 33400, 33000, 0);
  check(cal.take_timing_changed(), "times changed from too short");
  near(c.timing.open_ms / 1000.0f, 33.0f, 0.6f, "open measured");

  // Rejections leave the times alone.
  c.timing = make_timing(40000, 40000);
  c.model.position = 1.0f;
  c.model.tilt = 1.0f;
  now += 1000;
  calibrate(c, cal, now, 30000, 45000, 0);
  check(!cal.take_timing_changed(), "25% difference rejected");
  check(c.timing.open_ms == 40000 && c.timing.close_ms == 40000, "times kept");
  check(std::strstr(cal.take_message(), "rejected") != nullptr, "says rejected");

  c.model.position = 1.0f;
  c.model.tilt = 1.0f;
  now += 1000;
  calibrate(c, cal, now, 0, 33000, 0);
  check(!cal.take_timing_changed(), "motor never ran on the timed run");
  check(c.timing.open_ms == 40000, "times kept after no cut");

  // With no true position to go back to, it stays at the top.
  c.timing = make_timing(40000, 40000);
  c.model.position = 0.5f;
  c.model.tilt = 0.0f;
  c.mark_position_unknown();
  now += 1000;
  calibrate(c, cal, now, 33400, 33000, 3000);
  check(cal.take_timing_changed(), "times changed from an unknown start");
  near(c.model.position, 1.0f, 0.001f, "left fully open");
  near(c.model.tilt, 1.0f, 0.001f, "slats open");
  check(std::strstr(cal.take_message(), "fully open") != nullptr, "says where it left the blind");

  // A cover command aborts it and that command runs.
  c.model.position = 0.5f;
  now += 1000;
  cal.start(now, c);
  now += 1000;
  c.tick(now);
  cal.abort();
  c.request_position(now, 0.2f);
  check(!cal.active(), "aborted");
  check(!c.relay_up(), "the new command takes over");
}

static uint32_t limit_step(MotionController &c, LimitSequence &seq, uint32_t now, uint32_t ms, float watts = -1.0f) {
  const uint32_t end = now + ms;
  while (now < end) {
    now += 10;
    c.tick(now);
    const RunEnd run_end = c.take_run_end();
    if (run_end != RunEnd::NONE)
      seq.on_run_end(now, run_end, c);
    seq.tick(now, c);
    check(!(c.relay_up() && c.relay_down()), "controller never closes both");
    if (seq.relay_up() || seq.relay_down())
      check(c.idle(), "controller is idle while the sequence holds a relay");
    if (watts >= 0.0f && now % 250 == 0)
      seq.on_power(now, watts);
  }
  return now;
}

// Parks the blind and waits for the moment both relays close.
static uint32_t limit_enter(MotionController &c, LimitSequence &seq, uint32_t now, LimitKind kind) {
  seq.start(now, kind, c);
  for (int i = 0; i < 20000 && !(seq.relay_up() && seq.relay_down()); i++) {
    now = limit_step(c, seq, now, 10);
  }
  return now;
}

static void test_limit_sequence() {
  MotionController c;
  c.timing = make_timing(33000, 33400);
  LimitSequence seq;
  uint32_t now = 1000;

  // Parking ends on an up move at mid height, from above and from below.
  c.model.position = 0.8f;
  c.model.tilt = 1.0f;
  seq.start(now, LimitKind::LOWER, c);
  check(c.relay_down(), "parking from above goes down first");
  while (!c.relay_up() && now < 100000)
    now = limit_step(c, seq, now, 10);
  check(c.relay_up(), "then up");
  bool last_up = true;
  bool both = false;
  uint32_t stopped_at = 0;
  while (!both && now < 200000) {
    if (c.relay_down())
      last_up = false;
    if (c.relay_up())
      stopped_at = now;
    now = limit_step(c, seq, now, 10);
    both = seq.relay_up() && seq.relay_down();
  }
  check(both, "both relays close after parking");
  check(last_up, "the last move before entry went up");
  check(now - stopped_at >= LIMIT_PARK_PAUSE_MS, "long pause before entry");
  near(c.model.position, 0.5f, 0.001f, "parked at mid height");

  seq.on_power(now + 5100, NAN);
  check(seq.relay_up() && seq.relay_down(), "a bad reading is not an acknowledgement");

  // Power before 5 s is ignored: releasing early would miss setting mode.
  const uint32_t entered = now;
  now = limit_step(c, seq, now, 4500, 100.0f);
  check(seq.relay_up() && seq.relay_down(), "both held through early power");
  check(!seq.store(now), "store refused during entry");
  while (seq.relay_up() && now < entered + 9000)
    now = limit_step(c, seq, now, 10, 100.0f);
  check(!seq.relay_up() && !seq.relay_down(), "released after the jog");
  near((now - entered) / 1000.0f, 8.0f, 0.02f, "power doesn't cut the jog short");
  check(std::strstr(seq.take_message(), "In setting mode") != nullptr, "reports the acknowledgement");

  // No store before a move, or the parked height becomes the limit.
  check(!seq.store(now), "store refused before any move");
  check(seq.run(now), "run accepted");
  check(!seq.relay_down(), "run waits the reversal gap after both relays");
  now = limit_step(c, seq, now, 500, 100.0f);
  check(seq.relay_down() && !seq.relay_up(), "lower limit drives down only");
  now = limit_step(c, seq, now, 5000, 100.0f);
  seq.stop(now, c);
  check(!seq.relay_down(), "stop cuts the relay");
  check(seq.store(now), "store accepted after a move");
  check(!seq.relay_up(), "store press waits the reversal gap");
}

static void test_limit_store() {
  MotionController c;
  c.timing = make_timing(33000, 33400);
  uint32_t now = 1000;

  // Lower limit: drive down, store with a single up press.
  {
    LimitSequence seq;
    c.model.position = 0.2f;
    c.model.tilt = 0.0f;
    now = limit_enter(c, seq, now, LimitKind::LOWER);
    now = limit_step(c, seq, now, 8100, 100.0f);
    seq.run(now);
    now = limit_step(c, seq, now, 3000, 100.0f);
    seq.stop(now, c);
    now = limit_step(c, seq, now, 100);
    check(seq.store(now), "store accepted");
    check(!seq.relay_up(), "store press waits the reversal gap");
    uint32_t press_ms = 0;
    for (int i = 0; i < 400 && seq.active(); i++) {
      now = limit_step(c, seq, now, 10, 100.0f);
      if (seq.relay_up())
        press_ms += 10;
      check(!seq.relay_down(), "store never drives down on a lower limit");
    }
    near(press_ms / 1000.0f, 1.0f, 0.05f, "one store press");
    check(!seq.active(), "sequence over");
    check(!seq.store(now) && !seq.run(now), "no more presses after the store");
    near(c.model.position, 0.5f, 0.001f, "store doesn't claim the blind reached the bottom");
    check(std::strstr(seq.take_message(), "Store press done") != nullptr, "reports the press, not a stored limit");
    check(seq.take_store_finished(), "a finished store is reported, so Service Mode can end");
    check(!seq.take_store_finished(), "reported once");

    // The next close is anchored wherever the motor stops, without a false fault.
    now += 1000;
    c.request_position(now, 0.0f);
    now = run_motor(c, now, 1000);
    check(c.take_run_end() == RunEnd::AT_LIMIT, "a stop after the sequence is the limit");
    check(!c.home_error_valid(), "no home error from an unknown start");
    near(c.model.position, 0.0f, 0.001f, "anchored at the bottom");

    // That applies to one stop only.
    now += 1000;
    c.model.position = 0.5f;
    c.request_position(now, 0.0f);
    now = run_motor(c, now, 3000);
    check(c.take_run_end() == RunEnd::MID_TRAVEL_CUT, "later mid-travel stops are faults again");
  }

  // Upper limit: drive up, store with a down press.
  {
    LimitSequence seq;
    c.model.position = 0.2f;
    c.model.tilt = 0.0f;
    now += 1000;
    now = limit_enter(c, seq, now, LimitKind::UPPER);
    now = limit_step(c, seq, now, 8100, 100.0f);
    seq.nudge(now);
    now = limit_step(c, seq, now, 500);
    check(seq.relay_up() && !seq.relay_down(), "upper limit drives up only");
    now = limit_step(c, seq, now, 510);
    check(!seq.relay_up(), "nudge ends itself");
    seq.store(now);
    now = limit_step(c, seq, now, 500);
    check(seq.relay_down() && !seq.relay_up(), "upper limit stores with down");
    now = limit_step(c, seq, now, 1100);
    check(std::strstr(seq.take_message(), "no movement seen") != nullptr, "no store movement seen is reported");
  }

  // The cap: nothing holds both relays long enough to reach the factory reset.
  {
    now += 1000;
    c.request_position(now, 0.0f);  // a full run gives the model a true position after the store
    now = run_motor(c, now, 3000);
    c.take_run_end();
    LimitSequence seq;
    c.model.position = 0.2f;
    c.model.tilt = 0.0f;
    now += 1000;
    now = limit_enter(c, seq, now, LimitKind::LOWER);
    uint32_t both_ms = 0;
    for (int i = 0; i < 2000; i++) {
      now = limit_step(c, seq, now, 10, 0.5f);
      if (seq.relay_up() && seq.relay_down())
        both_ms += 10;
    }
    near(both_ms / 1000.0f, LIMIT_ENTRY_HOLD_MS / 1000.0f, 0.05f, "both relays stop at the cap with no acknowledgement");
    check(LIMIT_ENTRY_HOLD_MS < 15000, "well short of the factory reset");
    check(std::strstr(seq.take_message(), "No movement") != nullptr, "says no acknowledgement was seen");
    check(seq.active(), "still waiting for a decision");
    check(seq.run(now), "run still possible if the blind did jog");
    now = limit_step(c, seq, now, 600, 0.5f);
    check(seq.relay_down(), "driving");
    now = limit_step(c, seq, now, LIMIT_NO_MOVE_MS, 0.5f);
    check(!seq.relay_down(), "no movement stops the drive");
    check(std::strstr(seq.take_message(), "factory locked") != nullptr, "says it may be factory locked");
    seq.cancel(now, c);
    check(!seq.active() && !seq.relay_up() && !seq.relay_down(), "cancel ends it with relays off");
    check(std::strstr(seq.take_message(), "setting mode") != nullptr, "warns about setting mode");
    c.request_position(now, 0.8f);
    check(!c.relay_up(), "a normal move waits the reversal gap after the sequence");
    now += REVERSAL_DEAD_TIME_MS;
    c.tick(now);
    check(c.relay_up(), "then runs");
    c.request_stop(now + 300);
    now += 800;
    c.tick(now);
  }

  // After a store the height is unknown, so parking finds the limit first and reaches true mid-height.
  {
    LimitSequence seq;
    c.model.position = 0.5f;
    c.model.tilt = 1.0f;
    c.mark_position_unknown();
    now += 1000;
    seq.start(now, LimitKind::UPPER, c);
    check(c.relay_down(), "goes down to find the limit first");
    uint32_t t0 = now;
    while (c.relay_down() && now < t0 + 60000) {
      now += 10;
      c.tick(now);
      if ((now - t0) % 500 == 0)
        c.on_power(now, Direction::DOWN, now - t0 < 3000 ? 100.0f : 0.5f);
    }
    check(!c.relay_down(), "the limit stops it");
    near(c.model.position, 0.0f, 0.001f, "position is true again");
    now = limit_step(c, seq, now, 700);
    check(seq.active() && c.relay_up(), "parking continues upward");
    while (c.relay_up() && now < t0 + 100000)
      now = limit_step(c, seq, now, 10);
    near(c.model.position, 0.5f, 0.001f, "parked at true mid height");
    check(seq.active(), "on its way to setting mode");
    seq.cancel(now, c);
    now = limit_step(c, seq, now, 100);
    seq.take_message();
    c.model.position = 0.5f;
    c.request_position(now, 0.0f);
    now = run_motor(c, now, 3000);
    c.take_run_end();
  }

  // After a cancel inside setting mode, nothing is pressed until the motor is entered again.
  {
    LimitSequence seq;
    c.model.position = 0.2f;
    c.model.tilt = 0.0f;
    now += 1000;
    now = limit_enter(c, seq, now, LimitKind::LOWER);
    now = limit_step(c, seq, now, 1000);
    seq.cancel(now, c);
    seq.take_message();
    now += 1000;
    seq.start(now, LimitKind::LOWER, c);
    check(!c.relay_up() && !c.relay_down(), "no parking move after a cancel in setting mode");
    check(std::strstr(seq.take_message(), "stays put") != nullptr, "says why it isn't parking");
    now = limit_step(c, seq, now, LIMIT_PARK_PAUSE_MS - 20);
    check(!seq.relay_up() && !c.relay_up() && !c.relay_down(), "still pressing nothing");
    now = limit_step(c, seq, now, 40);
    check(seq.relay_up() && seq.relay_down(), "enters again after the pause");
    now = limit_step(c, seq, now, LIMIT_ENTRY_HOLD_MS + 100, 100.0f);
    check(seq.ready() && !seq.can_store(), "ready, but no store before a move");
    seq.cancel(now, c);
    seq.take_message();
    c.model.position = 0.5f;
    c.request_position(now, 0.0f);
    now = run_motor(c, now, 3000);
    c.take_run_end();
  }

  // Stop during parking ends it without touching the motor's settings.
  {
    LimitSequence seq;
    c.model.position = 0.8f;
    now += 1000;
    seq.start(now, LimitKind::UPPER, c);
    now = limit_step(c, seq, now, 1000);
    seq.stop(now, c);
    now = limit_step(c, seq, now, 100);
    check(!seq.active() && c.idle(), "stop during parking ends it");
    check(std::strstr(seq.take_message(), "unchanged") != nullptr, "limits unchanged");
  }
}

static void test_validation() {
  check(make_timing(39000, 38500).valid(), "normal times are valid");
  check(!make_timing(1000, 38500).valid(), "open shorter than tilt plus activation is rejected");
  check(!make_timing(39000, 6000).valid(), "under 5 s net travel is rejected");
  Timing t = make_timing(39000, 38500);
  t.tilt_ms = 0;
  check(!t.valid(), "zero tilt is rejected");
}

struct Expect {
  const char *name;
  PowerEvent event;
  float min_s;
  float max_s;
};

static void test_t1_traces() {
  const Expect expected[] = {
      {"16:22:31", PowerEvent::NONE, 0, 0},       // run 1: stopped by the model, no cut
      {"16:24:10", PowerEvent::CUT, 7.4f, 8.5f},  // run 2: the "no cut" burst
      {"16:24:27", PowerEvent::NEVER_RAN, 0, 0},  // at the lower limit
      {"16:24:43", PowerEvent::NEVER_RAN, 0, 0},  // at the lower limit
      {"16:25:44", PowerEvent::NONE, 0, 0},       // run 3: stopped by the model
      {"16:27:20", PowerEvent::CUT, 3.0f, 4.0f},  // run 4: top
      {"16:28:53", PowerEvent::NONE, 0, 0},       // tilt
      {"16:29:08", PowerEvent::NONE, 0, 0},       // tilt over-run
      {"16:30:07", PowerEvent::NONE, 0, 0},       // tilt
      {"16:30:19", PowerEvent::NONE, 0, 0},       // tilt over-run
      {"16:31:26", PowerEvent::CUT, 8.4f, 9.5f},  // run 10
  };
  const unsigned traces = sizeof(T1_TRACES) / sizeof(T1_TRACES[0]);
  check(traces == sizeof(expected) / sizeof(expected[0]), "trace count");
  for (unsigned i = 0; i < traces; i++) {
    const Trace &trace = T1_TRACES[i];
    CutDetector d;
    d.reset();
    d.readings_below = 2;  // the logs are 1 Hz, so two readings span what three do at 500 ms
    PowerEvent event = PowerEvent::NONE;
    for (unsigned s = 0; s < trace.count && event == PowerEvent::NONE; s++)
      event = d.feed(trace.samples[s].t_ms, trace.samples[s].watts);
    printf("  %s %-4s relay %6.2f s  event %-9s cut at %5.2f s\n", trace.name, trace.channel, trace.relay_ms / 1000.0f,
           event == PowerEvent::CUT ? "CUT" : (event == PowerEvent::NEVER_RAN ? "NEVER_RAN" : "none"),
           d.cut_ms() / 1000.0f);
    if (i >= sizeof(expected) / sizeof(expected[0]))
      continue;
    check(std::strcmp(trace.name, expected[i].name) == 0, "trace order");
    check(event == expected[i].event, trace.name);
    if (expected[i].event == PowerEvent::CUT) {
      check(d.cut_ms() / 1000.0f >= expected[i].min_s && d.cut_ms() / 1000.0f <= expected[i].max_s, "cut time");
    }
  }
}

int main() {
  test_position_scale();
  test_close_longer_than_open();
  test_duration_change_mid_move();
  test_reversal_dead_time();
  test_min_command_length();
  test_requests_at_ends();
  test_tilt_keeps_position();
  test_time_limit();
  test_homing();
  test_calibration();
  test_limit_sequence();
  test_limit_store();
  test_validation();
  test_t1_traces();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
