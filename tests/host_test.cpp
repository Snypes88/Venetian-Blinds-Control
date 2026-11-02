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
  now = run_until_idle(c, now);
  now += 1000;
  c.request_position(now, 0.0f);
  const uint32_t on = relay_time(c, now);
  near(on / 1000.0f, 50.0f, 0.1f, "full close takes the whole close time");
}

static void test_duration_change_mid_move() {
  // Upstream left the relay on forever here.
  MotionController c;
  c.timing = make_timing(60000, 60000);
  uint32_t now = 1000;
  c.request_position(now, 1.0f);
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
  near(c.model.position, 1.0f, 0.0001f, "arrived at the top");

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
  check(!c.request_position(1000, 0.0f), "close at bottom with closed slats is no move");
  c.model.tilt = 0.4f;
  check(c.request_position(2000, 0.0f), "close at bottom with open slats moves");
  uint32_t now = run_until_idle(c, 2000);
  near(c.model.tilt, 0.0f, 0.0001f, "slats closed");

  c.model.position = 1.0f;
  c.model.tilt = 0.35f;
  now += 1000;
  check(c.request_position(now, 1.0f), "open at top with slats part open moves");
  now = run_until_idle(c, now);
  near(c.model.tilt, 1.0f, 0.0001f, "slats fully open");

  now += 1000;
  check(c.request_position(now, 1.0f), "open when fully open probes the top");
  const uint32_t on = relay_time(c, now);
  near(on / 1000.0f, TOP_PROBE_MS / 1000.0f, 0.05f, "probe length");
  near(c.model.position, 1.0f, 0.0001f, "probe leaves the position");
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
  now += 5000;
  c.tick(now);
  c.timing.open_ms = 170000;  // the model can no longer arrive within the limit set at the start
  const uint32_t end = run_until_idle(c, now, 400000);
  check(c.idle(), "time limit releases the relay");
  near((end - 1000) / 1000.0f, 40.0f - 1.75f + 1.75f + 15.0f, 0.5f, "stops at expected time plus margin");
  check(c.take_run_end() == RunEnd::TIME_LIMIT, "reported as a time limit");
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
  test_validation();
  test_t1_traces();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
