#pragma once
// NAP: night speed-sign short exposure (param NAPSpeedSignShortExposure, default Off).
// Road camera only. Trades exposure time for analog gain at the SAME EV, so the
// frame brightness modeld sees is preserved; only sensor noise rises. Bounded:
// exposure is never cut below 1/4 of the AE choice-equivalent cap and analog gain
// never rises more than NAP_SE_MAX_GAIN_MULT over the AE choice.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>

constexpr float NAP_SE_MAX_GAIN_MULT = 4.0f;   // at most +2 stops of gain
constexpr int NAP_SE_CAP_DIV = 4;              // cap = exposure_time_max / 4
constexpr int64_t NAP_SE_MAX_WINDOW_NS = 3000000000LL;  // a request lasts <= 3 s
constexpr int NAP_SE_MAX_ACTIVE_FRAMES = 80;   // 4 s at 20 Hz, then cooldown
constexpr int NAP_SE_COOLDOWN_FRAMES = 80;
constexpr const char *NAP_SE_REQUEST_PATH = "/dev/shm/nap_speedsign_exp";

struct NapExpChoice { int t; int g; };

// Pure: given the AE's (t, g) for desired_ev, pick a shorter t with higher gain
// at (approximately) the same EV. Never returns a longer t or lower gain.
inline NapExpChoice nap_short_exposure(float desired_ev, int t, int g, int cap_t, int t_min,
                                       const float *gains, int g_max, float gain_factor) {
  if (t <= cap_t || desired_ev <= 0.f) return {t, g};
  const float limit = gains[g] * gain_factor * NAP_SE_MAX_GAIN_MULT;
  int best = g;
  for (int gg = g; gg <= g_max; gg++) {
    float gain = gains[gg] * gain_factor;
    if (gain > limit + 1e-6f) break;
    best = gg;
    int tt = (int)std::lround(desired_ev / gain);
    if (tt <= cap_t) return {std::clamp(tt, t_min, t), gg};
  }
  float gain = gains[best] * gain_factor;
  return {std::clamp((int)std::lround(desired_ev / gain), t_min, t), best};
}

inline int64_t nap_se_mono_ns() {
  timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

// Request file holds a CLOCK_MONOTONIC deadline in ns (Python time.monotonic_ns()).
inline bool nap_se_request_live(int64_t now_ns) {
  FILE *f = fopen(NAP_SE_REQUEST_PATH, "r");
  if (!f) return false;
  long long dl = 0;
  int n = fscanf(f, "%lld", &dl);
  fclose(f);
  return n == 1 && dl > now_ns && dl - now_ns <= NAP_SE_MAX_WINDOW_NS;
}

// Frame-counted gate: bounded active run, then a forced cooldown.
struct NapSeGate {
  int active = 0, cooldown = 0;
  bool step(bool want) {
    if (cooldown > 0) { cooldown--; return false; }
    if (!want) { active = 0; return false; }
    if (++active > NAP_SE_MAX_ACTIVE_FRAMES) { active = 0; cooldown = NAP_SE_COOLDOWN_FRAMES; return false; }
    return true;
  }
};
