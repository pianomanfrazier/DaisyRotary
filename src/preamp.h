#pragma once
#include "daisysp.h"

float Gain(float norm, float minDb, float maxDb);

// Simple "grit" stage for organ:
// HPF -> Overdrive -> LPF, parallel blend, auto level match.
class Grit
{
  public:
    void Init(float sample_rate);

    // Call once per audio block (or when knob changes)
    void SetAmount(float knob01);

    // Call per-sample when enabled (caller guards)
    float Process(float x);

    // Optional: call per-block or per-sample when disabled
    // to gently return trim toward unity.
    void Relax();

  private:
    static float Clamp01(float x);
    static float Clamp(float x, float lo, float hi);

    // ---- Tweakables (start here; adjust by ear)
    static constexpr float kHPF_Hz   = 120.0f;  // rumble control
    static constexpr float kLPF_Hz   = 5000.0f; // fizz control

    static constexpr float kStart    = 0.20f;   // knob start of useful sweep
    static constexpr float kWidth    = 0.60f;   // knob width of useful sweep
    static constexpr float kMaxDrive = 0.75f;   // clamp internal drive

    static constexpr float kMixMin   = 0.15f;
    static constexpr float kMixMax   = 0.80f;

    static constexpr float kEnvA     = 0.001f;  // env follower speed
    static constexpr float kTrimA    = 0.002f;  // trim smoothing
    static constexpr float kTrimMin  = 0.25f;
    static constexpr float kTrimMax  = 4.00f;

    daisysp::Svf       hpf_;
    daisysp::Svf       lpf_;
    daisysp::Overdrive od_;

    float t_       = 0.0f;
    float mix_     = 0.0f;
    float dry_env_ = 0.0f;
    float wet_env_ = 0.0f;
    float trim_    = 1.0f;
};
