#pragma once
#include "daisysp.h"

float Gain(float norm, float minDb, float maxDb);

class Grit
{
  public:
    void Init(float sample_rate);

    // knob01 = your drive knob (0..1)
    void SetAmount(float knob01);

    // caller guards with if(driveOn)
    float Process(float x);

  private:
    // --- tone shaping
    daisysp::Svf hpf_;
    daisysp::Svf lpf_;
    daisysp::Overdrive od_;

    // cached per-block params
    float drive_param_ = 0.0f;   // internal drive (0..1)
    float mix_         = 0.7f;   // wet mix
    float trim_        = 1.0f;   // linear gain compensation

    // ---- TUNABLES (start here)
    static constexpr float kHPF_Hz = 120.0f;     // rumble control
    static constexpr float kLPF_Hz = 5000.0f;    // fizz control

    // knob -> “useful drive” mapping
    static constexpr float kStart  = 0.00f;      // make the whole knob do something
    static constexpr float kWidth  = 1.00f;

    // clamp the nastiest top end of DaisySP Overdrive
    static constexpr float kMaxDrive = 0.7f;    // try 0.70–0.90

    // your proven “volume stays sane” curve endpoints
    static constexpr float kTrimDbAt0 = +4.0f;
    static constexpr float kTrimDbAt1 = -40.0f;

    // mix range (optional: tie to knob; or just keep 0.7 constant)
    static constexpr float kMixMin = 0.55f;
    static constexpr float kMixMax = 0.75f;
};
