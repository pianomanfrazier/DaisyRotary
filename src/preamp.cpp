#include "preamp.h"
#include <cmath>

float Gain(float norm, float minDb, float maxDb)
{
    if(norm < 0.0f) norm = 0.0f;
    if(norm > 1.0f) norm = 1.0f;

    float db = minDb + (maxDb - minDb) * norm;
    return powf(10.0f, db / 20.0f);
}

float Grit::Clamp01(float x)
{
    if(x < 0.0f) return 0.0f;
    if(x > 1.0f) return 1.0f;
    return x;
}

float Grit::Clamp(float x, float lo, float hi)
{
    if(x < lo) return lo;
    if(x > hi) return hi;
    return x;
}

void Grit::Init(float sample_rate)
{
    hpf_.Init(sample_rate);
    hpf_.SetRes(0.7f);
    hpf_.SetFreq(kHPF_Hz);

    lpf_.Init(sample_rate);
    lpf_.SetRes(0.7f);
    lpf_.SetFreq(kLPF_Hz);

    od_.SetDrive(0.0f);

    t_       = 0.0f;
    mix_     = kMixMin;
    dry_env_ = 0.0f;
    wet_env_ = 0.0f;
    trim_    = 1.0f;
}

void Grit::SetAmount(float knob01)
{
    float k = Clamp01(knob01);

    // Stretch "useful" range across the knob and clamp the rest
    float t = (k - kStart) / kWidth;
    t = Clamp01(t);

    // More resolution in subtle zone
    // t = t * t;

    t_   = t;
    mix_ = kMixMin + (kMixMax - kMixMin) * t_;

    // Keep out of the fizzy top end
    od_.SetDrive(kMaxDrive * t_);
}

float Grit::Process(float x)
{
    float dry = x;

    // Pre-drive high-pass (kills rumble before clipping)
    hpf_.Process(x);
    float hp = hpf_.High();

    // Drive only the high-passed content
    float wet = od_.Process(hp);

    // Post-drive low-pass (tames fizz)
    lpf_.Process(wet);
    wet = lpf_.Low();

    // Add grit "on top" (parallel style)
    float y = dry + mix_ * wet;

    // Auto level match so toggling feels consistent
    dry_env_ += kEnvA * (fabsf(dry) - dry_env_);
    wet_env_ += kEnvA * (fabsf(y)   - wet_env_);

    float target = (wet_env_ > 1e-6f) ? (dry_env_ / wet_env_) : 1.0f;
    target = Clamp(target, kTrimMin, kTrimMax);

    trim_ += kTrimA * (target - trim_);

    return y * trim_;
}

void Grit::Relax()
{
    trim_ += kTrimA * (1.0f - trim_);
}
