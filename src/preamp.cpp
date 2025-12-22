#include "preamp.h"
#include <cmath>

float Gain(float norm, float minDb, float maxDb)
{
    if(norm < 0.0f) norm = 0.0f;
    if(norm > 1.0f) norm = 1.0f;

    float db = minDb + (maxDb - minDb) * norm;
    return powf(10.0f, db / 20.0f);
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
}

void Grit::SetAmount(float knob01)
{
    float t = knob01;

    // Ease-out curve: lots of action early, but no infinite slope at 0
    float shaped = t * (2.0f - t);   // 0..1

    // If you have a toggle, you usually don’t need minDrive.
    // Start at 0 and let the knob actually control the onset.
    float drive_param = kMaxDrive * shaped;
    od_.SetDrive(drive_param);

    float trimDb = kTrimDbAt0 + (kTrimDbAt1 - kTrimDbAt0) * shaped;
    trim_ = powf(10.0f, trimDb / 20.0f);
}

float Grit::Process(float x)
{
    float dry = x;

    // Pre-drive HPF (kills rumble going into nonlinearity)
    hpf_.Process(x);
    float hp = hpf_.High();

    // Distort only high-passed content
    float wet = od_.Process(hp);

    // Post-drive LPF (tames fizz)
    lpf_.Process(wet);
    wet = lpf_.Low();

    // Parallel blend + your predictable trim
    float y = mix_ * wet + (1.0f - mix_) * dry;
    return y * trim_;
}
