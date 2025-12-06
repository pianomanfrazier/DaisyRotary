#pragma once

#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

// =====================================================================
// Public data structures
// =====================================================================

struct Stereo
{
    float l;
    float r;
};

template <size_t MAX_DELAY>
struct RotorBand
{
    DelayLine<float, MAX_DELAY> delayL;
    DelayLine<float, MAX_DELAY> delayR;

    void Init()
    {
        delayL.Init();
        delayR.Init();
    }
};

struct Rotor
{
    float ampDepth;
    float panDepth;
    float baseDelay;
    float delayDepth;
    float micOffset;
};

struct RotorMotion
{
    float slowSpeed;
    float fastSpeed;

    float accel;
    float decel;

    float speed;
    float targetSpeed;
    float phase;
};

enum RotorMode
{
    MODE_SLOW,
    MODE_STOP,
    MODE_FAST,
};

static const size_t LESLIE_MAX_DELAY_SAMPLES = 9600;
static const size_t LESLIE_NUM_REF_TAPS      = 3;

struct LeslieState
{
    float sr;
    float dt;

    RotorBand<LESLIE_MAX_DELAY_SAMPLES> hornBand;
    RotorBand<LESLIE_MAX_DELAY_SAMPLES> drumBand;

    Rotor       hornRotor;
    Rotor       drumRotor;
    RotorMotion hornMotion;
    RotorMotion drumMotion;

    Svf filter;

    // Reflections
    DelayLine<float, LESLIE_MAX_DELAY_SAMPLES> cabDelayL[LESLIE_NUM_REF_TAPS];
    DelayLine<float, LESLIE_MAX_DELAY_SAMPLES> cabDelayR[LESLIE_NUM_REF_TAPS];
    Svf cabRefLpL;
    Svf cabRefLpR;

    bool      enableReflections;
    RotorMode mode;
};

// =====================================================================
// Public API
// =====================================================================

// Initialize all internal state
void Leslie_Init(LeslieState& ls, float sampleRate);

// Process one mono sample → stereo Leslie output
Stereo Leslie_ProcessSample(LeslieState& ls, float inSample);

void UpdateRotorParamsFromKnobs(LeslieState& ls,
                                float slowKnob,   // 0–1
                                float fastKnob,   // 0–1
                                float accelKnob,  // 0–1
                                float decelKnob);  // 0–1

void UpdateVoicingFromSpread(LeslieState& ls, float spreadKnob);