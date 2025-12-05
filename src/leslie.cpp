#include "leslie.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

// ---------------------------------------------------------------------
// All your constants (PRE_HP_MIN_FREQ, DRIVE_PRE_MIN, etc.)
// and all the helper functions go here, unchanged,
// **except** anything that referenced `hw` or GPIO.
// ---------------------------------------------------------------------

// (copy all your existing constants: PRE_* ranges, DRIVE_* ranges,
// CAB_REF_DELAY_SECS, CAB_REF_TAP_GAINS, CAB_REF_MIX, etc.)
// =====================================================================
// Constants & configuration
// =====================================================================

// Max delay ~ 9600 samples (at 48kHz ~ 200ms) for rotor Doppler
static const size_t MAX_DELAY_SAMPLES = 9600;
static const float  STOP_SPEED        = 0.0f;

// Crossover freq for drum vs horn
static const float CROSS_OVER_FREQ    = 800.0f;

// Cabinet reflection network
static const size_t CAB_REF_MAX_SAMPLES = 9600;   // ~200 ms max at 48k

// We'll use 3 early reflection taps per channel
static const size_t CAB_REF_NUM_TAPS = 3;

// Per-tap delays (seconds)
static const float CAB_REF_DELAY_SECS[CAB_REF_NUM_TAPS] = {
    4e-3f,   // 4 ms
    9e-3f,   // 9 ms
    15e-3f,  // 15 ms
};

// Per-tap gains
static const float CAB_REF_TAP_GAINS[CAB_REF_NUM_TAPS] = {
    0.40f,
    0.25f,
    0.15f,
};

// Overall wet level
static const float CAB_REF_MIX = 0.6f;

// Darken above this freq for "cab" tone
static const float CAB_REF_TONE_FREQ = 3000.0f;

// -----------------------------
// Helper functions
// -----------------------------

// ApplyPreEq, SoftClip, ApplyTubeStage, SplitDrumHorn,
// ProcessRotorBand, ProcessRotors, ApplyCabinetReflections,
// UpdateRotorTarget/Speed/Phase, UpdateAllRotorMotion,
// Leslie_InitVoicing, Leslie_InitMotion, Leslie_InitFiltersAndBands,
// Leslie_InitCabinetEq, Leslie_InitDrive, Leslie_InitCabinetReflections
//
// -> literally copy/paste from your monolithic file,
//    and replace MAX_DELAY_SAMPLES with LESLIE_MAX_DELAY_SAMPLES,
//    CAB_REF_NUM_TAPS with LESLIE_NUM_REF_TAPS, etc.

// =====================================================================
// Initialization helpers (for LeslieState)
// =====================================================================

void Leslie_InitVoicing(LeslieState& ls)
{
    // Horn voicing
    ls.hornRotor.ampDepth   = 0.7f;
    ls.hornRotor.panDepth   = 0.3f;
    ls.hornRotor.baseDelay  = 0.00040f;
    ls.hornRotor.delayDepth = 0.00030f;
    ls.hornRotor.micOffset  = M_PI / 2.0f;

    // Drum voicing
    ls.drumRotor.ampDepth   = 0.4f;
    ls.drumRotor.panDepth   = 0.15f;
    ls.drumRotor.baseDelay  = 0.00020f;
    ls.drumRotor.delayDepth = 0.00010f;
    ls.drumRotor.micOffset  = M_PI / 2.0f;
}

void Leslie_InitMotion(LeslieState& ls)
{
    // Roughly Leslie-like ballpark:
    // Horn a bit faster than drum
    ls.hornMotion.slowSpeed = 0.6f;  // Hz
    ls.hornMotion.fastSpeed = 7.0f;  // Hz

    ls.drumMotion.slowSpeed = 0.33f; // Hz
    ls.drumMotion.fastSpeed = 5.5f;  // Hz

    // Accel/decel times (seconds) → per-sample coeffs
    float hornAccelTime = 0.8f; // rise
    float hornDecelTime = 2.4f; // fall

    float drumAccelTime = 5.0f; // rise
    float drumDecelTime = 5.5f; // fall

    ls.hornMotion.accel = ls.dt / hornAccelTime;
    ls.hornMotion.decel = ls.dt / hornDecelTime;
    ls.drumMotion.accel = ls.dt / drumAccelTime;
    ls.drumMotion.decel = ls.dt / drumDecelTime;

    ls.hornMotion.speed       = STOP_SPEED;
    ls.hornMotion.targetSpeed = STOP_SPEED;
    ls.hornMotion.phase       = 0.0f;

    ls.drumMotion.speed       = STOP_SPEED;
    ls.drumMotion.targetSpeed = STOP_SPEED;
    ls.drumMotion.phase       = 0.0f;

    ls.mode = MODE_STOP;
}

void Leslie_InitFiltersAndBands(LeslieState& ls)
{
    ls.hornBand.Init();
    ls.drumBand.Init();

    // Crossover
    ls.filter.Init(ls.sr);
    ls.filter.SetFreq(CROSS_OVER_FREQ);
}

void Leslie_InitCabinetReflections(LeslieState& ls)
{
    // Init all tap delays
    for(size_t i = 0; i < CAB_REF_NUM_TAPS; ++i)
    {
        ls.cabDelayL[i].Init();
        ls.cabDelayR[i].Init();

        float refSamples = CAB_REF_DELAY_SECS[i] * ls.sr;
        ls.cabDelayL[i].SetDelay(refSamples);
        ls.cabDelayR[i].SetDelay(refSamples);
    }

    // LPF for summed reflections
    ls.cabRefLpL.Init(ls.sr);
    ls.cabRefLpR.Init(ls.sr);
    ls.cabRefLpL.SetFreq(CAB_REF_TONE_FREQ);
    ls.cabRefLpR.SetFreq(CAB_REF_TONE_FREQ);

    ls.enableReflections = false; // start OFF; button enables it
}

// =====================================================================
// Rotor motion helpers
// =====================================================================

inline void UpdateRotorTarget(RotorMotion& m, RotorMode currentMode)
{
    switch(currentMode)
    {
        case MODE_SLOW: m.targetSpeed = m.slowSpeed; break;
        case MODE_FAST: m.targetSpeed = m.fastSpeed; break;
        case MODE_STOP: m.targetSpeed = STOP_SPEED;  break;
    }
}

inline void UpdateRotorSpeed(RotorMotion& m)
{
    float coeff = (m.targetSpeed > m.speed) ? m.accel : m.decel;
    m.speed += (m.targetSpeed - m.speed) * coeff;
}

inline void UpdateRotorPhase(RotorMotion& m, float sampleRate)
{
    m.phase += 2.0f * M_PI * m.speed / sampleRate;
    if(m.phase > 2.0f * M_PI)
        m.phase -= 2.0f * M_PI;
}

// Update both horn & drum motion per sample
inline void UpdateAllRotorMotion(LeslieState& ls)
{
    UpdateRotorTarget(ls.hornMotion, ls.mode);
    UpdateRotorTarget(ls.drumMotion, ls.mode);

    UpdateRotorSpeed(ls.hornMotion);
    UpdateRotorSpeed(ls.drumMotion);

    UpdateRotorPhase(ls.hornMotion, ls.sr);
    UpdateRotorPhase(ls.drumMotion, ls.sr);
}

// =====================================================================
// Rotor band DSP
// =====================================================================

inline Stereo ProcessRotorBand(
    RotorBand<MAX_DELAY_SAMPLES>& band,
    const Rotor&                  rotor,
    float                         in,
    float                         phase,
    float                         sampleRate)
{
    Stereo out;

    // Virtual mic phases
    float phaseL = phase;
    float phaseR = phase + rotor.micOffset;

    // Doppler: time-varying delay per side
    float delayTimeL    = rotor.baseDelay + rotor.delayDepth * cosf(phaseL);
    float delayTimeR    = rotor.baseDelay + rotor.delayDepth * cosf(phaseR);
    float delaySamplesL = delayTimeL * sampleRate;
    float delaySamplesR = delayTimeR * sampleRate;

    band.delayL.SetDelay(delaySamplesL);
    band.delayR.SetDelay(delaySamplesR);

    band.delayL.Write(in);
    band.delayR.Write(in);

    float yL = band.delayL.Read();
    float yR = band.delayR.Read();

    // Amplitude modulation (distance / beaming)
    float amL = 1.0f + rotor.ampDepth * cosf(phaseL);
    float amR = 1.0f + rotor.ampDepth * cosf(phaseR);

    yL *= amL;
    yR *= amR;

    // Optional constant extra stereo spread
    float panL = 1.0f - 0.5f * rotor.panDepth;
    float panR = 1.0f + 0.5f * rotor.panDepth;

    out.l = yL * panL;
    out.r = yR * panR;

    return out;
}

// Stage 1: split input into drum/horn bands
inline void SplitDrumHorn(LeslieState& ls, float inMono, float& drumIn, float& hornIn)
{
    ls.filter.Process(inMono);
    drumIn = ls.filter.Low();   // below CROSS_OVER_FREQ
    hornIn = ls.filter.High();  // above CROSS_OVER_FREQ
}

// Stage 2: process both rotors and sum to stereo
inline Stereo ProcessRotors(LeslieState& ls, float drumIn, float hornIn)
{
    Stereo drumOut = ProcessRotorBand(
        ls.drumBand,
        ls.drumRotor,
        drumIn,
        ls.drumMotion.phase,
        ls.sr);

    Stereo hornOut = ProcessRotorBand(
        ls.hornBand,
        ls.hornRotor,
        hornIn,
        ls.hornMotion.phase,
        ls.sr);

    Stereo sum;
    sum.l = drumOut.l + hornOut.l;
    sum.r = drumOut.r + hornOut.r;
    return sum;
}

// Stage 3: cabinet reflections (3-tap early reflection layer)
inline Stereo ApplyCabinetReflections(LeslieState& ls, const Stereo& in)
{
    if(!ls.enableReflections)
        return in;

    Stereo out;

    float accL = 0.0f;
    float accR = 0.0f;

    for(size_t i = 0; i < CAB_REF_NUM_TAPS; ++i)
    {
        float dL = ls.cabDelayL[i].Read();
        float dR = ls.cabDelayR[i].Read();

        accL += CAB_REF_TAP_GAINS[i] * dL;
        accR += CAB_REF_TAP_GAINS[i] * dR;

        ls.cabDelayL[i].Write(in.l);
        ls.cabDelayR[i].Write(in.r);
    }

    // Darken reflections
    ls.cabRefLpL.Process(accL);
    ls.cabRefLpR.Process(accR);
    float refL = ls.cabRefLpL.Low();
    float refR = ls.cabRefLpR.Low();

    // Mix reflections back in
    out.l = in.l + CAB_REF_MIX * refL;
    out.r = in.r + CAB_REF_MIX * refR;

    return out;
}

// -----------------------------
// Public API implementation
// -----------------------------

void Leslie_Init(LeslieState& ls, float sampleRate)
{
    ls.sr = sampleRate;
    ls.dt = 1.0f / sampleRate;

    // your existing init sequence:
    Leslie_InitVoicing(ls);
    Leslie_InitFiltersAndBands(ls);
    Leslie_InitCabinetReflections(ls);
    Leslie_InitMotion(ls);
}

Stereo Leslie_ProcessSample(LeslieState& ls, float inSample)
{
    // This is basically your AudioCallback *inner loop*:

    float x = inSample;

    // 1) Motion
    UpdateAllRotorMotion(ls);

    // 2) Split bands
    float drumIn, hornIn;
    SplitDrumHorn(ls, x, drumIn, hornIn);

    // 3) Rotors
    Stereo leslie = ProcessRotors(ls, drumIn, hornIn);

    // 4) Reflections
    Stereo out = ApplyCabinetReflections(ls, leslie);

    return out;
}
