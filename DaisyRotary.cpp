#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

// =====================================================================
// Constants & configuration
// =====================================================================

// Max delay ~ 9600 samples (at 48kHz ~ 200ms) for rotor Doppler
static const size_t MAX_DELAY_SAMPLES = 9600;
static const float  STOP_SPEED        = 0.0f;

// Crossover freq for drum vs horn
static const float CROSS_OVER_FREQ    = 800.0f;

// Cabinet EQ constants (good starting points)
static const float CAB_HP_FREQ        = 80.0f;    // tighten low end
static const float CAB_LP_FREQ        = 9000.0f;  // remove some top fizz

// Cabinet reflection network
static const size_t CAB_REF_MAX_SAMPLES = 2048;   // ~42 ms max at 48k
static const float  CAB_REF_DELAY_SEC    = 0.0025f; // ~2.5 ms reflection
static const float  CAB_REF_GAIN         = 0.35f;   // how loud the reflection is
static const float  CAB_REF_TONE_FREQ    = 4000.0f; // darken above ~4 kHz

// =====================================================================
// Data structures
// =====================================================================

struct Stereo
{
    float l;
    float r;
};

// Rotor band abstraction (per-band L/R delay lines)
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

// Holds voicing/settings for a rotor (horn or drum)
struct Rotor
{
    float ampDepth;    // 0..1 amplitude modulation depth
    float panDepth;    // 0..1 extra stereo spread
    float baseDelay;   // seconds
    float delayDepth;  // seconds
    float micOffset;   // radians between L/R mics
};

// Holds motion parameters + state for a rotor
struct RotorMotion
{
    float slowSpeed;   // Hz (chorale)
    float fastSpeed;   // Hz (tremolo)

    float accel;       // per-sample accel coeff
    float decel;       // per-sample decel coeff

    float speed;       // current speed (Hz)
    float targetSpeed; // target speed (Hz)
    float phase;       // current phase (radians)
};

// 3-way switch mode
enum RotorMode
{
    MODE_SLOW,
    MODE_STOP,
    MODE_FAST,
};

// All DSP state for the Leslie lives here
struct LeslieState
{
    float sr;
    float dt;

    // Two bands: horn (high) + drum (low)
    RotorBand<MAX_DELAY_SAMPLES> hornBand;
    RotorBand<MAX_DELAY_SAMPLES> drumBand;

    // Rotor voicing configs
    Rotor hornRotor;
    Rotor drumRotor;

    // Rotor motion (separate horn/drum speeds & inertia)
    RotorMotion hornMotion;
    RotorMotion drumMotion;

    // Crossover filter (drum vs horn)
    Svf filter;

    // Cabinet EQ filters
    Svf cabHpL, cabHpR; // high-pass (low cut)
    Svf cabLpL, cabLpR; // low-pass (high cut)

    // Cabinet reflection network
    DelayLine<float, CAB_REF_MAX_SAMPLES> cabDelayL;
    DelayLine<float, CAB_REF_MAX_SAMPLES> cabDelayR;
    Svf cabRefLpL;
    Svf cabRefLpR;

    // Feature toggles
    bool enableCabinetEq;
    bool enableReflections;

    // Current mode
    RotorMode mode;
};

// =====================================================================
// Globals: hardware + one Leslie state
// =====================================================================

DaisySeed   hw;
GPIO        pinFast;
GPIO        pinSlow;
GPIO        ledPin;
GPIO        ledPin2;

LeslieState g_leslie;

// =====================================================================
// Switch handling (writes into LeslieState.mode)
// =====================================================================

void UpdateLeslieSwitch(LeslieState& ls)
{
    bool upState   = !pinFast.Read(); // switch connects to GND
    bool downState = !pinSlow.Read();

    if(upState)
        ls.mode = MODE_FAST;
    else if(downState)
        ls.mode = MODE_SLOW;
    else
        ls.mode = MODE_STOP;
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

// =====================================================================
// DSP stages: split / rotors / EQ / reflections
// =====================================================================

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

// Stage 3: cabinet EQ (HP -> LP)
inline Stereo ApplyCabinetEq(LeslieState& ls, const Stereo& in)
{
    if(!ls.enableCabinetEq)
        return in;

    Stereo out;

    // High-pass per channel
    ls.cabHpL.Process(in.l);
    ls.cabHpR.Process(in.r);
    float hpL = ls.cabHpL.High();
    float hpR = ls.cabHpR.High();

    // Low-pass per channel
    ls.cabLpL.Process(hpL);
    ls.cabLpR.Process(hpR);
    out.l = ls.cabLpL.Low();
    out.r = ls.cabLpR.Low();

    return out;
}

// Stage 4: cabinet reflections (early reflection layer)
inline Stereo ApplyCabinetReflections(LeslieState& ls, const Stereo& in)
{
    if(!ls.enableReflections)
        return in;

    Stereo out;

    // Read previous reflections (fixed delay)
    float refL = ls.cabDelayL.Read();
    float refR = ls.cabDelayR.Read();

    // Darken the reflections with LPF
    ls.cabRefLpL.Process(refL);
    ls.cabRefLpR.Process(refR);
    refL = ls.cabRefLpL.Low();
    refR = ls.cabRefLpR.Low();

    // Write current signal into delay lines
    ls.cabDelayL.Write(in.l);
    ls.cabDelayR.Write(in.r);

    // Mix reflections back in (feed-forward)
    out.l = in.l + CAB_REF_GAIN * refL;
    out.r = in.r + CAB_REF_GAIN * refR;

    return out;
}

// =====================================================================
// Audio Callback — orchestrate pipeline per sample
// =====================================================================

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    LeslieState& ls = g_leslie;

    for(size_t i = 0; i < size; i += 2)
    {
        float x = in[i]; // mono input

        // 1) Update rotor motion (speeds + phases)
        UpdateAllRotorMotion(ls);

        // 2) Split into drum/horn bands
        float drumIn, hornIn;
        SplitDrumHorn(ls, x, drumIn, hornIn);

        // 3) Process rotors (AM + Doppler + pan)
        Stereo leslie = ProcessRotors(ls, drumIn, hornIn);

        // 4) Cabinet EQ
        Stereo eq = ApplyCabinetEq(ls, leslie);

        // 5) Cabinet reflections
        Stereo finalOut = ApplyCabinetReflections(ls, eq);

        out[i]     = finalOut.l;
        out[i + 1] = finalOut.r;
    }
}

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
    float hornAccelTime = 1.8f; // rise
    float hornDecelTime = 2.4f; // fall

    float drumAccelTime = 7.0f; // rise
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

void Leslie_InitCabinetEq(LeslieState& ls)
{
    // Cabinet HP/LP filters
    ls.cabHpL.Init(ls.sr);
    ls.cabHpR.Init(ls.sr);
    ls.cabHpL.SetFreq(CAB_HP_FREQ);
    ls.cabHpR.SetFreq(CAB_HP_FREQ);

    ls.cabLpL.Init(ls.sr);
    ls.cabLpR.Init(ls.sr);
    ls.cabLpL.SetFreq(CAB_LP_FREQ);
    ls.cabLpR.SetFreq(CAB_LP_FREQ);

    ls.enableCabinetEq = true;
}

void Leslie_InitCabinetReflections(LeslieState& ls)
{
    ls.cabDelayL.Init();
    ls.cabDelayR.Init();
    {
        float refSamples = CAB_REF_DELAY_SEC * ls.sr;
        ls.cabDelayL.SetDelay(refSamples);
        ls.cabDelayR.SetDelay(refSamples);
    }

    ls.cabRefLpL.Init(ls.sr);
    ls.cabRefLpR.Init(ls.sr);
    ls.cabRefLpL.SetFreq(CAB_REF_TONE_FREQ);
    ls.cabRefLpR.SetFreq(CAB_REF_TONE_FREQ);

    ls.enableReflections = true;
}

void Leslie_Init(LeslieState& ls, float sampleRate)
{
    ls.sr = sampleRate;
    ls.dt = 1.0f / sampleRate;

    Leslie_InitVoicing(ls);
    Leslie_InitFiltersAndBands(ls);
    Leslie_InitCabinetEq(ls);
    Leslie_InitCabinetReflections(ls);
    Leslie_InitMotion(ls);
}

// =====================================================================
// Hardware init
// =====================================================================

void InitHardware()
{
    hw.Init();

    pinFast.Init(D0, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    pinSlow.Init(D1, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    ledPin.Init(D2, GPIO::Mode::OUTPUT);
    ledPin2.Init(D3, GPIO::Mode::OUTPUT);
}

// =====================================================================
// main
// =====================================================================

int main(void)
{
    InitHardware();

    float sr = hw.AudioSampleRate();
    Leslie_Init(g_leslie, sr);

    hw.StartAudio(AudioCallback);

    while(1)
    {
        UpdateLeslieSwitch(g_leslie);
        ledPin.Write(g_leslie.mode == MODE_FAST);
        ledPin2.Write(g_leslie.mode == MODE_SLOW);
    }
}
