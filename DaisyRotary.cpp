#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

// -----------------------------------------
// Rotor band abstraction (per-band L/R delay lines)
// -----------------------------------------
template <size_t MAX_DELAY_SAMPLES>
struct RotorBand
{
    DelayLine<float, MAX_DELAY_SAMPLES> delayL;
    DelayLine<float, MAX_DELAY_SAMPLES> delayR;

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

// Max delay ~ 9600 samples (at 48kHz ~ 200ms) for rotor Doppler
static const size_t MAX_DELAY_SAMPLES = 9600;
static const float  STOP_SPEED        = 0.0f;

// Two bands: horn (high) + drum (low)
RotorBand<MAX_DELAY_SAMPLES> hornBand;
RotorBand<MAX_DELAY_SAMPLES> drumBand;

// Rotor voicing configs
Rotor hornRotor = {
    0.7f,          // ampDepth
    0.3f,          // panDepth
    0.00040f,      // baseDelay (s)
    0.00030f,      // delayDepth (s)
    M_PI / 2.0f,   // micOffset
};

Rotor drumRotor = {
    0.4f,          // ampDepth
    0.15f,         // panDepth
    0.00020f,      // baseDelay (s)
    0.00010f,      // delayDepth (s)
    M_PI / 2.0f,   // micOffset
};

// Rotor motion (separate horn/drum speeds & inertia)
RotorMotion hornMotion;
RotorMotion drumMotion;

// Crossover filter (drum vs horn)
Svf filter;

// Cabinet EQ filters
Svf cabHpL, cabHpR; // high-pass (low cut)
Svf cabLpL, cabLpR; // low-pass (high cut)

// Cabinet EQ constants (good starting points)
const float CAB_HP_FREQ = 80.0f;    // tighten low end
const float CAB_LP_FREQ = 6000.0f;  // remove some top fizz

// -----------------------------------------
// Cabinet reflection network
// -----------------------------------------
static const size_t CAB_REF_MAX_SAMPLES = 2048; // ~42 ms max at 48k
DelayLine<float, CAB_REF_MAX_SAMPLES> cabDelayL;
DelayLine<float, CAB_REF_MAX_SAMPLES> cabDelayR;
Svf cabRefLpL;
Svf cabRefLpR;

// ~2.5 ms reflection
const float CAB_REF_DELAY_SEC = 0.0025f;
// how loud the reflection is
const float CAB_REF_GAIN      = 0.35f;
// darken reflection above ~4 kHz
const float CAB_REF_TONE_FREQ = 4000.0f;

DaisySeed hw;
GPIO     pinFast;
GPIO     pinSlow;
GPIO     ledPin;
GPIO     ledPin2;

float sr;
float dt;

// -----------------------------------------
// 3-way switch
// -----------------------------------------
enum RotorMode
{
    MODE_SLOW,
    MODE_STOP,
    MODE_FAST
};
volatile RotorMode mode = MODE_STOP;

// Crossover freq for drum vs horn
const float CROSS_OVER_FREQ = 800.0f;

// -----------------------------------------
// Switch handling
// -----------------------------------------
void UpdateLeslieSwitch()
{
    bool upState   = !pinFast.Read(); // switch connects to GND
    bool downState = !pinSlow.Read();

    if(upState)
        mode = MODE_FAST;
    else if(downState)
        mode = MODE_SLOW;
    else
        mode = MODE_STOP;
}

// -----------------------------------------
// Update target speed per rotor (based on mode)
// -----------------------------------------
inline void UpdateRotorTarget(RotorMotion& m, RotorMode mode)
{
    switch(mode)
    {
        case MODE_SLOW: m.targetSpeed = m.slowSpeed; break;
        case MODE_FAST: m.targetSpeed = m.fastSpeed; break;
        case MODE_STOP: m.targetSpeed = STOP_SPEED;  break;
    }
}

// Smooth rotor speed toward target (per-sample)
// -----------------------------------------
inline void UpdateRotorSpeed(RotorMotion& m)
{
    float coeff = (m.targetSpeed > m.speed) ? m.accel : m.decel;
    m.speed += (m.targetSpeed - m.speed) * coeff;
}

// -----------------------------------------
// Rotor band processor abstraction
// -----------------------------------------
inline void ProcessRotorBand(
    RotorBand<MAX_DELAY_SAMPLES>& band,
    const Rotor&                  rotor,
    float                         in,
    float                         phase,
    float                         sr,
    float&                        outL,
    float&                        outR)
{
    // Virtual mic phases
    float phaseL = phase;
    float phaseR = phase + rotor.micOffset;

    // Doppler: time-varying delay per side
    float delayTimeL    = rotor.baseDelay + rotor.delayDepth * cosf(phaseL);
    float delayTimeR    = rotor.baseDelay + rotor.delayDepth * cosf(phaseR);
    float delaySamplesL = delayTimeL * sr;
    float delaySamplesR = delayTimeR * sr;

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

    outL = yL * panL;
    outR = yR * panR;
}

// -----------------------------------------
// Audio Callback — split drum/horn, separate rotor motion
// -----------------------------------------
void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    for(size_t i = 0; i < size; i += 2)
    {
        float x = in[i]; // mono input

        // -------------------------------------
        // Update target speeds per rotor
        // -------------------------------------
        UpdateRotorTarget(hornMotion, mode);
        UpdateRotorTarget(drumMotion, mode);

        // -------------------------------------
        // Smoothed accel/decel per rotor
        // -------------------------------------
        UpdateRotorSpeed(hornMotion);
        UpdateRotorSpeed(drumMotion);

        // -------------------------------------
        // Phase update per rotor
        // -------------------------------------
        hornMotion.phase += 2.0f * M_PI * hornMotion.speed / sr;
        drumMotion.phase += 2.0f * M_PI * drumMotion.speed / sr;

        if(hornMotion.phase > 2.0f * M_PI)
            hornMotion.phase -= 2.0f * M_PI;
        if(drumMotion.phase > 2.0f * M_PI)
            drumMotion.phase -= 2.0f * M_PI;

        // -------------------------------------
        // Split into drum (low) and horn (high)
        // -------------------------------------
        filter.Process(x);
        float drumIn = filter.Low();   // below ~800 Hz
        float hornIn = filter.High();  // above ~800 Hz

        // -------------------------------------
        // Process each band through its own rotor motion
        // -------------------------------------
        float drumL, drumR;
        float hornL, hornR;

        ProcessRotorBand(drumBand, drumRotor, drumIn, drumMotion.phase, sr, drumL, drumR);
        ProcessRotorBand(hornBand, hornRotor, hornIn, hornMotion.phase, sr, hornL, hornR);

        // Sum drum + horn to stereo
        float left  = drumL + hornL;
        float right = drumR + hornR;

        // -------------------------------------
        // Cabinet EQ: HP -> LP per channel
        // -------------------------------------

        // High-pass (remove deep lows)
        cabHpL.Process(left);
        cabHpR.Process(right);
        float hpL = cabHpL.High();
        float hpR = cabHpR.High();

        // Low-pass (remove top-end fizz)
        cabLpL.Process(hpL);
        cabLpR.Process(hpR);
        float lpL = cabLpL.Low();
        float lpR = cabLpR.Low();

        // -------------------------------------
        // Cabinet reflections (early reflection layer)
        // -------------------------------------

        // Read previous reflections (fixed delay)
        float refL = cabDelayL.Read();
        float refR = cabDelayR.Read();

        // Darken the reflections with LPF
        cabRefLpL.Process(refL);
        cabRefLpR.Process(refR);
        refL = cabRefLpL.Low();
        refR = cabRefLpR.Low();

        // Write current signal into delay lines
        cabDelayL.Write(lpL);
        cabDelayR.Write(lpR);

        // Mix reflections back in
        float outL = lpL + CAB_REF_GAIN * refL;
        float outR = lpR + CAB_REF_GAIN * refR;

        out[i]     = outL;
        out[i + 1] = outR;
    }
}

int main(void)
{
    hw.Init();
    sr = hw.AudioSampleRate();
    dt = 1.0f / sr;

    hornBand.Init();
    drumBand.Init();

    // Crossover
    filter.Init(sr);
    filter.SetFreq(CROSS_OVER_FREQ);

    // Cabinet HP/LP filters
    cabHpL.Init(sr);
    cabHpR.Init(sr);
    cabHpL.SetFreq(CAB_HP_FREQ);
    cabHpR.SetFreq(CAB_HP_FREQ);

    cabLpL.Init(sr);
    cabLpR.Init(sr);
    cabLpL.SetFreq(CAB_LP_FREQ);
    cabLpR.SetFreq(CAB_LP_FREQ);

    // Cabinet reflections
    cabDelayL.Init();
    cabDelayR.Init();
    {
        float refSamples = CAB_REF_DELAY_SEC * sr;
        cabDelayL.SetDelay(refSamples);
        cabDelayR.SetDelay(refSamples);
    }

    cabRefLpL.Init(sr);
    cabRefLpR.Init(sr);
    cabRefLpL.SetFreq(CAB_REF_TONE_FREQ);
    cabRefLpR.SetFreq(CAB_REF_TONE_FREQ);

    pinFast.Init(D0, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    pinSlow.Init(D1, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    ledPin.Init(D2, GPIO::Mode::OUTPUT);
    ledPin2.Init(D3, GPIO::Mode::OUTPUT);

    // -------------------------------------
    // Initialize rotor motion parameters
    // -------------------------------------
    // Roughly Leslie-like ballpark:
    // Horn a bit faster than drum
    hornMotion.slowSpeed = 0.6f;  // Hz
    hornMotion.fastSpeed = 7.0f;  // Hz

    drumMotion.slowSpeed = 0.33f; // Hz
    drumMotion.fastSpeed = 5.5f;  // Hz

    // Accel/decel times (seconds) → per-sample coeffs
    float hornAccelTime = 1.8f; // rise
    float hornDecelTime = 2.4f; // fall

    float drumAccelTime = 7.0f; // rise
    float drumDecelTime = 5.5f; // fall

    hornMotion.accel = dt / hornAccelTime;
    hornMotion.decel = dt / hornDecelTime;
    drumMotion.accel = dt / drumAccelTime;
    drumMotion.decel = dt / drumDecelTime;

    hornMotion.speed       = STOP_SPEED;
    hornMotion.targetSpeed = STOP_SPEED;
    hornMotion.phase       = 0.0f;

    drumMotion.speed       = STOP_SPEED;
    drumMotion.targetSpeed = STOP_SPEED;
    drumMotion.phase       = 0.0f;

    hw.StartAudio(AudioCallback);

    while(1)
    {
        UpdateLeslieSwitch();
        ledPin.Write(mode == MODE_FAST);
        ledPin2.Write(mode == MODE_SLOW);
    }
}
