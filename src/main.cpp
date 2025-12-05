#include "daisy_seed.h"
#include "daisysp.h"
#include "leslie.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

// Hardware
DaisySeed   hw;
GPIO        pinFast;
GPIO        pinSlow;
GPIO        led1;
GPIO        led2;
GPIO        btn1;
GPIO        btn2;
GPIO        btn3;

AdcChannelConfig adc_cfg[8];

// DSP state
LeslieState g_leslie;

// ======================
// Switch helpers
// ======================

RotorMode ReadLeslieMode()
{
    bool upState   = !pinFast.Read();
    bool downState = !pinSlow.Read();

    if(upState)
    {
        return MODE_FAST;
    } else if(downState)
    {
        return MODE_SLOW;
    } else
    {
        return MODE_STOP;
    }
}

void UpdateFeatureButtons()
{
    // active-low buttons
    g_leslie.enableReflections = !btn1.Read();
}

// ======================
// Preamp
// ======================

inline float Gain(float norm, float minDb, float maxDb)
{
    if(norm < 0.0f) norm = 0.0f;
    if(norm > 1.0f) norm = 1.0f;

    float db = minDb + (maxDb - minDb) * norm;
    return powf(10.0f, db / 20.0f);
}

inline float DbToLin(float db)
{
    return powf(10.0f, db / 20.0f);
}

inline float TubeWaveshape(float x, float driveNorm)
{
    // driveNorm 0–1 -> more curvature
    float k = 1.0f + 19.0f * driveNorm; // 1..20
    float y = tanhf(k * x);
    // normalize so output stays roughly in [-1,1]
    return y / tanhf(k);
}

// Tube preamp:
//  - driveNorm: 0–1
//  - minDb/maxDb: pre-gain range in dB
inline float TubePreampSample(float in, float driveNorm,
                              float minDb, float maxDb)
{
    // pre-gain from knob
    float preGain = Gain(driveNorm, minDb, maxDb);
    float x       = in * preGain;

    // asymmetric bit (very subtle) to feel more “tube” than pure tanh
    float even    = TubeWaveshape(x, driveNorm);
    float odd     = TubeWaveshape(x + 0.1f, driveNorm) - TubeWaveshape(0.1f, driveNorm);
    float y       = 0.7f * even + 0.3f * odd;

    // optional output trim so higher drive doesn’t explode level
    float makeup  = Gain(driveNorm, -6.0f, 0.0f); // from -6 dB at 0 to 0 dB at 1
    return y * makeup;
}

// Simple active Baxandall: Bass + Treble
struct BaxandallTone
{
    // Filters
    Biquad bassLp;
    Biquad trebleLp;

    // sample rate
    float fs = 48000.0f;

    // control ranges
    float bassMinDb   = -15.0f;
    float bassMaxDb   = +15.0f;
    float trebleMinDb = -15.0f;
    float trebleMaxDb = +15.0f;

    // shelving corner freqs
    float bassCutoff   = 120.0f;   // tweak to taste
    float trebleCutoff = 4000.0f;  // tweak to taste

    // current linear gains
    float bassGainLin   = 1.0f;
    float trebleGainLin = 1.0f;

    void Init(float sampleRate)
    {
        fs = sampleRate;

        bassLp.Init(fs);
        bassLp.SetCutoff(bassCutoff);
        bassLp.SetRes(0.7f); // fairly gentle

        trebleLp.Init(fs);
        trebleLp.SetCutoff(trebleCutoff);
        trebleLp.SetRes(0.7f);
    }

    // bassNorm / trebleNorm: 0–1 from your knobs
    void SetBass(float bassNorm)
    {
        float db      = bassMinDb + (bassMaxDb - bassMinDb) * bassNorm;
        bassGainLin   = DbToLin(db);
    }

    void SetTreble(float trebleNorm)
    {
        float db        = trebleMinDb + (trebleMaxDb - trebleMinDb) * trebleNorm;
        trebleGainLin   = DbToLin(db);
    }

    inline float Process(float x)
    {
        // --- Low shelf stage (using bassLp as LP) ---
        float low   = bassLp.Process(x);
        float yLow  = x + (bassGainLin - 1.0f) * low;
        //   -> below bassCutoff: ~bassGainLin * x
        //   -> above bassCutoff: ~x

        // --- High shelf stage (using trebleLp as LP on yLow) ---
        float lp    = trebleLp.Process(yLow);
        float yHigh = trebleGainLin * yLow - (trebleGainLin - 1.0f) * lp;
        //   -> below trebleCutoff: ~yLow
        //   -> above trebleCutoff: ~trebleGainLin * yLow

        return yHigh;
    }
};

BaxandallTone bax;

// ======================
// Audio callback
// ======================

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    // Read knobs -> float[8]
    float knobs[8];
    for(int i = 0; i < 8; ++i)
        knobs[i] = hw.adc.GetFloat(i);

    bax.SetBass(knobs[2]);
    bax.SetTreble(knobs[3]);

    for(size_t i = 0; i < size; i += 2)
    {
        float x = in[i]; // mono input

        // Preamp
        if(!btn2.Read())
        {
            x = TubePreampSample(x, knobs[1], -12.0f, +30.0f);
        }
        if(!btn3.Read())
        {
            x = bax.Process(x);
        }
        x *= Gain(knobs[0], -20.0f, 6.0f);

        Stereo s = Leslie_ProcessSample(g_leslie, x);

        out[i]     = s.l;
        out[i + 1] = s.r;
    }
}

// ======================
// Hardware init
// ======================

void InitHardware()
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(48);

    pinFast.Init(D0, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    pinSlow.Init(D1, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    led1.Init(D2, GPIO::Mode::OUTPUT);
    led2.Init(D3, GPIO::Mode::OUTPUT);

    btn1.Init(D4, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    btn2.Init(D5, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    btn3.Init(D6, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    // ADC for 7 knobs on A0..A6
    adc_cfg[0].InitSingle(A0);
    adc_cfg[1].InitSingle(A1);
    adc_cfg[2].InitSingle(A2);
    adc_cfg[3].InitSingle(A3);
    adc_cfg[4].InitSingle(A4);
    adc_cfg[5].InitSingle(A5);
    adc_cfg[6].InitSingle(A6);
    adc_cfg[7].InitSingle(A7);

    hw.adc.Init(adc_cfg, 7);
    hw.adc.Start();
}

// ======================
// main
// ======================

int main(void)
{
    InitHardware();

    float sr = hw.AudioSampleRate();
    Leslie_Init(g_leslie, sr);
    bax.Init(sr);

    hw.StartAudio(AudioCallback);

    while(1)
    {
        // Poll switches in control loop
        g_leslie.mode = ReadLeslieMode();
        UpdateFeatureButtons();

        led2.Write(g_leslie.enableReflections);
    }
}
