#include "daisy_seed.h"
#include "daisysp.h"
#include "leslie.h"
#include "preamp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;


static Biquad filter;

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

    // map knob to g_leslie
    UpdateRotorParamsFromKnobs(g_leslie, knobs[1], knobs[2], knobs[3], knobs[4]);
    UpdateVoicingFromSpread(g_leslie, knobs[5]);

    filter.SetCutoff(knobs[6] * 7000);

    if (g_leslie.drumMotion.speed < g_leslie.drumMotion.slowSpeed * 0.5f)
        led2.Write(true);
    else
        led2.Write(g_leslie.drumMotion.phase <= M_PI);


    for(size_t i = 0; i < size; i += 2)
    {
        float x = in[i]; // mono input

        x = filter.Process(x);

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

    filter.Init(sr);
    filter.SetRes(0.7);

    hw.StartAudio(AudioCallback);

    while(1)
    {
        // Poll switches in control loop
        g_leslie.mode = ReadLeslieMode();
        UpdateFeatureButtons();
    }
}
