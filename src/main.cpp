#include "daisy_seed.h"
#include "daisysp.h"
#include "leslie.h"
#include "preamp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

Grit grit;

// Hardware
DaisySeed   hw;
GPIO        pinFast;
GPIO        pinSlow;
GPIO        led1;
GPIO        led2;
GPIO        btn1;

AdcChannelConfig adc_cfg[6];

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

// ======================
// Audio callback
// ======================
void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    float knobs[6];
    for(int i = 0; i < 6; ++i)
        knobs[i] = hw.adc.GetFloat(i);
    bool driveOn = !btn1.Read();

    UpdateRotorParamsFromKnobs(g_leslie, knobs[1], knobs[2], knobs[3], knobs[4]);
    float outGain = Gain(knobs[0], -40.0f, 8.0f);
    grit.SetAmount(knobs[5]);

    if (g_leslie.drumMotion.speed < g_leslie.drumMotion.slowSpeed * 0.5f)
        led2.Write(true);
    else
        led2.Write(g_leslie.drumMotion.phase <= M_PI);

    led1.Write(driveOn);

    for(size_t i = 0; i < size; i += 2)
    {
        float x = in[i]; // mono input

        if(driveOn)
        {
            x = grit.Process(x);
        }

        x *= outGain;

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

    pinFast.Init(D13, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    pinSlow.Init(D14, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    led1.Init(D23, GPIO::Mode::OUTPUT);
    led2.Init(D22, GPIO::Mode::OUTPUT);

    btn1.Init(D7, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    // ADC for 6 knobs on A1..A6
    adc_cfg[0].InitSingle(A1); // GAIN
    adc_cfg[1].InitSingle(A3); // FAST
    adc_cfg[2].InitSingle(A2); // SLOW
    adc_cfg[3].InitSingle(A6); // ACC
    adc_cfg[4].InitSingle(A5); // DEC
    adc_cfg[5].InitSingle(A4); // DRIVE

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
    grit.Init(sr);

    hw.StartAudio(AudioCallback);

    while(1)
    {
        // Poll switches in control loop
        g_leslie.mode = ReadLeslieMode();
    }
}
