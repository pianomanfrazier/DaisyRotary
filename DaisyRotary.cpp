#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

DaisySeed hw;
GPIO pinFast;
GPIO pinSlow;
GPIO ledPin;

float sr;
float dt;

float accel;
float decel;

// -----------------------------------------
// 3-way switch
// -----------------------------------------
enum RotorMode { MODE_SLOW, MODE_STOP, MODE_FAST };
volatile RotorMode mode = MODE_STOP;

// -----------------------------------------
// Simple speed parameters (Hz)
// -----------------------------------------
float slowSpeed = 0.33f;   // 20 RPM
float fastSpeed = 7.0f;    // 420 RPM
float stopSpeed = 0.0f;


// -----------------------------------------
// State
// -----------------------------------------
float rotorSpeed  = 0.33f;
float rotorTarget = 0.33f;

float phase = 0.0f;

// AM depth
// 0 <= D <= 1
float ampDepth = 0.6f;

void UpdateLeslieSwitch()
{
    bool upState   = !pinFast.Read();  // switch connects to GND
    bool downState = !pinSlow.Read();

    if(upState)
        mode = MODE_FAST;
    else if(downState)
        mode = MODE_SLOW;
    else
        mode = MODE_STOP;
}

// -----------------------------------------
// Audio Callback — AM ONLY, NO PAN
// -----------------------------------------
void AudioCallback(AudioHandle::InterleavingInputBuffer in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t size)
{
    for(size_t i = 0; i < size; i += 2)
    {
        float x = in[i];  // mono input

        // -------------------------------------
        // TARGET SPEED
        // -------------------------------------
        switch(mode)
        {
            case MODE_SLOW: rotorTarget = slowSpeed; break;
            case MODE_FAST: rotorTarget = fastSpeed; break;
            case MODE_STOP: rotorTarget = stopSpeed; break;
        }

        // -------------------------------------
        // Smoothed accel/decel
        // -------------------------------------
        float smooth = (rotorTarget > rotorSpeed) ? accel : decel;
        rotorSpeed += (rotorTarget - rotorSpeed) * smooth;

        // -------------------------------------
        // Phase update
        // -------------------------------------
        phase += 2.0f * M_PI * rotorSpeed / sr;
        if(phase > 2 * M_PI)
            phase -= 2 * M_PI;

        // -------------------------------------
        // Simple MONO amplitude modulation
        // -------------------------------------
        float am = 1.0f + ampDepth * cosf(phase);
        float y = x * am;

        // SAME OUTPUT TO LEFT AND RIGHT
        out[i]   = y;
        out[i+1] = y;
    }
}

int main(void)
{
    hw.Init();

    pinFast.Init(D0, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    pinSlow.Init(D1, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    ledPin.Init(D2, GPIO::Mode::OUTPUT);

    sr = hw.AudioSampleRate();
    dt = 1.0f / sr;

    accel = dt / 0.5f;
    decel = dt / 1.5f;

    hw.StartAudio(AudioCallback);

    while(1)
    {
        UpdateLeslieSwitch();
        ledPin.Write(mode == MODE_STOP);
    }
}
