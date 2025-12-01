#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

static DelayLine<float, 9600> dopplerDelay;

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

// 0 <= D <= 1
const float AMP_DEPTH = 0.6f;

// 0 = mono, 1 = full Leslie
const float PAN_DEPTH = 0.3f;

const float BASE_DELAY  = 0.00035f;
const float DELAY_DEPTH = 0.00020f;

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
// Audio Callback — AM + PAN + DOPPLER DELAY
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
        if(phase > 2.0f * M_PI)
            phase -= 2.0f * M_PI;

        // -------------------------------------
        // Amplitude modulation
        // -------------------------------------
        float am = 1.0f + AMP_DEPTH * cosf(phase);
        float amOut = x * am;

        // -------------------------------------
        // Doppler true time-delay (distance modulation)
        // -------------------------------------
        float delayTime    = BASE_DELAY + DELAY_DEPTH * cosf(phase);
        float delaySamples = delayTime * sr;

        dopplerDelay.SetDelay(delaySamples);
        dopplerDelay.Write(amOut);

        float y = dopplerDelay.Read();

        // -------------------------------------
        // Stereo panning based on rotor speed
        // -------------------------------------
        float normalized = rotorSpeed / fastSpeed;
        if(normalized > 1.0f) normalized = 1.0f;
        if(normalized < 0.0f) normalized = 0.0f;

        float dynamicDepth = PAN_DEPTH * normalized;

        float pan = sinf(phase) * dynamicDepth;

        float left  = y * (0.5f - 0.5f * pan);
        float right = y * (0.5f + 0.5f * pan);

        out[i]   = left;
        out[i+1] = right;
    }
}

int main(void)
{
    hw.Init();
    dopplerDelay.Init();

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
