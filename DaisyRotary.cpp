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

// -----------------------------------------
// 3-way switch
// -----------------------------------------
enum RotorMode { MODE_SLOW, MODE_STOP, MODE_FAST };
volatile RotorMode mode = MODE_STOP;

// -----------------------------------------
// 2-pole rotor model
// -----------------------------------------
struct Rotor2Pole
{
    float speed;
    float velocity;
    float target;
    float omega;   // natural frequency
    float damp;    // damping ratio
};

Rotor2Pole rotor;

// -----------------------------------------
// Simple speed parameters (Hz)
// -----------------------------------------
float slowSpeed  = 0.33f;   // 20 RPM
float fastSpeed  = 6.0f;    // 420 RPM

float omegaFast = 1.0f;
float omegaSlow = 0.5f;
float omegaStop = 0.25f;

float damp = 0.3f;

// -----------------------------------------
// State
// -----------------------------------------
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

inline void UpdateRotor(Rotor2Pole& r, float dt)
{
    float accel = (r.omega * r.omega) * (r.target - r.speed)
                  - 2.0f * r.damp * r.omega * r.velocity;

    r.velocity += accel * dt;
    r.speed    += r.velocity * dt;

    if(r.speed < 0.0f)
        r.speed = 0.0f;
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

        switch(mode)
        {
            case MODE_SLOW:
                rotor.target = slowSpeed;
                rotor.omega  = omegaSlow;
                break;

            case MODE_FAST:
                rotor.target = fastSpeed;
                rotor.omega  = omegaFast;
                break;

            case MODE_STOP:
                rotor.target = 0.0f;
                rotor.omega  = omegaStop;
                break;
        }

        UpdateRotor(rotor, dt);

        phase += 2.0f * M_PI * rotor.speed * dt;
        if(phase > 2.0f * M_PI)
            phase -= 2.0f * M_PI;

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

    // Init rotor
    rotor.speed = 0.0f;
    rotor.velocity = 0.0f;
    rotor.target = 0.0f;
    rotor.omega = omegaStop;
    rotor.damp  = damp;

    hw.StartAudio(AudioCallback);

    while(1)
    {
        UpdateLeslieSwitch();
        ledPin.Write(!pinFast.Read());
    }
}
