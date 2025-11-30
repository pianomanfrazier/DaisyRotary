#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

// -------------------------------------------------
// Globals & hardware
// -------------------------------------------------
DaisySeed hw;
GPIO pinFast;
GPIO pinSlow;
GPIO ledPin;

float sr;
float dt;

// -------------------------------------------------
// Modes
// -------------------------------------------------
enum RotorMode { MODE_SLOW, MODE_STOP, MODE_FAST };
volatile RotorMode mode = MODE_STOP;

// -------------------------------------------------
// Rotor state (speed & velocity)
// -------------------------------------------------
struct RotorState
{
    float speed;     // Hz
    float velocity;  // first derivative (Hz per second)
};

RotorState rotor;

// -------------------------------------------------
// Motor & physical parameters
// -------------------------------------------------
const float DRAG = 0.15f;       // increase until FAST → STOP glides fully to zero
const float FRICTION = 0.15f;   // mechanical friction on velocity
const float DEAD_ZONE = 0.00001f;

float slowSpeed = 0.33f;    // ~20 RPM
float fastSpeed = 6.00f;     // ~360 RPM

// Slow motor parameters
float slowOmega = 1.00f;
float slowDamp  = 0.55f;

// Fast motor parameters
float fastOmega = 1.00f;
float fastDamp  = 0.40f;

float torque = 0.0f;

// -------------------------------------------------
// AM LFO state
// -------------------------------------------------
float phase = 0.0f;
float ampDepth = 0.6f;   // 0–1

// -------------------------------------------------
// Read FAST/OFF/SLOW switch
// -------------------------------------------------
void UpdateLeslieSwitch()
{
    bool upState   = !pinFast.Read(); // active low
    bool downState = !pinSlow.Read(); // active low

    if(upState)
        mode = MODE_FAST;
    else if(downState)
        mode = MODE_SLOW;
    else
        mode = MODE_STOP;
}

// -------------------------------------------------
// Real Leslie motor physics
// -------------------------------------------------
inline void UpdateRotorRealMotor(RotorState &r, RotorMode mode, float dt)
{
    switch(mode)
    {
        case MODE_FAST:
            torque =
                (fastOmega * fastOmega) * (fastSpeed - r.speed)
                - 2.0f * fastDamp * fastOmega * r.velocity;
            break;

        case MODE_SLOW:
            torque =
                (slowOmega * slowOmega) * (slowSpeed - r.speed)
                - 2.0f * slowDamp * slowOmega * r.velocity;
            break;

        case MODE_STOP:
        {
            // Air and bearing drag decelerate speed directly
            torque = -DRAG * r.speed - FRICTION * r.velocity;

            // Dead-zone to prevent micro-motion forever
            if(fabsf(r.speed) < DEAD_ZONE && fabsf(r.velocity) < DEAD_ZONE)
            {
                r.speed    = 0.0f;
                r.velocity = 0.0f;
            }
        }
        break;

    }

    // Apply physics
    r.velocity += torque * dt;
    r.speed += r.velocity * dt;

    // Prevent negative numerical drift
    if(r.speed < 0.0f)
    {
        r.speed = 0.0f;
        if(r.velocity < 0.0f)
            r.velocity = 0.0f;
    }
}

// -------------------------------------------------
// Audio processing
// -------------------------------------------------
void AudioCallback(AudioHandle::InterleavingInputBuffer in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t size)
{
    for(size_t i = 0; i < size; i += 2)
    {
        float x = in[i];

        UpdateRotorRealMotor(rotor, mode, dt);

        // Update rotor phase from speed
        phase += 2.0f * M_PI * rotor.speed * dt;
        if(phase > 2.0f * M_PI)
            phase -= 2.0f * M_PI;

        float am = 1.0f + ampDepth * cosf(phase);
        float y = x * am;

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

    rotor.speed = 0.0f;
    rotor.velocity = 0.0f;

    hw.StartAudio(AudioCallback);

    while(1)
    {
        UpdateLeslieSwitch();
        ledPin.Write(!pinFast.Read());
    }
}
