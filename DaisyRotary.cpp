#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

static DelayLine<float, 9600> dopplerDelayL;
static DelayLine<float, 9600> dopplerDelayR;

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

const float BASE_DELAY  = 0.00040f;
const float DELAY_DEPTH = 0.00030f;
const float MIC_PHASE_OFFSET = M_PI / 2;

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
        // Per-channel phases (virtual mic positions)
        // -------------------------------------

        float phaseL = phase;
        float phaseR = phase + MIC_PHASE_OFFSET;

        // -------------------------------------
        // Doppler: true time-varying delay per channel
        // -------------------------------------
        float delayTimeL    = BASE_DELAY + DELAY_DEPTH * cosf(phaseL);
        float delayTimeR    = BASE_DELAY + DELAY_DEPTH * cosf(phaseR);

        float delaySamplesL = delayTimeL * sr;
        float delaySamplesR = delayTimeR * sr;

        // Write same mono input into both delay lines
        dopplerDelayL.SetDelay(delaySamplesL);
        dopplerDelayR.SetDelay(delaySamplesR);

        dopplerDelayL.Write(x);
        dopplerDelayR.Write(x);

        float yL = dopplerDelayL.Read();
        float yR = dopplerDelayR.Read();

        // -------------------------------------
        // Amplitude modulation per channel
        // (distance / beaming effect)
        // -------------------------------------
        float amL = 1.0f + AMP_DEPTH * cosf(phaseL);
        float amR = 1.0f + AMP_DEPTH * cosf(phaseR);

        yL *= amL;
        yR *= amR;

        // -------------------------------------
        // Optional extra stereo spread based on rotor speed
        // (you may find you don't even need this anymore)
        // -------------------------------------
        float normalized = rotorSpeed / fastSpeed;
        if(normalized > 1.0f) normalized = 1.0f;
        if(normalized < 0.0f) normalized = 0.0f;

        float dynamicDepth = PAN_DEPTH * normalized;

        // Keep overall level roughly stable:
        float panL = 1.0f - 0.5f * dynamicDepth;
        float panR = 1.0f + 0.5f * dynamicDepth;

        float left  = yL * panL;
        float right = yR * panR;

        out[i]   = left;
        out[i+1] = right;
    }
}

int main(void)
{
    hw.Init();
    dopplerDelayL.Init();
    dopplerDelayR.Init();

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
