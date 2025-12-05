#include "daisy_seed.h"
#include "daisysp.h"
#include "leslie.h"

using namespace daisy;
using namespace daisy::seed;
using namespace daisysp;

float Gain(float norm, float minDb, float maxDb);
float DbToLin(float db);
float TubeWaveshape(float x, float driveNorm);
float TubePreampSample(float in, float driveNorm, float minDb, float maxDb);

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

    void Init(float sampleRate);
    void SetBass(float bassNorm);
    void SetTreble(float trebleNorm);
    float Process(float x);
};