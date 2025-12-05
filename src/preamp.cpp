#include "preamp.h"

float Gain(float norm, float minDb, float maxDb)
{
    if(norm < 0.0f) norm = 0.0f;
    if(norm > 1.0f) norm = 1.0f;

    float db = minDb + (maxDb - minDb) * norm;
    return powf(10.0f, db / 20.0f);
}

float DbToLin(float db)
{
    return powf(10.0f, db / 20.0f);
}

float TubeWaveshape(float x, float driveNorm)
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
float TubePreampSample(float in, float driveNorm,
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

void BaxandallTone::Init(float sampleRate)
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
void BaxandallTone::SetBass(float bassNorm)
{
    float db      = bassMinDb + (bassMaxDb - bassMinDb) * bassNorm;
    bassGainLin   = DbToLin(db);
}

void BaxandallTone::SetTreble(float trebleNorm)
{
    float db        = trebleMinDb + (trebleMaxDb - trebleMinDb) * trebleNorm;
    trebleGainLin   = DbToLin(db);
}

float BaxandallTone::Process(float x)
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