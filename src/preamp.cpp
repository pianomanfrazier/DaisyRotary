#include "preamp.h"

float Gain(float norm, float minDb, float maxDb)
{
    if(norm < 0.0f) norm = 0.0f;
    if(norm > 1.0f) norm = 1.0f;

    float db = minDb + (maxDb - minDb) * norm;
    return powf(10.0f, db / 20.0f);
}