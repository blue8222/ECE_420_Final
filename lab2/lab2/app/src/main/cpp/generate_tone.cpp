// generate_tone.cpp
#include "generate_tone.h"
#include <cmath>
#include <climits>
#include <jni.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::vector<uint8_t> generateTonePCM_LE16(
        int sampleRate,
        int durationSeconds,
        double freqHz
) {
    std::vector<uint8_t> pcm;

    if (sampleRate <= 0 || durationSeconds <= 0) {
        return pcm;    // return empty vector
    }

    const int numSamples = sampleRate * durationSeconds;
    pcm.reserve(numSamples * 2);  // 2 bytes per sample

    const double twoPiF = 2.0 * M_PI * freqHz;
    for (int i = 0; i < numSamples; ++i) {
        double t = (double) i / (double) sampleRate;
        double sample = std::sin(twoPiF * t);
        double scaled = sample * (double) INT16_MAX;

        if (scaled > INT16_MAX)  scaled = INT16_MAX;
        if (scaled < INT16_MIN)  scaled = INT16_MIN;
        int16_t s = (int16_t) std::lrint(scaled);

        // little-endian
        uint8_t low  = (uint8_t) (s & 0xFF);
        uint8_t high = (uint8_t) ((s >> 8) & 0xFF);
        pcm.push_back(low);
        pcm.push_back(high);
    }

    return pcm;
}


extern "C" JNIEXPORT jbyteArray JNICALL
Java_com_ece420_lab2_MainActivity_generateTonePCMNative(
        JNIEnv* env, jclass,
        jint sampleRate, jint durationSeconds, jdouble freqHz) {

    std::vector<uint8_t> pcm = generateTonePCM_LE16(
            sampleRate,
            durationSeconds,
            freqHz
    );

    jsize outSize = (jsize) pcm.size();
    jbyteArray out = env->NewByteArray(outSize);
    if (!out) return nullptr;

    env->SetByteArrayRegion(out, 0, outSize, reinterpret_cast<const jbyte*>(pcm.data()));
    return out;
}