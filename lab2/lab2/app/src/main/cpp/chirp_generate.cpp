//
// Created by dager on 11/21/2025.
//


#include "chirp_generate.h"
#include <cmath>
#include <climits>
#include <jni.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif




static inline float getHanningCoef(int N, int idx) {
    return (float)(0.5 * (1.0 - cos(2.0 * M_PI * idx / (N - 1))));
}

std::vector<uint8_t> generateChirpPCM_LE16(
        int sampleRate_,
        double minFreq_,
        double bandwidth_,
        double sweepTime_

) {
    std::vector<uint8_t> pcm;



    const int totalSamples = static_cast<int>(sampleRate_ * sweepTime);
    pcm.reserve(totalSamples * 2);

    double maxFreq = minFreq_ + bandwidth_;
    double k = (maxFreq - minFreq_) / sweepTime_;  // sweep rate (Hz per sec)

    for (int n = 0; n < totalSamples; ++n) {
        double t = (double)n / (double)sampleRate_;
        // instantaneous phase for linear chirp:
        // φ(t) = 2π ( f0 t + (k/2) t^2 )
        double phase = 2.0 * M_PI * (minFreq_ * t + 0.5 * k * t * t);
        double sampleValue = sin(phase);

        // apply Hanning window
        sampleValue *= getHanningCoef(totalSamples, n);

        // scale to signed 16-bit
        double scaled = sampleValue * (double)INT16_MAX;
        if (scaled > INT16_MAX) scaled = INT16_MAX;
        if (scaled < INT16_MIN) scaled = INT16_MIN;
        int16_t s = (int16_t)std::lrint(scaled);

        // little endian
        pcm.push_back((uint8_t)(s & 0xFF));
        pcm.push_back((uint8_t)((s >> 8) & 0xFF));
    }

    return pcm;
}

extern "C" JNIEXPORT jbyteArray JNICALL
Java_com_ece420_lab2_MainActivity_generateChirpPCMNative(
        JNIEnv* env, jclass) {

    std::vector<uint8_t> pcm = generateChirpPCM_LE16(
            48000,
            minFreq,
            bandwidth,
            sweepTime
    );

    jbyteArray out = env->NewByteArray((jsize)pcm.size());
    if (!out) return nullptr;
    env->SetByteArrayRegion(out, 0, (jsize)pcm.size(), reinterpret_cast<jbyte*>(pcm.data()));
    return out;
}