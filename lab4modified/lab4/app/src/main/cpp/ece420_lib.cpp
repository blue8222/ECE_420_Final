//
// Created by daran on 1/12/2017 to be used in ECE420 Sp17 for the first time.
// Modified by dwang49 on 1/1/2018 to adapt to Android 7.0 and Shield Tablet updates.
//

#include <cmath>
#include "ece420_lib.h"

#include "kiss_fft/kiss_fft.h"
#include <cstdint>
#include <vector>
#include <complex>
#include <cstddef>  // For size_t

#include "ece420_main.h"

#define FS 48000.0
#define T 0.5
#define F_min 8000.0
#define B 10000.0
#define FRAME_SIZE 1024 //size of the recorded buffer (should be greater than chirp length)
#define V_s = 343.0  // Speed of sound in air (m/s)






// https://en.wikipedia.org/wiki/Hann_function
float getHanningCoef(int N, int idx) {
    return (float) (0.5 * (1.0 - cos(2.0 * M_PI * idx / (N - 1))));
}

int findMaxArrayIdx(float *array, int minIdx, int maxIdx) {
    int ret_idx = minIdx;

    for (int i = minIdx; i < maxIdx; i++) {
        if (array[i] > array[ret_idx]) {
            ret_idx = i;
        }
    }

    return ret_idx;
}

int findClosestIdxInArray(float *array, float value, int minIdx, int maxIdx) {
    int retIdx = minIdx;
    float bestResid = abs(array[retIdx] - value);

    for (int i = minIdx; i < maxIdx; i++) {
        if (abs(array[i] - value) < bestResid) {
            bestResid = abs(array[i] - value);
            retIdx = i;
        }
    }

    return retIdx;
}

// TODO: These should really be templatized
int findClosestInVector(std::vector<int> vec, float value, int minIdx, int maxIdx) {
    int retIdx = minIdx;
    float bestResid = abs(vec[retIdx] - value);

    for (int i = minIdx; i < maxIdx; i++) {
        if (abs(vec[i] - value) < bestResid) {
            bestResid = abs(vec[i] - value);
            retIdx = i;
        }
    }

    return retIdx;
}

