//
// Created by daran on 1/12/2017 to be used in ECE420 Sp17 for the first time.
// Modified by dwang49 on 1/1/2018 to adapt to Android 7.0 and Shield Tablet updates.
//

#include "ece420_main.h"

#include "kiss_fft/kiss_fft.h"
#include <cmath>
#include <jni.h>
#include <cstdint>
#include <vector>
#include <complex>
#include <cstddef>  // For size_t
#include <array>

// Student Variables

#define F_min 8000
#define B 10000
#define T 1
#define FS 48000

const int FRAME_SIZE = static_cast<int>(FS*T);


float Freq = -1;

std::vector<int16_t> generatePlaybackAudio(bool window) {

    int number_samples = static_cast< int>(FS * T);

    std::vector<int16_t> outputBuffer(number_samples + number_samples);

    std::vector<float> t_chirp(number_samples);

    float dt = static_cast<float>(T) / static_cast<float>(number_samples);


    for (int i = 0; i < number_samples; ++i) {
        t_chirp[i] = i * dt;
    }

    std::vector<float> phase(number_samples);
    for (int i = 0; i < number_samples; ++i) {
        float t = t_chirp[i];
        phase[i] = 2.0f * std::acos(-1.0f) * (F_min * t + (B / (2.0f * T)) * t * t);
    }

    std::vector<float> single_chirp(number_samples);
    for (int i = 0; i < number_samples; ++i) {
        single_chirp[i] = std::cos(phase[i]);
    }

    float max_val = 0.0f;
    for (float val : single_chirp) {
        float abs_val = std::abs(val);
        if (abs_val > max_val) {
            max_val = abs_val;
        }
    }

    if (max_val > 0.0f) {
        for (float& val : single_chirp) {
            val = (val / max_val) * 32767.0f;
        }
    }

    // optional window function
    if (window) {
        std::vector<float> hamm(number_samples);
        if (number_samples > 1) {
            float denom = number_samples - 1.0f;
            float pi = std::acos(-1.0f);
            for (int n = 0; n < number_samples; ++n) {
                hamm[n] = 0.54f - 0.46f * std::cos(2.0f * pi * n / denom);
            }
        } else {
            hamm[0] = 1.0f; // trivial case
        }
        for (int i = 0; i < number_samples; ++i) {
            single_chirp[i] *= hamm[i];
        }
    }

    for (int i = 0; i < number_samples; ++i) {
        outputBuffer[i] = static_cast<int16_t>(std::round(single_chirp[i]));
    }
    LOGD("Generated chirp of %d samples for playback.", number_samples);
    return outputBuffer;
}



static std::vector<int16_t> chirp = generatePlaybackAudio(true);

void ece420ProcesssFrame(sample_buf *dataBuf) {
    // Keep in mind, we only have a small amount of time to process each buffer!
    struct timeval start;
    gettimeofday(&start, NULL);

    // Using {} initializes all values in the array to zero
    //int16_t bufferIn[2*FRAME_SIZE] = {};


    std::vector<int16_t> bufferOut(2 * FRAME_SIZE);





    // Your buffer conversion (unpacking) here
    // Fetch data sample from dataBuf->buf_[], unpack and put into bufferIn[]
    // ******************** START YOUR CODE HERE ******************** //

    for (int bufferIndex = 0; bufferIndex < 2*FRAME_SIZE; bufferIndex++) {

        //bufferIn[bufferIndex] = (int16_t)((dataBuf->buf_[2*bufferIndex + 1] << 8) | dataBuf->buf_[2*bufferIndex]);
        if(bufferIndex < FRAME_SIZE) {
            bufferOut[bufferIndex] = chirp[bufferIndex];
        } else {
            bufferOut[bufferIndex] = 0;
        }
        dataBuf->buf_[2 * bufferIndex] = (uint8_t)(bufferOut[bufferIndex] & 0xFF); //most significant byte

        dataBuf->buf_[2 * bufferIndex + 1] = (uint8_t)((bufferOut[bufferIndex] >> 8) & 0xFF); //least sig byte


    }

    /*
    for(int outBufferIndex = 0; outBufferIndex < 2*FRAME_SIZE; outBufferIndex++) {



    }
    */

    // ********************* END YOUR CODE HERE ********************* //


    // Your buffer conversion (packing) here
    // Fetch data sample from bufferOut[], pack them and put back into dataBuf->buf_[]
    // ******************** START YOUR CODE HERE ******************** //



    // ********************* END YOUR CODE HERE ********************* //

	// Log the processing time to Android Monitor or Logcat window at the bottom
    struct timeval end;
    gettimeofday(&end, NULL);
    LOGD("Loop timer: %ld us",  ((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec)));

}


extern "C"
JNIEXPORT jfloat JNICALL
Java_com_ece420_lab2_MainActivity_getFreqUpdate(JNIEnv *env, jclass clazz) {

    return Freq;
}