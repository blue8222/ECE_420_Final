//
// Created by daran on 1/12/2017 to be used in ECE420 Sp17 for the first time.
// Modified by dwang49 on 1/1/2018 to adapt to Android 7.0 and Shield Tablet updates.
//

#include "ece420_main.h"
#include "ece420_lib.h"
#include "kiss_fft/kiss_fft.h"
#include <math.h>
#include <jni.h>

#include <cstdint>
#include <vector>
#include <complex>
#include <cstddef>  // For size_t


#define FS 48000.0
#define T 0.5
#define F_min 8000.0
#define B 10000.0
#define FRAME_SIZE 1024 //size of the recorded buffer (should be greater than chirp length)
#define V_s 343.0  // Speed of sound in air (m/s)


struct Peak {
    int index;           // peak index
    float value;         // peak value
    float distance_m;    // calculated distance
};


extern "C" {
JNIEXPORT float JNICALL
Java_com_ece420_lab4_MainActivity_getDistanceUpdate(JNIEnv *env, jclass);
}


// Define the global buffers and playback position
std::vector<float> fullRecordingBuffer;
std::vector<float> playbackBuffer;
std::atomic<int> playbackBufferPos(0);
// The distance measured
//float distance = -1
// Function to generate a chirp
std::vector<float> generatePlaybackAudio(bool window) {

    std::vector<float> outputBuffer(FRAME_SIZE);
    int number_samples = static_cast<int>(FS * T);
    std::vector<float> t_chirp(number_samples);
    float dt = T / number_samples;
    for (int i = 0; i < number_samples; ++i) {
        t_chirp[i] = i * dt;
    }

    std::vector<float> phase(number_samples);
    for (int i = 0; i < number_samples; ++i) {
        float t = t_chirp[i];
        phase[i] = 2.0 * std::acos(-1.0) * (F_min * t + (B / (2.0 * T)) * t * t);
    }

    std::vector<float> single_chirp(number_samples);
    for (int i = 0; i < number_samples; ++i) {
        single_chirp[i] = std::cos(phase[i]);
    }

    double max_val = 0.0;
    for (int16_t val : single_chirp) {
        float abs_val = std::abs(val);
        if (abs_val > max_val) {
            max_val = abs_val;
        }
    }

    if (max_val > 0.0) {
        for (float& val : single_chirp) {
            val = (uint16_t) ((val / max_val) * 32767.0);
        }
    }

    // optional window function
    if (window) {
        std::vector<float> hamm(number_samples);
        if (number_samples > 1) {
            float denom = number_samples - 1.0;
            float pi = M_PI;
            for (int n = 0; n < number_samples; ++n) {
                hamm[n] = 0.54 - 0.46 * std::cos(2.0 * pi * n / denom);
            }
        } else {
            hamm[0] = 1.0; // trivial case
        }
        for (int i = 0; i < number_samples; ++i) {
            single_chirp[i] *= hamm[i];
        }
    }

    for (int i = 0; i < number_samples; ++i) {  // assuming numSamples is a typo and should be number_samples
        outputBuffer[i] = static_cast<float>(single_chirp[i]);
    }
    LOGD("Generated chirp of %d samples for playback.", number_samples);
    return outputBuffer;
}

std::vector<float> correlate(const std::vector<float>& a, const std::vector<float>& b) {
    size_t len_a = a.size();
    size_t len_b = b.size();
    if (len_a == 0 || len_b == 0) return {};

    // Compute minimum padded length (len_a + len_b - 1)
    size_t nfft = len_a + len_b - 1;

    // Allocate Kiss FFT configs
    kiss_fft_cfg fwd = kiss_fft_alloc(static_cast<int>(nfft), 0, nullptr, nullptr);
    kiss_fft_cfg inv = kiss_fft_alloc(static_cast<int>(nfft), 1, nullptr, nullptr);
    if (!fwd || !inv) {
        // Handle allocation failure
        if (fwd) kiss_fft_free(fwd);
        if (inv) kiss_fft_free(inv);
        return {};
    }

    // Pad inputs with zeros and treat as complex (imag = 0)
    std::vector<std::complex<float>> padded_a(nfft, 0.0);
    std::vector<std::complex<float>> padded_b(nfft, 0.0);
    for (size_t i = 0; i < len_a; ++i) padded_a[i] = a[i];
    for (size_t i = 0; i < len_b; ++i) padded_b[i] = b[i];

    // Compute FFTs
    std::vector<std::complex<float>> fft_a(nfft);
    std::vector<std::complex<float>> fft_b(nfft);
    kiss_fft(fwd, reinterpret_cast<kiss_fft_cpx*>(padded_a.data()), reinterpret_cast<kiss_fft_cpx*>(fft_a.data()));
    kiss_fft(fwd, reinterpret_cast<kiss_fft_cpx*>(padded_b.data()), reinterpret_cast<kiss_fft_cpx*>(fft_b.data()));

    // Multiply: fft_a * conj(fft_b)
    for (size_t k = 0; k < nfft; ++k) {
        fft_a[k] *= std::conj(fft_b[k]);
    }

    // Inverse FFT
    std::vector<std::complex<float>> corr(nfft);
    kiss_fft(inv, reinterpret_cast<kiss_fft_cpx*>(fft_a.data()), reinterpret_cast<kiss_fft_cpx*>(corr.data()));

    // Free configs
    kiss_fft_free(fwd);
    kiss_fft_free(inv);

    // Extract real part, normalize by nfft (Kiss FFT doesn't scale automatically)
    std::vector<float> result(nfft);
    for (size_t i = 0; i < nfft; ++i) {
        result[i] = corr[(i + nfft - (len_b - 1)) % nfft].real() / static_cast<float>(nfft);  // Shift for zero-centered lags
    }

    return result;
}

std::vector<float> findPeaks(const std::vector<float>& correlation, const std::vector<float>& reference_chirp, const float threshold = 0.3){
    std::vector<Peak> peaks;
    
    float max_corr = 0.0;
    for (float val : correlation) {
        float abs_val = std::abs(val) 
        if (abs_val > max_corr) {
            max_corr = abs_val;
        }
    }
    
    int chirp_len = reference_chirp.size();
    // find peak, the gap between two peaks must be larger than a threshold
    int min_distance = FS * 0.01;  
    
    for (size_t i = min_distance; i < correlation.size() - min_distance; ++i) {
        float normalized_val = correlation[i] / max_corr;
        // check if larger than threshold and min distance
        if (normalized_val > threshold) {
            bool is_peak = true;
            // check if the local maximum
            for (int j = -min_distance/2; j <= min_distance/2; ++j) {
                if (j != 0 && correlation[i + j] > correlation[i]) {
                    is_peak = false;
                    break;
                }
            }
            
            if (is_peak) {
                Peak peak;
                peak.index = i;
                peak.value = normalized_val;
                // include time delay
                int lag = i - chirp_len + 1;
                float time_delay = (float)lag / FS;
                
                peak.distance_m = (time_delay * V_S) / 2.0;
                
                if (peak.distance_m > 0) {
                    peaks.push_back(peak);
                    // LOGD("Peak found at index %d, normalized value: %.3f, distance: %.2f m", 
                    //      peak.index, peak.value, peak.distance_m);
                }
            }
        }
    }
    
    return peaks;
}

float distanceEstimation(){
    if(fullRecordingBuffer.empty() || playbackBuffer.empty()){
        return -1.0;
    }
    std::vector<float> correlation = correlate(fullRecordingBuffer, playbackBuffer);
    std::vector<float> peaks = findPeaks(correlation, playbackBuffer, 0.3);
    // minimum distance measured will be at least 20cm
    float min_distance = 0.2;
    Peak best_peak;
    best_peak.value = 0.0;
    // find the best peak
    for(Peak& peak:peaks){
        if(peak.distance_m>min_distance && peak.value > best_peak.value){
            best_peak = peak;
        }
    }
    if(best_peak.value == 0.0){
        return -1.0;
    }

    return best_peak.distance_m;
}

// Define a new JNI function to start the process
extern "C" JNIEXPORT void JNICALL
Java_com_ece420_lab4_MainActivity_startEcho(JNIEnv *env, jobject thiz) {
    LOGD("Starting playback and recording process");

    // Clear previous data
    fullRecordingBuffer.clear();
    playbackBuffer.clear();
    playbackBufferPos = 0;

    // Pre-generate the chirp to play.



    playbackBuffer = generatePlaybackAudio(true);

    // Reserve space for the recording buffer to avoid re-allocations
    // Example: Reserve 10 seconds of recording space
    fullRecordingBuffer.reserve(FS * 10);

}

// Return measured distance
Java_com_ece420_lab4_MainActivity_getDistanceUpdate(JNIEnv *env, jclass){
    float distance = distanceEstimation()
    return distance;
}



float lastFreqDetected = -1;

//Modified this to write the recording buffer

void ece420ProcessFrame(sample_buf *dataBuf) {
    // This function now just copies the recorded data into our long-term buffer.

    //Convert PCM 16-bit to float
    float bufferIn[FRAME_SIZE];
    for (int i = 0; i < FRAME_SIZE; i++) {
        int16_t val = ((uint16_t)dataBuf->buf_[2 * i]) | (((uint16_t)dataBuf->buf_[2 * i + 1]) << 8);
        bufferIn[i] = (float)val;
    }

    // Append the data to our main recording buffer
    // This needs to be thread-safe if other threads access it, but in this design,
    // only the recorder thread writes to it.
    fullRecordingBuffer.insert(fullRecordingBuffer.end(), bufferIn, bufferIn + FRAME_SIZE);



}













