#include "process_recording.h"
#include <cmath>
#include <complex>
#include <algorithm>
#include <cstring>
#include <cassert>
#include <jni.h>


#include "kiss_fft/kiss_fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


static constexpr float SPEED_OF_SOUND_MS = 343.0f;


// convert little-endian bytes -> int16_t samples
static std::vector<int16_t> bytesToInt16(const std::vector<uint8_t>& pcmBytes) {
    size_t bytes = pcmBytes.size();
    size_t nSamples = bytes / 2;
    std::vector<int16_t> samples;
    samples.reserve(nSamples);
    for (size_t i = 0; i < nSamples; ++i) {
        uint8_t low = pcmBytes[2*i];
        uint8_t high = pcmBytes[2*i + 1];
        int16_t s = static_cast<int16_t>((high << 8) | low); // little-endian
        samples.push_back(s);
    }
    return samples;
}

// convert int16 samples -> float normalized (-1..1)
static std::vector<float> int16ToFloat(const std::vector<int16_t>& in) {
    std::vector<float> out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        out.push_back(static_cast<float>(in[i]) / static_cast<float>(INT16_MAX));
    }
    return out;
}










// ----------------- Correlation / Peak-finding for chirp ranging -----------------

// Cross-correlate two real float sequences using Kiss FFT in frequency domain.
// Returns correlation of length (len_a + len_b - 1). Output arranged with lag zero at index (len_b - 1).
static std::vector<float> correlate(const std::vector<float>& a, const std::vector<float>& b) {
    size_t len_a = a.size();
    size_t len_b = b.size();
    if (len_a == 0 || len_b == 0) return {};

    size_t nfft = len_a + len_b - 1; // as in your snippet

    // Allocate Kiss FFT configs
    kiss_fft_cfg fwd = kiss_fft_alloc(static_cast<int>(nfft), 0, nullptr, nullptr);
    kiss_fft_cfg inv = kiss_fft_alloc(static_cast<int>(nfft), 1, nullptr, nullptr);
    if (!fwd || !inv) {
        if (fwd) kiss_fft_free(fwd);
        if (inv) kiss_fft_free(inv);
        return {};
    }

    // Prepare input buffers as kiss_fft_cpx arrays (real -> imag 0)
    std::vector<kiss_fft_cpx> in_a(nfft);
    std::vector<kiss_fft_cpx> in_b(nfft);
    std::memset(in_a.data(), 0, nfft * sizeof(kiss_fft_cpx));
    std::memset(in_b.data(), 0, nfft * sizeof(kiss_fft_cpx));
    for (size_t i = 0; i < len_a; ++i) { in_a[i].r = a[i]; in_a[i].i = 0.0f; }
    for (size_t i = 0; i < len_b; ++i) { in_b[i].r = b[i]; in_b[i].i = 0.0f; }

    // FFT outputs
    std::vector<kiss_fft_cpx> fft_a(nfft);
    std::vector<kiss_fft_cpx> fft_b(nfft);
    kiss_fft(fwd, in_a.data(), fft_a.data());
    kiss_fft(fwd, in_b.data(), fft_b.data());

    // Multiply: fft_a * conj(fft_b)
    std::vector<kiss_fft_cpx> prod(nfft);
    for (size_t k = 0; k < nfft; ++k) {
        float ar = fft_a[k].r;
        float ai = fft_a[k].i;
        float br = fft_b[k].r;
        float bi = fft_b[k].i;
        // conj(b) = (br, -bi)
        prod[k].r = ar * br + ai * bi;
        prod[k].i = ai * br - ar * bi;
    }

    // inverse FFT
    std::vector<kiss_fft_cpx> corr_cpx(nfft);
    kiss_fft(inv, prod.data(), corr_cpx.data());

    kiss_fft_free(fwd);
    kiss_fft_free(inv);

    // Extract real part and normalize by nfft (kiss_fft doesn't scale)
    std::vector<float> result(nfft);
    for (size_t i = 0; i < nfft; ++i) {
        // shift so index (len_b-1) corresponds to lag 0
        size_t shifted_idx = (i + nfft - (len_b - 1)) % nfft;
        result[shifted_idx] = corr_cpx[i].r / static_cast<float>(nfft);
    }
    return result;
}

struct Peak {
    int index;
    float value;     // normalized 0..1 relative to max
    float distance_m;
};

// find peaks in correlation and convert to distances (meters)
// reference_chirp: the chirp samples (float) used for computing correlation length
static std::vector<Peak> findPeaks(const std::vector<float>& correlation,
                                   const std::vector<float>& reference_chirp,
                                   int sampleRate,
                                   float threshold = 0.3f)
{
    std::vector<Peak> peaks;
    if (correlation.empty() || reference_chirp.empty()) return peaks;

    float max_corr = 0.0f;
    for (float val : correlation) {
        float abs_val = (val >= 0.0f) ? val : -val;
        if (abs_val > max_corr) max_corr = abs_val;
    }
    if (max_corr <= 0.0f) return peaks;

    int chirp_len = static_cast<int>(reference_chirp.size());
    // minimum distance between peaks (in samples) — here 10ms default
    int min_distance = std::max<int>(1, (int)std::floor(sampleRate * 0.01f));

    // scan correlation for peaks avoiding edges by min_distance
    for (size_t i = min_distance; i + min_distance < correlation.size(); ++i) {
        float normalized_val = correlation[i] / max_corr;
        if (normalized_val > threshold) {
            bool is_peak = true;
            // check local max across neighborhood
            int half_win = min_distance / 2;
            for (int j = -half_win; j <= half_win; ++j) {
                if (j == 0) continue;
                if (correlation[i + j] > correlation[i]) { is_peak = false; break; }
            }
            if (!is_peak) continue;
            Peak p;
            p.index = static_cast<int>(i);
            p.value = normalized_val;
            int lag = p.index - (chirp_len - 1); // because correlation lag 0 is at index (chirp_len - 1)
            float time_delay = static_cast<float>(lag) / static_cast<float>(sampleRate);
            // two-way travel: distance = (time_delay * speed) / 2
            p.distance_m = (time_delay * SPEED_OF_SOUND_MS) / 2.0f;
            // only add positive, sensible distances
            if (p.distance_m > 0.0f) peaks.push_back(p);
        }
    }
    return peaks;
}

// Top-level function used earlier as distanceEstimation; now embedded in analyzeRecordedBuffer
static double estimateDistanceFromBuffers(const std::vector<float>& recorded,
                                          const std::vector<float>& reference_chirp,
                                          int sampleRate,
                                          float correlationThreshold = 0.3f,
                                          float minDistanceMeters = 0.2f)
{
    if (recorded.empty() || reference_chirp.empty()) return -1.0;
    std::vector<float> corr = correlate(recorded, reference_chirp);
    if (corr.empty()) return -1.0;
    std::vector<Peak> peaks = findPeaks(corr, reference_chirp, sampleRate, correlationThreshold);
    if (peaks.empty()) return -1.0;
    // find best peak above minDistanceMeters
    Peak best; best.value = 0.0f; best.distance_m = -1.0f;
    for (const Peak& p : peaks) {
        if (p.distance_m > minDistanceMeters && p.value > best.value) {
            best = p;
        }
    }
    if (best.value <= 0.0f) return -1.0;
    return static_cast<double>(best.distance_m);
}

// ----------------- Main analyze function -----------------

AnalysisResult analyzeRecordedBuffer(const std::vector<uint8_t>& pcmBytes, int sampleRate,
                                     const std::vector<uint8_t>* referenceChirpBytes /*=nullptr*/) {
    AnalysisResult res;

    res.distance_valid = false;
    res.distance_m = -1.0;

    // Convert to int16 samples
    std::vector<int16_t> samples = bytesToInt16(pcmBytes);
    if (samples.empty()) return res;



    // If a reference chirp is provided, attempt correlation-based ranging
    if (referenceChirpBytes && !referenceChirpBytes->empty()) {
        // Convert both recorded and reference to float normalized -1..1
        std::vector<float> recordedFloat = int16ToFloat(samples);

        // reference bytes -> int16 -> float
        std::vector<int16_t> refInt16 = bytesToInt16(*referenceChirpBytes);
        if (!refInt16.empty()) {
            std::vector<float> refFloat = int16ToFloat(refInt16);



            double dist = estimateDistanceFromBuffers(recordedFloat, refFloat, sampleRate, 0.30f, 0.20f);
            if (dist > 0.0) {
                res.distance_valid = true;
                res.distance_m = dist;
            } else {
                res.distance_valid = false;
                res.distance_m = -1.0;
            }
        }
    }

    return res;
}


extern "C"
JNIEXPORT jobject JNICALL
Java_com_ece420_lab2_MainActivity_analyzeRecordedBuffer(JNIEnv *env, jclass clazz,
                                                        jbyteArray pcmBytes, jint sampleRate,
                                                        jbyteArray referenceChirpBytes) {
    // Convert pcmBytes to std::vector<uint8_t>
    std::vector<uint8_t> pcmVec;
    if (pcmBytes != NULL) {
        jsize len = env->GetArrayLength(pcmBytes);
        pcmVec.resize(static_cast<size_t>(len));
        env->GetByteArrayRegion(pcmBytes, 0, len, reinterpret_cast<jbyte*>(pcmVec.data()));
    }

    // Handle optional referenceChirpBytes
    std::vector<uint8_t> refVec;
    const std::vector<uint8_t>* refPtr = nullptr;
    if (referenceChirpBytes != NULL) {
        jsize len = env->GetArrayLength(referenceChirpBytes);
        refVec.resize(static_cast<size_t>(len));
        env->GetByteArrayRegion(referenceChirpBytes, 0, len, reinterpret_cast<jbyte*>(refVec.data()));
        refPtr = &refVec;
    }

    // Call the C++ method
    AnalysisResult res = analyzeRecordedBuffer(pcmVec, sampleRate, refPtr);

    // Create and populate the Java AnalysisResult object
    jclass resultClass = env->FindClass("com/ece420/lab2/MainActivity$AnalysisResult");
    if (resultClass == NULL) {
        // Handle error (e.g., throw exception or log)
        return NULL;
    }

    jmethodID constructor = env->GetMethodID(resultClass, "<init>", "()V");
    if (constructor == NULL) {
        return NULL;
    }

    jobject obj = env->NewObject(resultClass, constructor);
    if (obj == NULL) {
        return NULL;
    }

    jfieldID validField = env->GetFieldID(resultClass, "distance_valid", "Z");
    if (validField == NULL) {
        return NULL;
    }
    env->SetBooleanField(obj, validField, res.distance_valid ? JNI_TRUE : JNI_FALSE);

    jfieldID distField = env->GetFieldID(resultClass, "distance_m", "D");
    if (distField == NULL) {
        return NULL;
    }
    env->SetDoubleField(obj, distField, static_cast<jdouble>(res.distance_m));

    return obj;
}
