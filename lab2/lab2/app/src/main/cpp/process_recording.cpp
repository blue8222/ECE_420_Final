#include "process_recording.h"
#include <cmath>
#include <complex>
#include <algorithm>
#include <cstring>
#include <cassert>
#include <jni.h>
#include "debug_utils.h"
#include <android/log.h>
#include "android_debug.h"


#include "kiss_fft/kiss_fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


//static constexpr float SPEED_OF_SOUND_MS = 343.0f;


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

void takeFFT(const std::vector<float>& inReal,
                         std::vector<std::complex<float>>& outSpectrum,
                         std::vector<float>& outMagnitudes)
{
    int N = static_cast<int>(inReal.size());
    outSpectrum.clear();
    outMagnitudes.clear();
    if (N <= 0) return;

    // optional: copy and window
    std::vector<float> data(inReal);

    // Prepare kiss input and output buffers
    std::vector<kiss_fft_cpx> fin(N);
    std::vector<kiss_fft_cpx> fout(N);

    for (int i = 0; i < N; ++i) {
        fin[i].r = data[i];
        fin[i].i = 0;
    }

    // Allocate config (inverse_fft = 0 for forward FFT)
    kiss_fft_cfg cfg = kiss_fft_alloc(N, 0, nullptr, nullptr);
    if (!cfg) {
        // allocation failed
        return;
    }

    // Execute FFT
    kiss_fft(cfg, fin.data(), fout.data());

    // Convert to std::complex and compute magnitudes
    outSpectrum.resize(N);
    outMagnitudes.resize(N);
    for (int k = 0; k < N; ++k) {
        outSpectrum[k] = std::complex<float>(fout[k].r, fout[k].i);
        outMagnitudes[k] = std::hypot(fout[k].r, fout[k].i); // sqrt(r^2 + i^2)
    }

    // free config
    kiss_fft_free(cfg);

    // Note: kiss_fft does NOT normalize the transform. If you want amplitude to reflect
    // input-sample amplitudes, divide magnitudes (or the spectrum) by N:
    // for (auto &m : outMagnitudes) m /= static_cast<float>(N);
}


static Peak findMaxPeak(const std::vector<float>& correlation)
{
    Peak result{};
    result.index = -1;      // signal "no peak found"
    result.value = 0.0f;
    result.distance_m = 0.0f;

    if (correlation.empty()) return result;

    int best_idx = 0;
    float best_val = correlation[0];

    for (size_t i = 1; i < correlation.size(); ++i) {
        if (correlation[i] > best_val) {
            best_val = correlation[i];
            best_idx = static_cast<int>(i);
        }
    }

    result.index = best_idx;
    result.value = best_val;
    return result;
}

static Peak findFFTPeak(const std::vector<float>& arr,
                        int startIndex,
                        int endIndex)
{
    Peak result{};
    result.index = -1;      // signal "no peak found"
    result.value = 0.0f;
    result.distance_m = 0.0f;

    if (arr.empty()) return result;

    // clamp bounds defensively
    if (startIndex < 0) startIndex = 0;
    if (endIndex >= static_cast<int>(arr.size())) endIndex = static_cast<int>(arr.size()) - 1;
    if (startIndex > endIndex) return result; // invalid search range

    int best_idx = startIndex;
    float best_val = arr[startIndex];

    for (int i = startIndex + 1; i <= endIndex; ++i) {
        if (arr[i] > best_val) {
            best_val = arr[i];
            best_idx = i;
        }
    }

    result.index = best_idx;
    result.value = best_val;
    return result;
}

// Top-level function used earlier as distanceEstimation; now embedded in analyzeRecordedBuffer
static double estimateDistanceFromBuffers(const std::vector<float>& recorded,
                                          const std::vector<float>& reference_chirp,
                                          int sampleRate,
                                          float minDistanceMeters = 0.2f)
{
    if (recorded.empty() || reference_chirp.empty()) return -1.0;
    std::vector<float> corr = correlate(recorded, reference_chirp);
    if (corr.empty()) return -1.0;
    Peak peak = findMaxPeak(corr);

    int index = peak.index;
    double cancel_factor = 0.5;

    std::vector<float> arraySliced;

// safety check
    if (index < 0) index = 0;
    if (index > (int)recorded.size()) index = recorded.size();

    arraySliced = std::vector<float>(recorded.begin() + index, recorded.end());

    arraySliced.resize(reference_chirp.size(), 0.0f);

    LOGI("arraySliced.size() = %zu, reference_chirp.size() = %zu", arraySliced.size(), reference_chirp.size());
    assert(arraySliced.size() == reference_chirp.size());

    //apply cancelation

    for(int i = 0; i < arraySliced.size(); i++) {
        arraySliced[i] = arraySliced[i] - cancel_factor*reference_chirp[i];
    }

    //apply hamming window
    float denom = arraySliced.size() - 1.0f;

    for (int n = 0; n < arraySliced.size(); n++) {
        arraySliced[n] = 0.54f - 0.46f * std::cos(2.0f * M_PI * n / denom);
    }

    //multply signals
    std::vector<float> mult(arraySliced.size());

    for (int n = 0; n < arraySliced.size(); n++) {
        mult[n] = arraySliced[n]*reference_chirp[n];
    }

    //take FFT
    std::vector<std::complex<float>> FFT;
    std::vector<float> FFT_mag;

    takeFFT(mult, FFT, FFT_mag);

    Peak Distance = findFFTPeak(FFT_mag, 0, FFT_mag.size() - 1);

    return Distance.distance_m;





    //return static_cast<double>(peak.distance_m);
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



            double dist = estimateDistanceFromBuffers(recordedFloat, refFloat, sampleRate, 0.20f);
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
