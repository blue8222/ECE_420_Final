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
#include "chirp_generate.h"


#include "kiss_fft/kiss_fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define V_s 343.0




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
             std::vector<float>& outMagnitudes,
             bool applyHamming = false,
             bool returnDb = false)
{
    int N = static_cast<int>(inReal.size());
    outSpectrum.clear();
    outMagnitudes.clear();
    if (N <= 0) return;

    // copy input (and optionally apply Hamming window)
    std::vector<float> data(inReal);
    if (applyHamming) {
        float denom = (N > 1) ? static_cast<float>(N - 1) : 1.0f;
        for (int n = 0; n < N; ++n) {
            float w = 0.54f - 0.46f * std::cos(2.0f * M_PI * n / denom);
            data[n] *= w;
        }
    }

    // Prepare kiss input and output buffers
    std::vector<kiss_fft_cpx> fin(N);
    std::vector<kiss_fft_cpx> fout(N);
    for (int i = 0; i < N; ++i) {
        fin[i].r = data[i];
        fin[i].i = 0.0f;
    }

    // Allocate config (forward FFT)
    kiss_fft_cfg cfg = kiss_fft_alloc(N, 0, nullptr, nullptr);
    if (!cfg) {
        return;
    }

    // Execute FFT
    kiss_fft(cfg, fin.data(), fout.data());

    // Build full complex spectrum (size N)
    outSpectrum.resize(N);
    for (int k = 0; k < N; ++k) {
        outSpectrum[k] = std::complex<float>(fout[k].r, fout[k].i);
    }

    // Normalize and compute single-sided magnitudes (bins 0 .. halfN-1)
    int halfN = N/2 + 1; // works for even/odd N
    outMagnitudes.resize(halfN);

    const float scale = 1.0f / static_cast<float>(N); // normalization by N
    for (int k = 0; k < halfN; ++k) {
        float r = fout[k].r;
        float i = fout[k].i;
        float mag = std::hypot(r, i) * scale; // normalized magnitude

        // for real input, double the energy in non-DC and non-Nyquist bins
        if (k != 0 && !(N % 2 == 0 && k == N/2)) {
            mag *= 2.0f;
        }

        if (returnDb) {
            // convert to dB (guard against log(0))
            float db = 20.0f * std::log10f(mag + 1e-12f);
            outMagnitudes[k] = db;
        } else {
            outMagnitudes[k] = mag;
        }
    }

    kiss_fft_free(cfg);
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

int maxIndex(std::vector<float>& arr,
             int startIndex_,
             int endIndex_)
{

    // Validate bounds
    if (startIndex_ < 0 || endIndex_ > static_cast<int>(arr.size()) || startIndex_ >= endIndex_) {
        return -1; // or throw std::out_of_range(...)
    }

    int maxIdx = startIndex_;
    float maxVal = arr[startIndex_];

    for (int i = startIndex_ + 1; i < endIndex_; ++i) {
        if (arr[i] > maxVal) {
            maxVal = arr[i];
            maxIdx = i;
        }
    }

    return maxIdx;
}


// Top-level function used earlier as distanceEstimation; now embedded in analyzeRecordedBuffer
static double estimateDistanceFromBuffers(const std::vector<float>& recorded,
                                          const std::vector<float>& reference_chirp,
                                          int sampleRate,
                                          std::vector<float>& FFT_return)
{
    if (recorded.empty() || reference_chirp.empty() || sampleRate <= 0) return -1.0;

    std::vector<float> corr = correlate(recorded, reference_chirp);
    if (corr.empty()) return -1.0;

    Peak peak = findMaxPeak(corr);
    if (peak.index < 0) return -1.0;

    // convert correlation index -> lag (samples)
    // correlate() places lag=0 at index (len_b - 1)
    int refLen = static_cast<int>(reference_chirp.size());
    int lag = peak.index - (refLen - 1);
    if (lag < 0) lag = 0;
    if (lag > static_cast<int>(recorded.size())) lag = static_cast<int>(recorded.size());

    // slice recorded from lag
    std::vector<float> arraySliced;
    if (lag < static_cast<int>(recorded.size()))
        arraySliced = std::vector<float>(recorded.begin() + lag, recorded.end());
    else
        arraySliced.clear();

    // ensure we have at least reference length; pad/truncate
    arraySliced.resize(reference_chirp.size(), 0.0f);

    // cancellation
    const float cancel_factor = 0.5f;
    for (size_t i = 0; i < arraySliced.size(); ++i) {
        arraySliced[i] = arraySliced[i] - cancel_factor * reference_chirp[i];
    }

    // apply Hamming window (guard denom)
    int N = static_cast<int>(arraySliced.size());
    if (N <= 0) return -1.0;
    if (N > 1) {
        float denom = static_cast<float>(N - 1);
        for (int n = 0; n < N; ++n) {
            float w = 0.54f - 0.46f * std::cos(2.0f * M_PI * n / denom);
            arraySliced[n] *= w;
        }
    }

    // multiply with reference
    std::vector<float> mult(N);
    for (int n = 0; n < N; ++n) {
        mult[n] = arraySliced[n] * reference_chirp[n];
    }

    // take FFT
    std::vector<std::complex<float>> FFT;
    std::vector<float> FFT_mag;
    takeFFT(mult, FFT, FFT_mag);
    FFT_return = FFT_mag; // pass back magnitudes for UI/debug

    if (FFT_mag.empty()) return -1.0;

   // Peak F_p = findFFTPeak(FFT_mag, 50, static_cast<int>(FFT_mag.size()) - 1);
    Peak F_p;
    F_p.index = maxIndex(FFT_mag, 10, 1000);
    if (F_p.index < 0) return -1.0;

    // convert bin -> frequency (Hz)
    // takeFFT produced magnitudes length = halfN (N/2+1), but frequency resolution is sampleRate / N
    double freq_hz = static_cast<double>(F_p.index) * static_cast<double>(sampleRate) / static_cast<double>(N);
    LOGI("N = %d", N);

    // Validate chirp params (sweepTime, bandwidth) - make sure they are non-zero and available
    if (bandwidth <= 0.0 || sweepTime <= 0.0) {
        LOGI("bandwidth or sweepTime invalid: bw=%f, t=%f", bandwidth, sweepTime);
        return -1.0;
    }

    // compute range R and distance D
    float R = (V_s * sweepTime * freq_hz) / static_cast<float>(bandwidth);

    LOGI("R = %f", R);
    LOGI("V_s = %f", V_s);
    LOGI("sweepTime= %f", sweepTime);
    LOGI("freq_hz = %f", freq_hz);

    float D = R / 2.0;

    LOGI("D = %f", D);

    return D;
}

// ----------------- Main analyze function -----------------

AnalysisResult analyzeRecordedBuffer(const std::vector<uint8_t>& pcmBytes, int sampleRate,
                                     const std::vector<uint8_t>* referenceChirpBytes /*=nullptr*/) {
    AnalysisResult res;

    res.distance_valid = false;
    res.distance_m = -1.0;
    res.FFT = {};

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


            std::vector<float> FFT_return;
            double dist = estimateDistanceFromBuffers(recordedFloat, refFloat, sampleRate, FFT_return);

            LOGI("dist = %f", dist);




            res.FFT = FFT_return;

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




    // -------- Call C++ analyzer --------
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



    jfieldID FFTField = env->GetFieldID(resultClass, "FFT", "[F");
    if (FFTField == NULL) {
        return NULL;
    }

// create jfloatArray (size as jint)
    jsize fftSize = static_cast<jsize>(res.FFT.size());
    jfloatArray jFFT = env->NewFloatArray(fftSize);
    if (jFFT != NULL && fftSize > 0) {
        // SetFloatArrayRegion expects jfloat*, so cast from float*
        env->SetFloatArrayRegion(jFFT, 0, fftSize, reinterpret_cast<const jfloat*>(res.FFT.data()));
    }
// attach array to the Java object

    env->SetObjectField(obj, FFTField, jFFT);







    return obj;
}
