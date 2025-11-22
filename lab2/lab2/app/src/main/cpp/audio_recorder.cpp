// audio_recorder.cpp  -- collects large PCM buffer while preserving queue behavior

#include <cstring>
#include <cstdlib>
#include <vector>
#include <mutex>
#include <atomic>

#include <jni.h>

#include "audio_recorder.h"



// Add near top of file (after includes and existing globals)
static JavaVM* gJvm = nullptr;  // set by JNI_OnLoad

// Called when the native library is loaded - cache the JavaVM
jint JNI_OnLoad(JavaVM* vm, void* /*reserved*/) {
    gJvm = vm;
    return JNI_VERSION_1_6;
}

// Helper: call MainActivity.onNativeAnalysisResult(boolean, int)
static void notifyJavaAnalysis(bool voiced, int freq) {
    if (!gJvm) return;

    JNIEnv* env = nullptr;
    bool attached = false;
    jint getEnvStat = gJvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    if (getEnvStat == JNI_EDETACHED) {
        // Attach current native thread to JVM so we can call Java methods
        if (gJvm->AttachCurrentThread(&env, nullptr) != 0) {
            return; // can't attach
        }
        attached = true;
    } else if (getEnvStat != JNI_OK) {
        return; // unexpected
    }

    jclass clazz = env->FindClass("com/ece420/lab2/MainActivity");
    if (clazz) {
        jmethodID mid = env->GetStaticMethodID(clazz, "onNativeAnalysisResult", "(ZI)V");
        if (mid) {
            env->CallStaticVoidMethod(clazz, mid, (jboolean)voiced, (jint)freq);
        }
        env->DeleteLocalRef(clazz);
    }

    if (attached) {
        gJvm->DetachCurrentThread();
    }
}

/*
 * bqRecorderCallback(): called for every buffer is full;
 *                       pass directly to handler
 */
void bqRecorderCallback(SLAndroidSimpleBufferQueueItf bq, void *rec) {
    (static_cast<AudioRecorder *>(rec))->ProcessSLCallback(bq);
}

// Global (module-level) collector storage and control.
// We place it here to avoid touching the header; this recorder is single-instance
// in your app, so a module global is acceptable.
static std::vector<uint8_t> g_recordedPCM;
static std::mutex          g_recordedMutex;
static std::atomic<bool>   g_collecting(false);
static size_t              g_expectedCapacityBytes = 0;

static std::vector<uint8_t> g_completedPCM;   // finished recording ready for Java

// Java JNI helpers (call from Java to control collection):
// Java side signatures:
//   public static native void nativeStartCollect(int expectedBytes);
//   public static native byte[] nativeStopAndGetRecording();


extern "C" JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_nativeStartCollect(JNIEnv * /*env*/, jclass /*clazz*/, jint expectedBytes) {
    std::lock_guard<std::mutex> lock(g_recordedMutex);
    g_recordedPCM.clear();
    g_recordedPCM.shrink_to_fit();
    g_completedPCM.clear();
    g_completedPCM.shrink_to_fit();
    if (expectedBytes > 0) {
        g_recordedPCM.reserve(static_cast<size_t>(expectedBytes));
        g_expectedCapacityBytes = static_cast<size_t>(expectedBytes);
    } else {
        g_expectedCapacityBytes = 0;
    }
    g_collecting.store(true);
}

extern "C" JNIEXPORT jbyteArray JNICALL
Java_com_ece420_lab2_MainActivity_nativeStopAndGetRecording(JNIEnv *env, jclass /*clazz*/) {
    // Stop collecting first
    g_collecting.store(false);

    std::lock_guard<std::mutex> lock(g_recordedMutex);

    // Prefer returning completed buffer if the callback moved it there
    std::vector<uint8_t> *src = nullptr;
    if (!g_completedPCM.empty()) {
        src = &g_completedPCM;
    } else {
        src = &g_recordedPCM;
    }

    jsize outLen = static_cast<jsize>(src->size());
    LOGI("nativeStopAndGetRecording outLen = %d" , outLen);

    jbyteArray outArr = env->NewByteArray(outLen);
    if (outArr && outLen > 0) {
        env->SetByteArrayRegion(outArr, 0, outLen, reinterpret_cast<const jbyte*>(src->data()));
    }

    // clear both buffers after handing data back to Java
    g_recordedPCM.clear();
    g_recordedPCM.shrink_to_fit();
    g_completedPCM.clear();
    g_completedPCM.shrink_to_fit();
    g_expectedCapacityBytes = 0;

    return outArr;
}
// ---- Original AudioRecorder methods, modified to append to g_recordedPCM ----

void AudioRecorder::ProcessSLCallback(SLAndroidSimpleBufferQueueItf bq) {
#ifdef ENABLE_LOG
    recLog_->logTime();
#endif
    assert(bq == recBufQueueItf_);
    sample_buf *dataBuf = NULL;
    if (!devShadowQueue_->front(&dataBuf)) {
        // unexpected: callback but no devShadow buffer available
        LOGE("ProcessSLCallback: devShadowQueue empty");
        return;
    }
    devShadowQueue_->pop();
    dataBuf->size_ = dataBuf->cap_;           // device only calls us when it is really full



    // Append to recQueue for downstream processing as before
    recQueue_->push(dataBuf);

    // Append raw bytes into global collector if enabled.

    if (g_collecting.load()) {
        std::vector<uint8_t> localCopy;
        {
            std::lock_guard<std::mutex> lock(g_recordedMutex);
            // If expected capacity was set, avoid growing beyond it (optional safety).
            LOGI("Callback: dataBuf->cap_=%d dataBuf->size_=%d g_recordedPCM.size()=%zu g_expectedCapacityBytes=%zu g_collecting=%d",
                 dataBuf->cap_, dataBuf->size_, g_recordedPCM.size(), g_expectedCapacityBytes, (int)g_collecting.load());
            if (g_expectedCapacityBytes == 0 || (g_recordedPCM.size() + dataBuf->size_ <= g_expectedCapacityBytes)) {
                g_recordedPCM.insert(g_recordedPCM.end(), dataBuf->buf_, dataBuf->buf_ + dataBuf->size_);
            } else {
                size_t remaining = g_expectedCapacityBytes > g_recordedPCM.size()
                                   ? (g_expectedCapacityBytes - g_recordedPCM.size())
                                   : 0;
                if (remaining > 0) {
                    g_recordedPCM.insert(g_recordedPCM.end(), dataBuf->buf_, dataBuf->buf_ + (std::min<size_t>(remaining, dataBuf->size_)));
                } else {
                    // Already full; drop further data
                }
            }

            // If we reached or exceeded expected capacity, take a copy and reset collecting
            if (g_expectedCapacityBytes > 0 && g_recordedPCM.size() >= g_expectedCapacityBytes) {
                // Move the collected bytes to the completed buffer under lock, then stop collecting.
                g_completedPCM = std::move(g_recordedPCM); // ownership transferred; g_recordedPCM now empty
                g_recordedPCM.clear();
                g_recordedPCM.shrink_to_fit();
                // stop collecting until Java restarts it
                g_collecting.store(false);
                g_expectedCapacityBytes = 0;
                // we still have a local copy variable out of the lock in previous code path for analysis,
                // but now the data we want to return to Java lives in g_completedPCM.
                // If you want to analyze here as well, you can move from g_completedPCM or use the existing code.
                // (If analysis needs to happen outside the lock, copy or move to a local vector.)
                localCopy = g_completedPCM; // optional: make a local copy for analysis
            }
        } // unlock here

        // If we made a local copy, analyze it now (off the lock)
        if (!localCopy.empty()) {
            // Example analysis: simple RMS + zero-crossing frequency estimate

            // interpret bytes as int16 little-endian
            size_t bytes = localCopy.size();
            const size_t numSamples = bytes / 2;
            if (numSamples == 0) {
                notifyJavaAnalysis(false, 0);
            } else {
                // compute RMS
                double sumSq = 0.0;
                int16_t *samples = reinterpret_cast<int16_t*>(localCopy.data());
                // careful: depends on the platform's alignment/endianness; Android little-endian matches our data
                int zeroCrossings = 0;
                int16_t prev = samples[0];
                for (size_t i = 0; i < numSamples; ++i) {
                    int16_t s = samples[i];
                    double d = (double)s;
                    sumSq += d * d;
                    if ((s >= 0 && prev < 0) || (s < 0 && prev >= 0)) {
                        zeroCrossings++;
                    }
                    prev = s;
                }
                double rms = std::sqrt(sumSq / (double)numSamples);

                // compute simple zero-crossing based frequency estimate:
                // freq ≈ zeroCrossings * sampleRate / (2 * numSamples)
                int sampleRate = 16000; // default fallback
                // if AudioRecorder has sampleInfo_ with sample rate, use it. Adjust to your sampleInfo field name.
#ifdef HAS_SAMPLEINFO_RATE
                sampleRate = (int) sampleInfo_.sampleRate; // example; modify to actual field if available
#endif

                double estimatedFreq = 0.0;
                if (numSamples > 0) {
                    estimatedFreq = (double)zeroCrossings * (double)sampleRate / (2.0 * (double)numSamples);
                }

                // Choose threshold for voiced vs unvoiced (example: RMS > 500)
                bool voiced = (rms > 500.0); // tweak this threshold based on real data

                int freqInt = static_cast<int>(estimatedFreq + 0.5);

                // Notify Java with results
                notifyJavaAnalysis(voiced, freqInt);
            }
        }
    }
}

AudioRecorder::AudioRecorder(SampleFormat *sampleFormat, SLEngineItf slEngine) :
        freeQueue_(nullptr), recQueue_(nullptr), devShadowQueue_(nullptr),
        callback_(nullptr)
{
    SLresult result;
    sampleInfo_ = *sampleFormat;
    SLAndroidDataFormat_PCM_EX format_pcm;
    ConvertToSLSampleFormat(&format_pcm, &sampleInfo_);

    // configure audio source
    SLDataLocator_IODevice loc_dev = {SL_DATALOCATOR_IODEVICE,
                                      SL_IODEVICE_AUDIOINPUT,
                                      SL_DEFAULTDEVICEID_AUDIOINPUT,
                                      NULL };
    SLDataSource audioSrc = {&loc_dev, NULL };

    // configure audio sink
    SLDataLocator_AndroidSimpleBufferQueue loc_bq = {
            SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE,
            DEVICE_SHADOW_BUFFER_QUEUE_LEN };

    SLDataSink audioSnk = {&loc_bq, &format_pcm};

    // create audio recorder
    // (requires the RECORD_AUDIO permission)
    const SLInterfaceID id[2] = {SL_IID_ANDROIDSIMPLEBUFFERQUEUE,
                                 SL_IID_ANDROIDCONFIGURATION };
    const SLboolean req[2] = {SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE};
    result = (*slEngine)->CreateAudioRecorder(slEngine,
                                              &recObjectItf_,
                                              &audioSrc,
                                              &audioSnk,
                                              sizeof(id)/sizeof(id[0]),
                                              id, req);
    SLASSERT(result);

    // Configure the voice recognition preset which has no
    // signal processing for lower latency.
    SLAndroidConfigurationItf inputConfig;
    result = (*recObjectItf_)->GetInterface(recObjectItf_,
                                            SL_IID_ANDROIDCONFIGURATION,
                                            &inputConfig);
    if (SL_RESULT_SUCCESS == result) {
        SLuint32 presetValue = SL_ANDROID_RECORDING_PRESET_VOICE_RECOGNITION;
        (*inputConfig)->SetConfiguration(inputConfig,
                                         SL_ANDROID_KEY_RECORDING_PRESET,
                                         &presetValue,
                                         sizeof(SLuint32));
    }
    result = (*recObjectItf_)->Realize(recObjectItf_, SL_BOOLEAN_FALSE);
    SLASSERT(result);
    result = (*recObjectItf_)->GetInterface(recObjectItf_,
                                            SL_IID_RECORD, &recItf_);
    SLASSERT(result);

    result = (*recObjectItf_)->GetInterface(recObjectItf_,
                                            SL_IID_ANDROIDSIMPLEBUFFERQUEUE, &recBufQueueItf_);
    SLASSERT(result);

    result = (*recBufQueueItf_)->RegisterCallback(recBufQueueItf_,
                                                  bqRecorderCallback, this);
    SLASSERT(result);

    devShadowQueue_ = new AudioQueue(DEVICE_SHADOW_BUFFER_QUEUE_LEN);
    assert(devShadowQueue_);
#ifdef ENABLE_LOG
    std::string name = "rec";
    recLog_ = new AndroidLog(name);
#endif
}

SLboolean AudioRecorder::Start(void) {
    if(!freeQueue_ || !recQueue_ || !devShadowQueue_) {
        LOGE("====NULL poiter to Start(%p, %p, %p)", freeQueue_, recQueue_, devShadowQueue_);
        return SL_BOOLEAN_FALSE;
    }
    audioBufCount = 0;

    SLresult result;
    // in case already recording, stop recording and clear buffer queue
    result = (*recItf_)->SetRecordState(recItf_, SL_RECORDSTATE_STOPPED);
    SLASSERT(result);
    result = (*recBufQueueItf_)->Clear(recBufQueueItf_);
    SLASSERT(result);

    for(int i =0; i < RECORD_DEVICE_KICKSTART_BUF_COUNT; i++ ) {
        sample_buf *buf = NULL;
        if(!freeQueue_->front(&buf)) {
            LOGE("=====OutOfFreeBuffers @ startingRecording @ (%d)", i);
            break;
        }
        freeQueue_->pop();
        assert(buf->buf_ && buf->cap_ && !buf->size_);

        result = (*recBufQueueItf_)->Enqueue(recBufQueueItf_, buf->buf_,
                                             buf->cap_);
        SLASSERT(result);
        devShadowQueue_->push(buf);
    }


    result = (*recItf_)->SetRecordState(recItf_, SL_RECORDSTATE_RECORDING);
    SLASSERT(result);

    // if collection was not explicitly started via nativeStartCollect, ensure collection is off
    // (g_collecting is not modified here; it is controlled by JNI helper)
    return (result == SL_RESULT_SUCCESS? SL_BOOLEAN_TRUE:SL_BOOLEAN_FALSE);
}

SLboolean  AudioRecorder::Stop(void) {
    // in case already recording, stop recording and clear buffer queue
    SLuint32 curState;

    SLresult result = (*recItf_)->GetRecordState(recItf_, &curState);
    SLASSERT(result);
    if( curState == SL_RECORDSTATE_STOPPED) {
        return SL_BOOLEAN_TRUE;
    }
    result = (*recItf_)->SetRecordState(recItf_, SL_RECORDSTATE_STOPPED);
    SLASSERT(result);
    result = (*recBufQueueItf_)->Clear(recBufQueueItf_);
    SLASSERT(result);


#ifdef ENABLE_LOG
    recLog_->flush();
#endif

    // Return any remaining devShadowQueue_ buffers back into freeQueue_ now that recording is stopped
    if (devShadowQueue_ && freeQueue_) {
        sample_buf *buf = nullptr;
        while (devShadowQueue_->front(&buf)) {
            devShadowQueue_->pop();
            freeQueue_->push(buf);
        }
    }

    // We do NOT automatically turn off collection here because the Java side will call
    // nativeStopAndGetRecording() to retrieve data.
    return SL_BOOLEAN_TRUE;
}

AudioRecorder::~AudioRecorder() {
    // destroy audio recorder object, and invalidate all associated interfaces
    if (recObjectItf_ != NULL) {
        (*recObjectItf_)->Destroy(recObjectItf_);
    }

    if(devShadowQueue_) {
        sample_buf *buf = NULL;
        while(devShadowQueue_->front(&buf)) {
            devShadowQueue_->pop();
            freeQueue_->push(buf);
        }
        delete (devShadowQueue_);
    }
#ifdef  ENABLE_LOG
    if(recLog_) {
        delete recLog_;
    }
#endif
}

void AudioRecorder::SetBufQueues(AudioQueue *freeQ, AudioQueue *recQ) {
    assert(freeQ && recQ);
    freeQueue_ = freeQ;
    recQueue_ = recQ;
}

void AudioRecorder::RegisterCallback(ENGINE_CALLBACK cb, void *ctx) {
    callback_ = cb;
    ctx_ = ctx;
}
int32_t AudioRecorder::dbgGetDevBufCount(void) {
    return devShadowQueue_->size();
}
