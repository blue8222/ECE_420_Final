// audio_main.cpp  -- updated to use StaticLikeAudioPlayer
#include <cassert>
#include <cstring>
#include <jni.h>

#include <sys/types.h>
#include <SLES/OpenSLES.h>

#include "audio_common.h"
#include "audio_recorder.h"
#include "audio_player.h"
// NEW: include the static-like player header
#include "StaticAudioPlayer.h"

struct EchoAudioEngine {
    SLmilliHertz fastPathSampleRate_;
    uint32_t     fastPathFramesPerBuf_;
    uint16_t     sampleChannels_;
    uint16_t     bitsPerSample_;

    SLObjectItf  slEngineObj_;
    SLEngineItf  slEngineItf_;

    // recorder (unchanged)
    AudioRecorder  *recorder_;
    AudioQueue     *freeBufQueue_;    // Owner for recorder
    AudioQueue     *recBufQueue_;     // Owner for recorder

    sample_buf  *bufs_;
    uint32_t     bufCount_;
    uint32_t     frameCount_;

    // static-like player fields (new)
    StaticAudioPlayer *staticPlayer_;
    uint8_t               *staticPcmBuffer_;
    uint32_t               staticPcmSizeBytes_;
};
static EchoAudioEngine engine;

bool EngineService(void* ctx, uint32_t msg, void* data );

extern "C" {
JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_createSLEngine(JNIEnv *env, jclass, jint sampleRate, jint framesPerBuf);

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_deleteSLEngine(JNIEnv *env, jclass type);

JNIEXPORT jboolean JNICALL
Java_com_ece420_lab2_MainActivity_createSLBufferQueueAudioPlayer(JNIEnv *env, jclass);

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_deleteSLBufferQueueAudioPlayer(JNIEnv *env, jclass type);

JNIEXPORT jboolean JNICALL
Java_com_ece420_lab2_MainActivity_createAudioRecorder(JNIEnv *env, jclass type);

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_deleteAudioRecorder(JNIEnv *env, jclass type);

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_startPlay(JNIEnv *env, jclass type);

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_stopPlay(JNIEnv *env, jclass type);

/* New helper to upload PCM data from Java to native engine memory */
JNIEXPORT jboolean JNICALL
Java_com_ece420_lab2_MainActivity_loadPCMBuffer(JNIEnv *env, jclass type, jbyteArray pcm);
} // extern "C"



JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_createSLEngine(
        JNIEnv *env, jclass type, jint sampleRate, jint framesPerBuf) {
    SLresult result;
    memset(&engine, 0, sizeof(engine));

    engine.fastPathSampleRate_   = static_cast<SLmilliHertz>(sampleRate) * 1000;
    engine.fastPathFramesPerBuf_ = static_cast<uint32_t>(framesPerBuf);
    engine.sampleChannels_   = AUDIO_SAMPLE_CHANNELS;
    engine.bitsPerSample_    = SL_PCMSAMPLEFORMAT_FIXED_16;

    engine.staticPlayer_ = nullptr;
    engine.staticPcmBuffer_ = nullptr;
    engine.staticPcmSizeBytes_ = 0;

    engine.slEngineObj_ = nullptr;
    engine.slEngineItf_ = nullptr;

    result = slCreateEngine(&engine.slEngineObj_, 0, NULL, 0, NULL, NULL);
    SLASSERT(result);

    result = (*engine.slEngineObj_)->Realize(engine.slEngineObj_, SL_BOOLEAN_FALSE);
    SLASSERT(result);

    result = (*engine.slEngineObj_)->GetInterface(engine.slEngineObj_, SL_IID_ENGINE, &engine.slEngineItf_);
    SLASSERT(result);

    // compute recommended fast audio buffer size (bytes)
    uint32_t bufSize = engine.fastPathFramesPerBuf_ * engine.sampleChannels_ * engine.bitsPerSample_;
    bufSize = (bufSize + 7) >> 3;            // bits --> byte

    engine.bufCount_ = BUF_COUNT;
    engine.bufs_ = allocateSampleBufs(engine.bufCount_, bufSize);
    assert(engine.bufs_);

    engine.freeBufQueue_ = new AudioQueue(engine.bufCount_);
    engine.recBufQueue_  = new AudioQueue(engine.bufCount_);
    assert(engine.freeBufQueue_ && engine.recBufQueue_);
    for(uint32_t i = 0; i < engine.bufCount_; i++) {
        engine.freeBufQueue_->push(&engine.bufs_[i]);
    }

    // recorder not created here — created by createAudioRecorder()
}


JNIEXPORT jboolean JNICALL
Java_com_ece420_lab2_MainActivity_createSLBufferQueueAudioPlayer(JNIEnv *env, jclass) {
    // In the static-player world we expect the PCM buffer to be loaded first
    if (engine.staticPcmBuffer_ == nullptr || engine.staticPcmSizeBytes_ == 0) {
        LOGE("StaticAudioPlayer: no PCM buffer loaded - call loadPCMBuffer() first");
        return JNI_FALSE;
    }

    // ---- Logging block ----
    uint32_t sampleRateHz = static_cast<uint32_t>(engine.fastPathSampleRate_ / 1000);
    uint32_t channels = engine.sampleChannels_;
    uint32_t pcmBytes = engine.staticPcmSizeBytes_;

    LOGE("StaticAudioPlayer: PCM buffer loaded");
    LOGE("  sampleRateHz  = %u", sampleRateHz);
    LOGE("  channels      = %u", channels);
    LOGE("  pcmBytes      = %u", pcmBytes);

    // Preview first 16 bytes
    const uint32_t previewLen = (pcmBytes < 16 ? pcmBytes : 16);
    char preview[128];
    char *ptr = preview;
    for (uint32_t i = 0; i < previewLen; i++) {
        ptr += sprintf(ptr, "%02X ", engine.staticPcmBuffer_[i]);
    }
    LOGE("  first %u bytes: %s", previewLen, preview);
    // ---- End logging block ----

    // delete any existing static player
    if (engine.staticPlayer_) {
        engine.staticPlayer_->Stop();
        delete engine.staticPlayer_;
        engine.staticPlayer_ = nullptr;
    }

    uint32_t bits = 16; // We use 16-bit PCM in this project

    engine.staticPlayer_ = new StaticAudioPlayer(
            engine.slEngineItf_,
            engine.staticPcmBuffer_,
            engine.staticPcmSizeBytes_,
            sampleRateHz,
            channels,
            bits
    );

    if (!engine.staticPlayer_) {
        LOGE("StaticAudioPlayer: allocation failed after PCM load");
        return JNI_FALSE;
    }

    LOGE("StaticAudioPlayer: created successfully");
    return JNI_TRUE;
}

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_deleteSLBufferQueueAudioPlayer(JNIEnv *env, jclass type) {
    if (engine.staticPlayer_) {
        engine.staticPlayer_->Stop();
        delete engine.staticPlayer_;
        engine.staticPlayer_ = nullptr;
    }
    // Note: We keep engine.staticPcmBuffer_ in memory until deleteSLEngine() or a new load.
}


JNIEXPORT jboolean JNICALL
Java_com_ece420_lab2_MainActivity_createAudioRecorder(JNIEnv *env, jclass type) {
    if (!engine.slEngineItf_) {
        LOGE("createAudioRecorder: engine not initialized");
        return JNI_FALSE;
    }

    SampleFormat sampleFormat;
    memset(&sampleFormat, 0, sizeof(sampleFormat));
    sampleFormat.pcmFormat_ = static_cast<uint16_t>(engine.bitsPerSample_);
    sampleFormat.channels_ = engine.sampleChannels_;
    sampleFormat.sampleRate_ = engine.fastPathSampleRate_;
    sampleFormat.framesPerBuf_ = engine.fastPathFramesPerBuf_;

    engine.recorder_ = new AudioRecorder(&sampleFormat, engine.slEngineItf_);
    if (!engine.recorder_) {
        LOGE("createAudioRecorder: allocation failed");
        return JNI_FALSE;
    }
    engine.recorder_->SetBufQueues(engine.freeBufQueue_, engine.recBufQueue_);
    engine.recorder_->RegisterCallback(EngineService, (void*)&engine);

    if (!engine.recorder_->Start()) {
        LOGE("createAudioRecorder: failed to start recorder");
        return JNI_FALSE;
    }

    LOGI("createAudioRecorder: recorder started");
    return JNI_TRUE;
}

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_deleteAudioRecorder(JNIEnv *env, jclass type) {
    if (engine.recorder_) {
        delete engine.recorder_;
        engine.recorder_ = nullptr;
    }
}


JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_startPlay(JNIEnv *env, jclass type) {
    engine.frameCount_ = 0;

    if (engine.staticPlayer_) {
        // Start playback (no loop)
        if (!engine.staticPlayer_->Start(false)) {
            LOGE("StaticLikeAudioPlayer::Start failed");
            return;
        }
        // If recorder exists but you don't want recorder running with static playback,
        // do NOT call recorder_->Start(); user can start recorder separately.
        return;
    }

    // fallback to old behavior if static player not used (if you have an old AudioPlayer)
    if (engine.recorder_) {
        engine.recorder_->Start();
    }
}

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_stopPlay(JNIEnv *env, jclass type) {
    if (engine.recorder_) {
        engine.recorder_->Stop();
    }

    if (engine.staticPlayer_) {
        engine.staticPlayer_->Stop();
    }
}

JNIEXPORT void JNICALL
Java_com_ece420_lab2_MainActivity_deleteSLEngine(JNIEnv *env, jclass type) {
    // stop and free static player
    if (engine.staticPlayer_) {
        engine.staticPlayer_->Stop();
        delete engine.staticPlayer_;
        engine.staticPlayer_ = nullptr;
    }

    // free PCM upload buffer
    if (engine.staticPcmBuffer_) {
        delete [] engine.staticPcmBuffer_;
        engine.staticPcmBuffer_ = nullptr;
        engine.staticPcmSizeBytes_ = 0;
    }

    // delete recorder resources
    if (engine.recorder_) {
        delete engine.recorder_;
        engine.recorder_ = nullptr;
    }

    // queues & sample buffers
    if (engine.recBufQueue_) {
        delete engine.recBufQueue_;
        engine.recBufQueue_ = nullptr;
    }
    if (engine.freeBufQueue_) {
        delete engine.freeBufQueue_;
        engine.freeBufQueue_ = nullptr;
    }
    releaseSampleBufs(engine.bufs_, engine.bufCount_);
    engine.bufs_ = nullptr;
    engine.bufCount_ = 0;

    if (engine.slEngineObj_ != NULL) {
        (*engine.slEngineObj_)->Destroy(engine.slEngineObj_);
        engine.slEngineObj_ = NULL;
        engine.slEngineItf_ = NULL;
    }
}

uint32_t dbgEngineGetBufCount(void) {
    uint32_t count = 0;
    // static player doesn't use device-buffer queues; only recorder and free queues count
    if (engine.recorder_) {
        count += engine.recorder_->dbgGetDevBufCount();
    }
    if (engine.freeBufQueue_) count += engine.freeBufQueue_->size();
    if (engine.recBufQueue_)  count += engine.recBufQueue_->size();

    // log distribution; keep names for backward compatibility
    LOGE("Buf Distributions: PlayerDev=%d (static), RecDev=%d, FreeQ=%d, RecQ=%d",
         0,
         engine.recorder_ ? engine.recorder_->dbgGetDevBufCount() : 0,
         engine.freeBufQueue_ ? engine.freeBufQueue_->size() : 0,
         engine.recBufQueue_ ? engine.recBufQueue_->size() : 0);

    if (count != engine.bufCount_) {
        LOGE("====Lost Bufs among the queue(supposed = %d, found = %d)",
             engine.bufCount_, count);
    }
    return count;
}

/*
 * simple message passing for player/recorder to communicate with engine
 */
bool EngineService(void* ctx, uint32_t msg, void* data ) {
    assert(ctx == &engine);

    switch (msg) {
        case ENGINE_SERVICE_MSG_RETRIEVE_DUMP_BUFS:
            *(static_cast<uint32_t*>(data)) = dbgEngineGetBufCount();
            break;
        default:
            assert(false);
            return false;
    }

    return true;
}



JNIEXPORT jboolean JNICALL
Java_com_ece420_lab2_MainActivity_loadPCMBuffer(JNIEnv *env, jclass type, jbyteArray pcm) {
    if (pcm == nullptr) return JNI_FALSE;

    jsize len = env->GetArrayLength(pcm);
    if (len <= 0) return JNI_FALSE;

    // free old buffer if any
    if (engine.staticPcmBuffer_) {
        delete [] engine.staticPcmBuffer_;
        engine.staticPcmBuffer_ = nullptr;
        engine.staticPcmSizeBytes_ = 0;
    }

    engine.staticPcmBuffer_ = new uint8_t[len];
    if (!engine.staticPcmBuffer_) {
        LOGE("loadPCMBuffer: allocation failed for %d bytes", len);
        return JNI_FALSE;
    }

    env->GetByteArrayRegion(pcm, 0, len, reinterpret_cast<jbyte*>(engine.staticPcmBuffer_));
    engine.staticPcmSizeBytes_ = static_cast<uint32_t>(len);
    return JNI_TRUE;
}
