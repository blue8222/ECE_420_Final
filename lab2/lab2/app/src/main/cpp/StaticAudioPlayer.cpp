#include "StaticAudioPlayer.h"
#include <cstring>
#include <android/log.h>

#define LOGTAG "StaticAudioPlayer"
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOGTAG, __VA_ARGS__)
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOGTAG, __VA_ARGS__)

StaticAudioPlayer::StaticAudioPlayer(
        SLEngineItf engine,
        uint8_t *pcmData,
        uint32_t pcmSizeBytes,
        uint32_t sampleRate,
        uint32_t channels,
        uint32_t bits)
        : outputMixObject_(nullptr),
          playerObject_(nullptr),
          playItf_(nullptr),
          bufferQueueItf_(nullptr),
          seekItf_(nullptr),
          pcmData_(pcmData),
          pcmSizeBytes_(pcmSizeBytes) {

    if (engine == nullptr) {
        ALOGE("StaticAudioPlayer: engine is null");
        return;
    }

    SLresult r;

    // ---- Output mix ----
    r = (*engine)->CreateOutputMix(engine, &outputMixObject_, 0, nullptr, nullptr);
    if (r != SL_RESULT_SUCCESS) {
        ALOGE("CreateOutputMix failed: 0x%08x", r);
        outputMixObject_ = nullptr;
        return;
    }
    r = (*outputMixObject_)->Realize(outputMixObject_, SL_BOOLEAN_FALSE);
    if (r != SL_RESULT_SUCCESS) {
        ALOGE("Realize outputMix failed: 0x%08x", r);
        (*outputMixObject_)->Destroy(outputMixObject_);
        outputMixObject_ = nullptr;
        return;
    }

    // ---- PCM format ----
    SLAndroidDataFormat_PCM_EX formatPCM;
    std::memset(&formatPCM, 0, sizeof(formatPCM));
    formatPCM.formatType = SL_ANDROID_DATAFORMAT_PCM_EX;
    formatPCM.numChannels = static_cast<SLuint32>(channels);
    formatPCM.sampleRate = static_cast<SLuint32>(sampleRate) * 1000; // millihertz
    formatPCM.bitsPerSample = static_cast<SLuint32>(bits);
    formatPCM.containerSize = static_cast<SLuint32>(bits);
    formatPCM.channelMask = (channels == 1) ? SL_SPEAKER_FRONT_CENTER
                                            : (SL_SPEAKER_FRONT_LEFT | SL_SPEAKER_FRONT_RIGHT);
    formatPCM.endianness = SL_BYTEORDER_LITTLEENDIAN;
    formatPCM.representation = SL_ANDROID_PCM_REPRESENTATION_SIGNED_INT;

    // ---- Source using Android simple buffer queue (1 buffer only) ----
    SLDataLocator_AndroidSimpleBufferQueue locBQ;
    locBQ.locatorType = SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE;
    locBQ.numBuffers = 1;

    SLDataSource audioSrc;
    audioSrc.pLocator = &locBQ;
    audioSrc.pFormat = &formatPCM;

    // ---- Sink ----
    SLDataLocator_OutputMix locOutmix;
    locOutmix.locatorType = SL_DATALOCATOR_OUTPUTMIX;
    locOutmix.outputMix = outputMixObject_;

    SLDataSink audioSnk;
    audioSnk.pLocator = &locOutmix;
    audioSnk.pFormat = nullptr;

    // ---- Required interfaces ----
    const SLInterfaceID ids[] = { SL_IID_PLAY, SL_IID_BUFFERQUEUE };
    const SLboolean req[] = { SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE };
    const SLuint32 interfaceCount = sizeof(ids) / sizeof(ids[0]);

    // ---- Player ----
    r = (*engine)->CreateAudioPlayer(engine, &playerObject_, &audioSrc, &audioSnk,
                                     interfaceCount, ids, req);
    if (r != SL_RESULT_SUCCESS) {
        ALOGE("CreateAudioPlayer failed: 0x%08x", r);
        playerObject_ = nullptr;
        return;
    }

    r = (*playerObject_)->Realize(playerObject_, SL_BOOLEAN_FALSE);
    if (r != SL_RESULT_SUCCESS) {
        ALOGE("Realize playerObject failed: 0x%08x", r);
        (*playerObject_)->Destroy(playerObject_);
        playerObject_ = nullptr;
        return;
    }

    // ---- Interfaces ----
    r = (*playerObject_)->GetInterface(playerObject_, SL_IID_PLAY, &playItf_);
    if (r != SL_RESULT_SUCCESS) {
        ALOGE("GetInterface PLAY failed: 0x%08x", r);
        playItf_ = nullptr; // still try to cleanup later
    }

    r = (*playerObject_)->GetInterface(playerObject_, SL_IID_BUFFERQUEUE, &bufferQueueItf_);
    if (r != SL_RESULT_SUCCESS) {
        ALOGE("GetInterface BUFFERQUEUE failed: 0x%08x", r);
        bufferQueueItf_ = nullptr;
    }

    // Note: We intentionally do NOT request SL_IID_SEEK because SEEK is not supported
    // for buffer-queue data sources on most Android implementations.
}

StaticAudioPlayer::~StaticAudioPlayer() {
    // Stop playback first if necessary
    if (playItf_) {
        (*playItf_)->SetPlayState(playItf_, SL_PLAYSTATE_STOPPED);
    }

    if (playerObject_) {
        (*playerObject_)->Destroy(playerObject_);
        playerObject_ = nullptr;
    }

    if (outputMixObject_) {
        (*outputMixObject_)->Destroy(outputMixObject_);
        outputMixObject_ = nullptr;
    }

    // pcmData_ is not owned by this class (caller retains ownership)
    playItf_ = nullptr;
    bufferQueueItf_ = nullptr;
    seekItf_ = nullptr;
}

bool StaticAudioPlayer::Start(bool /*loop*/) {
    if (!playItf_ || !bufferQueueItf_) {
        ALOGE("Start called but interfaces missing (playItf=%p, bufferQueueItf=%p)",
              playItf_, bufferQueueItf_);
        return false;
    }

    // Stop & clear queue before restarting
    (*playItf_)->SetPlayState(playItf_, SL_PLAYSTATE_STOPPED);
    (*bufferQueueItf_)->Clear(bufferQueueItf_);

    // Submit the PCM buffer once — no callback required for a single-shot play
    SLresult res = (*bufferQueueItf_)->Enqueue(bufferQueueItf_, pcmData_, pcmSizeBytes_);
    if (res != SL_RESULT_SUCCESS) {
        ALOGE("Enqueue failed: 0x%08x", res);
        return false;
    }

    res = (*playItf_)->SetPlayState(playItf_, SL_PLAYSTATE_PLAYING);
    if (res != SL_RESULT_SUCCESS) {
        ALOGE("SetPlayState PLAYING failed: 0x%08x", res);
        return false;
    }

    return true;
}

void StaticAudioPlayer::Stop() {
    if (!playItf_) return;
    SLresult r = (*playItf_)->SetPlayState(playItf_, SL_PLAYSTATE_STOPPED);
    if (r != SL_RESULT_SUCCESS) {
        ALOGE("Stop SetPlayState failed: 0x%08x", r);
    }
}
