#pragma once
#include <SLES/OpenSLES.h>
#include <SLES/OpenSLES_Android.h>
#include <cstdint>

class StaticAudioPlayer {
public:
    StaticAudioPlayer(SLEngineItf engine,
                          uint8_t *pcmData,
                          uint32_t pcmSizeBytes,
                          uint32_t sampleRate,
                          uint32_t channels,
                          uint32_t bits);
    ~StaticAudioPlayer();

    bool Start(bool loop = false);
    void Stop();

private:
    SLObjectItf outputMixObject_ = nullptr;
    SLObjectItf playerObject_ = nullptr;
    SLPlayItf playItf_ = nullptr;
    SLAndroidSimpleBufferQueueItf bufferQueueItf_ = nullptr;
    SLSeekItf seekItf_ = nullptr; // used for looping

    uint8_t *pcmData_;
    uint32_t pcmSizeBytes_;
};