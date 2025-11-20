/*
 * Copyright 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cstdlib>
#include "audio_player.h"
#include "ece420_main.h"
#include "ece420_lib.h"
#include "kiss_fft/kiss_fft.h"

#define FRAME_SIZE 1024


/*
 * Called by OpenSL SimpleBufferQueue for every audio buffer played
 * directly pass thru to our handler.
 * The regularity of this callback from openSL/Android System affects
 * playback continuity. If it does not callback in the regular time
 * slot, you are under big pressure for audio processing[here we do
 * not do any filtering/mixing]. Callback from fast audio path are
 * much more regular than other audio paths by my observation. If it
 * very regular, you could buffer much less audio samples between
 * recorder and player, hence lower latency.
 */
void bqPlayerCallback(SLAndroidSimpleBufferQueueItf bq, void *ctx) {
    (static_cast<AudioPlayer *>(ctx))->ProcessSLCallback(bq);
}
void AudioPlayer::ProcessSLCallback(SLAndroidSimpleBufferQueueItf bq) {
#ifdef ENABLE_LOG
    logFile_->logTime();
#endif
    std::lock_guard<std::mutex> lock(stopMutex_);

    // This callback is triggered when the player has finished playing a buffer.
    // The buffer that just finished is waiting for us in devShadowQueue_.
    // We retrieve it and immediately put it back on the freeQueue_ so it can be used again.
    sample_buf* completedBuf;
    if (!devShadowQueue_->front(&completedBuf)) {
        LOGE("Player callback with no buffer in shadow queue. This should not happen.");
        return;
    }
    devShadowQueue_->pop();

    if (completedBuf != &silentBuf_) {
        freeQueue_->push(completedBuf);
    }

    // Now, get a fresh buffer from the freeQueue_ to fill with the *next* chunk of audio.
    sample_buf* bufferToFill;
    if (!freeQueue_->front(&bufferToFill)) {
        LOGW("Player underrun: No free buffers available to fill!");
        // To prevent buggy behavior, we'll re-enqueue the silent buffer and stop.
        (*bq)->Enqueue(bq, silentBuf_.buf_, silentBuf_.size_);
        devShadowQueue_->push(&silentBuf_);
        return;
    }
    freeQueue_->pop();

    // === NEW PLAYBACK LOGIC ===
    // We will fill `bufferToFill` with data from our global `playbackBuffer`.

    int currentPos = playbackBufferPos.load(std::memory_order_relaxed);
    int totalPlaybackSamples = playbackBuffer.size();
    int samplesToCopy = 0;

    if (currentPos < totalPlaybackSamples) {
        // We still have audio left to play from our generated buffer.
        int samplesLeft = totalPlaybackSamples - currentPos;
        samplesToCopy = (samplesLeft > FRAME_SIZE) ? FRAME_SIZE : samplesLeft;

        // Copy the pre-generated audio into the buffer, converting float to int16_t
        for (int i = 0; i < samplesToCopy; ++i) {
            // Clamp the float value to the valid range for int16_t before casting
            float sample = playbackBuffer[currentPos + i];
            if (sample > 32767.0f) sample = 32767.0f;
            if (sample < -32768.0f) sample = -32768.0f;
            int16_t pcm_val = static_cast<int16_t>(sample);

            bufferToFill->buf_[2 * i] = (uint8_t)(pcm_val & 0x00FF);
            bufferToFill->buf_[2 * i + 1] = (uint8_t)((pcm_val & 0xFF00) >> 8);
        }
        // Atomically update the playback position
        playbackBufferPos.fetch_add(samplesToCopy, std::memory_order_relaxed);
    }

    // If we copied less than a full frame (or if playback is done), fill the rest with silence.
    if (samplesToCopy < FRAME_SIZE) {
        memset(&bufferToFill->buf_[2 * samplesToCopy], 0, (FRAME_SIZE - samplesToCopy) * 2);
    }

    bufferToFill->size_ = FRAME_SIZE * 2; // size in bytes

    // === END OF NEW LOGIC ===

    // Enqueue the newly filled buffer to be played by the speaker.
    SLresult result = (*bq)->Enqueue(bq, bufferToFill->buf_, bufferToFill->size_);
    if (SL_RESULT_SUCCESS != result) {
        LOGE("Player Enqueue failed!");
        // If enqueue fails, return the buffer to the free queue.
        freeQueue_->push(bufferToFill);
    } else {
        // If it succeeds, push it to the shadow queue so we know it's with the device.
        devShadowQueue_->push(bufferToFill);
    }
}

AudioPlayer::AudioPlayer(SampleFormat *sampleFormat, SLEngineItf slEngine) :
    freeQueue_(nullptr), playQueue_(nullptr), devShadowQueue_(nullptr),
    callback_(nullptr)
{
    SLresult result;
    assert(sampleFormat);
    sampleInfo_ = *sampleFormat;

    result = (*slEngine)->CreateOutputMix(slEngine, &outputMixObjectItf_,
                                          0, NULL, NULL);
    SLASSERT(result);

    // realize the output mix
    result = (*outputMixObjectItf_)->Realize(outputMixObjectItf_, SL_BOOLEAN_FALSE);
    SLASSERT(result);

    // configure audio source
    SLDataLocator_AndroidSimpleBufferQueue loc_bufq = {
            SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE,
            DEVICE_SHADOW_BUFFER_QUEUE_LEN };

    SLAndroidDataFormat_PCM_EX format_pcm;
    ConvertToSLSampleFormat(&format_pcm, &sampleInfo_);
    SLDataSource audioSrc = {&loc_bufq, &format_pcm};

    // configure audio sink
    SLDataLocator_OutputMix loc_outmix = {SL_DATALOCATOR_OUTPUTMIX, outputMixObjectItf_};
    SLDataSink audioSnk = {&loc_outmix, NULL};
    /*
     * create fast path audio player: SL_IID_BUFFERQUEUE and SL_IID_VOLUME interfaces ok,
     * NO others!
     */
    SLInterfaceID  ids[2] = { SL_IID_BUFFERQUEUE, SL_IID_VOLUME};
    SLboolean      req[2] = {SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE};
    result = (*slEngine)->CreateAudioPlayer(slEngine, &playerObjectItf_, &audioSrc, &audioSnk,
                                            sizeof(ids)/sizeof(ids[0]), ids, req);
    SLASSERT(result);

    // realize the player
    result = (*playerObjectItf_)->Realize(playerObjectItf_, SL_BOOLEAN_FALSE);
    SLASSERT(result);

    // get the play interface
    result = (*playerObjectItf_)->GetInterface(playerObjectItf_, SL_IID_PLAY, &playItf_);
    SLASSERT(result);

    // get the buffer queue interface
    result = (*playerObjectItf_)->GetInterface(playerObjectItf_, SL_IID_BUFFERQUEUE,
                                             &playBufferQueueItf_);
    SLASSERT(result);

    // register callback on the buffer queue
    result = (*playBufferQueueItf_)->RegisterCallback(playBufferQueueItf_, bqPlayerCallback, this);
    SLASSERT(result);

    result = (*playItf_)->SetPlayState(playItf_, SL_PLAYSTATE_STOPPED);
    SLASSERT(result);

    // create an empty queue to track deviceQueue
    devShadowQueue_ = new AudioQueue(DEVICE_SHADOW_BUFFER_QUEUE_LEN);
    assert(devShadowQueue_);

    silentBuf_.cap_ = (format_pcm.containerSize >> 3) *
                      format_pcm.numChannels * sampleInfo_.framesPerBuf_;
    silentBuf_.buf_ = new uint8_t[silentBuf_.cap_];
    memset(silentBuf_.buf_, 0, silentBuf_.cap_);
    silentBuf_.size_ = silentBuf_.cap_;

#ifdef  ENABLE_LOG
    std::string name = "play";
    logFile_ = new AndroidLog(name);
#endif
}

AudioPlayer::~AudioPlayer() {

  std::lock_guard<std::mutex> lock(stopMutex_);

    // destroy buffer queue audio player object, and invalidate all associated interfaces
    if (playerObjectItf_ != NULL) {
        (*playerObjectItf_)->Destroy(playerObjectItf_);
    }
    // Consume all non-completed audio buffers
    sample_buf *buf = NULL;
    while(devShadowQueue_->front(&buf)) {
      buf->size_ = 0;
      devShadowQueue_->pop();
      freeQueue_->push(buf);
    }
    delete devShadowQueue_;

    while(playQueue_->front(&buf)) {
      buf->size_ = 0;
      playQueue_->pop();
      freeQueue_->push(buf);
    }

    // destroy output mix object, and invalidate all associated interfaces
    if (outputMixObjectItf_) {
        (*outputMixObjectItf_)->Destroy(outputMixObjectItf_);
    }

    delete [] silentBuf_.buf_;
}

void AudioPlayer::SetBufQueue(AudioQueue *playQ, AudioQueue *freeQ) {
    playQueue_ = playQ;
    freeQueue_ = freeQ;
}

SLresult AudioPlayer::Start(void) {
    SLuint32   state;
    SLresult  result = (*playItf_)->GetPlayState(playItf_, &state);
    if (result != SL_RESULT_SUCCESS) {
        return SL_BOOLEAN_FALSE;
    }
    if(state == SL_PLAYSTATE_PLAYING) {
        return SL_BOOLEAN_TRUE;
    }

    result = (*playItf_)->SetPlayState(playItf_, SL_PLAYSTATE_STOPPED);
    SLASSERT(result);

    result = (*playBufferQueueItf_)->Enqueue(playBufferQueueItf_,
                                             silentBuf_.buf_,
                                             silentBuf_.size_);
    SLASSERT(result);
    devShadowQueue_->push(&silentBuf_);

    result = (*playItf_)->SetPlayState(playItf_, SL_PLAYSTATE_PLAYING);
    SLASSERT(result);
    return SL_BOOLEAN_TRUE;
}

void AudioPlayer::Stop(void) {
    SLuint32   state;

    SLresult   result = (*playItf_)->GetPlayState(playItf_, &state);
    SLASSERT(result);

    if(state == SL_PLAYSTATE_STOPPED)
        return;

    std::lock_guard<std::mutex> lock(stopMutex_);

    result = (*playItf_)->SetPlayState(playItf_, SL_PLAYSTATE_STOPPED);
    SLASSERT(result);
    (*playBufferQueueItf_)->Clear(playBufferQueueItf_);

#ifdef ENABLE_LOG
    if (logFile_) {
        delete logFile_;
        logFile_ = nullptr;
    }
#endif
}

void AudioPlayer::RegisterCallback(ENGINE_CALLBACK cb, void *ctx) {
    callback_ = cb;
    ctx_ = ctx;
}

uint32_t  AudioPlayer::dbgGetDevBufCount(void) {
    return (devShadowQueue_->size());
}