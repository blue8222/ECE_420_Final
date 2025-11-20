//
// Created by daran on 1/12/2017 to be used in ECE420 Sp17 for the first time.
// Modified by dwang49 on 1/1/2018 to adapt to Android 7.0 and Shield Tablet updates.
//

#ifndef ECE420_MAIN_H
#define ECE420_MAIN_H


#include <cstdlib>
#include "audio_player.h"

#include "ece420_lib.h"
#include "kiss_fft/kiss_fft.h"

#include <vector> // Add this for std::vector

// Define the function that will be the main processing loop
void ece420ProcessFrame(sample_buf *dataBuf);

// Buffer to store the entire recording
extern std::vector<float> fullRecordingBuffer;

// Buffer that holds the pre-generated audio to be played
extern std::vector<float> playbackBuffer;

// Keep track of our position in the playback buffer
extern std::atomic<int> playbackBufferPos;

// Function to generate the audio we want to play
extern std::vector<float> generatePlaybackAudio(bool window);

#endif //ECE420_MAIN_H
