//
// Created by dager on 11/21/2025.
//

#ifndef PROCESS_RECORDING_H
#define PROCESS_RECORDING_H

#include <vector>
#include <cstdint>

struct AnalysisResult {


    bool distance_valid;  // whether a distance estimate is available
    double distance_m;    // distance in meters (if distance_valid == true)
};

/**
 * Analyze a recorded PCM buffer (16-bit little-endian mono).
 *
 * @param pcmBytes    input buffer of raw bytes (little-endian 16-bit PCM)
 * @param sampleRate  sample rate in Hz
 * @return AnalysisResult with estimates
 */
AnalysisResult analyzeRecordedBuffer(const std::vector<uint8_t>& pcmBytes,
                                     int sampleRate,
                                     const std::vector<uint8_t>* referenceChirpBytes = nullptr);

#endif // PROCESS_RECORDING_H
