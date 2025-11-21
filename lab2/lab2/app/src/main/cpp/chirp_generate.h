//
// Created by dager on 11/21/2025.
//



#ifndef GENERATE_CHIRP_H
#define GENERATE_CHIRP_H

#include <vector>
#include <cstdint>

/**
 * Generate a linear-frequency chirp with Hanning window, 16-bit PCM (mono).
 *
 * @param sampleRate   sample rate in Hz
 * @param minFreq      starting frequency (Hz)
 * @param bandwidth    B value (maxFreq = minFreq + bandwidth)
 * @param sweepTime    duration in seconds
 * @return vector<uint8_t> (16-bit signed little-endian)
 */
std::vector<uint8_t> generateChirpPCM_LE16(
        int sampleRate,
        double minFreq,
        double bandwidth,
        double sweepTime
);

#endif // GENERATE_CHIRP_H
