//
// Created by dager on 11/21/2025.
//



#ifndef GENERATE_CHIRP_H
#define GENERATE_CHIRP_H

#include <vector>
#include <cstdint>


#define sweepTime 0.5
#define minFreq 8000.0
#define bandwidth 10000.0


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
        double minFreq_,
        double bandwidth_,
        double sweepTime_
);

#endif // GENERATE_CHIRP_H
