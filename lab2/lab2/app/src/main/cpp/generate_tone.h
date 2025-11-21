//
// Created by dager on 11/21/2025.
//



#ifndef GENERATE_TONE_H
#define GENERATE_TONE_H

#include <cstdint>
#include <vector>

/**
 * Generate a PCM sine wave tone.
 *
 * @param sampleRate      Sample rate in Hz (e.g. 48000)
 * @param durationSeconds Duration in seconds
 * @param freqHz          Tone frequency in Hz (e.g. 440.0)
 * @return A std::vector<uint8_t> containing 16-bit PCM little-endian mono data.
 *
 * Layout: each sample is two bytes (little-endian), signed int16 range.
 * Total size = sampleRate * durationSeconds * 2
 */
std::vector<uint8_t> generateTonePCM_LE16(
        int sampleRate,
        int durationSeconds,
        double freqHz
);

#endif // GENERATE_TONE_H