package com.ece420.lab2; // change to your package

public final class AudioUtils {

    private AudioUtils() { /* no instances */ }

    /**
     * Apply a band-pass FIR filter (windowed-sinc, Hamming window) to a PCM16 short[] buffer.
     *
     * @param input      Mono PCM16 samples (short[]).
     * @param sampleRate Sample rate in Hz (e.g., 48000).
     * @param lowHz      Lower cutoff frequency in Hz (e.g., 17000).
     * @param highHz     Upper cutoff frequency in Hz (e.g., 23000).
     * @param taps       Number of FIR taps (must be odd, e.g., 129 or 161). Larger -> sharper filter, more CPU.
     * @return filtered PCM16 samples (short[]), same length as input.
     */
    public static short[] applyBandpassFilter(short[] input, double sampleRate,
                                              double lowHz, double highHz, int taps) {
        if (input == null) return null;
        if (input.length == 0) return new short[0];
        if (lowHz <= 0 || highHz <= lowHz || highHz >= sampleRate / 2.0) {
            throw new IllegalArgumentException("Invalid cutoff frequencies");
        }
        if (taps % 2 == 0) taps++; // require odd length for symmetric (linear-phase)
        // design filter kernel (double)
        double[] kernel = designBandpassKernel(taps, sampleRate, lowHz, highHz);
        // convolve (simple time-domain FIR)
        return applyFir(input, kernel);
    }

    // --- helper: design the bandpass kernel via windowed-sinc (Hamming) ---
    private static double[] designBandpassKernel(int taps, double fs, double lowHz, double highHz) {
        int M = (taps - 1) / 2; // center index
        double[] h = new double[taps];

        // normalized cutoffs (cycles per sample)
        double f1 = lowHz / fs;
        double f2 = highHz / fs;

        for (int n = -M; n <= M; n++) {
            int idx = n + M;
            if (n == 0) {
                // h_lp(f) = 2 * f
                double val = 2.0 * (f2 - f1);
                h[idx] = val;
            } else {
                double pi_n = Math.PI * n;
                // lowpass at f2 minus lowpass at f1
                double h1 = Math.sin(2.0 * Math.PI * f2 * n) / (pi_n);
                double h2 = Math.sin(2.0 * Math.PI * f1 * n) / (pi_n);
                h[idx] = h1 - h2;
            }
            // apply Hamming window
            h[idx] *= hammingWindow(idx, taps);
        }

        // (Optional) normalize gain at DC or at band center — for bandpass, we normalize so max magnitude ~1
        // Compute sum of absolute kernel; use that to normalize energy roughly
        double sum = 0.0;
        for (double v : h) sum += Math.abs(v);
        if (sum > 0.0) {
            for (int i = 0; i < h.length; i++) h[i] /= sum;
        }
        return h;
    }

    /**
     * Apply a low-pass FIR filter (windowed-sinc, Hamming window) to a PCM16 short[] buffer.
     *
     * @param input      Mono PCM16 samples (short[]).
     * @param sampleRate Sample rate in Hz (e.g., 48000).
     * @param cutoffHz   Cutoff frequency in Hz (e.g., 8000).
     * @param taps       Number of FIR taps (must be odd, e.g., 101). Larger -> sharper filter, more CPU.
     * @return filtered PCM16 samples (short[]), same length as input.
     */
    public static short[] applyLowpassFilter(short[] input, double sampleRate,
                                             double cutoffHz, int taps) {
        if (input == null) return null;
        if (input.length == 0) return new short[0];
        if (cutoffHz <= 0 || cutoffHz >= sampleRate / 2.0) {
            throw new IllegalArgumentException("Invalid cutoff frequency");
        }
        if (taps % 2 == 0) taps++; // require odd length for symmetric (linear-phase)

        double[] kernel = designLowpassKernel(taps, sampleRate, cutoffHz);
        return applyFir(input, kernel);
    }

    // --- helper: design lowpass kernel via windowed-sinc (Hamming) ---
    private static double[] designLowpassKernel(int taps, double fs, double cutoffHz) {
        int M = (taps - 1) / 2; // center index
        double[] h = new double[taps];

        // normalized cutoff (cycles per sample)
        double fc = cutoffHz / fs; // 0 < fc < 0.5

        for (int n = -M; n <= M; n++) {
            int idx = n + M;
            if (n == 0) {
                h[idx] = 2.0 * fc;
            } else {
                double pi_n = Math.PI * n;
                h[idx] = Math.sin(2.0 * Math.PI * fc * n) / (pi_n);
            }
            // apply Hamming window (reuse your existing helper)
            h[idx] *= hammingWindow(idx, taps);
        }

        // Normalize kernel so DC gain ~= 1.0
        double sum = 0.0;
        for (double v : h) sum += v;
        if (Math.abs(sum) > 0.0) {
            for (int i = 0; i < h.length; i++) h[i] /= sum;
        }
        return h;
    }

    private static double hammingWindow(int idx, int taps) {
        // Hamming window: w[n] = 0.54 - 0.46*cos(2*pi*n/(N-1))
        return 0.54 - 0.46 * Math.cos(2.0 * Math.PI * idx / (taps - 1));
    }

    // --- helper: simple time-domain FIR convolution with zero-padding ---
    private static short[] applyFir(short[] input, double[] kernel) {
        int N = input.length;
        int K = kernel.length;
        int M = (K - 1) / 2;
        short[] out = new short[N];

        // convert input to doubles for accumulation (avoid overflow)
        double[] inD = new double[N];
        for (int i = 0; i < N; i++) inD[i] = input[i];

        // For each output sample, compute dot product with kernel centered at sample
        for (int n = 0; n < N; n++) {
            double sum = 0.0;
            for (int k = 0; k < K; k++) {
                int inIndex = n + k - M;
                double x = 0.0;
                if (inIndex >= 0 && inIndex < N) x = inD[inIndex];
                sum += kernel[k] * x;
            }
            // clamp back to short
            out[n] = doubleToShortClamped(sum);
        }
        return out;
    }

    private static short doubleToShortClamped(double v) {
        if (v > Short.MAX_VALUE) return Short.MAX_VALUE;
        if (v < Short.MIN_VALUE) return Short.MIN_VALUE;
        return (short) Math.round(v);
    }
}
