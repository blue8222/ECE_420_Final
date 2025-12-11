package com.ece420.lab2;

import android.util.Log;
import java.util.Arrays;
import org.jtransforms.fft.DoubleFFT_1D;

public class ChirpAligner {

    private static final String TAG = "ChirpAligner";

    // Result container
    public static class Result {
        public final short[] aligned;
        public final double peakCorrelation;
        public final int peakIndex;
        public final int lag;

        public Result(short[] aligned, double peakCorrelation, int peakIndex, int lag) {
            this.aligned = aligned;
            this.peakCorrelation = peakCorrelation;
            this.peakIndex = peakIndex;
            this.lag = lag;
        }
    }

    // Changed signature: returns Result now
    public static Result alignChirps(short[] referenceChirp, short[] recordedChirp) {
        if (referenceChirp == null) throw new IllegalArgumentException("referenceChirp is null");
        if (recordedChirp == null) recordedChirp = new short[0];

        final int N = referenceChirp.length;
        final int M = recordedChirp.length;

        Log.i(TAG, "reference length (N): " + N);
        Log.i(TAG, "recorded length (input M): " + M);

        if (N == 0) return new Result(new short[0], 0.0, 0, 0);
        if (M == 0) return new Result(new short[N], 0.0, 0, 0);

        int convLen = M + N - 1;
        int fftSize = nextPowerOfTwo(convLen);

        Log.i(TAG, "convLen: " + convLen + ", fftSize: " + fftSize);

        double[] recC = new double[2 * fftSize];
        double[] revRefC = new double[2 * fftSize];

        for (int i = 0; i < M; i++) {
            recC[2 * i] = recordedChirp[i];
            recC[2 * i + 1] = 0.0;
        }

        for (int i = 0; i < N; i++) {
            revRefC[2 * i] = referenceChirp[N - 1 - i];
            revRefC[2 * i + 1] = 0.0;
        }

        DoubleFFT_1D fft = new DoubleFFT_1D(fftSize);
        fft.complexForward(recC);
        fft.complexForward(revRefC);

        double[] prod = new double[2 * fftSize];
        for (int i = 0; i < fftSize; i++) {
            double aRe = recC[2 * i], aIm = recC[2 * i + 1];
            double bRe = revRefC[2 * i], bIm = revRefC[2 * i + 1];
            prod[2 * i]     = aRe * bRe - aIm * bIm;
            prod[2 * i + 1] = aRe * bIm + aIm * bRe;
        }

        fft.complexInverse(prod, true);

        // Peak detection using full complex magnitude
        int bestIndex = 0;
        double bestVal = Math.hypot(prod[0], prod[1]);
        for (int i = 1; i < convLen; i++) {
            double re = prod[2 * i];
            double im = prod[2 * i + 1];
            double v = Math.hypot(re, im);
            if (v > bestVal) {
                bestVal = v;
                bestIndex = i;
            }
        }

        Log.i(TAG, "max correlation (magnitude): " + bestVal);
        Log.i(TAG, "peak correlation at convolution index: " + bestIndex);

        int lag = bestIndex - (N - 1);
        Log.i(TAG, "computed lag: " + lag);

        // Output
        short[] aligned = new short[N];
        Arrays.fill(aligned, (short) 0);

        int copied = 0;
        for (int i = 0; i < N; i++) {
            int src = i + lag;
            if (src >= 0 && src < M) {
                aligned[i] = recordedChirp[src];
                copied++;
            }
        }

        Log.i(TAG, "aligned output length: " + aligned.length);
        Log.i(TAG, "recorded samples copied into output: " + copied);

        Log.i(TAG, "first 100 samples of aligned: " + firstNSamples(aligned, 100));
        Log.i(TAG, "first 100 samples of reference: " + firstNSamples(referenceChirp, 100));

        return new Result(aligned, bestVal, bestIndex, lag);
    }

    private static int nextPowerOfTwo(int x) {
        if (x <= 1) return 1;
        return Integer.highestOneBit(x - 1) << 1;
    }

    private static String firstNSamples(short[] arr, int n) {
        if (arr == null || arr.length == 0) return "[]";
        int len = Math.min(n, arr.length);
        StringBuilder sb = new StringBuilder();
        sb.append('[');
        for (int i = 0; i < len; i++) {
            sb.append(arr[i]);
            if (i < len - 1) sb.append(", ");
        }
        if (arr.length > len) sb.append(", ...");
        sb.append(']');
        return sb.toString();
    }
}