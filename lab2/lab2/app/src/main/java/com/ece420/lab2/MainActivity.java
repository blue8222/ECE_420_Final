package com.ece420.lab2;

import static java.lang.Math.cos;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.Manifest;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import android.media.MicrophoneInfo;
import android.os.Bundle;
import android.os.Handler;

import org.jtransforms.fft.FloatFFT_1D;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import android.os.AsyncTask;
import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import java.util.Locale;




public class MainActivity extends Activity
        implements ActivityCompat.OnRequestPermissionsResultCallback {

    // UI Variables
    Button   controlButton;
    TextView statusView;

    TextView distanceView;
    String  nativeSampleRate;
    String  nativeSampleBufSize;
    boolean supportRecording;
    Boolean isPlaying = false;
    // Static Values
    private static final int AUDIO_ECHO_REQUEST = 0;

    public static native void nativeInit();

    private static final int sampleRate_ = 48000;

    private static final int V_s = 343;

    private static final int B = 10000;

    private static final double sweepTime = 0.035;
    private static MainActivity instance;

    public static byte[] monoToStereoLeft(byte[] monoPCM) {
        if (monoPCM == null || monoPCM.length % 2 != 0) {
            throw new IllegalArgumentException("monoPCM must be 16-bit PCM (even length)");
        }

        int monoSamples = monoPCM.length / 2;
        byte[] stereo = new byte[monoSamples * 4]; // 2 channels * 2 bytes per sample

        int monoIndex = 0;
        int stereoIndex = 0;

        while (monoIndex < monoPCM.length) {
            byte lo = monoPCM[monoIndex];
            byte hi = monoPCM[monoIndex + 1];

            // ----- LEFT CHANNEL (100% right pan => left is silent) -----
            stereo[stereoIndex + 2]     = 0;
            stereo[stereoIndex + 3] = 0;

            // ----- RIGHT CHANNEL (copy input mono sample) -----
            stereo[stereoIndex] = lo;
            stereo[stereoIndex + 1] = hi;

            monoIndex += 2;
            stereoIndex += 4;
        }

        return stereo;
    }
    public static short[] multiplyPcm(short[] a, short[] b) {
        int n = Math.min(a.length, b.length);
        short[] out = new short[n];

        for (int i = 0; i < n; i++) {
            // Convert to normalized float [-1, 1]
            float fa = a[i] / 32768f;
            float fb = b[i] / 32768f;

            // Multiply in floating-point
            float fm = fa * fb;

            // Convert back, scale up
            int sample = Math.round(fm * 32767f);

            // Clamp to prevent clipping (just in case)
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;

            out[i] = (short) sample;
        }

        return out;
    }

    public static short[] trimLeadingZeros(short[] input) {
        if (input == null || input.length == 0) return input;

        int idx = 0;

        // find first non-zero sample
        while (idx < input.length && input[idx] == 0) {
            idx++;
        }

        // if entire buffer was zeros → return empty array
        if (idx == input.length) {
            return new short[0];
        }

        // copy from first non-zero index to end
        return Arrays.copyOfRange(input, idx, input.length);
    }
    public static short[] trimLeadingBelowThreshold(short[] input, int threshold) {
        if (input == null || input.length == 0) return input;

        int idx = 0;

        // Find first sample whose magnitude is >= threshold
        while (idx < input.length && Math.abs(input[idx]) < threshold) {
            idx++;
        }

        // If no sample ever crosses the threshold → return empty array
        if (idx == input.length) {
            return new short[0];
        }

        return Arrays.copyOfRange(input, idx, input.length);
    }
    public static float[] fftShortArray(short[] pcm) {
        if (pcm == null || pcm.length == 0) return new float[0];

        int n = pcm.length;

        Log.i("FFT Length = ", String.valueOf(n));

        // Convert short[] -> float[] for JTransforms
        float[] fftData = new float[n];
        for (int i = 0; i < n; i++) {
            fftData[i] = pcm[i]; // no scaling by default
        }

        // Perform FFT (real-valued forward transform)
        FloatFFT_1D fft = new FloatFFT_1D(n);
        fft.realForward(fftData);

        // fftData now contains packed FFT:
        // index 0      -> real(0 Hz)
        // index 1      -> real(N/2)  (Nyquist)
        // index 2k     -> real(k)
        // index 2k + 1 -> imag(k), for k = 1..N/2-1
        return fftData;
    }
    public static float[] computeMagnitude(float[] fftData) {
        if (fftData == null || fftData.length == 0) return new float[0];

        int n = fftData.length;
        int bins = n / 2 + 1;
        float[] mag = new float[bins];

        // bin 0 (DC)
        mag[0] = Math.abs(fftData[0]);

        // bin N/2 (Nyquist) - stored at index 1
        mag[bins - 1] = Math.abs(fftData[1]);

        // bins 1..N/2-1
        for (int k = 1; k < bins - 1; k++) {
            float real = fftData[2 * k];
            float imag = fftData[2 * k + 1];
            // use sqrt(real^2 + imag^2)
            mag[k] = (float) Math.sqrt((double) real * real + (double) imag * imag);
        }

        return mag;
    }
    public static int peakIndexInRange(float[] arr, int start, int end) {
        if (arr == null || arr.length == 0) return -1;

        start = Math.max(0, start);
        end   = Math.min(arr.length - 1, end);

        if (start > end) return -1;

        float maxVal = Float.NEGATIVE_INFINITY;
        int maxIdx = -1;

        for (int i = start; i <= end; i++) {
            if (arr[i] > maxVal) {
                maxVal = arr[i];
                maxIdx = i;
            }
        }

        return maxIdx;
    }
    public static double[] correlate(short[] signal, short[] template) {
        int N = signal.length;
        int M = template.length;
        int convLen = N + M - 1;

        // FFT size (next pow2 >= convLen)
        int fftSize = 1;
        while (fftSize < convLen) fftSize <<= 1;

        // complex arrays interleaved: [re0, im0, re1, im1, ...]
        float[] A = new float[2 * fftSize];
        float[] B = new float[2 * fftSize];

        // copy signal (real parts)
        for (int i = 0; i < N; i++) A[2 * i] = signal[i];
        // copy template (do NOT reverse if we multiply by conj(B))
        for (int i = 0; i < M; i++) B[2 * i] = template[i];

        FloatFFT_1D fft = new FloatFFT_1D(fftSize);
        fft.complexForward(A);
        fft.complexForward(B);

        // Multiply A *= conj(B): produces cross-correlation in time domain after IFFT
        for (int k = 0; k < 2 * fftSize; k += 2) {
            float ar = A[k], ai = A[k + 1];
            float br = B[k], bi = B[k + 1];
            // conj(B) = (br, -bi)
            float rr = ar * br + ai * bi;
            float ii = ai * br - ar * bi;
            A[k] = rr;
            A[k + 1] = ii;
        }

        fft.complexInverse(A, true);

        // extract raw real cross-correlation (length convLen)
        double[] rawCorr = new double[convLen];
        for (int i = 0; i < convLen; i++) rawCorr[i] = A[2 * i];

        // compute template energy and running sum of signal^2
        double templateEnergy = 0.0;
        for (int i = 0; i < M; i++) templateEnergy += ((double) template[i]) * template[i];

        double[] sqCum = new double[N + 1]; // sqCum[t] = sum_{j< t} signal[j]^2
        sqCum[0] = 0.0;
        for (int i = 0; i < N; i++) {
            sqCum[i + 1] = sqCum[i] + ((double) signal[i]) * signal[i];
        }

        // normalized correlation (NCC)
        double[] nCorr = new double[convLen];
        final double EPS = 1e-12;
        for (int i = 0; i < convLen; i++) {
            // valid full-overlap index range:
            if (i >= (M - 1) && i <= (N - 1)) {
                int start = i - (M - 1); // segment start in signal
                double segEnergy = sqCum[start + M] - sqCum[start];
                double denom = Math.sqrt(Math.max(templateEnergy * segEnergy, EPS));
                nCorr[i] = rawCorr[i] / denom;
            } else {
                nCorr[i] = 0.0; // outside valid alignment -> ignore
            }
        }

        return nCorr;
    }

    public static short[] applyHannWindowShortNormalized(short[] input) {
        int N = input.length;
        double[] temp = new double[N];
        double maxAbs = 0.0;

        // 1. Apply Hann window (double precision)
        for (int n = 0; n < N; n++) {
            double w = 0.5 * (1.0 - Math.cos((2.0 * Math.PI * n) / (N - 1)));
            double v = input[n] * w;
            temp[n] = v;

            double abs = Math.abs(v);
            if (abs > maxAbs) maxAbs = abs;
        }

        // 2. If the signal is silent, avoid divide-by-zero
        if (maxAbs == 0.0) {
            return new short[N]; // all zeros
        }

        // 3. Compute normalization scale
        double scale = Short.MAX_VALUE / maxAbs;

        // 4. Normalize + convert to short
        short[] out = new short[N];
        for (int n = 0; n < N; n++) {
            out[n] = (short) Math.round(temp[n] * scale);
        }

        return out;
    }
    /** Find best correlation peak (max absolute). */
    public static int findPeak(double[] corr, int signalLen, int templateLen) {
        int start = templateLen - 1;
        int end   = signalLen - 1;
        if (start < 0) start = 0;
        if (end >= corr.length) end = corr.length - 1;

        double best = Double.NEGATIVE_INFINITY;
        int bestIdx = start;
        for (int i = start; i <= end; i++) {
            double val = Math.abs(corr[i]); // use abs (or magnitude if complex)
            if (val > best) {
                best = val;
                bestIdx = i;
            }
        }
        return bestIdx;
    }
    public static byte[] shortsToBytes(short[] pcm) {
        byte[] out = new byte[pcm.length * 2];
        for (int i = 0; i < pcm.length; i++) {
            out[i * 2]     = (byte) (pcm[i] & 0xFF);       // low byte
            out[i * 2 + 1] = (byte) ((pcm[i] >> 8) & 0xFF); // high byte
        }
        return out;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        instance = this;

        nativeInit();

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);
        distanceView = findViewById(R.id.distanceView);
        super.setRequestedOrientation (ActivityInfo.SCREEN_ORIENTATION_UNSPECIFIED);

        // UI wiring
        controlButton = (Button)findViewById((R.id.capture_control_button));
        statusView = (TextView)findViewById(R.id.statusView);
        queryNativeAudioParameters();
        // initialize native audio system
        updateNativeAudioUI();

        // IMPORTANT: create the SL engine regardless of whether recording is supported,
        // so we can play static buffers without needing the mic.
        Log.i("MainActivity", "nativeSampleBufSize: " + nativeSampleBufSize);
        createSLEngine(Integer.parseInt(nativeSampleRate), Integer.parseInt(nativeSampleBufSize), 2);




    }

    @Override
    protected void onDestroy() {
        if (isPlaying) {
            stopPlay();
            isPlaying = false;
        }
        // full cleanup including engine
        deleteSLBufferQueueAudioPlayer();
        deleteAudioRecorder();
        deleteSLEngine();
        instance = null;
        super.onDestroy();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == R.id.action_settings) return true;
        return super.onOptionsItemSelected(item);
    }
    private void releasePlayerAndRecorder() {
        // stop playback if running
        try {
            stopPlay();
        } catch (Exception e) {
            Log.w("MainActivity", "stopPlay() threw: " + e.getMessage());
        }

        // delete objects associated with playback/recording
        deleteSLBufferQueueAudioPlayer();
        deleteAudioRecorder();

        // DO NOT delete the SL engine here and DO NOT null out instance.
        // deleteSLEngine() and instance = null should only happen in onDestroy()
    }
    private void startEcho() {

        int sampleRate = sampleRate_;
        byte[] chirp = generateChirpPCMNative(true);
        int expectedBytes = (int) (sampleRate * 2 * 1); // 16-bit mono, 0.5s

        // ─────────────────────────────────────────────
        // START STATE
        // ─────────────────────────────────────────────
        if (!isPlaying) {

            // Load chirp as stereo-left
            byte[] stereoLeft = monoToStereoLeft(chirp);
            if (!loadPCMBuffer(stereoLeft)) {
                statusView.setText("Error loading PCM buffer");
                return;
            }

            // Create playback
            if (!createSLBufferQueueAudioPlayer()) {
                statusView.setText("Error creating player");
                return;
            }

            // Create recorder
            if (!createAudioRecorder()) {
                statusView.setText("Error creating recorder");
                deleteSLBufferQueueAudioPlayer();
                return;
            }

            // Tell native recorder how many bytes we want
            nativeStartCollect(expectedBytes);

            // Start chirp playback
            startPlay();
            statusView.setText(getString(R.string.status_echoing));
        }

        // ─────────────────────────────────────────────
        // STOP STATE
        // ─────────────────────────────────────────────
        else {

            stopPlay();

            byte[] recorded = nativeStopAndGetRecording();



            if (recorded != null && recorded.length != expectedBytes) {
                Log.w("ECHO_DEBUG", "Expected " + expectedBytes + " bytes, got " + recorded.length);
            }

            // Convert recorded → short[]
            short[] pcm = null;
            if (recorded != null) {
                pcm = new short[recorded.length / 2];
                for (int i = 0; i < pcm.length; i++) {
                    pcm[i] = (short) ((recorded[i * 2] & 0xFF) |
                            (recorded[i * 2 + 1] << 8));
                }
            }

            pcm = trimLeadingBelowThreshold(pcm, 1000);
            //pcm = trimLeadingZeros(pcm);
            // Convert chirp → short[]
            short[] chirp_pcm = null;
            if (chirp != null) {
                chirp_pcm = new short[chirp.length / 2];
                for (int i = 0; i < chirp_pcm.length; i++) {
                    chirp_pcm[i] = (short) ((chirp[i * 2] & 0xFF) |
                            (chirp[i * 2 + 1] << 8));
                }
            }

            // Show chirp waveform
            WaveformView waveformView2 = findViewById(R.id.waveformView2);
            if (chirp_pcm != null && waveformView2 != null) {
                waveformView2.setAudioData(chirp_pcm);
            }

            // Debug recorded bytes
            if (recorded != null) {
                Log.i("ECHO_DEBUG", "Recorded buffer length = " + recorded.length);

                StringBuilder sb = new StringBuilder();
                sb.append("First samples: ");
                for (int i = 0; i < recorded.length; i++) {
                    sb.append(String.format("%02X ", recorded[i]));
                }
                Log.i("ECHO_DEBUG", sb.toString());
            } else {
                Log.e("ECHO_DEBUG", "Recorded buffer is null");
            }

            assert chirp_pcm != null;
            assert pcm != null;

            // Correlation to align
            double[] corr = correlate(pcm, chirp_pcm);
            int corr_idx = findPeak(corr, pcm.length, chirp_pcm.length);
            int shift = corr_idx - (chirp_pcm.length - 1); // shift >= 0 means chirp starts at pcm[shift]
            Log.i("peak correlation index from java", String.valueOf(corr_idx));
            Log.i("alignment (pcm sample index)", String.valueOf(shift));
            short[] pcm_shifted = Arrays.copyOfRange(pcm, Math.max(0, shift), pcm.length);

            short[] pcm_final  = Arrays.copyOf(pcm_shifted, chirp_pcm.length); // zero padded

            assert (pcm_final.length == chirp_pcm.length);

            // Window


            // Multiply with chirp
            short[] pcm_final_win = applyHannWindowShortNormalized(pcm_final);

            short[] mult_java = multiplyPcm(pcm_final_win, chirp_pcm);




            // Show time-domain
            WaveformView waveformView = findViewById(R.id.waveformView);
            if (waveformView != null) {
                waveformView.setAudioData(pcm_final_win);
            }

            // FFT
            float[] FFT_java = fftShortArray(mult_java);
            float[] FFT_mag = computeMagnitude(FFT_java);

            FFTView FFTView = findViewById(R.id.FFTView);

            int peakIndex = peakIndexInRange(FFT_mag, 0, 200); //range max of ~7m
            int N = pcm_final_win.length;
            double freq = peakIndex * (sampleRate_/ (double) N);

            double D = ((V_s * freq * sweepTime )/ (2 * B) )/ 2;

            TextView distance = findViewById(R.id.distanceView);

            String s1 = String.format("%.3f", D);
            String str = "distance = " + s1 + " m";

            distance.setText(str);

            double reflection_generated = FFT_mag[peakIndex];

            double reflection_ideal = 1000000.0; //<<<<<<<<<<<<<<TUNE THIS

            double reflected_calculation = reflection_generated/reflection_ideal;

            if (reflected_calculation > 1.0) reflected_calculation = 1.0;

            TextView reflection = findViewById(R.id.reflectionView);

            String s2 = String.format("%.3f", reflected_calculation);


            String str2 = "reflection_coeff = " + s2;

            reflection.setText(str2);


            FFTView.setMarkerIndex(peakIndex);
            FFTView.setXLimitsIndices(0, 200);
            FFTView.setAudioData(FFT_mag);

            releasePlayerAndRecorder();
            statusView.setText("Stopped");
        }

        // Toggle state
        isPlaying = !isPlaying;
        controlButton.setText(
                getString(isPlaying ? R.string.StopEcho : R.string.StartEcho)
        );
    }



    public void onEchoClick(View view) {
        // For static playback we do not strictly require RECORD_AUDIO permission.
        // If you want to enable recording-based features, request permissions elsewhere.
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
                != PackageManager.PERMISSION_GRANTED) {

            ActivityCompat.requestPermissions(
                    this,
                    new String[]{Manifest.permission.RECORD_AUDIO},
                    AUDIO_ECHO_REQUEST);

            return; // wait for user response before starting
        }

        /*
        // Delay execution by 2000 ms (2 seconds)
        statusView.setText("Starting in 2 seconds...");


        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                startEcho();
            }
        }, 2000);

        */
        startEcho();


    }

    public void getLowLatencyParameters(View view) {
        updateNativeAudioUI();
        return;
    }

    private void queryNativeAudioParameters() {
        AudioManager myAudioMgr = (AudioManager) getSystemService(Context.AUDIO_SERVICE);
        nativeSampleRate  =  myAudioMgr.getProperty(AudioManager.PROPERTY_OUTPUT_SAMPLE_RATE);
        nativeSampleBufSize = myAudioMgr.getProperty(AudioManager.PROPERTY_OUTPUT_FRAMES_PER_BUFFER);
        //nativeSampleBufSize = String.valueOf(1024);
        int recBufSize = AudioRecord.getMinBufferSize(
                Integer.parseInt(nativeSampleRate),
                AudioFormat.CHANNEL_IN_MONO,
                AudioFormat.ENCODING_PCM_16BIT);
        supportRecording = true;
        if (recBufSize == AudioRecord.ERROR ||
                recBufSize == AudioRecord.ERROR_BAD_VALUE) {
            supportRecording = false;
        }
    }
    private void updateNativeAudioUI() {
        if (!supportRecording) {
            statusView.setText(getString(R.string.error_no_mic));
            controlButton.setEnabled(true); // still allow static playback
            return;
        }


    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        if (AUDIO_ECHO_REQUEST != requestCode) {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
            return;
        }
        if (grantResults.length != 1  ||
                grantResults[0] != PackageManager.PERMISSION_GRANTED) {
            statusView.setText(getString(R.string.error_no_permission));
            Toast.makeText(getApplicationContext(),
                    getString(R.string.prompt_permission),
                    Toast.LENGTH_SHORT).show();
            return;
        }
        statusView.setText("RECORD_AUDIO permission granted, touch " +
                getString(R.string.StartEcho) + " to begin");
        // If the user asked for permission earlier, they can now press button again
    }



    public static class AnalysisResult {
        public boolean distance_valid;
        public float[] FFT;
        public int peakBin;
        public double distance_m;
    }

    /*
     * Loading our Libs
     */
    static {
        System.loadLibrary("echo");
    }

    /*
     * jni function implementations...
     */
    public static native void createSLEngine(int rate, int framesPerBuf, int recordLength);
    public static native void deleteSLEngine();

    // old names kept for compatibility: createSLBufferQueueAudioPlayer now
    // creates the StaticAudioPlayer that consumes the uploaded PCM buffer
    public static native boolean createSLBufferQueueAudioPlayer();
    public static native void deleteSLBufferQueueAudioPlayer();

    public static native boolean createAudioRecorder();
    public static native void deleteAudioRecorder();
    public static native void startPlay();
    public static native void stopPlay();

    // NEW: upload a Java byte[] PCM buffer (16-bit little-endian) to native
    public static native boolean loadPCMBuffer(byte[] pcm);

    public static native void nativeStartCollect(int expectedBytes);
    public static native byte[] nativeStopAndGetRecording();

    public static native byte[] generateTonePCMNative(int sampleRate, int durationSeconds, double freqHz);

    public static native byte[] generateChirpPCMNative(boolean jApplyWindow);

    private static native AnalysisResult analyzeRecordedBuffer(byte[] pcmBytes, int sampleRate, byte[] referenceChirpBytes);


    public static void onNativeAnalysisResult(final boolean voiced, final int freq) {
        // instance might be null if activity is gone or destroyed
        final MainActivity act = instance;
        if (act == null) return;

//        act.runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                if (voiced) {
//                    act.statusView.setText(Integer.toString(freq) + " Hz");
//                } else {
//                    act.statusView.setText("Unvoiced");
//                }
//            }
//        });
    }
}