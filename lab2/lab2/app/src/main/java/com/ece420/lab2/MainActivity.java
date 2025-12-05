package com.ece420.lab2;

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
    public static float[] fftShortArray(short[] pcm) {
        if (pcm == null || pcm.length == 0) return new float[0];

        int n = pcm.length;

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

    public static double[] correlate(short[] signal, short[] template) {
        int N = signal.length;
        int M = template.length;
        int convLen = N + M - 1;

        // Next power of 2 >= convLen
        int fftSize = 1;
        while (fftSize < convLen) fftSize <<= 1;

        // Allocate real+imag (interleaved) buffers
        float[] A = new float[2 * fftSize];
        float[] B = new float[2 * fftSize];

        // Copy signal into A (real)
        for (int i = 0; i < N; i++) A[2 * i] = signal[i];

        // Copy template into B reversed (needed for correlation)
        // corr = conv(signal, reverse(template))
        for (int i = 0; i < M; i++) B[2 * i] = template[M - 1 - i];

        FloatFFT_1D fft = new FloatFFT_1D(fftSize);

        // Forward FFTs (complex)
        fft.complexForward(A);
        fft.complexForward(B);

        // Multiply A *= conj(B)
        for (int k = 0; k < 2 * fftSize; k += 2) {
            float ar = A[k], ai = A[k + 1];
            float br = B[k], bi = B[k + 1];

            // conj(B) = (br, -bi)
            A[k]     = ar * br + ai * bi;   // real
            A[k + 1] = ai * br - ar * bi;   // imag
        }

        // IFFT
        fft.complexInverse(A, true);

        // Extract real part of convolution output (valid: convLen samples)
        double[] corr = new double[convLen];
        for (int i = 0; i < convLen; i++) {
            corr[i] = A[2 * i];
        }

        return corr;
    }


    /** Find best correlation peak (max absolute). */
    public static int findPeak(double[] corr) {
        int best = 0;
        double bestVal = Math.abs(corr[0]);
        for (int i = 1; i < corr.length; i++) {
            double v = Math.abs(corr[i]);
            if (v > bestVal) {
                bestVal = v;
                best = i;
            }
        }
        return best;
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

        // Generate chirp
        byte[] chirp = generateChirpPCMNative(true);

        int expectedBytes = (int) (sampleRate * 2 * 0.4);;

        if (!isPlaying) {



            // 1. Load PCM into native buffer
            byte[] stereoLeft = monoToStereoLeft(chirp);

            boolean ok = loadPCMBuffer(stereoLeft);

            if (!ok) {
                statusView.setText("Error loading PCM buffer");
                return;
            }

            // 2. Create static audio player
            if (!createSLBufferQueueAudioPlayer()) {
                statusView.setText("Error creating player");
                return;
            }

            // 3. Create recorder (calls recorder->Start() in JNI)
            if (!createAudioRecorder()) {
                statusView.setText("Error creating recorder");
                deleteSLBufferQueueAudioPlayer();
                return;
            }

            // 4. Tell native how many bytes of recording we want
            nativeStartCollect(expectedBytes);

            // 5. Start playback of chirp
            startPlay();
            statusView.setText(getString(R.string.status_echoing));

        } else {

            // STOP STATE --------------------------------------

            stopPlay();

            // Get recorded buffer from native
            byte[] recorded = nativeStopAndGetRecording();


            if (recorded != null && recorded.length != expectedBytes) {
                Log.w("ECHO_DEBUG", "Expected " + expectedBytes + " bytes, got " + recorded.length);
            }


            WaveformView waveformView = findViewById(R.id.waveformView);

            short[] pcm = null;

            if (recorded != null) {
                // Convert byte[] → short[] PCM
                pcm = new short[recorded.length / 2];
                for (int i = 0; i < pcm.length; i++) {
                    pcm[i] = (short) ((recorded[i * 2] & 0xFF) | (recorded[i * 2 + 1] << 8));
                }

            }

            WaveformView waveformView2 = findViewById(R.id.waveformView2);

            short[] chirp_pcm = null;

            if (chirp != null) {
                // Convert byte[] → short[] PCM
                chirp_pcm = new short[chirp.length / 2];
                for (int i = 0; i < chirp_pcm.length; i++) {
                    chirp_pcm[i] = (short) ((chirp[i * 2] & 0xFF) | (chirp[i * 2 + 1] << 8));
                }

                if (waveformView2 != null) {
                    waveformView2.setAudioData(chirp_pcm);
                }

            }


            assert chirp_pcm != null;
            assert pcm != null;
            double[] corr = correlate(pcm, chirp_pcm);

            int corr_idx = findPeak(corr);

            Log.i("peak correlation index from java", String.valueOf(corr_idx));


            short[] pcm_shifted = Arrays.copyOfRange(pcm, corr_idx, pcm.length - 1);

            short[] pcm_final = Arrays.copyOf(pcm_shifted, pcm.length); //this should zero pad the end if its too short

            waveformView.setAudioData(pcm_final);

            // Debug log
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

            byte[] byte_final = shortsToBytes(pcm_final);
            // Analyze the recorded audio

            assert(pcm_final.length == pcm.length);

            short[] mult_java = multiplyPcm(pcm, pcm_final); //multiplication in float to avoid clipping

            float[] FFT_java = fftShortArray(mult_java);



            float[] FFT_mag = computeMagnitude(FFT_java);




            /*
            AnalysisResult result = analyzeRecordedBuffer(byte_final, sampleRate, chirp);



            Log.i("peakBin", String.valueOf(result.peakBin));
            */

            FFTView FFTView = findViewById(R.id.FFTView);

            FFTView.setXLimitsIndices(0, 300);

            //FFTView.setMarkerIndex(result.peakBin);

            FFTView.setAudioData(FFT_mag);


            //FFT logging
            /*
            float[] fft = result.FFT;
            String full = Arrays.toString(fft);
            int maxLogSize = 1000; // safe length for Logcat

            for (int i = 0; i <= full.length() / maxLogSize; i++) {
                int start = i * maxLogSize;
                int end = Math.min((i + 1) * maxLogSize, full.length());
                Log.i("ECHO_DEBUG", full.substring(start, end));
            }

             */

            /*
            if (result != null) {
                String msg = String.format(Locale.US,
                        "Distance: %f m\n",
                        result.distance_m
                );
                Log.i("Distance = ", msg);
                distanceView.setText(msg);


            } else {
                distanceView.setText("Analysis failed.");
            }
            */

            //stop recorder

            releasePlayerAndRecorder();

            statusView.setText("Stopped");
        }

        isPlaying = !isPlaying;
        controlButton.setText(getString(isPlaying ? R.string.StopEcho : R.string.StartEcho));
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

        // Delay execution by 2000 ms (2 seconds)
        statusView.setText("Starting in 2 seconds...");

        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                startEcho();
            }
        }, 2000);
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

        act.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (voiced) {
                    act.statusView.setText(Integer.toString(freq) + " Hz");
                } else {
                    act.statusView.setText("Unvoiced");
                }
            }
        });
    }
}