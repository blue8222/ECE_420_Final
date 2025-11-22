package com.ece420.lab2;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.Manifest;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import android.os.Bundle;
import android.os.Handler;

import java.util.Arrays;
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
    private static MainActivity instance;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        instance = this;
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
        // cleanup native engine
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

    private void startEcho() {

        int sampleRate = Integer.parseInt(nativeSampleRate);

        // Generate chirp
        byte[] chirp = generateChirpPCMNative();

        if (!isPlaying) {

            int expectedBytes = (int)(sampleRate * 2 * 2.0); // 1 second recording

            // 1. Load PCM into native buffer
            boolean ok = loadPCMBuffer(chirp);
            if (!ok) {
                statusView.setText("Error loading PCM buffer");
                return;
            }

            // 2. Create static audio player
            if (!createSLBufferQueueAudioPlayer()) {
                statusView.setText("Error creating player");
                return;
            }

            // ⭐ 3. Create recorder (calls recorder->Start() in JNI)
            if (!createAudioRecorder()) {
                statusView.setText("Error creating recorder");
                deleteSLBufferQueueAudioPlayer();
                return;
            }

            // ⭐ 4. Tell native how many bytes of recording we want
            nativeStartCollect(expectedBytes);

            // 5. Start playback of chirp
            startPlay();
            statusView.setText(getString(R.string.status_echoing));

        } else {

            // STOP STATE --------------------------------------

            // Stop playback and recording
            stopPlay();

            // Get recorded buffer from native
            byte[] recorded = nativeStopAndGetRecording();

            WaveformView waveformView = findViewById(R.id.waveformView);

            if (recorded != null) {
                // Convert byte[] → short[] PCM
                short[] pcm = new short[recorded.length / 2];
                for (int i = 0; i < pcm.length; i++) {
                    pcm[i] = (short) ((recorded[i*2] & 0xFF) | (recorded[i*2 + 1] << 8));
                }

                waveformView.setAudioData(pcm);
            }

            // Debug log
            if (recorded != null) {
                Log.i("ECHO_DEBUG", "Recorded buffer length = " + recorded.length);

                StringBuilder sb = new StringBuilder();
                sb.append("First samples: ");
                for (int i = 0; i < Math.min(16, recorded.length); i++) {
                    sb.append(String.format("%02X ", recorded[i]));
                }
                Log.i("ECHO_DEBUG", sb.toString());
            } else {
                Log.e("ECHO_DEBUG", "Recorded buffer is null");
            }

            // Analyze the recorded audio

            AnalysisResult result = analyzeRecordedBuffer(recorded, sampleRate, chirp);

            FFTView FFTView = findViewById(R.id.FFTView);

            FFTView.setAudioData(result.FFT);




            Log.i("ECHO_DEBUG", "FFT = " + Arrays.toString(result.FFT));


            if (result != null) {
                String msg = String.format(Locale.US,
                        "Distance: %.2f m\n",
                        result.distance_m
                );
                distanceView.setText(msg);



            } else {
                distanceView.setText("Analysis failed.");
            }

            deleteAudioRecorder();
            deleteSLBufferQueueAudioPlayer();
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

    public static native byte[] generateChirpPCMNative();

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