package com.ece420.lab2;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

public class FFTView extends View {

    private float[] fftData = null;
    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint backgroundPaint = new Paint();


    public void setAudioData(float[] fft) {
        this.fftData = fft;
        invalidate(); // trigger redraw
    }

    public FFTView(Context context) {
        super(context);
        init();
    }

    public FFTView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public FFTView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        paint.setColor(0xFF00FF00); // Bright green bars
        paint.setStrokeWidth(2f);

        backgroundPaint.setColor(0xFF000000); // Black background
    }

    /** Call this to update the FFT buffer and trigger visualization */
    /*public void updateFFT(float[] newFFT) {
        this.fftData = newFFT;
        invalidate();  // Request redraw on UI thread
    }*/

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        canvas.drawRect(0, 0, getWidth(), getHeight(), backgroundPaint);

        if (fftData == null || fftData.length == 0) {
            return;
        }

        int width = getWidth();
        int height = getHeight();
        int n = fftData.length;

        float barWidth = (float) width / n;

        // Find max value for automatic scaling
        float maxVal = 0f;
        for (float v : fftData) {
            if (v > maxVal) maxVal = v;
        }
        if (maxVal < 1e-6f) maxVal = 1e-6f;

        for (int i = 0; i < n; i++) {
            float value = fftData[i] / maxVal;
            float barHeight = value * height;

            // Draw from bottom up
            float x = i * barWidth;
            canvas.drawLine(
                    x,
                    height,
                    x,
                    height - barHeight,
                    paint
            );
        }
    }
}
