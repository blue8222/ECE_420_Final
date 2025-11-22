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
    private final Paint axisPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // x-axis display limits
    private int xStartIndex = -1;
    private int xEndIndex = -1;

    // Space reserved for axis labels (in pixels)
    private final int axisPadding = 60;

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
        paint.setColor(0xFF00FF00); // bars
        paint.setStrokeWidth(2f);

        backgroundPaint.setColor(0xFF000000);

        axisPaint.setColor(0xFFFFFFFF);
        axisPaint.setStrokeWidth(2f);

        labelPaint.setColor(0xFFFFFFFF);
        labelPaint.setTextSize(28f);
        labelPaint.setTextAlign(Paint.Align.CENTER);
    }

    public void setAudioData(float[] fft) {
        this.fftData = fft;
        if (fft != null && xStartIndex >= fft.length)
            resetXLimits();
        invalidate();
    }

    public void setXLimitsIndices(int startIndex, int endIndex) {
        if (fftData == null || fftData.length == 0) {
            this.xStartIndex = startIndex;
            this.xEndIndex = endIndex;
            invalidate();
            return;
        }

        if (endIndex < startIndex) {
            int tmp = startIndex;
            startIndex = endIndex;
            endIndex = tmp;
        }

        int n = fftData.length;
        startIndex = Math.max(0, startIndex);
        endIndex = Math.min(n - 1, endIndex);

        if (startIndex > endIndex) {
            resetXLimits();
            return;
        }

        this.xStartIndex = startIndex;
        this.xEndIndex = endIndex;
        invalidate();
    }

    public void setXLimitsFractions(float startFrac, float endFrac) {
        startFrac = Math.max(0f, Math.min(1f, startFrac));
        endFrac = Math.max(0f, Math.min(1f, endFrac));

        if (endFrac < startFrac) {
            float t = startFrac;
            startFrac = endFrac;
            endFrac = t;
        }

        if (fftData == null) {
            this.xStartIndex = -1;
            this.xEndIndex = -1;
            invalidate();
            return;
        }

        int n = fftData.length;
        int startIndex = (int) Math.floor(startFrac * (n - 1));
        int endIndex = (int) Math.ceil(endFrac * (n - 1));

        setXLimitsIndices(startIndex, endIndex);
    }

    public void resetXLimits() {
        xStartIndex = -1;
        xEndIndex = -1;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getWidth();
        int height = getHeight() - axisPadding; // leave room for axis
        canvas.drawRect(0, 0, width, getHeight(), backgroundPaint);

        if (fftData == null || fftData.length == 0)
            return;

        int n = fftData.length;

        int start = (xStartIndex >= 0) ? xStartIndex : 0;
        int end = (xEndIndex >= 0) ? xEndIndex : (n - 1);

        start = Math.max(0, Math.min(start, n - 1));
        end = Math.max(0, Math.min(end, n - 1));

        if (start > end) {
            start = 0;
            end = n - 1;
        }

        int displayCount = end - start + 1;
        float barWidth = (float) width / displayCount;

        // Scale bars
        float maxVal = 0f;
        for (int i = start; i <= end; i++)
            if (fftData[i] > maxVal) maxVal = fftData[i];
        if (maxVal < 1e-6f) maxVal = 1e-6f;

        // Draw FFT bars
        for (int i = 0; i < displayCount; i++) {
            int dataIndex = start + i;
            float value = fftData[dataIndex] / maxVal;
            float barHeight = value * height;

            float x = i * barWidth + barWidth / 2f;

            canvas.drawLine(
                    x,
                    height,
                    x,
                    height - barHeight,
                    paint
            );
        }

        // ==== Draw X axis line ====
        float axisY = height + 10;
        canvas.drawLine(0, axisY, width, axisY, axisPaint);

        // ==== Draw labels ====
        int numTicks = Math.min(10, displayCount);
        if (numTicks < 2) numTicks = 2;

        for (int t = 0; t < numTicks; t++) {
            float frac = (float) t / (numTicks - 1);
            float x = frac * width;

            int bin = start + Math.round(frac * (displayCount - 1));

            // Tick mark
            canvas.drawLine(x, axisY, x, axisY + 10, axisPaint);

            // Label (bin index)
            canvas.drawText(
                    String.valueOf(bin),
                    x,
                    axisY + 40,
                    labelPaint
            );
        }
    }
}