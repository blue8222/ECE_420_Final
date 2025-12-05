package com.ece420.lab2;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import java.util.Locale;

public class FFTView extends View {

    private float[] fftData = null;

    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint backgroundPaint = new Paint();
    private final Paint axisPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint markerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // x-axis display limits
    private int xStartIndex = -1;
    private int xEndIndex = -1;

    // Marker index for vertical line
    private int markerIndex = -1;

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

        markerPaint.setColor(0xFFFF0000); // red for marker line
        markerPaint.setStrokeWidth(2f);
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

    public void setMarkerIndex(int index) {
        this.markerIndex = index;
        invalidate();
    }

    public void resetMarker() {
        this.markerIndex = -1;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int w = getWidth();
        int hFull = getHeight();

        // respect padding
        int leftPad = getPaddingLeft() + 8;
        int rightPad = getPaddingRight() + 8;
        int bottomPad = getPaddingBottom() + axisPadding; // space for axis/labels
        int topPad = getPaddingTop() + 8;

        canvas.drawRect(0, 0, w, hFull, backgroundPaint);

        if (fftData == null || fftData.length == 0) return;

        int n = fftData.length;
        int start = (xStartIndex >= 0) ? xStartIndex : 0;
        int end = (xEndIndex >= 0) ? xEndIndex : (n - 1);
        start = Math.max(0, Math.min(start, n - 1));
        end = Math.max(0, Math.min(end, n - 1));
        if (start > end) { start = 0; end = n - 1; }

        int displayCount = end - start + 1;
        int availableWidth = Math.max(1, w - leftPad - rightPad);
        float barWidth = (float) availableWidth / displayCount;
        int availableHeight = Math.max(1, hFull - bottomPad - topPad);

        // compute max absolute magnitude
        float maxVal = 1e-6f;
        for (int i = start; i <= end; i++) {
            float v = fftData[i];
            if (!Float.isFinite(v)) continue;
            v = Math.abs(v);
            if (v > maxVal) maxVal = v;
        }

        // draw bars
        paint.setAntiAlias(false);
        for (int i = 0; i < displayCount; i++) {
            int dataIndex = start + i;
            float v = fftData[dataIndex];
            if (!Float.isFinite(v)) v = 0f;
            v = Math.abs(v) / maxVal;
            float left = leftPad + i * barWidth;
            float right = left + Math.max(1f, barWidth);
            float top = topPad + (availableHeight * (1f - v));
            float bottom = topPad + availableHeight;
            canvas.drawRect(left, top, right, bottom, paint);
        }

        // axis
        float axisY = topPad + availableHeight + 6;
        canvas.drawLine(leftPad, axisY, w - rightPad, axisY, axisPaint);

        // ticks: show first/last + a few intermediates based on width
        int maxTicks = Math.min(6, displayCount);
        if (maxTicks < 2) maxTicks = 2;
        labelPaint.setTextAlign(Paint.Align.CENTER);
        for (int t = 0; t < maxTicks; t++) {
            float frac = (float) t / (maxTicks - 1);
            float x = leftPad + frac * (availableWidth);
            int bin = start + Math.round(frac * (displayCount - 1));
            String lbl;
            float fftSize = fftData.length;
            double sampleRate = 48000;
            double sweepTime = 0.35;
            int bandwidth = 10000;
            double V_s = 343.0;
            double fb = bin * (sampleRate / fftData.length);

            double R = ((V_s * fb * sweepTime) / (2.0 * bandwidth) )/ 2;




            if (fftSize > 0) {
                lbl = String.format(Locale.US, "%.2f m", R);
            } else {
                assert (fftSize != 0);

                lbl = ("axis failure");

                //freq_hz = peak_bin * (sampleRate / static_cast<double>(N));
                //(V_s * sweepTime * freq_hz) / static_cast<float>(bandwidth);
            }
            canvas.drawLine(x, axisY, x, axisY + 8, axisPaint);
            canvas.drawText(lbl, x, axisY + 30, labelPaint);
        }

        // Draw vertical marker line if set and within range
        if (markerIndex >= start && markerIndex <= end) {
            float markerX = leftPad + (markerIndex - start) * barWidth + barWidth / 2f; // Center on the bar
            canvas.drawLine(markerX, topPad, markerX, topPad + availableHeight, markerPaint);
        }
    }
}