package com.ece420.lab2;


import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

public class WaveformView extends View {

    private short[] samples = new short[0];
    private Paint paint = new Paint();

    public WaveformView(Context context, AttributeSet attrs) {
        super(context, attrs);
        paint.setColor(0xFF00FF00); // Green
        paint.setStrokeWidth(2f);
    }

    public void setAudioData(short[] pcm) {
        this.samples = pcm;
        invalidate(); // trigger redraw
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (samples.length == 0) return;

        float midY = getHeight() / 2f;
        float scaleX = (float)getWidth() / samples.length;
        float scaleY = (float)(getHeight() / 2) / 32768f;

        for (int i = 1; i < samples.length; i++) {
            float x1 = (i - 1) * scaleX;
            float y1 = midY - samples[i - 1] * scaleY;
            float x2 = i * scaleX;
            float y2 = midY - samples[i] * scaleY;
            canvas.drawLine(x1, y1, x2, y2, paint);
        }
    }
}

