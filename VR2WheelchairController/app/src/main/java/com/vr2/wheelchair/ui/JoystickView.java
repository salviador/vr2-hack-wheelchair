package com.vr2.wheelchair.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

/**
 * Custom Joystick View for wheelchair control
 * 
 * Features:
 * - Circular joystick area
 * - Returns values from -127 to +127 for both axes
 * - Visual feedback
 * - Optional deadzone
 */
public class JoystickView extends View {
    
    private static final String TAG = "JoystickView";
    
    // Paints
    private Paint backgroundPaint;
    private Paint borderPaint;
    private Paint stickPaint;
    private Paint stickBorderPaint;
    private Paint crosshairPaint;
    private Paint textPaint;
    
    // Dimensions
    private float centerX;
    private float centerY;
    private float baseRadius;
    private float stickRadius;
    private float stickX;
    private float stickY;
    
    // Configuration
    private float deadzone = 0.1f;  // 10% deadzone
    private int maxValue = 100;
    
    // Current values
    private int valueX = 0;
    private int valueY = 0;
    private boolean isTouching = false;
    // Aggiungi queste variabili dopo "private boolean isTouching = false;"
    private boolean dragMode = false;  // false = touch anywhere, true = drag only
    private boolean isDragging = false;

    // Callback
    public interface JoystickListener {
        void onJoystickMoved(int x, int y);
        void onJoystickReleased();
    }
    
    private JoystickListener listener;
    
    public JoystickView(Context context) {
        super(context);
        init();
    }
    
    public JoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }
    
    public JoystickView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }
    
    private void init() {
        // Background (outer circle)
        backgroundPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        backgroundPaint.setColor(Color.parseColor("#2D2D2D"));
        backgroundPaint.setStyle(Paint.Style.FILL);
        
        // Border
        borderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        borderPaint.setColor(Color.parseColor("#4CAF50"));
        borderPaint.setStyle(Paint.Style.STROKE);
        borderPaint.setStrokeWidth(6f);
        
        // Stick (inner circle)
        stickPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        stickPaint.setColor(Color.parseColor("#4CAF50"));
        stickPaint.setStyle(Paint.Style.FILL);
        
        // Stick border
        stickBorderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        stickBorderPaint.setColor(Color.parseColor("#81C784"));
        stickBorderPaint.setStyle(Paint.Style.STROKE);
        stickBorderPaint.setStrokeWidth(4f);
        
        // Crosshair
        crosshairPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        crosshairPaint.setColor(Color.parseColor("#666666"));
        crosshairPaint.setStyle(Paint.Style.STROKE);
        crosshairPaint.setStrokeWidth(2f);
        
        // Text
        textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setColor(Color.parseColor("#AAAAAA"));
        textPaint.setTextSize(32f);
        textPaint.setTextAlign(Paint.Align.CENTER);
    }
    
    public void setJoystickListener(JoystickListener listener) {
        this.listener = listener;
    }
    
    public void setDeadzone(float deadzone) {
        this.deadzone = Math.max(0f, Math.min(0.5f, deadzone));
    }
    
    public int getValueX() {
        return valueX;
    }
    
    public int getValueY() {
        return valueY;
    }
    
    public boolean isTouching() {
        return isTouching;
    }
    
    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        
        centerX = w / 2f;
        centerY = h / 2f;
        
        // Base radius is 90% of the smaller dimension
        baseRadius = Math.min(w, h) * 0.45f;
        
        // Stick radius is 30% of base radius
        stickRadius = baseRadius * 0.35f;
        
        // Initialize stick at center
        stickX = centerX;
        stickY = centerY;
    }
    
    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        
        // Draw background circle
        canvas.drawCircle(centerX, centerY, baseRadius, backgroundPaint);
        
        // Draw border
        canvas.drawCircle(centerX, centerY, baseRadius, borderPaint);
        
        // Draw crosshair
        canvas.drawLine(centerX - baseRadius * 0.8f, centerY, 
                       centerX + baseRadius * 0.8f, centerY, crosshairPaint);
        canvas.drawLine(centerX, centerY - baseRadius * 0.8f, 
                       centerX, centerY + baseRadius * 0.8f, crosshairPaint);
        
        // Draw deadzone circle
        if (deadzone > 0) {
            crosshairPaint.setAlpha(100);
            canvas.drawCircle(centerX, centerY, baseRadius * deadzone, crosshairPaint);
            crosshairPaint.setAlpha(255);
        }
        
        // Draw direction labels
        float labelOffset = baseRadius + 40;
        canvas.drawText("F", centerX, centerY - labelOffset + 10, textPaint);
        canvas.drawText("B", centerX, centerY + labelOffset + 10, textPaint);
        canvas.drawText("L", centerX - labelOffset, centerY + 10, textPaint);
        canvas.drawText("R", centerX + labelOffset, centerY + 10, textPaint);
        
        // Update stick color based on state
        if (isTouching) {
            stickPaint.setColor(Color.parseColor("#66BB6A"));
        } else {
            stickPaint.setColor(Color.parseColor("#4CAF50"));
        }
        
        // Draw stick
        canvas.drawCircle(stickX, stickY, stickRadius, stickPaint);
        canvas.drawCircle(stickX, stickY, stickRadius, stickBorderPaint);
        
        // Draw current values in center of stick
        if (isTouching && (valueX != 0 || valueY != 0)) {
            Paint valuePaint = new Paint(textPaint);
            valuePaint.setColor(Color.WHITE);
            valuePaint.setTextSize(24f);
            canvas.drawText(String.format("%d,%d", valueX, valueY), 
                           stickX, stickY + 8, valuePaint);
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                if (dragMode) {
                    // Check if touch is on the stick
                    float dx = event.getX() - stickX;
                    float dy = event.getY() - stickY;
                    float distance = (float) Math.sqrt(dx * dx + dy * dy);
                    if (distance <= stickRadius * 1.5f) {  // 1.5x for easier grab
                        isDragging = true;
                        isTouching = true;
                    }
                } else {
                    isTouching = true;
                    processTouch(event.getX(), event.getY());
                }
                break;

            case MotionEvent.ACTION_MOVE:
                if (dragMode) {
                    if (isDragging) {
                        processTouch(event.getX(), event.getY());
                    }
                } else {
                    processTouch(event.getX(), event.getY());
                }
                break;

            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_CANCEL:
                isTouching = false;
                isDragging = false;
                resetStick();
                break;
        }

        return true;
    }
    /**
     * Set drag mode
     * @param enabled true = must touch stick to drag, false = touch anywhere
     */
    public void setDragMode(boolean enabled) {
        this.dragMode = enabled;
    }

    public boolean isDragMode() {
        return dragMode;
    }
    private void processTouch(float touchX, float touchY) {
        // Calculate distance from center
        float dx = touchX - centerX;
        float dy = touchY - centerY;
        float distance = (float) Math.sqrt(dx * dx + dy * dy);
        
        // Limit to base radius
        if (distance > baseRadius - stickRadius) {
            float ratio = (baseRadius - stickRadius) / distance;
            dx *= ratio;
            dy *= ratio;
            distance = baseRadius - stickRadius;
        }
        
        // Update stick position
        stickX = centerX + dx;
        stickY = centerY + dy;
        
        // Calculate normalized values (-1 to +1)
        float normalizedX = dx / (baseRadius - stickRadius);
        float normalizedY = -dy / (baseRadius - stickRadius);  // Invert Y (up = positive)
        
        // Apply deadzone
        float magnitude = (float) Math.sqrt(normalizedX * normalizedX + normalizedY * normalizedY);
        if (magnitude < deadzone) {
            normalizedX = 0;
            normalizedY = 0;
        } else {
            // Scale values outside deadzone to full range
            float scale = (magnitude - deadzone) / (1 - deadzone) / magnitude;
            normalizedX *= scale;
            normalizedY *= scale;
        }
        
        // Convert to integer values
        int newX = Math.round(normalizedX * maxValue);
        int newY = Math.round(normalizedY * maxValue);
        
        // Clamp
        newX = Math.max(-maxValue, Math.min(maxValue, newX));
        newY = Math.max(-maxValue, Math.min(maxValue, newY));
        
        // Update if changed
        if (newX != valueX || newY != valueY) {
            valueX = newX;
            valueY = newY;
            
            if (listener != null) {
                listener.onJoystickMoved(valueX, valueY);
            }
        }
        
        invalidate();
    }
    
    private void resetStick() {
        stickX = centerX;
        stickY = centerY;
        valueX = 0;
        valueY = 0;
        
        if (listener != null) {
            listener.onJoystickReleased();
        }
        
        invalidate();
    }
    
    /**
     * Programmatically set stick position (for external control)
     */
    public void setValues(int x, int y) {
        valueX = Math.max(-maxValue, Math.min(maxValue, x));
        valueY = Math.max(-maxValue, Math.min(maxValue, y));
        
        // Update visual position
        stickX = centerX + (valueX / (float) maxValue) * (baseRadius - stickRadius);
        stickY = centerY - (valueY / (float) maxValue) * (baseRadius - stickRadius);
        
        invalidate();
    }
    
    /**
     * Set stick color
     */
    public void setStickColor(int color) {
        stickPaint.setColor(color);
        invalidate();
    }
    
    /**
     * Set border color
     */
    public void setBorderColor(int color) {
        borderPaint.setColor(color);
        stickBorderPaint.setColor(color);
        invalidate();
    }
}
