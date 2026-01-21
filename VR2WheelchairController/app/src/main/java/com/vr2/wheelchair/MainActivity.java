package com.vr2.wheelchair;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.material.button.MaterialButtonToggleGroup;
import com.vr2.wheelchair.ble.VR2BleConstants;
import com.vr2.wheelchair.ble.VR2BleManager;
import com.vr2.wheelchair.ui.JoystickView;

import java.util.ArrayList;
import java.util.List;

import android.view.WindowManager;

/**
 * Main Activity for VR2 Wheelchair Controller
 * 
 * Features:
 * - Virtual joystick for movement control
 * - Start/Stop buttons
 * - Speed +/- buttons
 * - Connection status display
 * - Telemetry display (state, speed, battery)
 */
public class MainActivity extends AppCompatActivity implements VR2BleManager.VR2Callback {
    
    private static final String TAG = "MainActivity";
    private static final int REQUEST_PERMISSIONS = 100;
    private static final long JOYSTICK_UPDATE_INTERVAL_MS = 50;  // 20Hz
    
    // BLE
    private VR2BleManager bleManager;
    private BluetoothDevice connectedDevice;
    
    // UI Components
    private JoystickView joystickView;
    private Button btnConnect;
    private Button btnStart;
    private Button btnStop;
    private Button btnSpeedUp;
    private Button btnSpeedDown;
    private TextView tvConnectionStatus;
    private TextView tvState;
    private TextView tvSpeed;
    private TextView tvBattery;
    private View statusIndicator;
    
    // Joystick update handler
    private Handler joystickHandler;
    private Runnable joystickUpdateRunnable;
    private int lastSentX = 0;
    private int lastSentY = 0;
    
    // Activity result launcher for scan
    private final ActivityResultLauncher<Intent> scanLauncher = registerForActivityResult(
        new ActivityResultContracts.StartActivityForResult(),
        result -> {
            if (result.getResultCode() == Activity.RESULT_OK && result.getData() != null) {
                BluetoothDevice device = result.getData().getParcelableExtra(ScanActivity.EXTRA_DEVICE);
                if (device != null) {
                    connectToDevice(device);
                }
            }
        }
    );
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Keep screen on while app is open
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        initUI();
        initBle();
        initJoystickHandler();
    }
    
    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopJoystickUpdates();
        if (bleManager != null) {
            bleManager.disconnectDevice();
        }
    }
    
    // =========================================================================
    // UI Initialization
    // =========================================================================

    private void initUI() {
        // Find views
        joystickView = findViewById(R.id.joystickView);
        btnConnect = findViewById(R.id.btnConnect);
        btnStart = findViewById(R.id.btnStart);
        btnStop = findViewById(R.id.btnStop);
        btnSpeedUp = findViewById(R.id.btnSpeedUp);
        btnSpeedDown = findViewById(R.id.btnSpeedDown);
        tvConnectionStatus = findViewById(R.id.tvConnectionStatus);
        tvState = findViewById(R.id.tvState);
        tvSpeed = findViewById(R.id.tvSpeed);
        tvBattery = findViewById(R.id.tvBattery);
        statusIndicator = findViewById(R.id.statusIndicator);
        
        // Setup joystick listener
        joystickView.setJoystickListener(new JoystickView.JoystickListener() {
            @Override
            public void onJoystickMoved(int x, int y) {
                // Will be sent by joystickUpdateRunnable
            }
            
            @Override
            public void onJoystickReleased() {
                // Send center/stop immediately
                sendJoystickValues(0, 0);
            }
        });
        
        // Button listeners
        btnConnect.setOnClickListener(v -> onConnectClicked());
        btnStart.setOnClickListener(v -> onStartClicked());
        btnStop.setOnClickListener(v -> onStopClicked());
        btnSpeedUp.setOnClickListener(v -> onSpeedUpClicked());
        btnSpeedDown.setOnClickListener(v -> onSpeedDownClicked());
        
        // Initial state
        updateUIState(false);


        // Joystick mode toggle
        MaterialButtonToggleGroup toggleMode = findViewById(R.id.toggleJoystickMode);
        toggleMode.addOnButtonCheckedListener((group, checkedId, isChecked) -> {
            if (isChecked) {
                if (checkedId == R.id.btnModeDrag) {
                    joystickView.setDragMode(true);
                    Toast.makeText(this, "Drag mode", Toast.LENGTH_SHORT).show();
                } else {
                    joystickView.setDragMode(false);
                    Toast.makeText(this, "Touch mode", Toast.LENGTH_SHORT).show();
                }
            }
        });
    }
    
    private void initJoystickHandler() {
        joystickHandler = new Handler(Looper.getMainLooper());
        joystickUpdateRunnable = new Runnable() {
            @Override
            public void run() {
                if (joystickView.isTouching()) {
                    int x = joystickView.getValueX();
                    int y = joystickView.getValueY();
                    
                    // Only send if values changed
                    if (x != lastSentX || y != lastSentY) {
                        sendJoystickValues(x, y);
                    }
                }
                joystickHandler.postDelayed(this, JOYSTICK_UPDATE_INTERVAL_MS);
            }
        };
    }
    
    private void startJoystickUpdates() {
        joystickHandler.post(joystickUpdateRunnable);
    }
    
    private void stopJoystickUpdates() {
        joystickHandler.removeCallbacks(joystickUpdateRunnable);
    }
    
    // =========================================================================
    // BLE Initialization
    // =========================================================================
    
    private void initBle() {
        bleManager = new VR2BleManager(this);
        bleManager.setCallback(this);
        
        // Observe connection state
        bleManager.getConnectedState().observe(this, connected -> {
            updateUIState(connected);
            if (connected) {
                startJoystickUpdates();
            } else {
                stopJoystickUpdates();
            }
        });
        
        // Observe connection status
        bleManager.getConnectionStatus().observe(this, status -> {
            tvConnectionStatus.setText(status);
        });
        
        // Observe telemetry
        bleManager.getTelemetry().observe(this, this::updateTelemetry);
    }
    
    // =========================================================================
    // Permissions
    // =========================================================================
    
    private boolean checkPermissions() {
        List<String> permissions = new ArrayList<>();
        
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) 
                    != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.BLUETOOTH_SCAN);
            }
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) 
                    != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.BLUETOOTH_CONNECT);
            }
        } else {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) 
                    != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.ACCESS_FINE_LOCATION);
            }
        }
        
        if (!permissions.isEmpty()) {
            ActivityCompat.requestPermissions(this, 
                permissions.toArray(new String[0]), 
                REQUEST_PERMISSIONS);
            return false;
        }
        return true;
    }
    
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, 
                                          @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        
        if (requestCode == REQUEST_PERMISSIONS) {
            boolean allGranted = true;
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED) {
                    allGranted = false;
                    break;
                }
            }
            
            if (allGranted) {
                openScanActivity();
            } else {
                Toast.makeText(this, "Permissions required for BLE", Toast.LENGTH_LONG).show();
            }
        }
    }
    
    // =========================================================================
    // Button Handlers
    // =========================================================================
    
    private void onConnectClicked() {
        if (Boolean.TRUE.equals(bleManager.getConnectedState().getValue())) {
            // Disconnect
            bleManager.disconnectDevice();
            connectedDevice = null;
        } else {
            // Open scan activity
            if (checkPermissions()) {
                openScanActivity();
            }
        }
    }
    
    private void openScanActivity() {
        Intent intent = new Intent(this, ScanActivity.class);
        scanLauncher.launch(intent);
    }
    
    private void connectToDevice(BluetoothDevice device) {
        connectedDevice = device;
        try {
            Log.i(TAG, "Connecting to: " + device.getName() + " [" + device.getAddress() + "]");
        } catch (SecurityException e) {
            Log.i(TAG, "Connecting to: " + device.getAddress());
        }
        bleManager.connectToDevice(device);
    }
    
    private void onStartClicked() {
        if (Boolean.TRUE.equals(bleManager.getConnectedState().getValue())) {
            bleManager.sendStart();
            Toast.makeText(this, "START sent", Toast.LENGTH_SHORT).show();
        }
    }
    
    private void onStopClicked() {
        if (Boolean.TRUE.equals(bleManager.getConnectedState().getValue())) {
            bleManager.sendShutdown();
            Toast.makeText(this, "SHUTDOWN sent", Toast.LENGTH_SHORT).show();
        }
    }
    
    private void onSpeedUpClicked() {
        if (Boolean.TRUE.equals(bleManager.getConnectedState().getValue())) {
            bleManager.sendSpeedPlus();
            Toast.makeText(this, "SPEED+ sent", Toast.LENGTH_SHORT).show();
        }
    }
    
    private void onSpeedDownClicked() {
        if (Boolean.TRUE.equals(bleManager.getConnectedState().getValue())) {
            bleManager.sendSpeedMinus();
            Toast.makeText(this, "SPEED- sent", Toast.LENGTH_SHORT).show();
        }
    }
    
    private void sendJoystickValues(int x, int y) {
        lastSentX = x;
        lastSentY = y;
        
        if (Boolean.TRUE.equals(bleManager.getConnectedState().getValue())) {
            bleManager.sendJoystick(x, y);
        }
    }
    
    // =========================================================================
    // UI Updates
    // =========================================================================
    
    private void updateUIState(boolean connected) {
        btnConnect.setText(connected ? "DISCONNECT" : "CONNECT");
        
        // Enable/disable control buttons
        btnStart.setEnabled(connected);
        btnStop.setEnabled(connected);
        btnSpeedUp.setEnabled(connected);
        btnSpeedDown.setEnabled(connected);
        joystickView.setEnabled(connected);
        
        // Update status indicator
        statusIndicator.setBackgroundColor(connected ? 
            Color.parseColor("#4CAF50") :  // Green
            Color.parseColor("#F44336"));  // Red
        
        // Update joystick colors
        if (connected) {
            joystickView.setBorderColor(Color.parseColor("#4CAF50"));
        } else {
            joystickView.setBorderColor(Color.parseColor("#666666"));
        }
        
        if (!connected) {
            // Reset telemetry display
            tvState.setText("--");
            tvSpeed.setText("--");
            tvBattery.setText("--");
        }
    }
    
    private void updateTelemetry(VR2BleManager.TelemetryData data) {
        if (data == null) return;
        
        // Update state
        tvState.setText(data.getStateName());
        
        // Set state color
        int stateColor;
        switch (data.state) {
            case VR2BleConstants.STATE_RUN:
                stateColor = Color.parseColor("#4CAF50");  // Green
                break;
            case VR2BleConstants.STATE_INIT:
                stateColor = Color.parseColor("#FFC107");  // Yellow
                break;
            case VR2BleConstants.STATE_ERROR:
                stateColor = Color.parseColor("#F44336");  // Red
                break;
            case VR2BleConstants.STATE_SHUTDOWN:
                stateColor = Color.parseColor("#FF9800");  // Orange
                break;
            default:
                stateColor = Color.parseColor("#9E9E9E");  // Gray
        }
        tvState.setTextColor(stateColor);
        
        // Update speed (1-5 from ECU speed command)
        int speedLevel = getSpeedLevel(data.speed);
        tvSpeed.setText(String.valueOf(speedLevel));
        
        // Update battery
        if (data.hasError()) {
            tvBattery.setText(data.getErrorDescription());
            tvBattery.setTextColor(Color.parseColor("#F44336"));
        } else {
            tvBattery.setText(data.getBatteryLevel());
            tvBattery.setTextColor(Color.parseColor("#FFFFFF"));
        }
    }
    
    /**
     * Convert ECU speed command to level 1-5
     */
    private int getSpeedLevel(int speedCmd) {
        switch (speedCmd) {
            case 0x21: return 1;
            case 0x41: return 2;
            case 0x61: return 3;
            case 0x81: return 4;
            case 0xA1: return 5;
            default: return (speedCmd >> 5) + 1;  // Approximate
        }
    }
    
    // =========================================================================
    // VR2Callback Implementation
    // =========================================================================
    
    @Override
    public void onConnected() {
        runOnUiThread(() -> {
            Toast.makeText(this, "Connected to VR2", Toast.LENGTH_SHORT).show();
        });
    }
    
    @Override
    public void onDisconnected() {
        runOnUiThread(() -> {
            Toast.makeText(this, "Disconnected", Toast.LENGTH_SHORT).show();
        });
    }
    
    @Override
    public void onTelemetryReceived(VR2BleManager.TelemetryData data) {
        // Already handled by LiveData observer
    }
    
    @Override
    public void onError(String message) {
        runOnUiThread(() -> {
            Toast.makeText(this, "Error: " + message, Toast.LENGTH_LONG).show();
        });
    }
}
