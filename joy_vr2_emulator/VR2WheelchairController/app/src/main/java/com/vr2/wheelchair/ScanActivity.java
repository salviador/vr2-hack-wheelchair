package com.vr2.wheelchair;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.vr2.wheelchair.ble.VR2BleConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Activity to scan and select VR2 BLE devices
 */
public class ScanActivity extends AppCompatActivity {
    
    private static final String TAG = "ScanActivity";
    private static final int REQUEST_PERMISSIONS = 1;
    private static final long SCAN_DURATION_MS = 15000;
    
    public static final String EXTRA_DEVICE = "device";
    
    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLeScanner bleScanner;
    private Handler handler;
    private boolean isScanning = false;
    
    // UI
    private RecyclerView deviceList;
    private DeviceAdapter adapter;
    private Button btnScan;
    private ProgressBar progressBar;
    private TextView statusText;
    
    // Devices found
    private final List<ScanResult> devices = new ArrayList<>();
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_scan);
        
        handler = new Handler(Looper.getMainLooper());
        
        // Initialize UI
        deviceList = findViewById(R.id.deviceList);
        btnScan = findViewById(R.id.btnScan);
        progressBar = findViewById(R.id.progressBar);
        statusText = findViewById(R.id.statusText);
        
        adapter = new DeviceAdapter(this::onDeviceSelected);
        deviceList.setLayoutManager(new LinearLayoutManager(this));
        deviceList.setAdapter(adapter);
        
        btnScan.setOnClickListener(v -> {
            if (isScanning) {
                stopScan();
            } else {
                startScan();
            }
        });
        
        // Initialize Bluetooth
        BluetoothManager btManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = btManager.getAdapter();
        
        if (bluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth not supported", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        
        // Check permissions and start scan
        if (checkPermissions()) {
            startScan();
        }
    }
    
    @Override
    protected void onStop() {
        super.onStop();
        stopScan();
    }
    
    // =========================================================================
    // Permissions
    // =========================================================================
    
    private boolean checkPermissions() {
        List<String> permissions = new ArrayList<>();
        
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            // Android 12+
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) 
                    != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.BLUETOOTH_SCAN);
            }
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) 
                    != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.BLUETOOTH_CONNECT);
            }
        } else {
            // Android 11 and below
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
                startScan();
            } else {
                Toast.makeText(this, "Permissions required for BLE scanning", Toast.LENGTH_LONG).show();
            }
        }
    }
    
    // =========================================================================
    // BLE Scanning
    // =========================================================================

    private void startScan() {
        if (isScanning) return;

        if (!bluetoothAdapter.isEnabled()) {
            Toast.makeText(this, "Please enable Bluetooth", Toast.LENGTH_SHORT).show();
            return;
        }

        devices.clear();
        adapter.updateDevices(devices);

        bleScanner = bluetoothAdapter.getBluetoothLeScanner();
        if (bleScanner == null) {
            Toast.makeText(this, "BLE Scanner not available", Toast.LENGTH_SHORT).show();
            return;
        }

        // 1. CONFIGURA I FILTRI
        // Filtra solo i dispositivi che pubblicizzano il servizio VR2
        List<ScanFilter> filters = new ArrayList<>();
        filters.add(new ScanFilter.Builder()
                .setServiceUuid(new ParcelUuid(VR2BleConstants.SERVICE_UUID))
                .build());

        // 2. CONFIGURA I SETTINGS (Cruciale per il tuo ESP32)
        ScanSettings.Builder settingsBuilder = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY);

        // IMPORTANTE: Abilita il supporto per Bluetooth 5 Advertising Extension
        // Il tuo screenshot mostra che il dispositivo usa "Advertising Extension".
        // Senza setLegacy(false), i telefoni Android filtrano via questi pacchetti.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            settingsBuilder.setLegacy(false);
        }

        ScanSettings settings = settingsBuilder.build();

        try {
            // Avvia la scansione con Filtri e Settings aggiornati
            bleScanner.startScan(filters, settings, scanCallback);

            isScanning = true;
            updateUI();
            statusText.setText("Scanning for VR2 devices...");

            // Auto-stop after duration
            handler.postDelayed(this::stopScan, SCAN_DURATION_MS);

            Log.i(TAG, "BLE scan started (Legacy=false)");
        } catch (SecurityException e) {
            Log.e(TAG, "Scan permission denied", e);
            Toast.makeText(this, "Scan permission denied", Toast.LENGTH_SHORT).show();
        }
    }
    
    private void stopScan() {
        if (!isScanning) return;
        
        handler.removeCallbacksAndMessages(null);
        
        if (bleScanner != null) {
            try {
                bleScanner.stopScan(scanCallback);
            } catch (SecurityException e) {
                Log.e(TAG, "Stop scan error", e);
            }
        }
        
        isScanning = false;
        updateUI();
        
        if (devices.isEmpty()) {
            statusText.setText("No VR2 devices found. Tap SCAN to retry.");
        } else {
            statusText.setText(devices.size() + " device(s) found. Tap to connect.");
        }
        
        Log.i(TAG, "BLE scan stopped");
    }
    
    private final ScanCallback scanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            addDevice(result);
        }
        
        @Override
        public void onBatchScanResults(List<ScanResult> results) {
            for (ScanResult result : results) {
                addDevice(result);
            }
        }
        
        @Override
        public void onScanFailed(int errorCode) {
            Log.e(TAG, "Scan failed: " + errorCode);
            stopScan();
            statusText.setText("Scan failed. Error: " + errorCode);
        }
    };

    private void addDevice(ScanResult result) {
        // Check if device already in list
        for (int i = 0; i < devices.size(); i++) {
            if (devices.get(i).getDevice().getAddress().equals(result.getDevice().getAddress())) {
                // Update RSSI
                devices.set(i, result);
                adapter.notifyItemChanged(i);
                return;
            }
        }

        // Add new device
        devices.add(result);
        adapter.updateDevices(devices);
        statusText.setText("Found: " + devices.size() + " device(s)");

        Log.i(TAG, "Found device: " + getDeviceName(result.getDevice()) +
                " [" + result.getDevice().getAddress() + "] RSSI: " + result.getRssi());
    }
    
    private void updateUI() {
        btnScan.setText(isScanning ? "STOP" : "SCAN");
        progressBar.setVisibility(isScanning ? View.VISIBLE : View.GONE);
    }
    
    // =========================================================================
    // Device Selection
    // =========================================================================
    
    private void onDeviceSelected(ScanResult result) {
        stopScan();
        
        Intent resultIntent = new Intent();
        resultIntent.putExtra(EXTRA_DEVICE, result.getDevice());
        setResult(Activity.RESULT_OK, resultIntent);
        finish();
    }
    
    private String getDeviceName(BluetoothDevice device) {
        try {
            String name = device.getName();
            return name != null ? name : "Unknown";
        } catch (SecurityException e) {
            return "Unknown";
        }
    }
    
    // =========================================================================
    // RecyclerView Adapter
    // =========================================================================
// Interface FUORI dalla classe DeviceAdapter
    interface OnDeviceClickListener {
        void onDeviceClick(ScanResult result);
    }

    private class DeviceAdapter extends RecyclerView.Adapter<DeviceAdapter.ViewHolder> {

        private List<ScanResult> items = new ArrayList<>();
        private final OnDeviceClickListener listener;
        // QUESTO COSTRUTTORE DEVE ESSERCI!
        DeviceAdapter(OnDeviceClickListener listener) {
            this.listener = listener;
        }
        void updateDevices(List<ScanResult> devices) {
            this.items = new ArrayList<>(devices);
            notifyDataSetChanged();
        }
        
        @NonNull
        @Override
        public ViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            View view = LayoutInflater.from(parent.getContext())
                .inflate(R.layout.item_device, parent, false);
            return new ViewHolder(view);
        }
        
        @Override
        public void onBindViewHolder(@NonNull ViewHolder holder, int position) {
            ScanResult result = items.get(position);
            holder.bind(result);
        }
        
        @Override
        public int getItemCount() {
            return items.size();
        }
        
        class ViewHolder extends RecyclerView.ViewHolder {
            TextView deviceName;
            TextView deviceAddress;
            TextView deviceRssi;
            
            ViewHolder(View itemView) {
                super(itemView);
                deviceName = itemView.findViewById(R.id.deviceName);
                deviceAddress = itemView.findViewById(R.id.deviceAddress);
                deviceRssi = itemView.findViewById(R.id.deviceRssi);
                
                itemView.setOnClickListener(v -> {
                    int pos = getAdapterPosition();
                    if (pos != RecyclerView.NO_POSITION) {
                        listener.onDeviceClick(items.get(pos));
                    }
                });
            }
            
            void bind(ScanResult result) {
                deviceName.setText(getDeviceName(result.getDevice()));
                deviceAddress.setText(result.getDevice().getAddress());
                deviceRssi.setText(result.getRssi() + " dBm");
            }
        }
    }
}
