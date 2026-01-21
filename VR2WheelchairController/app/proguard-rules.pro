# VR2 Wheelchair Controller ProGuard Rules

# Nordic BLE Library
-keep class no.nordicsemi.android.ble.** { *; }

# Keep data classes
-keep class com.vr2.wheelchair.ble.VR2BleManager$TelemetryData { *; }
