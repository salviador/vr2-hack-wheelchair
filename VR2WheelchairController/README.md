# VR2 Wheelchair Controller - Android App

App Android per controllare la carrozzina elettrica tramite il bridge BLE ESP32 VR2 PG Drives.

## Caratteristiche

- **Joystick virtuale**: controllo movimento con feedback visivo
- **Pulsanti controllo**:
  - START: avvia la ECU
  - STOP: spegne la ECU (shutdown)
  - SPEED+/SPEED-: regola la velocità (1-5)
- **Telemetria in tempo reale**:
  - Stato ECU (IDLE/INIT/RUN/ERROR/SHUTDOWN)
  - Livello velocità
  - Stato batteria
- **Connessione BLE**: scan automatico per dispositivi "WHEELS Mic"

## Requisiti

- Android 8.0 (API 26) o superiore
- Bluetooth Low Energy
- Permessi: Bluetooth, Location (per BLE scan su Android < 12)

## Installazione

1. Apri il progetto in Android Studio
2. Sync Gradle
3. Build e installa sul dispositivo

## Protocollo BLE

### UUIDs
- **Service**: `da343711-d712-3230-8842-f9dea033233d`
- **Char TX** (write): `da343711-d712-3230-8842-f8dea033233d`
- **Char Telemetria** (notify): `da343711-d712-3230-8842-f6dea033233d`

### Comandi (write to joyTX)
| Comando | Byte | Descrizione |
|---------|------|-------------|
| SET_JOY | `[0x01][x][y][buttons]` | Imposta joystick (-127 a +127) |
| START | `[0x02]` | Avvia ECU |
| SHUTDOWN | `[0x03]` | Spegne ECU |
| SPEED+ | `[0x04]` | Aumenta velocità |
| SPEED- | `[0x05]` | Diminuisce velocità |

### Telemetria (notify from TELEMETRIA)
| Byte | Descrizione |
|------|-------------|
| 0 | State (0=IDLE, 1=INIT, 2=RUN, 3=ERROR, 4=SHUTDOWN) |
| 1 | Speed command |
| 2 | Battery code |
| 3 | Reserved |

## Dipendenze

```gradle
implementation 'no.nordicsemi.android:ble:2.7.4'
```

## Struttura Progetto

```
app/src/main/java/com/vr2/wheelchair/
├── MainActivity.java          # Activity principale con joystick
├── ScanActivity.java          # Scan BLE devices
├── ble/
│   ├── VR2BleConstants.java   # UUIDs e costanti protocollo
│   └── VR2BleManager.java     # Gestione connessione BLE (Nordic)
└── ui/
    └── JoystickView.java      # Custom joystick view
```

## Uso

1. **Avvia l'app** e premi **CONNECT**
2. **Seleziona** "WHEELS Mic" dalla lista dispositivi
3. Attendi la connessione (indicatore verde)
4. Premi **START** per attivare la ECU
5. Usa il **joystick** per muoverti
6. Usa **SPEED+/-** per regolare la velocità
7. Premi **STOP** per spegnere

## Note

- Il joystick invia valori a 20Hz quando toccato
- Al rilascio del joystick viene inviato automaticamente (0,0)
- La telemetria viene aggiornata dalla ECU ogni 100ms
