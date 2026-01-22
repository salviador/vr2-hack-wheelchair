# VR2 Wheelchair Hack

Sistema di controllo alternativo per carrozzine elettriche VR2 PG Drives. Questo progetto permette di controllare una carrozzina VR2 tramite app Android o head tracking con Raspberry Pi.

## Architettura del Sistema

```
┌──────────────────────────┐    ┌──────────────────────────┐
│      Android App         │    │     Raspberry Pi         │
│ (VR2WheelchairController)│    │ (Virtual_Joy_raspberry)  │
│                          │    │                          │
│  - Joystick virtuale     │    │  - Head tracking         │
│  - Controllo velocità    │    │  - Controllo con testa   │
│  - Telemetria            │    │  - Camera input          │
└────────────┬─────────────┘    └─────────────┬────────────┘
             │ BLE                            │ BLE
             └──────────────┬─────────────────┘
                            │
                ┌───────────▼───────────┐
                │   ESP32 Bridge        │
                │  (joy_vr2_emulator)   │
                │                       │
                │  - BLE GATT Server    │
                │  - Protocollo VR2     │
                │  - Controllo relè     │
                └───────────┬───────────┘
                            │ UART RS-485 (38400 baud)
                            │
                ┌───────────▼───────────┐
                │   VR2 PG Drives ECU   │
                │   (Carrozzina)        │
                └───────────────────────┘
```

---

## Firmware

### 1. joy_vr2_emulator (ESP32)

**Descrizione:** Firmware ESP32 che funge da bridge BLE tra i controller remoti e la centralina VR2 della carrozzina.

**Hardware:**
- ESP32 (C3, C6, S2, S3 supportati)
- Interfaccia RS-485/UART per comunicazione con ECU VR2
- Relè per controllo alimentazione

**Funzionalità principali:**
- Server BLE GATT per ricevere comandi
- Emulazione protocollo VR2 joystick
- Invio telemetria (stato, velocità, batteria) via BLE
- Gestione stati sistema (IDLE → STARTING → RUNNING → STOPPING)
- LED di stato (verde: sistema attivo, blu: BLE connesso)
- Supporto pulsanti fisici (SEC, SWL, SWR)

**Connessioni GPIO:**
| Funzione | GPIO |
|----------|------|
| VR2 TX (UART1) | 17 |
| VR2 RX (UART1) | 18 |
| Controllo Relè | 14 |
| RS-485 TX Enable | 11 |
| Pulsante SEC | 0 |
| Pulsante SWL | 45 |
| Pulsante SWR | 35 |

**Build:**
```bash
cd joy_vr2_emulator
idf.py build
idf.py flash
```

---

### 2. VR2WheelchairController (Android)

**Descrizione:** App Android per controllare la carrozzina tramite joystick virtuale touch.

**Requisiti:**
- Android 8.0+ (API 26)
- Bluetooth Low Energy

**Funzionalità principali:**
- Joystick virtuale con modalità drag e touch
- Pulsanti START/STOP per attivazione ECU
- Controllo velocità (livelli 1-5)
- Display telemetria in tempo reale:
  - Stato ECU (IDLE/INIT/RUN/ERROR/SHUTDOWN)
  - Livello velocità
  - Stato batteria con indicatori errore
- Scansione automatica dispositivo BLE "WHEELS Mic"
- Indicatori connessione (verde = connesso, rosso = disconnesso)

**Librerie utilizzate:**
- Nordic BLE Library
- AndroidX / Material Design
- EasyPermissions

**Build:**
```bash
cd VR2WheelchairController
./gradlew assembleDebug
```

---

### 3. Virtual_Joy_raspberry (Raspberry Pi)

**Descrizione:** Controller Python per Raspberry Pi che usa il tracking della testa per controllare la carrozzina. Muovendo la testa si controlla il joystick virtuale.

**Hardware:**
- Raspberry Pi 4/5 (consigliato)
- Raspberry Pi Camera 3 o webcam USB
- Adattatore Bluetooth

**Funzionalità principali:**
- Rilevamento volto con MediaPipe
- Stima pose della testa (yaw e pitch)
- Mappatura movimenti testa → comandi joystick:
  - Testa sinistra/destra → Sterzata (asse X)
  - Testa su/giù → Avanti/indietro (asse Y)
- Calibrazione posizione neutra
- Dead zone configurabile (default: 3°)
- Smoothing per controllo stabile
- Timeout sicurezza se volto non rilevato (0.5s)
- Display video con overlay joystick

**Controlli tastiera:**
| Tasto | Azione |
|-------|--------|
| `c` | Calibra posizione neutra |
| `r` | Reset calibrazione |
| `b` | Riconnetti BLE |
| `s` | Invia comando START |
| `x` | Invia comando SHUTDOWN |
| `+`/`-` | Regola velocità |
| `p` | Pausa/riprendi output joystick |
| `q` | Esci |

**Installazione:**
```bash
cd Virtual_Joy_raspberry
pip install -r requirements.txt
```

**Utilizzo:**
```bash
# Modalità normale
python main.py

# Test senza BLE
python main.py --no-ble

# Usa webcam USB invece di Pi Camera
python main.py --webcam

# Modalità headless (senza display)
python main.py --no-display
```

**Dipendenze:**
- OpenCV >= 4.5.0
- MediaPipe >= 0.10.0
- Bleak >= 0.21.0
- NumPy >= 1.20.0

---

## Protocollo BLE

I tre componenti comunicano usando un protocollo BLE unificato.

**UUIDs:**
| Caratteristica | UUID |
|----------------|------|
| Service | `da343711-d712-3230-8842-f9dea033233d` |
| TX (Comandi) | `da343711-d712-3230-8842-f7dea033233d` |
| Telemetria | `da343711-d712-3230-8842-f6dea033233d` |

**Comandi (verso carrozzina):**
| Comando | Codice | Formato |
|---------|--------|---------|
| SET_JOY | 0x01 | `[0x01][X][Y][Buttons][Checksum]` |
| START | 0x02 | `[0x02][Checksum]` |
| SHUTDOWN | 0x03 | `[0x03][Checksum]` |
| SPEED+ | 0x04 | `[0x04][Checksum]` |
| SPEED- | 0x05 | `[0x05][Checksum]` |

**Telemetria (dalla carrozzina):**
- Formato: `[State][Speed][Battery][Switches][Checksum]`
- Stati: IDLE, INIT, RUN, ERROR, SHUTDOWN
- Velocità: 1-5
- Batteria: indicatori LED (0x10-0xA0) o codici errore

**Checksum:** `(somma tutti i byte) XOR 0xFF`

---

## Licenza

Questo progetto è rilasciato per scopi educativi e di ricerca.

**ATTENZIONE:** Modificare sistemi di controllo di carrozzine elettriche può essere pericoloso. Usare solo in ambienti controllati e a proprio rischio.
