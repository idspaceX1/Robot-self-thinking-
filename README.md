# 🏔️ Mountain Exploration Robot (BeagleBone + Arduino)

A **fully offline, hardened exploration robot** designed for **mountain / outdoor environments**.
This system prioritizes **safety, reliability, and autonomy** over raw speed.

---

## 🚀 Project Overview

This robot uses a **dual-controller architecture**:

* **BeagleBone Black (BBB)** – High-level brain

  * Offline voice recognition (Vosk)
  * Decision making & autonomy
  * Secure packet communication
  * Logging & diagnostics

* **Arduino (Motor & Safety Controller)** – Low-level reflexes

  * Motor control
  * Obstacle detection (ultrasonic)
  * Hardware watchdog
  * Emergency stop logic

If **either side fails**, the robot **stops safely**.

---

## 🧠 System Architecture

```
Voice → BeagleBone Brain → Packet Protocol → Arduino Safety Controller → Motors
                                   ↑                 ↓
                               Heartbeat        Sensors & ACK
```

Key design principle:

> **Never trust a single component. Always fail safe.**

---

## 🔐 Communication Protocol (Critical)

All communication uses a **binary framed protocol with CRC8**:

```
[0xAA][TYPE][LEN][PAYLOAD...][CRC8]
```

### Packet Types

| Type   | Name      | Description                 |
| ------ | --------- | --------------------------- |
| `0x01` | COMMAND   | Movement / control commands |
| `0x02` | ACK       | Command acknowledgment      |
| `0x03` | ERROR     | Arduino error report        |
| `0x10` | SENSOR    | Distance sensor data        |
| `0x20` | HEARTBEAT | Link keep-alive             |
| `0x30` | EMERGENCY | Immediate stop              |

### Safety Features

* CRC8 integrity check
* Command sequence ID
* ACK timeout & retry
* Heartbeat dead-man switch

---

## ⚡ Safety & Fail-Safe Design

### ✅ Hardware Watchdog (Arduino)

* 2-second watchdog
* Any software freeze → **automatic reset**

### ✅ Heartbeat Monitoring

* BBB sends heartbeat every 500 ms
* Arduino stops motors if heartbeat lost

### ✅ Obstacle Protection

* Ultrasonic sensor (median filtered)
* Stops motion under **10 cm**

### ✅ Emergency Stop

Triggered by:

* Obstacle
* Heartbeat loss
* CRC flood
* Voice command
* Manual override

Emergency always overrides everything.

---

## 🔌 Wiring Overview

### 🧩 ASCII Wiring Diagram (Simplified)

```
┌──────────────────────────┐
│     BeagleBone Black     │
│                          │
│  P9_24  UART1_TX  ─────┐ │
│                         │ │
│  GND  ───────────────┐ │ │
└──────────────────────┘ │ │
                          │ │   (3.3V → 5V)
                    ┌─────▼─▼──────────┐
                    │ Logic Level      │
                    │ Shifter          │
                    └─────▲─▲──────────┘
                          │ │
┌──────────────────────┐ │ │
│        Arduino        │ │ │
│                       │ │ │
│  RX0  ◀──────────────┘ │ │
│  GND  ◀────────────────┘ │
│                           │
│  D2 ─────▶ Motor Driver IN1
│  D3 ─────▶ Motor Driver IN2
│  D4 ─────▶ Motor Driver IN3
│  D5 ─────▶ Motor Driver IN4
│                           │
│  D12 ───▶ Ultrasonic TRIG │
│  D11 ◀─── Ultrasonic ECHO │
└───────────────────────────┘
            │
            ▼
     ┌─────────────┐
     │ Motor Driver│
     │   (H-Bridge)│
     └─────▲───▲───┘
           │   │
        Motors Motors
```

⚠️ **All grounds must be common** (BBB, Arduino, motor driver).

---

### BeagleBone → Arduino (UART)

| BBB Pin          | Arduino Pin | Purpose       |
| ---------------- | ----------- | ------------- |
| P9_24 (UART1_TX) | RX0         | Command data  |
| GND              | GND         | Common ground |

⚠️ Use **logic level shifter** (3.3V → 5V).

### Arduino → Motor Driver

| Arduino | Driver | Function |
| ------- | ------ | -------- |
| D2      | IN1    | Motor A  |
| D3      | IN2    | Motor A  |
| D4      | IN3    | Motor B  |
| D5      | IN4    | Motor B  |

### Ultrasonic Sensor

| Sensor | Arduino |
| ------ | ------- |
| TRIG   | D12     |
| ECHO   | D11     |

---

## 🎙️ Voice Control (Offline)

* Powered by **Vosk**
* No internet required
* Works in remote locations

### Supported Commands

| Command      | Action                |
| ------------ | --------------------- |
| Forward / Go | Move forward          |
| Back         | Reverse               |
| Left / Right | Turn                  |
| Stop         | Immediate stop        |
| Emergency    | Full emergency halt   |
| Status       | System health report  |
| Reset        | Clear emergency state |

---

## 📊 Logging & Diagnostics

Logs are written to:

```
robot_brain.log
```

Includes:

* Packet counts
* CRC errors
* Distance readings
* Emergency triggers
* Uptime

Designed for **post-mission analysis**.

---

## 🛠️ Installation

### 1️⃣ BeagleBone

```bash
sudo apt update
sudo apt install python3 python3-pip espeak aplay
pip3 install pyserial vosk sounddevice
```

Copy Vosk model to:

```
./model/
```

Run:

```bash
python3 robot_brain.py
```

---

### 2️⃣ Arduino

* Flash the provided Arduino sketch
* Baud rate: **115200**
* Enable watchdog (already included)

---

## 🧪 Self-Test

Voice command:

```
run test
```

Checks:

* Speech
* Arduino link
* ACKs
* Sensors
* Heartbeat

---

## 🏔️ Designed For

✔ Mountain exploration
✔ Rough terrain
✔ No internet
✔ Cold / wind / dust
✔ Long runtimes

This is **not a toy robot**.

---

## 🔮 Future Extensions

* GPS return-to-home
* IMU (rollover & slope detection)
* SD-card blackbox
* Autonomous waypoint mode
* Solar-assisted power

---

## ⚠️ Disclaimer

This robot contains **moving motors** and can cause injury or damage.

Always:

* Test with wheels off ground
* Use current-limited power
* Keep emergency stop accessible

---

## 👤 Author

Built by **idspaceX1**

> "Robots should never assume. They should verify or stop." 🛑
