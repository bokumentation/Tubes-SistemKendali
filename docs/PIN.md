# Pin Configuration Guide

## ESP32 Devkit — Final Verified Pinout

**MCU:** ESP32 Devkit
**Target:** `idf.py set-target esp32`
**DRV8833 STBY:** Connect directly to **3.3V** (no GPIO needed)

### Pin Selection Notes

The following pins were tested and **did not work** for MCPWM:
- **GPIO12, GPIO13** — strapping pins, MCPWM output unreliable
- **GPIO25** — MCPWM didn't produce output on this board

### Channel vs Motor Label Note

DRV8833 channel A (AIN1/AIN2) controls the **left** physical motor.
DRV8833 channel B (BIN1/BIN2) controls the **right** physical motor.
The code aliases swap them so `MOTOR_FR` = right wheel, `MOTOR_FL` = left wheel.

---

## DRV8833 #1 — FRONT WHEELS (FR + FL)

| ESP32 GPIO | DRV8833 #1 | Channel | Motor |
|------------|------------|---------|-------|
| **GPIO14** | BIN1 | B (right) | **FR** (Front Right) |
| **GPIO27** | BIN2 | B (right) | **FR** |
| **GPIO26** | AIN1 | A (left) | **FL** (Front Left) |
| **GPIO32** | AIN2 | A (left) | **FL** |

**Terminal connections:**
- AO1, AO2 → Motor **FL** (left side)
- BO1, BO2 → Motor **FR** (right side)

---

## DRV8833 #2 — BACK WHEELS (BR + BL)

| ESP32 GPIO | DRV8833 #2 | Channel | Motor |
|------------|------------|---------|-------|
| **GPIO19** | BIN1 | B (right) | **BR** (Back Right) |
| **GPIO18** | BIN2 | B (right) | **BR** |
| **GPIO15** | AIN1 | A (left) | **BL** (Back Left) |
| **GPIO33** | AIN2 | A (left) | **BL** |

**Terminal connections:**
- AO1, AO2 → Motor **BL** (left side)
- BO1, BO2 → Motor **BR** (right side)

---

## Direction Logic

| Direction | IN1 (PWM) | IN2 (GPIO) |
|-----------|-----------|------------|
| **Forward** | Duty% (speed) | LOW (0V) |
| **Backward** | 0% | HIGH (3.3V) |
| **Stop/Coast** | 0% | LOW (0V) |

---

## MCPWM Mapping

| MCPWM Unit | Timer | Generator | GPIO | Motor |
|------------|-------|-----------|------|-------|
| UNIT_0 | TIMER_0 | MCPWM0B | 14 | FR |
| UNIT_0 | TIMER_0 | MCPWM0A | 26 | FL |
| UNIT_0 | TIMER_1 | MCPWM0A UNIT_1 | 19 | BR |
| UNIT_1 | TIMER_0 | MCPWM1A | 15 | BL |

Note: IN2 pins (27, 32, 18, 33) use `gpio_set_level()` — not MCPWM.

---

## Ultrasonic Sensors (RCWL-1601 / HC-SR04)

| Sensor | Position | TRIG | ECHO |
|--------|----------|------|------|
| Front | Depan | GPIO4 | GPIO5 |
| Left | Kiri | GPIO2 | GPIO16 |
| Right | Kanan | GPIO17 | GPIO21 |
| Back | Belakang | GPIO22 | GPIO23 |

**Wiring:**
- **VCC** → **5V** (not 3.3V — sensor needs 5V)
- **TRIG** → direct to ESP32 GPIO
- **ECHO** → voltage divider (10kΩ + 20kΩ) → ESP32 GPIO (sensor output is 5V)
- **GND** → common ground with ESP32

---

## Power Distribution

```
Battery (2S LiPo 7.4-8.4V)
│
├── JST+ ──┬── DRV8833 #1 VM
│          ├── DRV8833 #2 VM
│          └── MP1584 Buck IN+
│
├── JST- ──┬── DRV8833 #1 GND
│          ├── DRV8833 #2 GND
│          └── MP1584 Buck IN-
│
MP1584 OUT+ ──── ESP32 3V3 (VIN)
MP1584 OUT- ──── ESP32 GND
```

**Important:** All grounds must be connected (DRV8833, MP1584, ESP32, sensors).

---

## GPIO Usage Summary

| GPIO | Connected to | Function |
|------|-------------|----------|
| 2 | LEFT TRIG | Sensor output |
| 4 | FRONT TRIG | Sensor output |
| 5 | FRONT ECHO | Sensor input (via divider) |
| 14 | FR BIN1 | MCPWM0B |
| 15 | BL AIN1 | MCPWM1A |
| 16 | LEFT ECHO | Sensor input (via divider) |
| 17 | RIGHT TRIG | Sensor output |
| 18 | BR BIN2 | GPIO out (direction) |
| 19 | BR BIN1 | MCPWM0A UNIT_1 |
| 21 | RIGHT ECHO | Sensor input (via divider) |
| 22 | BACK TRIG | Sensor output |
| 23 | BACK ECHO | Sensor input (via divider) |
| 26 | FL AIN1 | MCPWM0A |
| 27 | FR BIN2 | GPIO out (direction) |
| 32 | FL AIN2 | GPIO out (direction) |
| 33 | BL AIN2 | GPIO out (direction) |

**Unused available GPIOs:** 0, 1, 3, 9, 10, 11, 12, 13, 25, 34, 35, 36, 37, 38, 39

---

## Build & Flash

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Exit monitor: `Ctrl+]`
