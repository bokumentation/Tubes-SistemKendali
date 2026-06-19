# Tugas Besar Sistem Kendali

## GOALS:
- Building simple PID implementation for Car Robot Obstacle Avoidance
- MCU: ESP32 Devkit
- Sensor: 4x RCWL-1601 (Compatible with HC-SR04)
- Output: 2x DRV8833 for 4 Wheels TT Gear Motor

---

## Verified Pin Assignment (ESP32 Devkit)

### DRV8833 #1 — FRONT (FR + FL)
| GPIO | DRV8833 | Channel | Motor |
|------|---------|---------|-------|
| 14 | BIN1 | B (right) | FR speed (MCPWM0B) |
| 27 | BIN2 | B (right) | FR direction |
| 26 | AIN1 | A (left) | FL speed (MCPWM0A) |
| 32 | AIN2 | A (left) | FL direction |

### DRV8833 #2 — BACK (BR + BL)
| GPIO | DRV8833 | Channel | Motor |
|------|---------|---------|-------|
| 19 | BIN1 | B (right) | BR speed (MCPWM0A UNIT_1) |
| 18 | BIN2 | B (right) | BR direction |
| 15 | AIN1 | A (left) | BL speed (MCPWM1A) |
| 33 | AIN2 | A (left) | BL direction |

### Ultrasonic Sensors
| Sensor | TRIG | ECHO |
|--------|------|------|
| FRONT | GPIO4 | GPIO5 |
| LEFT | GPIO2 | GPIO16 |
| RIGHT | GPIO17 | GPIO21 |
| BACK | GPIO22 | GPIO23 |

---

## System Block Diagram

### INPUT:
- SENSOR_FRONT  (GPIO4 TRIG, GPIO5 ECHO)
- SENSOR_RIGHT  (GPIO17 TRIG, GPIO21 ECHO)
- SENSOR_LEFT   (GPIO2 TRIG, GPIO16 ECHO)
- SENSOR_BACK   (GPIO22 TRIG, GPIO23 ECHO)

### PROCESS:
- ESP32 Devkit

### OUTPUT:
- DRV8833 #1  (GPIO26,32,14,27) → FRONT wheels (FR + FL)
- DRV8833 #2  (GPIO15,33,19,18) → BACK wheels (BR + BL)
- LOCAL WEB DASHBOARD via WiFi

---

## MCPWM Mapping

| UNIT | TIMER | GEN | GPIO | Motor |
|------|-------|-----|------|-------|
| 0 | 0 | B | 14 | FR |
| 0 | 0 | A | 26 | FL |
| 0 | 1 | UNIT_1 A | 19 | BR |
| 1 | 0 | A | 15 | BL |

Direction (IN2) pins: LOW=forward, HIGH=backward
