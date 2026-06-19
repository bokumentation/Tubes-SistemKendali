# Presentasi PID — Robot Car Obstacle Avoidance

## Tugas Besar Sistem Kendali

---

## 1. Pendahuluan

Robot mobil ini menggunakan **algoritma PID (Proportional-Integral-Derivative)** untuk menjaga jarak aman dari tembok/dinding secara otomatis. Robot dilengkapi **4 sensor ultrasonik RCWL-1601** yang ditempatkan di:

| Posisi | GPIO (TRIG, ECHO) | Sudut |
|--------|--------------------|-------|
| **Depan tengah** | GPIO4, GPIO5 | 0° (lurus) |
| **Depan kiri** | GPIO2, GPIO16 | ~30° kiri |
| **Depan kanan** | GPIO17, GPIO21 | ~30° kanan |
| **Belakang** | GPIO22, GPIO23 | 0° (lurus) |

**Tujuan PID:** Robot otomatis menjaga jarak **40 cm** dari objek di depannya. Jika terlalu dekat, robot memperlambat atau mundur. Sensor belakang berfungsi sebagai **safety** saat robot mundur.

---

## 2. Arsitektur File

| File | Peran |
|------|-------|
| `main/pid_controller.c` | **Implementasi inti algoritma PID** — fungsi `pid_ctrl_compute()` |
| `main/include/pid_controller.h` | **Deklarasi struktur `pid_ctrl_t`** dan fungsi PID |
| `main/obstacle_avoidance.c` | **Integrasi PID dengan sensor** — `avoid_task()` |
| `main/include/shared_data.h` | **Variabel global** — `speed_pid`, `speed_setpoint` |
| `main/web_dashboard.c` | **Endpoint API PID** — `POST /api/pid` dan status PID |
| `main/html/dashboard.html` | **UI tuning PID** — input Kp, Ki, Kd, Setpoint di browser |

---

## 3. Struktur Data PID

File: `main/include/pid_controller.h`

```c
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Akumulasi error (∫e dt)
    float prev_error;   // Error sebelumnya (untuk derivatif)
    float dt;           // Time step (0.03 detik)
    float out_min;      // Output minimum (0)
    float out_max;      // Output maximum (100)
} pid_ctrl_t;
```

Inisialisasi dilakukan di `main/obstacle_avoidance.c` baris 86:

```c
pid_ctrl_init(&speed_pid, 1.0f, 0.1f, 0.05f, PID_DT, 0, MAX_SPEED);
//                         Kp    Ki    Kd     dt     out_min  out_max
```

---

## 4. Algoritma PID

File: `main/pid_controller.c` — fungsi `pid_ctrl_compute()`

### Rumus Dasar

$$u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) d\tau + K_d \cdot \frac{de(t)}{dt}$$

### Implementasi Kode

**Error:**
```c
float error = setpoint - measurement;   // baris 18
```

**Integral (dengan anti-windup clamping):**
```c
pid->integral += error * pid->dt;       // baris 20
if (pid->integral > pid->out_max)       // baris 21
    pid->integral = pid->out_max;       // Clamp atas
else if (pid->integral < pid->out_min)  // baris 23
    pid->integral = pid->out_min;       // Clamp bawah
```

$$I = K_i \cdot \sum e(t) \cdot \Delta t$$

**Derivatif:**
```c
float derivative = (error - pid->prev_error) / pid->dt;  // baris 27
```

$$D = K_d \cdot \frac{e(t) - e(t-1)}{\Delta t}$$

**Output gabungan (dengan output clamping):**
```c
float output = (pid->kp * error)         // P
             + (pid->ki * pid->integral)  // I
             + (pid->kd * derivative);    // D
// baris 29-31

if (output > pid->out_max) output = pid->out_max;   // baris 33-34
else if (output < pid->out_min) output = pid->out_min; // baris 35-36
```

$$u(t) = \text{clamp}(K_p e + K_i \int e + K_d \frac{de}{dt},\ 0,\ 100)$$

---

## 5. Contoh Perhitungan

**Diketahui:**
- Setpoint = 40 cm
- Sensor depan mendeteksi = **20 cm**
- Kp = 1.0, Ki = 0.1, Kd = 0.05
- Δt = 0.03 detik
- Error sebelumnya = 0, Integral = 0

### Iterasi 1 (t = 0):

**Error:**
$$e = setpoint - measurement = 40 - 20 = \mathbf{20\ cm}$$

**Proportional:**
$$P = K_p \cdot e = 1.0 \times 20 = \mathbf{20}$$

**Integral:**
$$\int e = 0 + 20 \times 0.03 = 0.6$$
$$I = 0.1 \times 0.6 = \mathbf{0.06}$$

**Derivatif:**
$$\frac{de}{dt} = \frac{20 - 0}{0.03} = 666.67$$
$$D = 0.05 \times 666.67 = \mathbf{33.33}$$

**Output:**
$$u = 20 + 0.06 + 33.33 = 53.39 \approx \mathbf{53\%}$$

> **Interpretasi:** Sensor 20cm (lebih dekat dari setpoint 40cm) → Output 53%. Robot masih bergerak maju dengan kecepatan 53% — tidak full speed karena sudah dekat dinding.

---

### Iterasi 2 (t = 0.03s, sensor sekarang 15cm):

**Error:**
$$e = 40 - 15 = \mathbf{25\ cm}$$

**Proportional:**
$$P = 1.0 \times 25 = \mathbf{25}$$

**Integral:**
$$\int e = 0.6 + 25 \times 0.03 = 1.35$$
$$I = 0.1 \times 1.35 = \mathbf{0.135}$$

**Derivatif:**
$$\frac{de}{dt} = \frac{25 - 20}{0.03} = 166.67$$
$$D = 0.05 \times 166.67 = \mathbf{8.33}$$

**Output:**
$$u = 25 + 0.135 + 8.33 = 33.47 \approx \mathbf{33\%}$$

> Robot semakin mendekat ke dinding → output turun menjadi 33%.

---

### Iterasi 3 (t = 0.06s, sensor sekarang 8cm):

Karena **min_front = 8cm < 10cm** (threshold darurat, baris 106-111):

```c
if (min_front < 10.0f) {
    if (sensor_back_cm >= BACK_THRESHOLD || sensor_back_cm < 0) {
        speed_out = -50;  // Mundur pelan
    } else {
        speed_out = 0;    // STOP! Belakang mentok
    }
}
```

Jika sensor belakang aman (> 15cm):
$$u = \mathbf{-50\%} \text{ (mundur)}$$

Jika sensor belakang mentok (< 15cm):
$$u = \mathbf{0\%} \text{ (STOP total)}$$

---

### Iterasi 4 (robot mundur, sensor 20cm):

Setelah mundur, sensor depan = 20cm, kembali ke perhitungan PID normal:
$$e = 40 - 20 = 20$$
$$u \approx 53\%$$

> Robot kembali maju di kecepatan yang dikontrol PID.

---

## 6. Prioritas 3 Sensor Depan

File: `main/obstacle_avoidance.c` — fungsi `get_min_front()` (baris 73-80)

```c
static float get_min_front(void)
{
    float min_val = 999.0f;
    if (sensor_front_cm >= 0 && sensor_front_cm < min_val)
        min_val = sensor_front_cm;
    if (sensor_front_left_cm >= 0 && sensor_front_left_cm < min_val)
        min_val = sensor_front_left_cm;
    if (sensor_front_right_cm >= 0 && sensor_front_right_cm < min_val)
        min_val = sensor_front_right_cm;
    return (min_val > 998.0f) ? -1 : min_val;
}
```

### Logika:

1. **Baca ketiga sensor depan** (tengah, kiri, kanan)
2. **Ambil nilai terkecil** (jarak terdekat ke obstacle)
3. **Gunakan nilai terkecil sebagai input PID**

| Kondisi | Hasil |
|---------|-------|
| Depan=50cm, Kiri=20cm, Kanan=60cm | Pakai **20cm** (kiri paling dekat) |
| Depan=30cm, Kiri=80cm, Kanan=25cm | Pakai **25cm** (kanan paling dekat) |
| Semua > 40cm | Robot **full speed** |

**Mengapa pakai minimum?** Karena safety — robot harus merespon obstacle **terdekat**, bukan rata-rata. Jika sensor kiri mendeteksi tembok 15cm tapi yang lain 60cm, robot tetap harus menghindar.

---

## 7. Peran Sensor Belakang

File: `main/obstacle_avoidance.c` — baris 106-108 dan 137-146

### 7.1 Safety saat AUTO mode mundur

```c
if (min_front < 10.0f) {
    if (sensor_back_cm >= BACK_THRESHOLD || sensor_back_cm < 0) {
        speed_out = -50;    // Belakang aman → mundur
    } else {
        speed_out = 0;      // Belakang mentok → STOP
    }
}
```

**Threshold:** `BACK_THRESHOLD = 15.0f`

| Back Sensor | Aksi |
|-------------|------|
| > 15cm atau timeout | Mundur -50% |
| < 15cm | STOP (tidak bisa mundur) |

### 7.2 Safety saat manual reverse

```c
if (manual_override) {
    if (sensor_back_cm > 0 && sensor_back_cm < BACK_THRESHOLD &&
        (motor_fr_speed < 0 || ...)) {
        motor_all_stop();   // Force stop!
    }
}
```

Saat pengguna menekan **S/X (mundur)** dan sensor belakang < 15cm → robot **force stop** meskipun tombol masih ditekan.

---

## 8. EMA Filter (Signal Smoothing)

File: `main/obstacle_avoidance.c` — `read_all_sensors()` (baris 52-71)

```c
sensor_front_cm = sensor_front_cm * (1.0f - EMA_ALPHA) + val * EMA_ALPHA;
//                                      0.7                   0.3
```

$$x_{filtered} = x_{prev} \times 0.7 + x_{raw} \times 0.3$$

**Exponential Moving Average** dengan α = 0.3 menghaluskan noise sensor. Tanpa filter, pembacaan sensor bisa jitter ±3cm yang membuat output PID tidak stabil.

---

## 9. Feed-Forward Braking

File: `main/obstacle_avoidance.c` — baris 116-119

```c
float velocity = (prev_min_front - min_front) / PID_DT;
if (min_front < BRAKE_THRESHOLD && velocity > 10.0f) {
    speed_out -= velocity * 0.3f;
}
```

Jika robot mendekati tembok dengan **kecepatan > 10 cm/s** saat jarak < 60cm, output dikurangi:

$$u_{brake} = u_{PID} - v \times 0.3$$

> Ini adalah **preemptive braking** — robot mengantisipasi tabrakan bahkan sebelum PID sempat merespon penuh.

---

## 10. PID Tuning via Web Dashboard

File: `main/web_dashboard.c` — endpoint `POST /api/pid` (baris 155-195)

```json
POST /api/pid
{
  "setpoint": 40,
  "kp": 1.0,
  "ki": 0.1,
  "kd": 0.05
}
```

Dashboard memungkinkan **live tuning** — ubah Kp, Ki, Kd, Setpoint dari browser tanpa recompile.

File: `main/html/dashboard.html` — PID form dengan input `id="spkp"`, `id="spki"`, `id="spkd"`, `id="spsp"` dan tombol **SET**.

---

## 11. State Machine PID

```
┌─────────────┐     AUTO mode      ┌──────────────────┐
│  MANUAL     │ ◄─── STOP btn ──── │  PID ACTIVE       │
│  (WASD)     │                    │  Read sensors      │
│             │                    │  Compute min_front │
│  Back safety│                    │  PID compute       │
│  still active│                   │  Set motor speed   │
└─────────────┘                    └────────┬───────────┘
        ▲                                   │
        │         Space press               │
        └───────────────────────────────────┘
```

---

## 12. Rekomendasi Tuning

| Parameter | Efek | Rekomendasi |
|-----------|------|-------------|
| **Kp ↑** | Respons lebih cepat, tapi bisa overshoot | Mulai dari 1.0, naikkan bertahap |
| **Ki ↑** | Menghilangkan steady-state error | Mulai dari 0.1, naikkan jika robot "malas" |
| **Kd ↑** | Meredam overshoot, antisipasi perubahan | Mulai dari 0.05 |
| **Setpoint** | Jarak yang dijaga | 30-50cm cocok untuk obstacle avoidance |
