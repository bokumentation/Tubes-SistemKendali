# LAPORAN TUGAS BESAR SISTEM KENDALI

# OMNIDIRECTIONAL OBSTACLE AVOIDANCE PADA ROBOT 4 RODA MENGGUNAKAN KONTROL PID

Disusun untuk Memenuhi Tugas Mata Kuliah Sistem Kendali  
Dosen Pengampu:  
Marlindia Ike Sari S.T., M.T

Disusun oleh:

1. Muhammad Izzat Ramadhan (607022400050)  
2. Ibrahim Fauzi Ramadhan (607022400009)

PROGRAM STUDI D3 TEKNOLOGI KOMPUTER  
FAKULTAS ILMU TERAPAN  
UNIVERSITAS TELKOM  
BANDUNG  
2026

# 1. PENDAHULUAN

## 1.1. Latar Belakang

Perkembangan kendaraan otonom semakin pesat, salah satunya dipelopori oleh Tesla dengan fitur *autopilot*-nya. Mobil Tesla mampu mendeteksi pejalan kaki yang menyeberang maupun kendaraan lain di depannya, lalu secara otomatis mengurangi kecepatan atau berhenti untuk menghindari tabrakan. Konsep ini dikenal sebagai *obstacle avoidance*, yaitu kemampuan sistem untuk mengenali halangan di sekitar dan mengambil tindakan tanpa campur tangan manusia.

Terinspirasi dari konsep tersebut, proyek ini mencoba menerapkan *obstacle avoidance* dalam skala yang lebih kecil: robot mobil empat roda berbasis ESP32. Robot ini menggunakan sensor ultrasonik untuk membaca jarak ke objek di sekitarnya dan kontroler PID (*Proportional-Integral-Derivative*) untuk mengatur kecepatan motor secara otomatis agar jarak aman tetap terjaga.

Pada perancangan awal, robot direncanakan menggunakan roda *omniwheel*. Roda *omniwheel* memiliki kelebihan dapat bergerak ke segala arah tanpa perlu memutar badan robot terlebih dahulu — cukup dengan mengatur kombinasi putaran tiap roda, robot bisa bergeser secara lateral (kiri-kanan), diagonal, maupun rotasi di tempat. Ini sangat menguntungkan untuk manuver di ruang sempit dan menghindari halangan dengan lintasan yang lebih efisien. Sayangnya, implementasi roda *omniwheel* tidak berhasil karena keterbatasan waktu dan kompleksitas kode program yang dibutuhkan untuk koordinasi gerak multi-arah. Akhirnya dipilih roda biasa agar pengembangan lebih efisien dan fokus pada algoritma kontrol PID.

## 1.2. Rumusan Masalah

Berdasarkan latar belakang di atas, rumusan masalah pada proyek ini adalah:

1. Bagaimana cara mengukur jarak ke objek di depan robot menggunakan sensor ultrasonik RCWL-1601?
2. Bagaimana menerapkan kontroler PID agar robot dapat menjaga jarak aman terhadap halangan secara otomatis?
3. Bagaimana mengintegrasikan sistem ke dalam *dashboard* berbasis web agar pengguna dapat memantau dan mengendalikan robot dari jarak jauh?

## 1.3. Tujuan

Tujuan dari proyek ini adalah membangun robot *obstacle avoidance* empat roda berbasis ESP32 yang mampu menjaga jarak aman terhadap halangan di depannya menggunakan kontroler PID. Robot dilengkapi *dashboard* web yang menampilkan data sensor secara *real-time*, grafik respon PID, serta fitur kendali manual dan *tuning* parameter PID.

## 1.4. Batasan

Agar pengerjaan tetap terfokus, ditetapkan batasan-batasan sebagai berikut:

1. Mikrokontroler yang digunakan adalah ESP32 Devkit.
2. Sensor yang digunakan adalah empat buah RCWL-1601 (kompatibel dengan HC-SR04) yang dipasang di sisi depan tengah, depan kiri, depan kanan, dan belakang.
3. Kontroler PID hanya diterapkan untuk gerak maju-mundur (tidak mencakup *steering* atau belok).
4. Robot menggunakan empat roda biasa, bukan roda *omniwheel*.
5. *Dashboard* web hanya dapat diakses dalam jaringan WiFi lokal.

# 2. METODE PERANCANGAN

## 2.1. Perancangan Kontroler PID

Kontroler PID digunakan untuk menjaga jarak robot terhadap objek di depannya sesuai nilai *setpoint* yang diinginkan. Diagram blok kontrol sederhananya: sensor ultrasonik membaca jarak aktual ke objek, kemudian *error* dihitung dari selisih antara *setpoint* dan jarak aktual. Nilai *error* ini diumpankan ke kontroler PID yang menghasilkan sinyal kendali untuk motor.

Struktur data PID didefinisikan dalam *struct* `pid_ctrl_t` yang menyimpan parameter Kp, Ki, Kd, akumulasi integral, error sebelumnya, *time step* (*dt*), serta batas *output* minimum dan maksimum. Dalam kode, inisialisasi PID ditulis sebagai berikut:

```
pid_ctrl_init(&speed_pid, 0.8f, 0.05f, 0.1f, 0.03f, 0, 100);
```

Artinya, Kp = 0.8, Ki = 0.05, Kd = 0.1, *dt* = 0.03 detik, dengan *output* dibatasi antara 0 hingga 100 (persentase *duty cycle* PWM motor).

Rumus PID yang diimplementasikan di `pid_controller.c`:

```
u(t) = Kp * e(t) + Ki * integral(e) + Kd * de(t)/dt
```

di mana *e(t)* adalah selisih antara *setpoint* dan jarak terdepan terdekat (*min_front*) dari tiga sensor depan. Komponen *proportional* memberikan respon langsung terhadap *error*. Komponen *integral* mengakumulasi *error* seiring waktu untuk mengoreksi *steady-state error*, dengan perlindungan *anti-windup* melalui *clamping* nilai integral ke rentang 0-100. Komponen *derivative* memprediksi tren perubahan *error* untuk meredam *overshoot*.

Untuk memperhalus pembacaan sensor digunakan filter EMA (*Exponential Moving Average*) dengan *alpha* = 0.3. Rumusnya: `x_filtered = x_prev * 0.7 + x_raw * 0.3`. Filter ini mengurangi *noise* sehingga *output* PID tidak berfluktuasi liar akibat pembacaan sensor yang *jitter*.

Selain itu diterapkan *feed-forward braking*: jika robot mendekati halangan dengan kecepatan relatif di atas 10 cm/s pada jarak kurang dari 60 cm, *output* PID dikurangi secara proporsional terhadap kecepatan. Ini berfungsi sebagai pengereman dini (*preemptive braking*) sebelum PID sempat merespon penuh.

Logika keselamatan diterapkan jika jarak depan di bawah 10 cm: robot akan mundur dengan kecepatan 50% selama sensor belakang aman (jarak > 15 cm atau *timeout*). Jika belakang juga mentok (< 15 cm), robot berhenti total.


Berdasarkan uji coba, *setpoint* ideal yang digunakan adalah 25 cm. Robot tidak langsung mulus menjaga jarak — terdapat *error* kecil (sekitar 2-5 cm) yang membuat robot terus bergerak melakukan koreksi maju-mundur. Waktu yang dibutuhkan agar robot mencapai posisi stabil di sekitar *setpoint* (*settling time*) berkisar hingga 10 detik. Karakteristik ini wajar mengingat sensor ultrasonik memiliki *noise* bawaan dan *dt* pengukuran yang cukup panjang (30 ms).

## 2.2. Implementasi Perangkat Keras

Tabel 1. Daftar Komponen

| Komponen | Jumlah | Fungsi |
| :---: | --- | --- |
| ESP32 Devkit | 1 | Mikrokontroler utama. Memproses data sensor, menjalankan algoritma PID, dan menyediakan server WiFi untuk *dashboard*. |
| RCWL-1601 | 4 | Mengukur jarak ke objek di 4 sisi robot (depan tengah, depan kiri, depan kanan, belakang) dengan prinsip pantulan gelombang suara. |
| DRV8833 | 2 | Mengontrol kecepatan dan arah 4 motor DC. Satu *driver* bisa mengendalikan 2 motor. |
| Motor DC | 4 | Menggerakkan roda robot. Kecepatan diatur oleh sinyal PWM dari *driver*. |
| Baterai 18650 | 2 | Sumber daya utama (7.4V - 8.4V) untuk motor dan *driver*. Disusun secara seri. |
| Buck Converter 3.3V | 1 | Menurunkan tegangan 7.4V dari baterai menjadi 3.3V untuk ESP32 dan sensor. |



Roda yang digunakan adalah roda biasa, bukan roda *omniwheel* seperti yang direncanakan di awal. Kami gagal mengimplementasi roda *omniwheel* karena dua alasan. Pertama, waktu pengerjaan yang terbatas — koordinasi empat roda *omniwheel* membutuhkan perhitungan kinematika yang kompleks (vektor kecepatan translasi dan rotasi). Kedua, kode program untuk mengendalikan gerak *omnidirectional* memerlukan logika *mixing* empat kanal motor secara simultan dengan resolusi PWM yang presisi, yang tidak sempat kami selesaikan dalam masa pengerjaan. Oleh karena itu, kami memilih roda biasa agar pengembangan lebih efisien dan fokus pada algoritma kontrol PID serta integrasi sensor.

## 2.3. Perancangan Perangkat Lunak

Program utama (`main.c`) berjalan di atas FreeRTOS dengan alur sebagai berikut: inisialisasi motor via MCPWM, koneksi WiFi dengan mDNS (*project.local*), *start* server HTTP, lalu jalankan *task obstacle avoidance*. *Task* ini membaca empat sensor setiap 30 ms, menghitung *error*, menjalankan PID, dan mengatur kecepatan motor.

*Dashboard* web diimplementasikan menggunakan protokol HTTP di atas WiFi. ESP32 bertindak sebagai *server* yang melayani permintaan dari *browser*. Terdapat lima *endpoint* API:

1. `GET /` — mengirimkan halaman HTML *dashboard* yang sudah dikompilasi sebagai *binary array* dalam *firmware*.
2. `GET /api/status` — mengembalikan data sensor (depan, kiri, kanan, belakang dalam satuan cm), parameter PID (Kp, Ki, Kd, *error*, *output*), mode (*auto* / manual), dan data grafik (*error*, *output*, *setpoint*) dalam format JSON. *Endpoint* ini dipanggil setiap 300 ms oleh *frontend* untuk memperbarui tampilan secara *real-time*.
3. `POST /api/control` — menerima perintah gerak (*forward*, *backward*, *left*, *right*, *stop*) dan nilai *speed* (0-100%) untuk mode kendali manual. Digunakan oleh D-pad dan *keyboard* WASD pada *dashboard*.
4. `POST /api/mode` — beralih antara mode *auto* (PID aktif) dan *stop* (motor mati). Mode *stop* juga mereset status *manual override*.
5. `POST /api/pid` — menerima nilai *setpoint*, Kp, Ki, Kd baru dan langsung menerapkannya ke kontroler PID yang sedang berjalan. Fitur ini memungkinkan *live tuning* tanpa perlu mengkompilasi ulang *firmware*.

Fitur yang tersedia pada halaman *dashboard* meliputi:

- Tampilan *real-time* empat sensor dengan kode warna: hijau (aman), kuning (waspada), merah (bahaya).
- Grafik *error*, *output*, dan *setpoint* PID yang digambar langsung di *canvas* HTML5.
- D-pad virtual dan dukungan *keyboard* WASD + X untuk kendali manual robot.
- *Slider* kecepatan (0-100%) untuk mode manual.
- Form *tuning* PID: input *setpoint*, Kp, Ki, Kd dan tombol SET.
- Indikator koneksi *online*/*offline* (hijau/merah).
- Tombol AUTO untuk mengaktifkan mode *obstacle avoidance* otomatis dan tombol STOP untuk menghentikan robot.

# 3. HASIL DAN ANALISIS

## 3.1. Pengujian Sensor

Pengujian dilakukan dengan meletakkan objek (tembok atau kardus) pada berbagai jarak di depan masing-masing sensor dan membandingkan pembacaan di *dashboard* dengan pengukuran manual menggunakan meteran.

Hasil pengujian menunjukkan bahwa sensor RCWL-1601 berhasil mendeteksi objek dan menampilkan data di *dashboard* pada rentang jarak 5 cm hingga 100 cm. Pada rentang ini, pembacaan cukup akurat dengan deviasi sekitar 1-2 cm dari jarak sebenarnya berkat bantuan filter EMA.

Namun, sensor gagal membaca pada kondisi berikut:

- Jarak sangat dekat (< 5 cm): gelombang ultrasonik memantul terlalu cepat sehingga sensor belum siap menerima pantulan. Selain itu, efek *ringing* pada transduser menyebabkan sinyal pantul tidak bisa dibedakan dari sinyal kirim.
- Jarak sangat jauh (> 100 cm): intensitas gelombang pantul melemah secara signifikan. Sensor mengalami *timeout* (30 ms) dan mengembalikan nilai -1 yang diabaikan oleh filter EMA, sehingga pembacaan terakhir dipertahankan.

Kondisi kegagalan ini merupakan karakteristik bawaan sensor ultrasonik murah seperti RCWL-1601, bukan kesalahan program.

## 3.2. Pengujian Respon PID

Pengujian respon PID dilakukan dengan *setpoint* 25 cm dan parameter Kp = 0.8, Ki = 0.05, Kd = 0.1. Robot dihadapkan pada tembok, lalu mode AUTO diaktifkan dari *dashboard*. Respon diamati melalui grafik di *dashboard* dan perilaku fisik robot.

Saat robot diletakkan jauh dari tembok (> 25 cm), *error* negatif membuat PID menghasilkan *output* positif sehingga robot maju. Semakin dekat dengan *setpoint*, kecepatan berkurang secara bertahap. Ketika robot terlalu dekat (< 10 cm), sistem langsung memerintahkan mundur tanpa melalui perhitungan PID.

Hasil pengamatan menunjukkan bahwa robot tidak langsung mulus menjaga jarak 25 cm. Terdapat *error* kecil (sekitar 2-5 cm) yang menyebabkan robot terus bergerak — maju ketika terlalu jauh, mundur ketika terlalu dekat. Perilaku ini disebut *hunting*, yaitu kondisi di mana sistem terus mencari posisi ideal tanpa bisa benar-benar diam. Penyebab utamanya adalah *noise* pada sensor ultrasonik yang membuat pembacaan jarak sedikit berfluktuasi, ditambah dengan komponen *integral* yang lambat mengakumulasi koreksi.

*Settling time* — waktu yang dibutuhkan robot untuk mencapai dan bertahan di sekitar *setpoint* — berkisar hingga 10 detik. Setelah 10 detik, robot berada dalam kondisi "cukup stabil" di mana pergerakan maju-mundur sudah sangat kecil dan hampir tidak terlihat. Nilai Ki = 0.05 yang relatif kecil membuat koreksi *integral* berjalan lambat namun stabil, menghindari *overshoot* besar yang bisa menyebabkan robot menabrak tembok.


# 4. DISKUSI

Selama pengerjaan proyek, beberapa kendala teknis ditemukan dan diselesaikan sebagai berikut.

Kegagalan implementasi roda *omniwheel* merupakan kendala terbesar. Secara teori, roda *omniwheel* memungkinkan robot bergerak ke segala arah tanpa memutar badan, yang ideal untuk *obstacle avoidance*. Namun, koordinasi empat roda membutuhkan perhitungan kinematika *omnidirectional* dan logika *mixing* motor yang kompleks. Dalam waktu pengerjaan yang terbatas, kami memutuskan untuk menggunakan roda biasa dan memfokuskan upaya pada integrasi sensor serta *tuning* PID.

Keterbatasan sensor ultrasonik menjadi kendala berikutnya. Kegagalan pembacaan di bawah 5 cm berarti robot tidak bisa mendeteksi objek yang sudah sangat dekat, sehingga berisiko menabrak jika objek muncul tiba-tiba. Kegagalan di atas 100 cm berarti robot "buta" terhadap objek yang jauh, meskipun ini tidak terlalu kritis karena objek masih berada di luar jangkauan tabrakan. Solusi yang kami terapkan adalah menambahkan lapisan *safety* di level logika: jika sensor mengembalikan -1 (*timeout*), nilai lama dipertahankan oleh filter EMA, dan robot tetap beroperasi dengan data terakhir yang valid.

Dari sisi *tuning* PID, parameter Kp = 0.8, Ki = 0.05, Kd = 0.1 merupakan hasil kompromi. Nilai Kp yang lebih besar membuat robot lebih responsif tapi berisiko *overshoot* dan menabrak. Nilai Ki yang lebih besar mempercepat *settling time* tapi bisa menyebabkan *windup* dan osilasi. Konfigurasi ini menghasilkan gerakan yang relatif aman meskipun tidak sempurna — robot tidak menabrak tembok, namun tidak bisa diam sempurna di 25 cm.

Penggunaan *dashboard* web berbasis HTTP dan JSON terbukti memudahkan proses *debugging* dan *tuning*. Grafik *real-time* membantu memvisualisasikan hubungan antara *error*, *output* PID, dan *setpoint*, sehingga parameter bisa disesuaikan langsung dari *browser* tanpa perlu menyentuh kode.

# 5. KESIMPULAN

Berdasarkan hasil perancangan dan pengujian, dapat disimpulkan:

1. Sensor ultrasonik RCWL-1601 mampu mendeteksi objek dan menampilkan data pada *dashboard* di rentang 5-100 cm. Di bawah 5 cm dan di atas 100 cm sensor gagal membaca, sesuai karakteristik sensor ultrasonik.
2. Kontroler PID berhasil diimplementasikan dengan parameter Kp = 0.8, Ki = 0.05, Kd = 0.1 dan *setpoint* 25 cm. Robot mampu menyesuaikan kecepatan secara otomatis terhadap jarak halangan, meskipun terdapat *error* kecil 2-5 cm yang menyebabkan robot tidak bisa diam sempurna. *Settling time* berkisar hingga 10 detik.
3. *Dashboard* web berhasil diintegrasikan dengan protokol HTTP dan format JSON, menyediakan *monitoring* sensor, grafik PID, kendali manual, dan *live tuning* parameter PID tanpa perlu mengkompilasi ulang.
4. Roda *omniwheel* tidak berhasil diimplementasikan karena keterbatasan waktu dan kompleksitas kode, sehingga digunakan roda biasa yang lebih efisien untuk fokus pada kontrol PID.

# 6. LAMPIRAN

Kode Program: [https://github.com/bokumentation/Tubes-SistemKendali](https://github.com/bokumentation/Tubes-SistemKendali)
Dokumentasi: [https://drive.google.com/drive/folders/1JR2Kk3EQCOdPbi-odTxG8cz7G7xjlgce?usp=sharing](https://drive.google.com/drive/folders/1JR2Kk3EQCOdPbi-odTxG8cz7G7xjlgce?usp=sharing)

