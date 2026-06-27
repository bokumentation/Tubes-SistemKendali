# **LAPORAN TUGAS BESAR SISTEM KENDALI**

# **OMNIDIRECTIONAL OBSTACLE AVOIDANCE PADA ROBOT 4 RODA MENGGUNAKAN KONTROL PID**

# 

Disusun untuk Memenuhi Tugas Mata Kuliah Sistem Kendali  
Dosen Pengampu:   
**Marlindia Ike Sari S.T., M.T**

### **![][image1]**

Disusun oleh:

1. Muhammad Izzat Ramadhan 	(607022400050)  
2. Ibrahim Fauzi Ramadhan 	(607022400009) 

**PROGRAM STUDI D3 TEKNOLOGI KOMPUTER**  
**FAKULTAS ILMU TERAPAN**  
**UNIVERSITAS TELKOM**  
**BANDUNG**  
**2026**

# **1\.	PENDAHULUAN**

## 1.1.	Latar Belakang

## 1.2.	Rumusan Masalah

## 1.3.	Tujuan

## 1.4.	Batasan

# **2\.	METODE PERANCANGAN**

## 2.1.	Perancangan Kontroler PID

## 2.2.	Implementasi Perangkat Keras

Tabel 1\. Daftar Komponen

| Komponen  | Jumlah  | Fungsi  |
| :---: | ----- | ----- |
| ![][image2]  ESP32 Devkit  | 1 | Mikrokontroler utama. Memproses data sensor, menjalankan algoritma PID, dan menyediakan server WiFi untuk dashboard.  |
| ![][image3]  RCWL-1601  | 4 | Mengukur jarak ke objek di 4 sisi robot (depan, kiri, kanan, belakang) dengan prinsip pantulan gelombang suara.  |
| ![][image4]  DRV8833  | 2 | Mengontrol kecepatan dan arah 4 motor DC. Satu driver bisa mengendalikan 2 motor.  |
| ![][image5]  Motor DC  | 4       | Menggerakkan roda robot. Kecepatan diatur oleh sinyal PWM dari driver.  |
| ![][image6]  Baterai 18650  | 2  | Sumber daya utama (7.4V \- 8.4V) untuk motor dan driver. Disusun secara seri.  |
| ![][image7]  Buck Converter 3.3V  | 1  | Menurunkan tegangan 7.4V dari baterai menjadi 3.3V untuk ESP32 dan sensor.  |

![][image8]

Gambar 1\. Blok Diagram Perangkat Keras

## 2.3.	Perancangan Perangkat Lunak

![][image9]

Gambar 2\. Bagan Alir Program

# **3\.	HASIL DAN ANALISIS**

## 3.1.	Pengujian Sensor

## 3.2.	Pengujian Respon PID

## **4\.	DISKUSI**

## **5\.	KESIMPULAN**

## **6\.	LAMPIRAN**	

Kode Program: [https://github.com/bokumentation/Tubes-SistemKendali](https://github.com/bokumentation/Tubes-SistemKendali)  
Dokumentasi: [https://drive.google.com/drive/folders/1JR2Kk3EQCOdPbi-odTxG8cz7G7xjlgce?usp=sharing](https://drive.google.com/drive/folders/1JR2Kk3EQCOdPbi-odTxG8cz7G7xjlgce?usp=sharing)
