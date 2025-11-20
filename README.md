# RE702 Localization and Mapping - ATS

## Task  
**Pengantaran Barang dari Ruang 203 menuju ke Lobby BRAIL menggunakan TurtleBot4**

Proyek ini dilakukan untuk memenuhi ATS dalam mata kuliah **RE702 Lokalisasi dan Pemetaan**, dengan mengintegrasikan Practical Work 1-3, meliputi:

1. Setup & Configuration ROS  
2. Map Generation  
3. Navigation  

Sistem ini dibuat untuk mensimulasikan TurtleBot4 sebagai robot pengantar barang otonom yang dapat melakukan lokalisasi, pemetaan, dan navigasi otonom menggunakan ROS2.

---

## Studi Kasus

TurtleBot4 ini bertugas mengambil barang dari **Ruang 203** dan mengantarkan barang tersebut ke **Lobby BRAIL Lantai 2 Gedung Utama Polibatam**, dilengkapi dengan sistem indikator **buzzer** untuk menunjukkan proses **pick** dan **place** barang.

---

## Langkah-Langkah Sebelum Menjalankan Package

1. Pastikan sudah membuat map dengan environment yang digunakan.  
2. Simpan hasil mapping pada folder `maps/`.  
3. Jika ingin membuat map baru, dapat mengikuti dokumentasi resmi TurtleBot4:  
   https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html  
4. Pastikan perangkat (laptop/PC) terhubung ke LAN/WiFi yang sama dengan TurtleBot4, lalu masuk ke robot melalui SSH.

---

## Fitur Utama

### 1. Localization (Lokalisasi)
Robot melakukan lokalisasi menggunakan:
- AMCL (Adaptive Monte Carlo Localization)
- Pose diperoleh dari sensor ataupun simulasi (LiDAR/TurtleBot4)

### 2. Mapping (Pemetaan)
Robot melakukan pemetaan menggunakan:
- SLAM  
- Map disimpan dalam format `.yaml` dan `.pgm`

### 3. Navigation (Navigasi Otonom)
Robot bergerak dengan 2 waypoint:
- **Pick Point** → Ruang 203  
- **Place Point** → Lobby BRAIL  

Menggunakan:
- nav2 (Navigation2 stack)
- RViz2

### 4. Indikator Buzzer
Robot memiliki 2 kondisi:
- Sampai di titik *pick* (Ruang 203) → bunyi 1 kali → simulasi mengambil barang  
- Sampai di titik *place* (Lobby BRAIL) → bunyi 2 kali → simulasi mengantarkan barang  

---

## Cara Menjalankan Proyek

### 1. Setup Workspace
Buat workspace:
```bash
mkdir ros2_ws/src
cd ros2_ws/src
```

### 2. Menjalankan Mapping
Generate Map di TurtleBot4:
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=path/ke/map.yaml
```
Pastikan `path/ke/map.yaml` sudah diganti dengan file map yang benar.

Jalankan RViz2:
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

Teleop untuk menggerakkan robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Simpan Map:
```bash
ros2 run nav2_map_server map_saver_cli -f "map_name" \
 --ros-args -p map_subscribe_transient_local:=true -r __ns:=/namespace
```

### 3. Menjalankan Navigation
Load map:
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=office.yaml
```

Jalankan nav2:
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

RViz2 untuk menyesuaikan pose robot:
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

### 4. Menjalankan Misi Pengantaran (Pick → Place)
Jalankan program navigasi:
```bash
ros2 run re702_midterm_localization_mapping navigation
```

## Dokumentasi Video Demo TurtleBot4
https://www.youtube.com/watch?v=Wi7UTuRp9h0
