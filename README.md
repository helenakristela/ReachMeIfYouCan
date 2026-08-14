# Reach Me If You Can

Implementasi 2D Inverse Kinematics untuk lengan robot dengan N sendi (2-5), ditulis dalam C++17 dan raylib untuk visualisasi.

## Fitur

- Forward kinematics via matriks transformasi homogen 3x3 yang di-chain per sendi.
- Inverse kinematics numerik from scratch (Jacobian Transpose, Pseudo-Inverse (2x2 explicit inverse), dan Damped Least Squares (Levenberg-Marquardt)).
- Deteksi singularitas lewat manipulability measure `sqrt(det(J J^T))`, dengan fallback otomatis ke metode yang stabil saat mendekati singular.
- Constraint sudut sendi yang bisa diatur lewat slider.
- Obstacle avoidance untuk lengan N sendi berapa pun.
- Visualisasi step-by-step animasi lengan menuju target lewat GUI raylib + raygui.

## Video Presentasi

[Tonton di sini](https://drive.google.com/file/d/14cH-B2_9j36zAE2xveDRBccNu0bKaQTk/view?usp=sharing)

## Struktur

```
src/vec2.hpp
src/mat.hpp
src/obstacle.hpp
src/arm.hpp
src/main.cpp
test/test_arm.cpp
external/raygui.h
```

## Build

Cara termudah, di Linux (Ubuntu/Debian):

```bash
./build.sh
./build/reach_me
```

Script `build.sh` otomatis memakai raylib dari sistem (lewat `pkg-config`) kalau sudah terpasang, atau meng-clone dan build raylib 5.5 dari sumber ke `deps/raylib` kalau belum ada.

Kalau raylib sudah terpasang lewat package manager dan mendukung CMake config:

```bash
mkdir build && cd build
cmake ..
cmake --build .
./reach_me
```

Dependency sistem yang dibutuhkan raylib di Linux (kalau build dari sumber): `libgl1-mesa-dev libx11-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev`.

## Cara pakai

- Klik kiri di kanvas untuk menentukan titik target, lengan akan bergerak bertahap ke sana.
- Atur jumlah sendi, panjang tiap segmen, dan batas sudut lewat panel kiri.
- Pilih metode antara Transpose, Pseudo-Inverse, atau DLS.
- Centang "Hindari obstacle" untuk mengaktifkan mode obstacle, lalu klik kanan di kanvas untuk menaruh atau menghapus obstacle.
- Panel kiri menampilkan status: jumlah iterasi, error jarak ke target, dan nilai manipulability (indikator kedekatan dengan singularity).

## Uji numerik

```bash
g++ -std=c++17 -O2 test/test_arm.cpp -o test_arm -lm
./test_arm
```

Menguji konvergensi ketiga metode Inverse Kinematics, penanganan kondisi near-singular (memastikan tidak ada NaN/divergen), dan deteksi target unreachable.