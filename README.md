# TASK - FINAL

> **DEADLINE SENIN, 2 NOV 2025 PUKUL 18.00**

<p align="center">
<img src="./assets/image.png" alt="gambar robot humanoid">
</p>

### Contents:

- [DESKRIPSI TUGAS](#deskripsi-tugas)
- [DETAIL PENUGASAN](#detail-penugasan)
- [PROGRAM-REMOTE](#program-remote)
- [TRANSMITTER](#transmitter)
- [RECEIVER](#receiver)
- [WEBOTS](#webots)
- [CARA-BERJALAN](#cara-berjalan)
- [POSE-CONTROLLER](#pose-controller)

## DESKRIPSI TUGAS

Diminta untuk membuat kontroler robot humanoid yang dapat menerima input dari keyboard untuk menggerakkan robot tersebut di dalam simulasi Webots. Robot harus mampu melakukan perpindahan posisi dengan cara bergerak maju, mundur, geser kanan, kiri dan belok kanan, kiri sesuai dengan perintah yang diberikan melalui keyboard.

## DETAIL PENUGASAN

kalian diminta untuk membuat 2 program yaitu program receiver dan program kontroller robot diwebots.

<p align="center">
<img src="./assets/skema.png" alt="gambar robot humanoid">
Skema alur program receiver dan controller robot. Perintah keyboard dikirimkan dari Remote ke Webots laptop lain melalui perantara ESP-NOW.

</p>

#### **TODO 1 ESP32**

    Buatlah Program Receiver untuk menerima perintah dari keyboard yang dikirimkan dalam bentuk ESP-NOW.

#### **TODO 2 WEBOTS**

    Buatlah Program Kontroller Robot di Webots untuk menggerakkan robot sesuai perintah yang diterima dari Program Receiver.

Pengujian dan presentasi akan dilakukan pada: pada 2 November 2025. Pengujian dilakukan dengan penguji yang mengirimkan perintah dari keyboard pada program Remote.

# **PROGRAM**

<details id="program-remote-transmitter" open>
  <summary><b>PROGRAM_INPUT_KEYBOARD & TRANSMITTER</b></summary>
    <p align="center">
    <img src="./assets/remote.png" alt="gambar robot humanoid">
    </p>

    Deskripsi
    Program berfungsi untuk menerima input dari keyboard dan dikirimkan dalam bentuk Serial
        Perintah yang dapat dikirimkan adalah:
        - W, A, S, D, Q, E, C, V, R, F

    Hasil input dari keyboard akan dikirimkan melalui ESP-NOW dari Transmitter ke ESP_Receiver. dimana perintah yang dikirmkan berupa karakter ASCII atau nilainya.
    Panjang perintah hanya 1 yaitu berisi nilai key yang ditekan pada keyboard.

</details>
<br>
<details id="receiver" open>
  <summary><b>RECEIVER</b></summary>
<p>

    DESKRIPSI
    Program Receiver berfungsi untuk menerima perintah dari Transmitter dan meneruskannya ke Webots. Program ini akan berjalan di ESP32 yang terhubung dengan robot humanoid.

    KETENTUAN
    - Struktur folder atau program bersifat bebas.
    - Nilai identitas(MAC Address) dari Receiver disesuaikan dengan milik masing-masing.
    - MAC Address sudah dialokasikan pada file main.cpp, boleh dipindah-pindah sesuai kebutuhan tapi tidak boleh dirubah isinya.

</p>
<br>
</details id="webots" open>
<br>
<details>
  <summary><b>WEBOTS</b></summary>
  <p align="center">
    <img src="./assets/webots.png" alt="gambar robot humanoid">
  </p>
<p>

    DESKRIPSI
    Program Kontroller Robot di Webots berfungsi untuk menggerakkan robot sesuai perintah yang diterima dari Program Receiver. Program ini akan berjalan di dalam simulasi Webots pada laptop yang berbeda dengan laptop tempat Program Receiver berjalan.

    KETENTUAN
    - Struktur folder atau program bersifat bebas.
    - controller-fp.cpp hanya bersifat sebagai contoh atau placeholder, boleh dihapus atau dimodifikasi sesuai kebutuhan.

</p>
<div>
    <h3>Struktur Robot</h3>
    <a href="https://cyberbotics.com/doc/guide/robotis-op2?version=R2022a#robotisop2hinge2-variant" alt="Dokumentasi Robot">Webots Robotis OP2 Proto</a>
    <img src="./assets/robot.png" alt="gambar robotis op2">
</div>
</details>
<br>

# EXTRA

<details id="cara-berjalan" open>
  <summary><b>CARA BERJALAN</b></summary>
  <p>

    Pose berjalan dimasukin ke json, untuk setiap pose kalian bisa bikin sendiri atau nunggu aku, bakal tak buatin template dengan struktur json yang sama.
    
    Parameter Posisi dan asosiasinya sama device di webots di JSON.
    [Head, Neck, ShoulderL, ShoulderR, ArmUpperL, ArmUpperR, ArmLowerL, ArmLowerR, PelvYL, PelvYR, PelvL, PelvR, LegUpperL, LegUpperR, LegLowerL, LegLowerR, AnkleL, AnkleR, FootL, FootR]
  </p>

  <h3><b>Pose</b></h3>
  <ul>
    <li> <details>
        <summary> <B>Berdiri</B> </summary>
        https://github.com/virose-its/MODUL-PEMBELAJARAN/blob/a3a8ad769d44f037fd173d8aa18305681baf0f95/Programming/PENUGASAN/TASK-FINAL/Webots/poses/pose-berdiri.json#L1-L19
        <p align="center"><img src="./assets/berdiri.png" alt="visual-pose-berdiri"></p>
        </details>
    </li>
    <li> <details>
        <summary> <B>Jalan Maju</B> </summary>
        https://github.com/virose-its/MODUL-PEMBELAJARAN/blob/a3a8ad769d44f037fd173d8aa18305681baf0f95/Programming/PENUGASAN/TASK-FINAL/Webots/poses/pose-jalan-maju.json#L1-L132
        <p align="center"><img src="" alt="visual-pose-jalan-maju"></p>
        </details>
    </li>
    <li> <details>
        <summary> <B>Jalan Mundur</B> </summary>
        https://github.com/virose-its/MODUL-PEMBELAJARAN/blob/a3a8ad769d44f037fd173d8aa18305681baf0f95/Programming/PENUGASAN/TASK-FINAL/Webots/poses/pose-jalan-mundur.json#L1-L133
        <p align="center"><img src="" alt="visual-pose-jalan-mundur">
        </p>
        </details>
    </li>
    <li> <details>
        <summary> <B>Geser Kanan</B> </summary>
        https://github.com/virose-its/MODUL-PEMBELAJARAN/blob/a3a8ad769d44f037fd173d8aa18305681baf0f95/Programming/PENUGASAN/TASK-FINAL/Webots/poses/pose-geser-kanan.json#L1-L48
        <p align="center"><img src="" alt="visual-pose-geser-kanan"></p>
        </details>
    </li>
    <li> <details>
        <summary> <B>Geser Kiri</B> </summary>
        https://github.com/virose-its/MODUL-PEMBELAJARAN/blob/a3a8ad769d44f037fd173d8aa18305681baf0f95/Programming/PENUGASAN/TASK-FINAL/Webots/poses/pose-geser-kiri.json#L1-L48
        <p align="center"><img src="" alt="visual-pose-geser-kiri"></p>
        </details>
    </li>
    <li> <details>
        <summary> <B>Belok Kanan</B> </summary>
        https://github.com/virose-its/MODUL-PEMBELAJARAN/blob/a3a8ad769d44f037fd173d8aa18305681baf0f95/Programming/PENUGASAN/TASK-FINAL/Webots/poses/pose-belok-kanan.json#L1-L48
        <p align="center"><img src="" alt="visual-pose-belok-kanan"></p>
        </details>
    </li>
    <li> <details>
        <summary> <B>Belok Kiri</B> </summary>
        https://github.com/virose-its/MODUL-PEMBELAJARAN/blob/a3a8ad769d44f037fd173d8aa18305681baf0f95/Programming/PENUGASAN/TASK-FINAL/Webots/poses/pose-geser-kiri.json#L1-L48
        <p align="center"><img src="" alt="visual-pose-belok-kiri"></p>
        </details>
    </li>
  </ul>
</details>
<br>
<details id="pose-controller" open>
  <summary><b>POSE CONTROLLER</b></summary>
  
  
    Pose controller cuma controller tambahan yang bisa kalian pake buat ngebuat pose di robot. Kalian bisa pake ini buat ngebantu bikin gerakan di robot.
   
</details>
