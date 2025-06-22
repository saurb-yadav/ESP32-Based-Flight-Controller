
# 🚁 ESP32-Based Quadcopter Flight Controller

I chose the ESP32 as the flight controller primarily for its low cost, onboard WiFi & BLE capabilities, and potential for future ROS integration.

---

## 📷 Build Photos

<img src="assets/esp32_mounted.jpg" alt="ESP32 mounted on frame" width="600"/>
<br/>
<sub><i>ESP32 mounted on perfboard and attached to the F450 frame</i></sub>

<img src="assets/setup_with_laptop.jpg" alt="Drone on table with code on screen" width="600"/>
<br/>
<sub><i>PID tuning and motor testing workspace setup</i></sub>

---

## 🎥 Tutorial Videos

### 🛠️ How to Make?
[![How to Make?](https://img.youtube.com/vi/1zZbk0xWIrc/0.jpg)](https://youtu.be/1zZbk0xWIrc)

### 🛫 Dorm Flight
[![Dorm flight](https://img.youtube.com/vi/2goVaf8LJQM/0.jpg)](https://youtu.be/2goVaf8LJQM)

### 📈 Flight Stabilisation
[![Flight stabilisation](https://img.youtube.com/vi/PweCWXNrxg0/0.jpg)](https://youtu.be/PweCWXNrxg0?si=KJHE-g0SIH3p4Ixf)

### 🧠 Understanding PID Controller
[![Understanding PID Controller](https://img.youtube.com/vi/dMRDzicSvXk/0.jpg)](https://youtu.be/dMRDzicSvXk?si=4SlX_EvzxvCbH7zv)

### 🌐 Tuning PID Gains via Webserver
[![Tuning of PID Controller Gains](https://img.youtube.com/vi/kl3Dlm11AEQ/0.jpg)](https://youtu.be/kl3Dlm11AEQ?si=NrJVn8WFx9-ViMtl)

---

## ✅ Testing Before Flight

Use the testing codes provided in the `test/` folder:

- `reciever_pwm_esp32.ino` – Check transmitter & receiver signals
- `measure_angles_from_mpu.ino` – Validate IMU angle output
- `motor_calibration_esp32.ino` – Calibrate and sync ESCs/motors
- `Voltage_measurement_esp32.ino` – Test voltage sensing (optional)
- `anglemode_flightcontroller_ver3.1_PID_values_tuning_webserver.ino` – PID tuning via webserver

Refer to hand-drawn circuit diagram in:
