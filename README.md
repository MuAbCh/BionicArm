# BionicArm
A 3d printed robot arm controlled using EMG signals. It mimics your hand movements. Isn't that cool? 

# Smart Prosthetic Arm Controller

This repository contains the firmware and documentation for a low‑power, EMG‑driven prosthetic arm controller built using an STM32 Nucleo board, Myoware 2.0 EMG sensors, and PCA9685‑driven MG996R servos. The system interprets muscle signals in real time, detects flex patterns, and optimizes power usage for extended battery life.

---

## 🚀 Features

- **EMG‑Driven Actuation** – Real‑time acquisition of muscle signals via Myoware 2.0 sensors (ADC), processed with custom LPF/HPF filters and flex‑pattern detection.
- **Pattern Detection** – Configurable thresholds and a 4‑sample window to recognize intentional flex gestures.
- **Servo Control** – Smooth mapping of filtered EMG signals (0–4095 range) to five MG996R servos via PCA9685 PWM driver.
- **Power Optimization** – Dynamic frequency scaling: lowered STM32 clock from 168 MHz to 84 MHz, achieving ~9× reduction in average current draw.

---

## 📦 Hardware Requirements

- STM32 Nucleo development board (e.g., Nucleo‑F446RE)
- Myoware 2.0 EMG sensor module
- PCA9685 16‑channel PWM driver
- 5× MG996R servo motors
- Power supply: 7.2 V battery (recommended) or regulated 5 V for logic
- Breadboard, jumper wires, and connectors

---

## 🛠️ Software Requirements

- STM32CubeIDE or Keil µVision
- STM32CubeMX (for clock and peripheral configuration)
- ARM GCC toolchain (if using Makefile)
- C standard library and CMSIS headers (included via STM32CubeIDE)

---

## ⚙️ Setup & Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/MuAbCh/BionicArm.git
   cd BionicArm
   ```

2. **Open in STM32CubeIDE**
   - Import the project as an existing STM32CubeIDE project.
   - Ensure the correct MCU model is selected.

3. **Configure clock and peripherals**
   - Select the open project from existing `.ioc` file and use our `.ioc`. With this, CubeIDE will detup everything for you. You may need to replace the main.c it generates with ours.

4. **Build and Flash**
   - Click **Build** (hammer icon) to compile.
   - Connect your Nucleo board via ST‑Link and click **Run** to flash.

---

## 🔍 Usage

1. **Power on** the board and connect the Myoware sensor outputs to the ADC pins.
2. **Place sensors** on target muscle groups, ensuring good skin contact.
3. **Apply flex gestures**: the code will detect the pattern (small rise → peak → dip) and map to servo positions.
4. **Observe servo movement** on a linked prosthetic hand or test rig.

Tune `LOW_PASS_ALPHA`, `HIGH_PASS_ALPHA`, `PEAK_THRESHOLD`, and buffer sizes in `EMGFilter` as needed for your physiology.

---

## 📊 Performance & Power Analysis

- **Processing latency**: ~107 ms per cycle (106 ms busy, 1 ms ADC wait)  
- **Clock optimization**: 168 MHz → 84 MHz to maintain responsiveness  
- **Current draw**: reduced from 4.65 mA to ~0.51 mA (with sleep modes)  

---

## 📄 License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

