
# 🤖 Arduino Sumo Robot — Strategic Combat Bot

**Autonomous sumo robot built with Arduino**, equipped with opponent detection, line sensors, and multiple fighting strategies.
The robot can detect opponents, stay inside the ring, and decide between **attack**, **defense**, or **avoidance** strategies.

---

## ⚙️ Overview

This project implements a **fully autonomous sumo robot** controlled by an **Arduino board** and **Cytron motor drivers**.
It uses:

* **5 opponent detection sensors** (for directional awareness)
* **2 line sensors** (for ring boundary detection)
* **Start/stop module** for safe match control

The robot combines sensor readings with logic functions to **detect, chase, and attack opponents** or **retreat safely** when near the edge.

---

## 🧠 Features

* 🧭 **Opponent Detection:** Uses 5 infrared sensors for full front-arc scanning.
* ⚪ **Line Avoidance:** Two reflective line sensors prevent leaving the arena.
* ⚙️ **Modular Strategy System:**

  * `attackOpponent()` — aggressive frontal pursuit
  * `runAwayFromOpponent()` — evasive retreat strategy
  * `followWhiteLine()` — maintains position inside the ring
  * `lastPosition()` — uses last known enemy direction when sensors lose contact
* 🧩 **Start/Stop Module:** Enables or disables movement safely before each match.
* 🔁 **Real-time Updates:** Sensor readings and motor actions are printed to the Serial Monitor for debugging.

---

## 🪛 Hardware Setup

| Component             | Description                               |
| --------------------- | ----------------------------------------- |
| **Microcontroller**   | Arduino UNO / Arduino UNO R4              |
| **Motor Driver**      | 2 × Cytron MD10 (PWM + DIR)               |
| **Motors**            | DC gear motors                            |
| **Opponent Sensors**  | 5 × Infrared proximity sensors (pins 4–8) |
| **Line Sensors**      | 2 × Reflective IR sensors (pins 2–3)      |
| **Start/Stop Module** | Digital switch (pin 9)                    |
| **Power Supply**      | 7.4 V–12 V Li-Po or equivalent            |
| **Chassis**           | Custom Sumo robot frame                   |

---

## 🧩 Pin Configuration

| Pin   | Function          |
| ----- | ----------------- |
| 2–3   | Line sensors      |
| 4–8   | Opponent sensors  |
| 9     | Start/Stop module |
| 10–11 | Motor PWM         |
| 12–13 | Motor direction   |

---

## 🚀 How It Works

1. **Start/Stop module** is used to activate the robot at the beginning of a match.
2. **Line sensors** check the border — if detected, the robot backs up or turns.
3. **Opponent sensors** scan for the rival and estimate their direction.
4. Depending on conditions, the robot:

   * Attacks if an opponent is in front.
   * Retreats if the white border is detected.
   * Turns toward the last known opponent position if contact is lost.

---

## 🧱 Code Structure

```plaintext
setup()
 ├── initializeOppoSensors()
 ├── initializeStartStopModule()
loop()
 ├── check start/stop
 ├── read sensors
 ├── attackOpponent()
 ├── followWhiteLine()
 └── runAwayFromOpponent()
```

**Main Strategy Functions:**

* `attackOpponent()` — directs motors toward the opponent
* `runAwayFromOpponent()` — backs off if near the line
* `followWhiteLine()` — prevents falling off the ring
* `lastPosition()` — keeps pursuing based on last detected direction

---

## ⚔️ Future Improvements

* Add **gyro or compass module** for precise turns
* Use **PID control** for smoother speed adjustment
* Implement **adaptive strategy switching** (aggressive vs. defensive modes)
* Add **Bluetooth module** for wireless debugging

---

## 👩‍💻 Author

**Lidia Tomuș**
🎓 Technical University of Cluj-Napoca
💻 Robotics & Embedded Systems Enthusiast

---
