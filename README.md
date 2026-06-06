# 🚗⚡ Combined Automotive Simulation — Aero + Suspension + KERS

A unified physics-based simulation combining three automotive systems into one integrated model.

Built as **Phase 4** of my AI-Assisted Automotive Simulations series.

---

## 📐 System Architecture

The three systems work together in every time step:

Road Input feeds into Suspension Module which absorbs bumps using AI PD Controller.
Vehicle speed feeds into Aero Module which calculates drag resistance.
Braking phases feed into KERS Module which captures and stores energy.
All forces combine into final vehicle motion output.

---

## 📊 Simulation Results

![Combined Simulation](combined_simulation.png)

---

## ⚙️ Parameters

| System | Parameter | Value |
|---|---|---|
| Vehicle | Mass | 1200 kg |
| Aero | Drag Coefficient | 0.30 |
| Aero | Frontal Area | 2.2 m2 |
| Suspension | Spring Stiffness | 16000 N/m |
| Suspension | AI Damping Range | 500-5000 N/m |
| KERS | Efficiency | 85% |
| KERS | Max Battery | 400 kJ |
| KERS | Max Boost | 2000 N |

---

## 🚗 Drive Cycle

0s to 15s  — Acceleration
15s to 25s — Braking with KERS capture
25s to 40s — Acceleration with KERS boost
40s to 50s — Braking with KERS capture
50s to 60s — Acceleration with KERS boost

---

## 🛠️ Setup and Run

git clone https://github.com/MIz-1/combined-simulation.git
cd combined-simulation
python3 -m venv venv
source venv/bin/activate
pip install numpy matplotlib
python combined_simulation.py

---

## 🔭 Roadmap

- [x] Phase 1 — Aerodynamic Drag Simulator
- [x] Phase 2 — AI Smart Suspension System
- [x] Phase 3 — KERS Energy Recovery System
- [x] Phase 4 — Combined Aero + Suspension + KERS

---

## 👨‍💻 About

Self-taught simulation developer exploring AI + automotive physics.
Student project — built for learning, not production.
