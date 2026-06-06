import numpy as np
import matplotlib.pyplot as plt

# ══════════════════════════════════════════════════════════
# COMBINED SIMULATION — Aero + Suspension + KERS
# ══════════════════════════════════════════════════════════

# ── Vehicle Parameters ─────────────────────────────────────
m          = 1200      # Car mass (kg)
v0         = 30        # Initial speed (m/s)
F_engine   = 4000      # Engine force (N)

# ── Aero Parameters ────────────────────────────────────────
Cd         = 0.30      # Drag coefficient
A          = 2.2       # Frontal area (m²)
rho        = 1.225     # Air density (kg/m³)

# ── Suspension Parameters ──────────────────────────────────
ms         = 300       # Sprung mass (kg)
mu         = 45        # Unsprung mass (kg)
ks         = 16000     # Spring stiffness (N/m)
kt         = 190000    # Tyre stiffness (N/m)
cs_base    = 500       # Base damping (N·s/m)
Kp         = 8000      # PD Proportional gain
Kd         = 500       # PD Derivative gain
cs_max     = 5000      # Max damping (N·s/m)

# ── KERS Parameters ────────────────────────────────────────
F_brake    = 3000      # Braking force (N)
efficiency = 0.85      # KERS efficiency
battery    = 0.0       # Battery starts empty
battery_max= 400000    # Max battery (J)

# ── Time Setup ─────────────────────────────────────────────
dt = 0.001
t  = np.arange(0, 60, dt)

# ── Road Profile (bump at t=10s) ───────────────────────────
road = np.zeros(len(t))
road[10000:10050] = 0.05   # 5cm bump at t=10s

# ── Drive Cycle ────────────────────────────────────────────
def get_phase(time):
    if   0  <= time < 15: return "accelerate"
    elif 15 <= time < 25: return "brake"
    elif 25 <= time < 40: return "accelerate"
    elif 40 <= time < 50: return "brake"
    else:                 return "accelerate"

# ── Storage Arrays ─────────────────────────────────────────
v_log        = []
battery_log  = []
boost_log    = []
drag_log     = []
xs_log       = []
cs_log       = []

# ── Initial State ──────────────────────────────────────────
v       = v0
battery = 0.0
susp_state = np.zeros(4)   # [xs, xs_dot, xu, xu_dot]

# ══════════════════════════════════════════════════════════
# MAIN SIMULATION LOOP
# ══════════════════════════════════════════════════════════
for i in range(len(t)):
    phase = get_phase(t[i])
    zr    = road[i]

    # ── 1. AERO MODULE ─────────────────────────────────────
    F_drag = 0.5 * rho * Cd * A * v**2

    # ── 2. SUSPENSION MODULE ───────────────────────────────
    xs, xs_dot, xu, xu_dot = susp_state

    displacement_diff = abs(xs - xu)
    velocity_diff     = abs(xs_dot - xu_dot)
    c_ai = cs_base + Kp * displacement_diff + Kd * velocity_diff
    c_ai = min(c_ai, cs_max)

    Fs = ks * (xu - xs) + c_ai * (xu_dot - xs_dot)
    Ft = kt * (zr - xu)

    xs_ddot = Fs / ms
    xu_ddot = (Ft - Fs) / mu

    xs_dot += xs_ddot * dt
    xu_dot += xu_ddot * dt
    xs     += xs_dot  * dt
    xu     += xu_dot  * dt

    susp_state = [xs, xs_dot, xu, xu_dot]

    # ── 3. KERS MODULE ─────────────────────────────────────
    if phase == "brake" and v > 0:
        P_recovered = F_brake * v
        E_captured  = P_recovered * dt * efficiency
        battery     = min(battery + E_captured, battery_max)
        F_boost     = 0
        F_net       = -F_brake - F_drag

    elif phase == "accelerate":
        if battery > 0 and v > 0.1:
            F_boost = min(battery / (v * dt), 2000)
            E_used  = F_boost * v * dt
            battery = max(battery - E_used, 0)
        else:
            F_boost = 0
        F_net = F_engine + F_boost - F_drag
    else:
        F_boost = 0
        F_net   = -F_drag

    # ── 4. VEHICLE MOTION ──────────────────────────────────
    a = F_net / m
    v = max(v + a * dt, 0)

    # ── Log ────────────────────────────────────────────────
    v_log.append(v)
    battery_log.append(battery / 1000)
    boost_log.append(F_boost)
    drag_log.append(F_drag)
    xs_log.append(xs)
    cs_log.append(c_ai)

# ══════════════════════════════════════════════════════════
# PLOT
# ══════════════════════════════════════════════════════════
fig, axes = plt.subplots(6, 1, figsize=(14, 18))

axes[0].plot(t, v_log, color='blue', label='Vehicle Speed')
axes[0].set_ylabel('Speed (m/s)')
axes[0].legend(); axes[0].grid(True)
axes[0].set_title('Combined Simulation — Aero + Suspension + KERS')

axes[1].plot(t, drag_log, color='red', label='Aero Drag Force')
axes[1].set_ylabel('Drag (N)')
axes[1].legend(); axes[1].grid(True)

axes[2].plot(t, xs_log, color='green', label='Body Displacement (AI Suspension)')
axes[2].set_ylabel('Displacement (m)')
axes[2].legend(); axes[2].grid(True)

axes[3].plot(t, cs_log, color='purple', label='AI Damping')
axes[3].axhline(y=1000, color='gray', linestyle='--', label='Passive Damping')
axes[3].set_ylabel('Damping (N·s/m)')
axes[3].legend(); axes[3].grid(True)

axes[4].plot(t, battery_log, color='orange', label='Battery Charge')
axes[4].set_ylabel('Energy (kJ)')
axes[4].legend(); axes[4].grid(True)

axes[5].plot(t, boost_log, color='brown', label='KERS Boost Force')
axes[5].set_ylabel('Boost (N)')
axes[5].set_xlabel('Time (s)')
axes[5].legend(); axes[5].grid(True)

plt.tight_layout()
plt.savefig('combined_simulation.png')
plt.show()

print("Combined Simulation complete!")
print(f"Max Speed     : {max(v_log):.2f} m/s ({max(v_log)*3.6:.1f} km/h)")
print(f"Max Drag      : {max(drag_log):.2f} N")
print(f"Max Battery   : {max(battery_log):.2f} kJ")
print(f"Max KERS Boost: {max(boost_log):.2f} N")