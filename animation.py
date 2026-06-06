import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

# ══════════════════════════════════════════════════════════
# SIMULATION
# ══════════════════════════════════════════════════════════
m          = 1200
v0         = 30
F_engine   = 4000
Cd         = 0.30
A          = 2.2
rho        = 1.225
ms         = 300
mu         = 45
ks         = 16000
kt         = 190000
cs_base    = 500
Kp         = 8000
Kd         = 500
cs_max     = 5000
F_brake    = 3000
efficiency = 0.85
battery_max= 400000

# Small dt for suspension stability
dt   = 0.001
t    = np.arange(0, 60, dt)

road = np.zeros(len(t))
road[10000:10050] = 0.05

def get_phase(time):
    if   0  <= time < 15: return "accelerate"
    elif 15 <= time < 25: return "brake"
    elif 25 <= time < 40: return "accelerate"
    elif 40 <= time < 50: return "brake"
    else:                 return "accelerate"

v_log, battery_log, boost_log, drag_log, xs_log, cs_log = [], [], [], [], [], []

v          = v0
battery    = 0.0
susp_state = np.zeros(4)

for i in range(len(t)):
    phase = get_phase(t[i])
    zr    = road[i]

    F_drag = 0.5 * rho * Cd * A * v**2

    xs, xs_dot, xu, xu_dot = susp_state
    disp_diff = abs(xs - xu)
    vel_diff  = abs(xs_dot - xu_dot)
    c_ai = min(cs_base + Kp * disp_diff + Kd * vel_diff, cs_max)

    Fs = ks * (xu - xs) + c_ai * (xu_dot - xs_dot)
    Ft = kt * (zr - xu)
    xs_ddot = Fs / ms
    xu_ddot = (Ft - Fs) / mu

    xs_dot += xs_ddot * dt
    xu_dot += xu_ddot * dt
    xs     += xs_dot  * dt
    xu     += xu_dot  * dt
    susp_state = [xs, xs_dot, xu, xu_dot]

    if phase == "brake" and v > 0:
        P_recovered = F_brake * v
        battery     = min(battery + P_recovered * dt * efficiency, battery_max)
        F_boost     = 0
        F_net       = -F_brake - F_drag
    elif phase == "accelerate":
        if battery > 0 and v > 0.1:
            F_boost = min(battery / (v * dt), 2000)
            battery = max(battery - F_boost * v * dt, 0)
        else:
            F_boost = 0
        F_net = F_engine + F_boost - F_drag
    else:
        F_boost = 0
        F_net   = -F_drag

    a = F_net / m
    v = max(v + a * dt, 0)

    v_log.append(v)
    battery_log.append(battery / 1000)
    boost_log.append(F_boost)
    drag_log.append(F_drag)
    xs_log.append(xs * 100)
    cs_log.append(c_ai)

# ══════════════════════════════════════════════════════════
# ANIMATION
# ══════════════════════════════════════════════════════════
fig = plt.figure(figsize=(14, 8), facecolor='#0a0a0a')
gs  = GridSpec(3, 3, figure=fig, hspace=0.5, wspace=0.4)

ax_speed   = fig.add_subplot(gs[0, :])
ax_drag    = fig.add_subplot(gs[1, 0])
ax_susp    = fig.add_subplot(gs[1, 1])
ax_battery = fig.add_subplot(gs[1, 2])
ax_boost   = fig.add_subplot(gs[2, 0])
ax_damp    = fig.add_subplot(gs[2, 1])
ax_info    = fig.add_subplot(gs[2, 2])

for ax in [ax_speed, ax_drag, ax_susp, ax_battery, ax_boost, ax_damp, ax_info]:
    ax.set_facecolor('#111111')
    ax.tick_params(colors='white', labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#333333')

def style_ax(ax, title, ylabel, color, ymin, ymax):
    ax.set_title(title, color=color, fontsize=8, fontweight='bold')
    ax.set_ylabel(ylabel, color='#aaaaaa', fontsize=7)
    ax.set_xlim(0, t[-1])
    ax.set_ylim(ymin, ymax)
    ax.grid(True, alpha=0.2, color='#333333')

style_ax(ax_speed,   '🚗 VEHICLE SPEED',   'm/s',    '#4fc3f7', 0, 100)
style_ax(ax_drag,    '💨 AERO DRAG',        'N',      '#ef5350', 0, 3000)
style_ax(ax_susp,    '🔧 SUSPENSION',       'cm',     '#66bb6a', -2, 2)
style_ax(ax_battery, '⚡ KERS BATTERY',     'kJ',     '#ffa726', 0, 450)
style_ax(ax_boost,   '🚀 KERS BOOST',       'N',      '#ab47bc', 0, 2500)
style_ax(ax_damp,    '🎛️ AI DAMPING',       'N·s/m',  '#26c6da', 0, 5500)

line_speed,   = ax_speed.plot([],   [], color='#4fc3f7', lw=2)
line_drag,    = ax_drag.plot([],    [], color='#ef5350', lw=1.5)
line_susp,    = ax_susp.plot([],    [], color='#66bb6a', lw=1.5)
line_battery, = ax_battery.plot([], [], color='#ffa726', lw=1.5)
line_boost,   = ax_boost.plot([],   [], color='#ab47bc', lw=1.5)
line_damp,    = ax_damp.plot([],    [], color='#26c6da', lw=1.5)

time_text  = ax_speed.text(0.02, 0.88, '', transform=ax_speed.transAxes, color='white', fontsize=9)
phase_text = ax_speed.text(0.70, 0.88, '', transform=ax_speed.transAxes, color='yellow', fontsize=9, fontweight='bold')

ax_info.axis('off')
info = ax_info.text(0.05, 0.95, '', transform=ax_info.transAxes,
                    color='white', fontsize=8, verticalalignment='top',
                    fontfamily='monospace')

fig.suptitle('AI AUTOMOTIVE SIMULATION — Aero + Suspension + KERS',
             color='white', fontsize=13, fontweight='bold', y=0.98)

# Skip frames to speed up — show every 30th data point
STEP = 30

def update(frame):
    i = min(frame * STEP, len(t) - 1)
    x_data = t[:i]

    line_speed.set_data(x_data,   v_log[:i])
    line_drag.set_data(x_data,    drag_log[:i])
    line_susp.set_data(x_data,    xs_log[:i])
    line_battery.set_data(x_data, battery_log[:i])
    line_boost.set_data(x_data,   boost_log[:i])
    line_damp.set_data(x_data,    cs_log[:i])

    phase = get_phase(t[i])
    phase_label = {"accelerate": "🟢 ACCELERATING", "brake": "🔴 BRAKING"}

    time_text.set_text(f't = {t[i]:.1f}s')
    phase_text.set_text(phase_label.get(phase, ""))

    info.set_text(
        f"Speed  : {v_log[i]:.1f} m/s\n"
        f"         {v_log[i]*3.6:.1f} km/h\n\n"
        f"Drag   : {drag_log[i]:.0f} N\n\n"
        f"Battery: {battery_log[i]:.1f} kJ\n\n"
        f"Boost  : {boost_log[i]:.0f} N\n\n"
        f"Damping: {cs_log[i]:.0f} N·s/m"
    )

    return (line_speed, line_drag, line_susp, line_battery,
            line_boost, line_damp, time_text, phase_text, info)

frames = len(t) // STEP
ani = animation.FuncAnimation(fig, update, frames=frames, interval=30, blit=True)

print("Saving video — please wait...")
writer = animation.FFMpegWriter(fps=30, bitrate=2000)
ani.save('simulation_video.mp4', writer=writer, dpi=120)
print("Done! Video saved as simulation_video.mp4")

plt.show()