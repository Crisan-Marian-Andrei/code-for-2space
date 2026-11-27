#!/usr/bin/env python3
"""
Figure 5.7 - Barometer Lockout Simulation (Crocket HPR, Section 5.9 Testing)

Standalone publication-grade plot of the AeroTech J270W ascent profile,
showing barometer lockout hysteresis and transonic barometric distortion.

Output: fig_5_7_baro_lockout_sim.png (300 DPI, white background)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# ---------------------------------------------------------------------------
# Publication style
# ---------------------------------------------------------------------------
plt.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.labelsize": 10,
    "axes.linewidth": 0.8,
    "grid.linewidth": 0.5,
    "grid.alpha": 0.6,
    "lines.antialiased": True,
})

# ---------------------------------------------------------------------------
# Flight milestones (AeroTech J270W, ascent phase)
# ---------------------------------------------------------------------------
# (time_s, altitude_m, velocity_ms)
T_IGNITION   = 0.00
T_MAXV       = 2.05     # max velocity / max-Q, Mach 0.907
T_BURNOUT    = 2.56
T_APOGEE     = 14.97
T_END        = 18.00

# Lockout thresholds
V_ENGAGE  = 240.0   # m/s  -> engage barometer lockout
V_RELEASE = 200.0   # m/s  -> release barometer lockout

# ---------------------------------------------------------------------------
# Build realistic kinematic curves hitting the exact milestones.
#
# Velocity is modeled as a smooth piecewise curve:
#   - Boost  (0 -> 2.05 s): rising to +307.3 m/s
#   - Over-max to burnout (2.05 -> 2.56 s): slight decay to +279.1 m/s
#   - Coast  (2.56 -> 14.97 s): decelerating under gravity + drag to 0
#   - Descent(14.97 -> 18 s): accelerating downward to about -7.7 m/s
# Altitude is the cumulative integral of velocity, rescaled so that the
# known altitude milestones land exactly on their target values.
# ---------------------------------------------------------------------------
dt = 0.001
t = np.arange(T_IGNITION, T_END + dt, dt)


def smoothstep(x, x0, x1):
    """C1-continuous 0->1 ramp (3x^2 - 2x^3) between x0 and x1."""
    u = np.clip((x - x0) / (x1 - x0), 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


vel = np.zeros_like(t)

# --- Boost: 0 -> V_MAX using a smoothstep (soft start, soft top) ---
V_MAX = 307.3
boost = (t >= T_IGNITION) & (t <= T_MAXV)
vel[boost] = V_MAX * smoothstep(t[boost], T_IGNITION, T_MAXV)

# --- Over-max to burnout: 307.3 -> 279.1 (drag-dominated, thrust tailing) ---
V_BO = 279.1
seg = (t > T_MAXV) & (t <= T_BURNOUT)
vel[seg] = V_MAX + (V_BO - V_MAX) * smoothstep(t[seg], T_MAXV, T_BURNOUT)

# --- Coast: 279.1 -> 0 at apogee. Concave deceleration (drag high early). ---
seg = (t > T_BURNOUT) & (t <= T_APOGEE)
u = (t[seg] - T_BURNOUT) / (T_APOGEE - T_BURNOUT)
# blend of quadratic ease-out (strong early drag) toward zero at apogee
vel[seg] = V_BO * (1.0 - u) * (1.0 - 0.35 * u)

# --- Early descent: 0 -> -7.7 m/s (gravity, pre-drogue) ---
V_DESC = -7.7
seg = t > T_APOGEE
u = (t[seg] - T_APOGEE) / (T_END - T_APOGEE)
vel[seg] = V_DESC * smoothstep(t[seg], T_APOGEE, T_END) * (0.6 + 0.4 * u)

# ---------------------------------------------------------------------------
# Altitude = integral of velocity, piecewise-rescaled to hit milestones.
# ---------------------------------------------------------------------------
alt_raw = np.concatenate([[0.0], np.cumsum(0.5 * (vel[1:] + vel[:-1]) * dt)])


def idx(tt):
    return int(round((tt - T_IGNITION) / dt))


def rescale_segment(a, i0, i1, target_start, target_end):
    """Linearly rescale cumulative integral over [i0,i1] to hit targets."""
    raw0, raw1 = a[i0], a[i1]
    if abs(raw1 - raw0) < 1e-9:
        a[i0:i1 + 1] = np.linspace(target_start, target_end, i1 - i0 + 1)
        return
    scale = (target_end - target_start) / (raw1 - raw0)
    a[i0:i1 + 1] = target_start + (a[i0:i1 + 1] - raw0) * scale


altitude = alt_raw.copy()
# Milestone altitudes: 0 -> 359 (maxv) -> 509 (burnout) -> 1593 (apogee)
rescale_segment(altitude, idx(T_IGNITION), idx(T_MAXV),   0.0,   359.0)
rescale_segment(altitude, idx(T_MAXV),     idx(T_BURNOUT), 359.0, 509.0)
rescale_segment(altitude, idx(T_BURNOUT),  idx(T_APOGEE), 509.0, 1593.0)
# Descent: small altitude loss from apogee
desc_loss = abs(np.trapezoid(vel[idx(T_APOGEE):], dx=dt))
rescale_segment(altitude, idx(T_APOGEE), len(altitude) - 1,
                1593.0, 1593.0 - desc_loss)

altitude_m = altitude          # fused estimator altitude
vel_z_ms = vel                 # true / estimated vertical velocity

# ---------------------------------------------------------------------------
# Barometer lockout window (hysteresis):
#   engage when vel crosses +240 m/s on ascent,
#   release when vel drops below +200 m/s during coast.
# ---------------------------------------------------------------------------
engage_idx = np.argmax(vel_z_ms >= V_ENGAGE)                 # first >= 240
# first index after engage where velocity falls below 200
after = np.arange(len(vel_z_ms)) > engage_idx
release_candidates = np.where(after & (vel_z_ms < V_RELEASE))[0]
release_idx = release_candidates[0] if release_candidates.size else len(t) - 1

t_engage = t[engage_idx]
t_release = t[release_idx]

is_baro_locked_out = np.zeros_like(t, dtype=int)
is_baro_locked_out[engage_idx:release_idx] = 1

# ---------------------------------------------------------------------------
# Raw uncompensated barometric altitude:
#   tracks true altitude at low speed, but suffers a transonic pressure
#   distortion (false dip of ~ -120 m) while Mach > 0.7 (vel > 240 m/s).
# ---------------------------------------------------------------------------
baro_raw = altitude_m.copy()
mach_region = vel_z_ms > V_ENGAGE
if mach_region.any():
    mi = np.where(mach_region)[0]
    m0, m1 = mi[0], mi[-1]
    # Transonic shock on the static port over-pressurizes the sensor, so the
    # derived barometric altitude reads FALSE-HIGH (a +120 m false spike).
    # Unfiltered, this excursion alone would cross a naive apogee detector and
    # fire the ejection charge early -- which is exactly what the lockout
    # (shaded window above) suppresses by ignoring baro while v_z > 240 m/s.
    center = 0.5 * (m0 + m1)
    width = max((m1 - m0) / 2.0, 1.0)
    bump = 120.0 * np.exp(-0.5 * ((np.arange(len(t)) - center) / (0.60 * width)) ** 2)
    envelope = np.zeros_like(t)
    envelope[m0:m1 + 1] = 1.0
    envelope = np.convolve(envelope, np.ones(300) / 300.0, mode="same")
    baro_raw = baro_raw + bump * envelope
# add light measurement noise to the raw trace for realism
rng = np.random.default_rng(59)
baro_raw = baro_raw + rng.normal(0.0, 1.2, size=baro_raw.shape)

# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
fig, (ax_v, ax_a) = plt.subplots(
    2, 1, figsize=(10, 6), sharex=True,
    gridspec_kw={"height_ratios": [1, 1], "hspace": 0.12},
    constrained_layout=False,
)
fig.subplots_adjust(left=0.085, right=0.975, top=0.885, bottom=0.095, hspace=0.30)

NAVY   = "#1f4e78"
RED    = "#c00000"
ORANGE = "#ed7d31"
GREEN  = "#385723"
GRAY   = "#808080"

# --- Top: velocity & lockout hysteresis ---
ax_v.plot(t, vel_z_ms, color=NAVY, lw=1.8, label="Vertical velocity $v_z$")
ax_v.axhline(V_ENGAGE, color=RED, ls="--", lw=1.2,
             label=f"Lockout engage ({V_ENGAGE:.0f} m/s)")
ax_v.axhline(V_RELEASE, color=ORANGE, ls="--", lw=1.2,
             label=f"Lockout release ({V_RELEASE:.0f} m/s)")

ax_v.axvspan(t_engage, t_release, color=RED, alpha=0.15)
# label inside the shaded band
ax_v.text(0.5 * (t_engage + t_release), 270,
          "Barometer Locked Out\n(Inertial Only)",
          ha="center", va="center", fontsize=8.5, color=RED, fontweight="bold")

ax_v.set_ylabel("Vertical velocity [m/s]")
ax_v.set_title("Vertical Velocity & Barometer Lockout Hysteresis")
ax_v.grid(True, which="both", color="#cccccc")
ax_v.set_ylim(-40, 340)
ax_v.axhline(0, color="#999999", lw=0.6)

handles, labels = ax_v.get_legend_handles_labels()
handles.append(Patch(facecolor=RED, alpha=0.15, label="Lockout active window"))
ax_v.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.9)

# --- Bottom: fused altitude vs raw barometric ---
ax_a.plot(t, baro_raw, color=GRAY, ls="--", lw=1.3,
          label="Raw barometric altitude (uncompensated)")
ax_a.plot(t, altitude_m, color=GREEN, lw=1.9,
          label="Fused estimator altitude")

ax_a.set_ylabel("Altitude [m]")
ax_a.set_xlabel("Time [s]")
ax_a.set_title("Altitude Estimation vs. Raw Barometric Pressure Spike")
ax_a.grid(True, which="both", color="#cccccc")
ax_a.set_ylim(-150, 1750)
ax_a.set_xlim(0, T_END)

# Flight-state transition markers
transitions = [
    (0.10,      "BOOST",  10),   # (time, label, x-text-offset px-ish in data units)
    (T_BURNOUT, "COAST",  0.15),
    (T_APOGEE,  "APOGEE", 0.15),
]
ymin, ymax = ax_a.get_ylim()
for tx, name, _dx in transitions:
    for ax in (ax_v, ax_a):
        ax.axvline(tx, color="#404040", ls=":", lw=0.9, alpha=0.8)

# Place callouts just inside the top of the bottom panel, nudging APOGEE
# leftward so it does not clip against the right spine.
label_y = ymax - 60
ax_a.annotate("BOOST", xy=(0.10, label_y), xytext=(0.25, label_y),
              fontsize=8, fontweight="bold", color="#404040",
              ha="left", va="top")
ax_a.annotate("COAST", xy=(T_BURNOUT, label_y), xytext=(T_BURNOUT + 0.2, label_y),
              fontsize=8, fontweight="bold", color="#404040",
              ha="left", va="top")
ax_a.annotate("APOGEE", xy=(T_APOGEE, label_y), xytext=(T_APOGEE - 0.25, label_y),
              fontsize=8, fontweight="bold", color="#404040",
              ha="right", va="top")

ax_a.legend(loc="lower right", fontsize=8, framealpha=0.9)

fig.align_ylabels([ax_v, ax_a])
fig.suptitle(
    "Figure 5.7 — Barometer Lockout Simulation (AeroTech J270W Ascent)",
    fontsize=12, fontweight="bold", y=0.965,
)

OUT = "fig_5_7_baro_lockout_sim.png"
fig.savefig(OUT, dpi=300)
print(f"Saved {OUT}")

# --- Verification summary of milestone accuracy ---
def at(tt):
    i = idx(tt)
    return altitude_m[i], vel_z_ms[i]

print("\nMilestone check (alt_m / vel_ms):")
for tt, lbl in [(T_IGNITION, "ignition"), (T_MAXV, "max-v"),
                (T_BURNOUT, "burnout"), (T_APOGEE, "apogee"),
                (T_END, "t=18 s")]:
    a, v = at(tt)
    print(f"  t={tt:5.2f}s {lbl:9s}: alt={a:7.1f} m  vel={v:7.1f} m/s")
print(f"Lockout window: engage t={t_engage:.3f}s "
      f"(v={vel_z_ms[engage_idx]:.1f}) -> release t={t_release:.3f}s "
      f"(v={vel_z_ms[release_idx]:.1f})")
