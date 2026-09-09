"""
Switching-level Boost converter simulation — 50 ns step.
Compares Forward Euler vs Backward Euler discretization
as used in FPGA-based real-time HIL solvers.
"""

import numpy as np
import matplotlib.pyplot as plt


# ── Boost converter parameters ──
Vin    = 100.0      # V
L      = 1.0e-3     # H
C      = 470.0e-6   # F
R      = 20.0       # Ohm
fsw    = 20e3       # Hz
D      = 0.5        # duty cycle
Tsw    = 1.0 / fsw  # 50 us
h      = 50e-9      # 50 ns step
t_end  = 60e-3      # 60 ms (~1200 switching cycles)

n_steps = int(t_end / h)
n_sw    = int(Tsw / h)  # steps per switching period (1000)

print(f"Switching period Tsw = {Tsw*1e6:.2f} us")
print(f"Step size       h    = {h*1e9:.0f} ns")
print(f"Steps/period         = {n_sw}")
print(f"Total steps          = {n_steps}")
print()

# ── Continuous-time state matrices for each switch state ──
# State 0: S=ON,  D=OFF  (diL/dt = Vin/L,  dvC/dt = -vC/RC)
A0 = np.array([[0.0, 0.0],
               [0.0, -1.0/(R*C)]])
B0 = np.array([[1.0/L],
               [0.0]])

# State 1: S=OFF, D=ON   (diL/dt = (Vin - vC)/L,  dvC/dt = (iL - vC/R)/C)
A1 = np.array([[0.0,      -1.0/L],
               [1.0/C, -1.0/(R*C)]])
B1 = np.array([[1.0/L],
               [0.0]])

# State 2: DCM (iL=0)    (diL/dt = 0,  dvC/dt = -vC/RC)
A2 = np.array([[0.0, 0.0],
               [0.0, -1.0/(R*C)]])
B2 = np.array([[0.0],
               [0.0]])

u = np.array([[Vin]])


def discretize_fe(A, B, h):
    """Forward Euler: Ad = I + h*A,  Bd = h*B"""
    return np.eye(2) + h * A, h * B


def discretize_be(A, B, h):
    """Backward Euler: Ad = (I - h*A)^{-1},  Bd = (I - h*A)^{-1} * h*B"""
    I = np.eye(2)
    inv = np.linalg.inv(I - h * A)
    return inv, inv @ (h * B)


# ── Precompute all discrete matrices (offline) ──
print("Precomputing discrete state matrices ...")
mats = {}
for name, disc in [("fe", discretize_fe), ("be", discretize_be)]:
    mats[name] = {}
    for label, A, B in [("sw0", A0, B0), ("sw1", A1, B1), ("dcm", A2, B2)]:
        Ad, Bd = disc(A, B, h)
        mats[name][label] = (Ad, Bd)

print("  Forward Euler: 3 matrices x 2x2  (online = multiply only)")
print("  Backward Euler: 3 matrices x 2x2  (online = multiply only)")
print()


def select_mode(S_on, iL_prev):
    """Determine the operating mode based on switch state and inductor current."""
    if S_on:
        return "sw0"            # S=ON, D=OFF
    else:
        if iL_prev > 0:
            return "sw1"        # S=OFF, D=ON  (CCM)
        else:
            return "dcm"        # S=OFF, D=OFF (DCM: iL clamped to zero)


def simulate(mats, h):
    """Run switching-level simulation with precomputed discrete matrices."""
    iL = np.zeros(n_steps)
    vC = np.zeros(n_steps)
    x  = np.zeros((2, n_steps))

    for k in range(n_steps - 1):
        t_now = k * h
        # PWM generation
        S_on = (t_now % Tsw) < (D * Tsw)

        mode = select_mode(S_on, x[0, k])
        Ad, Bd = mats[mode]

        x_next = Ad @ x[:, k] + Bd @ u[:, 0]

        # DCM clamp
        if mode == "dcm":
            x_next[0] = 0.0  # force iL = 0

        x[:, k + 1] = x_next
        iL[k + 1] = x[0, k + 1]
        vC[k + 1] = x[1, k + 1]

    t = np.arange(n_steps) * h
    return t, iL, vC


# ── Run both methods ──
print("Running Forward Euler ...")
t_fe, iL_fe, vC_fe = simulate(mats["fe"], h)
print("Running Backward Euler ...")
t_be, iL_be, vC_be = simulate(mats["be"], h)

# ── Steady-state analysis (last switching cycle only) ──
# Use the final complete switching period to avoid contamination
# from LC resonant envelope oscillation.
ss_start = t_end - Tsw
ss_idx  = int(ss_start / h)

Vout_fe = np.mean(vC_fe[ss_idx:])
Vout_be = np.mean(vC_be[ss_idx:])
Vout_ideal = Vin / (1.0 - D)

iL_fe_mean = np.mean(iL_fe[ss_idx:])
iL_be_mean = np.mean(iL_be[ss_idx:])
iL_fe_ripple = np.max(iL_fe[ss_idx:]) - np.min(iL_fe[ss_idx:])
iL_be_ripple = np.max(iL_be[ss_idx:]) - np.min(iL_be[ss_idx:])

# ── Convergence check: compare last vs second-to-last cycle ──
ss_prev = int((t_end - 2 * Tsw) / h)
iL_fe_cycle1 = np.mean(iL_fe[ss_prev:ss_idx])
iL_fe_cycle2 = iL_fe_mean
iL_drift = abs(iL_fe_cycle2 - iL_fe_cycle1)

print(f"{'':>25} {'Forward Euler':>16} {'Backward Euler':>16}  {'Ideal':>10}")
print(f"{'Vout mean [V]':>25} {Vout_fe:>16.4f} {Vout_be:>16.4f}  {Vout_ideal:>10.2f}")
print(f"{'iL mean [A]':>25} {iL_fe_mean:>16.4f} {iL_be_mean:>16.4f}  {Vout_ideal/R/(1-D):>10.2f}")
print(f"{'iL ripple [A]':>25} {iL_fe_ripple:>16.4f} {iL_be_ripple:>16.4f}  {Vin*D*Tsw/L:>10.4f}")
print(f"{'iL drift/cycle [mA]':>25} {iL_drift*1e3:>16.4f}")
print()

# ── FPGA timing headroom ──
t_calc_fe = 10e-9  # assumed
t_calc_be = 15e-9  # BE needs matrix solve → slightly more
print(f"FPGA timing budget (Δt = {h*1e9:.0f} ns):")
print(f"  FE: t_calc ≈ {t_calc_fe*1e9:.0f} ns  →  headroom = {(h - t_calc_fe)*1e9:.0f} ns")
print(f"  BE: t_calc ≈ {t_calc_be*1e9:.0f} ns  →  headroom = {(h - t_calc_be)*1e9:.0f} ns")
print()

# ── Plots ──

# 1) Vout transient (full)
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t_fe * 1e6, vC_fe, label="Forward Euler")
ax.plot(t_be * 1e6, vC_be, "--", label="Backward Euler", alpha=0.8)
ax.axhline(Vout_ideal, linestyle=":", color="gray", label=f"Ideal {Vout_ideal:.0f}V")
ax.set_xlabel("Time [μs]")
ax.set_ylabel("Output voltage [V]")
ax.set_title("Boost Converter — Switching-Level (50 ns step)")
ax.legend()
ax.grid(True)
fig.tight_layout()
fig.savefig("boost_sw_vout.png", dpi=150)
print("Saved: boost_sw_vout.png")

# 2) iL transient (full)
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t_fe * 1e6, iL_fe, label="Forward Euler", linewidth=0.6)
ax.plot(t_be * 1e6, iL_be, "--", label="Backward Euler", linewidth=0.6, alpha=0.8)
ax.set_xlabel("Time [μs]")
ax.set_ylabel("Inductor current [A]")
ax.set_title("Boost Converter Inductor Current — Switching-Level (50 ns step)")
ax.legend()
ax.grid(True)
fig.tight_layout()
fig.savefig("boost_sw_il.png", dpi=150)
print("Saved: boost_sw_il.png")

# 3) Steady-state zoom (last 2 switching cycles)
zoom_start = t_end - 4 * Tsw
zoom_idx = int(zoom_start / h)
t_zoom = t_fe[zoom_idx:] * 1e6

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(t_zoom, vC_fe[zoom_idx:], label="Forward Euler")
ax1.plot(t_zoom, vC_be[zoom_idx:], "--", label="Backward Euler", alpha=0.8)
ax1.axhline(Vout_ideal, linestyle=":", color="gray", alpha=0.5)
ax1.set_ylabel("Vout [V]")
ax1.set_title(f"Steady-State Zoom (last 2 switching cycles, h = {h*1e9:.0f} ns)")
ax1.legend()
ax1.grid(True)

ax2.plot(t_zoom, iL_fe[zoom_idx:], label="Forward Euler", linewidth=0.8)
ax2.plot(t_zoom, iL_be[zoom_idx:], "--", label="Backward Euler", linewidth=0.8, alpha=0.8)
ax2.set_xlabel("Time [μs]")
ax2.set_ylabel("iL [A]")
ax2.legend()
ax2.grid(True)

fig.tight_layout()
fig.savefig("boost_sw_zoom.png", dpi=150)
print("Saved: boost_sw_zoom.png")

# 4) Difference (FE - BE)
fig, ax = plt.subplots(figsize=(10, 3))
diff_vC = vC_fe - vC_be
diff_iL = iL_fe - iL_be
ax.plot(t_fe * 1e6, diff_vC, label="ΔVout (FE - BE)", alpha=0.8)
ax.plot(t_fe * 1e6, diff_iL * 10, label="ΔiL ×10 (FE - BE)", alpha=0.8)
ax.axhline(0, linestyle=":", color="gray")
ax.set_xlabel("Time [μs]")
ax.set_ylabel("Difference")
ax.set_title("Numerical Difference: Forward Euler − Backward Euler")
ax.legend()
ax.grid(True)
fig.tight_layout()
fig.savefig("boost_sw_diff.png", dpi=150)
print("Saved: boost_sw_diff.png")

plt.close("all")
print("\nAll plots saved.")
