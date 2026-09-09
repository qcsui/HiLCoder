"""
Switching-level Buck converter simulation — 50 ns step.
Matching the paper's Asynchronous Buck parameters.
Forward Euler vs Backward Euler + DCM zero-current clamping.
"""

import numpy as np
import matplotlib.pyplot as plt


# ── Buck converter parameters (from paper) ──
Vin    = 24.0       # V
L      = 100.0e-6   # H
C      = 100.0e-6   # F
R      = 10.0       # Ohm
fsw    = 20e3       # Hz
D      = 0.5        # duty cycle → Vout_ideal = 12V
Tsw    = 1.0 / fsw  # 50 us
h      = 50e-9      # 50 ns step (matching paper)
t_end  = 10e-3      # 10 ms (~200 switching cycles)

n_steps = int(t_end / h)
n_sw    = int(Tsw / h)

Vout_ideal = D * Vin

print(f"{'Buck Converter Switching-Level Simulation':=^60}")
print(f"Vin={Vin}V, L={L*1e6:.0f}uH, C={C*1e6:.0f}uF, R={R}Ω")
print(f"fsw={fsw/1e3:.0f}kHz, D={D}, Vout_ideal={Vout_ideal}V")
print(f"Step h={h*1e9:.0f}ns, Tsw={Tsw*1e6:.1f}us, steps/period={n_sw}")
print()

# ── Continuous-time state matrices ──
# Mode A: S=ON,  D=OFF  (diL/dt = (Vin - vC)/L,  dvC/dt = (iL - vC/R)/C)
AA = np.array([[0.0,      -1.0/L],
               [1.0/C, -1.0/(R*C)]])
BA = np.array([[1.0/L],
               [0.0]])

# Mode B: S=OFF, D=ON   (diL/dt = -vC/L,         dvC/dt = (iL - vC/R)/C)
AB = np.array([[0.0,      -1.0/L],
               [1.0/C, -1.0/(R*C)]])
BB = np.array([[0.0],     # Vin disconnected
               [0.0]])

# Mode C: DCM (iL=0)    (diL/dt = 0,            dvC/dt = -vC/RC)
AC = np.array([[0.0, 0.0],
               [0.0, -1.0/(R*C)]])
BC = np.array([[0.0],
               [0.0]])

u = np.array([[Vin]])


def discretize_fe(A, B, h):
    return np.eye(2) + h * A, h * B


def discretize_be(A, B, h):
    I = np.eye(2)
    inv = np.linalg.inv(I - h * A)
    return inv, inv @ (h * B)


# ── Precompute discrete matrices (offline) ──
print("Precomputing discrete state matrices ...")
mats = {}
for name, disc in [("fe", discretize_fe), ("be", discretize_be)]:
    mats[name] = {}
    for lbl, A, B in [("on", AA, BA), ("off", AB, BB), ("dcm", AC, BC)]:
        Ad, Bd = disc(A, B, h)
        mats[name][lbl] = (Ad, Bd)

# Print numerical matrices for reference
for name in ["fe", "be"]:
    print(f"\n  {name.upper()}:")
    for lbl in ["on", "off", "dcm"]:
        Ad, Bd = mats[name][lbl]
        print(f"    {lbl}: Ad = {Ad[0]}  /  {Ad[1]}")


def select_mode(S_on, iL_prev):
    if S_on:
        return "on"             # S=ON, D=OFF
    else:
        if iL_prev > 0:
            return "off"        # S=OFF, D=ON (freewheel)
        else:
            return "dcm"        # DCM: iL clamped to 0


def simulate(mats, h):
    x = np.zeros((2, n_steps))

    for k in range(n_steps - 1):
        t_now = k * h
        S_on = (t_now % Tsw) < (D * Tsw)

        mode = select_mode(S_on, x[0, k])
        Ad, Bd = mats[mode]

        x_next = Ad @ x[:, k] + Bd @ u[:, 0]

        if mode == "dcm":
            x_next[0] = 0.0

        x[:, k + 1] = x_next

    t = np.arange(n_steps) * h
    return t, x[0, :], x[1, :]


# ── Run ──
print("\nRunning Forward Euler ...")
t_fe, iL_fe, vC_fe = simulate(mats["fe"], h)
print("Running Backward Euler ...")
t_be, iL_be, vC_be = simulate(mats["be"], h)

# ── Steady-state analysis (last switching cycle) ──
ss_start = t_end - Tsw
ss_idx   = int(ss_start / h)

Vout_fe = np.mean(vC_fe[ss_idx:])
Vout_be = np.mean(vC_be[ss_idx:])
iL_fe_mean = np.mean(iL_fe[ss_idx:])
iL_be_mean = np.mean(iL_be[ss_idx:])
iL_fe_ripple = np.max(iL_fe[ss_idx:]) - np.min(iL_fe[ss_idx:])
iL_be_ripple = np.max(iL_be[ss_idx:]) - np.min(iL_be[ss_idx:])

print()
print(f"{'':>25} {'Forward Euler':>16} {'Backward Euler':>16}  {'Ideal':>10}")
print(f"{'Vout mean [V]':>25} {Vout_fe:>16.4f} {Vout_be:>16.4f}  {Vout_ideal:>10.2f}")
print(f"{'iL mean [A]':>25} {iL_fe_mean:>16.4f} {iL_be_mean:>16.4f}  {Vout_ideal/R:>10.2f}")
print(f"{'iL ripple [A]':>25} {iL_fe_ripple:>16.4f} {iL_be_ripple:>16.4f}")

# Check for DCM
dcm_cycles_fe = 0
for k in range(ss_idx, n_steps - 1):
    t_now = k * h
    S_on = (t_now % Tsw) < (D * Tsw)
    mode = select_mode(S_on, iL_fe[k])
    if mode == "dcm":
        dcm_cycles_fe += 1
print(f"{'DCM steps (FE, last cycle)':>25} {dcm_cycles_fe:>16d}")

# ── Plots ──

# 1) Vout full transient
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t_fe * 1e6, vC_fe, label="Forward Euler", linewidth=0.8)
ax.plot(t_be * 1e6, vC_be, "--", label="Backward Euler", linewidth=0.8, alpha=0.8)
ax.axhline(Vout_ideal, linestyle=":", color="gray", label=f"Ideal {Vout_ideal:.1f}V")
ax.set_xlabel("Time [μs]")
ax.set_ylabel("Output voltage [V]")
ax.set_title(f"Buck Converter — Switching-Level (h = {h*1e9:.0f} ns)")
ax.legend()
ax.grid(True)
fig.tight_layout()
fig.savefig("buck_sw_vout.png", dpi=150)
print("Saved: buck_sw_vout.png")

# 2) iL full transient
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t_fe * 1e6, iL_fe, label="Forward Euler", linewidth=0.6)
ax.plot(t_be * 1e6, iL_be, "--", label="Backward Euler", linewidth=0.6, alpha=0.8)
ax.set_xlabel("Time [μs]")
ax.set_ylabel("Inductor current [A]")
ax.set_title("Buck Converter Inductor Current")
ax.legend()
ax.grid(True)
fig.tight_layout()
fig.savefig("buck_sw_il.png", dpi=150)
print("Saved: buck_sw_il.png")

# 3) Steady-state zoom (last 4 cycles)
zoom_start = t_end - 4 * Tsw
zoom_idx = int(zoom_start / h)
t_zoom = t_fe[zoom_idx:] * 1e6

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(t_zoom, vC_fe[zoom_idx:], label="Forward Euler", linewidth=0.8)
ax1.plot(t_zoom, vC_be[zoom_idx:], "--", label="Backward Euler", linewidth=0.8, alpha=0.8)
ax1.axhline(Vout_ideal, linestyle=":", color="gray", alpha=0.5)
ax1.set_ylabel("Vout [V]")
ax1.set_title(f"Buck Steady-State Zoom (last 4 cycles, h={h*1e9:.0f}ns)")
ax1.legend()
ax1.grid(True)

ax2.plot(t_zoom, iL_fe[zoom_idx:], label="Forward Euler", linewidth=0.8)
ax2.plot(t_zoom, iL_be[zoom_idx:], "--", label="Backward Euler", linewidth=0.8, alpha=0.8)
ax2.set_xlabel("Time [μs]")
ax2.set_ylabel("iL [A]")
ax2.legend()
ax2.grid(True)

fig.tight_layout()
fig.savefig("buck_sw_zoom.png", dpi=150)
print("Saved: buck_sw_zoom.png")

# 4) Difference (FE - BE)
fig, ax = plt.subplots(figsize=(10, 3))
diff_vC = vC_fe - vC_be
diff_iL = iL_fe - iL_be
ax.plot(t_fe * 1e6, diff_vC, label="ΔVout (FE - BE)", alpha=0.8)
ax.plot(t_fe * 1e6, diff_iL * 10, label="ΔiL ×10 (FE - BE)", alpha=0.8)
ax.axhline(0, linestyle=":", color="gray")
ax.set_xlabel("Time [μs]")
ax.set_ylabel("Difference")
ax.set_title("Numerical Difference: FE − BE")
ax.legend()
ax.grid(True)
fig.tight_layout()
fig.savefig("buck_sw_diff.png", dpi=150)
print("Saved: buck_sw_diff.png")

plt.close("all")
print("\nAll plots saved.")

# ── FPGA timing ──
t_calc_fe = 10e-9
t_calc_be = 15e-9
print(f"\nFPGA timing (Δt = {h*1e9:.0f} ns):")
print(f"  FE: t_calc ≈ {t_calc_fe*1e9:.0f} ns  →  headroom {(h - t_calc_fe)*1e9:.0f} ns")
print(f"  BE: t_calc ≈ {t_calc_be*1e9:.0f} ns  →  headroom {(h - t_calc_be)*1e9:.0f} ns")
