"""
Boost converter — ADC + MNA method
====================================
Follows Gao Fei's approach:
  1. Discretize L/C → Norton companion circuits  (first)
  2. Stamp companions into nodal admittance matrix Y
  3. Solve Y·v = i at each 50 ns step
  4. Update history sources

Comparison against State-Space method for validation.
"""

import numpy as np
import matplotlib.pyplot as plt

# ── Parameters ──
Vin    = 100.0
L      = 1.0e-3
C      = 470.0e-6
R      = 20.0
fsw    = 20e3
D      = 0.5
Tsw    = 1.0 / fsw
h      = 50e-9
t_end  = 60e-3

n_steps = int(t_end / h)
n_sw    = int(Tsw / h)

Vout_ideal = Vin / (1.0 - D)
iL_ideal   = Vout_ideal / (R * (1.0 - D))

print(f"{'Boost — ADC + MNA Simulation':=^60}")
print(f"Vin={Vin}V,  L={L*1e3:.2f}mH,  C={C*1e6:.0f}uF,  R={R}" + chr(937))
print(f"fsw={fsw/1e3:.0f}kHz  D={D}  h={h*1e9:.0f}ns  Tsw={Tsw*1e6:.1f}us")
print()

# ═══════════════════════════════════════════════════════════
# Step 1: Discretize L and C → companion circuit parameters
# ═══════════════════════════════════════════════════════════

G_L = h / L          # Norton conductance for inductor companion
G_C = C / h          # Norton conductance for capacitor companion
G_R = 1.0 / R        # Load conductance

print("Companion circuit parameters (Backward Euler):")
print(f"  G_L = h/L  = {G_L:.6e} S    (inductor)")
print(f"  G_C = C/h  = {G_C:.6e} S    (capacitor)")
print(f"  G_R = 1/R  = {G_R:.6e} S    (load)")
print()

# Switch & diode conductances (binary resistor model)
G_Son  = 1e6    # S ON  — near short
G_Soff = 1e-6   # S OFF — near open
G_Don  = 1e6    # D ON  — near short
G_Doff = 1e-6   # D OFF — near open

# ═══════════════════════════════════════════════════════════
# Step 2: Pre-assemble MNA matrix building blocks
# ═══════════════════════════════════════════════════════════
#
# Circuit nodes:
#   0 — GND (reference)
#   1 — node_a  (L_out / S_drain / D_anode)
#   2 — node_b  (D_cathode / C_top / R_top)
#
# Components and their stamps:
#   L companion:  G_L  between Vin(+) and node_a
#                 J_L  = iL[k]  (flows Vin → node_a)
#   C companion:  G_C  between node_b and GND
#                 J_C  = G_C * vC[k]  (flows GND → node_b)
#   R:            G_R  between node_b and GND
#   S (switch):   G_S  between node_a and GND
#   D (diode):    G_D  between node_a and node_b

def build_system(G_S, G_D, J_L, J_C):
    """
    Build Y matrix (2x2) and I vector (2x1) for MNA.

    KCL at node_a:
        G_L*(v_a - Vin) + J_L + G_S*v_a + G_D*(v_a - v_b) = 0
        (G_L + G_S + G_D)*v_a - G_D*v_b = G_L*Vin - J_L

    KCL at node_b:
        G_D*(v_b - v_a) + G_C*v_b - J_C + G_R*v_b = 0
        -G_D*v_a + (G_D + G_C + G_R)*v_b = J_C

    Note sign: J_L = iL[k]  flows INTO node_a from L's + terminal,
               so in KCL at node_a it appears as -J_L (leaving).
    """
    Y = np.array([
        [G_L + G_S + G_D,     -G_D          ],
        [-G_D,              G_D + G_C + G_R ]
    ])
    # KCL at node_a: (G_L+G_S+G_D)*v_a - G_D*v_b = G_L*Vin + J_L
    # KCL at node_b: -G_D*v_a + (G_D+G_C+G_R)*v_b = J_C
    I = np.array([
        G_L * Vin + J_L,
        J_C
    ])
    return Y, I


# ═══════════════════════════════════════════════════════════
# Step 3: Online simulation loop
# ═══════════════════════════════════════════════════════════

v_a = np.zeros(n_steps)
v_b = np.zeros(n_steps)
i_L = np.zeros(n_steps)

# History sources (initial conditions = 0)
J_L = 0.0   # = iL[k]
J_C = 0.0   # = G_C * vC[k] = G_C * v_b[k]
vC_prev = 0.0

print("Running ADC + MNA simulation ...")

for k in range(n_steps - 1):
    t_now = k * h

    # --- Determine switch states ---
    S_on = (t_now % Tsw) < (D * Tsw)
    G_S = G_Son if S_on else G_Soff

    # Diode state for Boost topology:
    #   - S=ON:  D is always reverse biased (anode ~ GND)
    #   - S=OFF: D conducts if inductor has current that needs a path
    #            (iL > 0 implies D is forced on by inductor action)
    if S_on:
        D_on = False
    else:
        D_on = (i_L[k] > 0)

    G_D = G_Don if D_on else G_Doff

    # --- Build and solve MNA ---
    Y, I = build_system(G_S, G_D, J_L, J_C)
    v = np.linalg.solve(Y, I)

    v_a[k + 1] = v[0]
    v_b[k + 1] = v[1]

    # --- Compute inductor current ---
    i_L[k + 1] = G_L * (Vin - v_a[k + 1]) + J_L

    # --- DCM clamp ---
    # If diode would be reverse-biased but current went negative,
    # force DCM: iL = 0, v_a = Vin (no current through L)
    if (not S_on) and (i_L[k + 1] <= 0):
        # Re-solve with D off and iL clamped to 0
        # With iL = 0: L is effectively open → v_a = Vin
        # Only node_b needs solving: G_C*v_b + G_R*v_b = J_C
        v_a[k + 1] = Vin
        v_b[k + 1] = J_C / (G_C + G_R)
        i_L[k + 1] = 0.0

    # --- Update history sources for next step ---
    J_L = i_L[k + 1]                # history source for L companion
    vC_prev = v_b[k + 1]
    J_C = G_C * vC_prev             # history source for C companion

t = np.arange(n_steps) * h

# ═══════════════════════════════════════════════════════════
# Analysis
# ═══════════════════════════════════════════════════════════

ss_start = t_end - Tsw
ss_idx   = int(ss_start / h)

Vout_mna = np.mean(v_b[ss_idx:])
iL_mna   = np.mean(i_L[ss_idx:])
iL_rip   = np.max(i_L[ss_idx:]) - np.min(i_L[ss_idx:])

print(f"\n{'':>25} {'ADC+MNA':>16}  {'Ideal':>10}")
print(f"{'Vout mean [V]':>25} {Vout_mna:>16.4f}  {Vout_ideal:>10.2f}")
print(f"{'iL mean [A]':>25} {iL_mna:>16.4f}  {iL_ideal:>10.2f}")
print(f"{'iL ripple [A]':>25} {iL_rip:>16.4f}")

# ═══════════════════════════════════════════════════════════
# Plots
# ═══════════════════════════════════════════════════════════

t_ms = t * 1e3
t_us = t * 1e6

fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t_ms, v_b, label="ADC+MNA", linewidth=0.8)
ax.axhline(Vout_ideal, linestyle=":", color="gray",
           label=f"Ideal {Vout_ideal:.0f}V")
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Output voltage [V]")
ax.set_title(f"Boost — ADC + MNA  (h={h*1e9:.0f}ns)")
ax.legend()
ax.grid(True)
fig.tight_layout()
fig.savefig("boost_mna_vout.png", dpi=150)
print("\nSaved: boost_mna_vout.png")

fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t_ms, i_L, label="ADC+MNA", linewidth=0.6)
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Inductor current [A]")
ax.set_title("Boost — Inductor Current (ADC+MNA)")
ax.legend()
ax.grid(True)
fig.tight_layout()
fig.savefig("boost_mna_il.png", dpi=150)
print("Saved: boost_mna_il.png")

# Steady-state zoom (last 4 cycles)
zoom_start = t_end - 4 * Tsw
zoom_idx = int(zoom_start / h)
t_zoom = t[zoom_idx:] * 1e6

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax1.plot(t_zoom, v_b[zoom_idx:], linewidth=0.8)
ax1.axhline(Vout_ideal, linestyle=":", color="gray", alpha=0.5)
ax1.set_ylabel("Vout [V]")
ax1.set_title(f"Boost ADC+MNA — Steady-State Zoom (h={h*1e9:.0f}ns)")
ax1.grid(True)

ax2.plot(t_zoom, i_L[zoom_idx:], linewidth=0.8)
ax2.set_xlabel("Time [μs]")
ax2.set_ylabel("iL [A]")
ax2.grid(True)
fig.tight_layout()
fig.savefig("boost_mna_zoom.png", dpi=150)
print("Saved: boost_mna_zoom.png")

plt.close("all")
print("\nDone.")
