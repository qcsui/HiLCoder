"""
golden_ref.py — Agent 1: Physics-Aware Modeling Agent
Generates fixed-point golden reference for State-Space and MNA routes.

Usage:
  python golden_ref.py --topology Buck --Vin 24 --L 100e-6 \\
      --C 100e-6 --R 10 --fsw 20e3 --D 0.5 --h 50e-9
"""

import argparse
import json
import numpy as np
from pathlib import Path

# ── Supported topologies ──
# Each topology defines (state_on, state_off, state_dcm) matrices
# All in continuous-time, 2-state: x = [iL, vC]^T, u = [Vin]

def topology_Buck(Vin, L, C, R):
    """Buck converter state-space matrices."""
    A_on  = np.array([[0,     -1/L ],
                      [1/C, -1/(R*C)]])
    B_on  = np.array([[1/L], [0]])
    # OFF mode: same A, Vin disconnected
    A_off = A_on.copy()
    B_off = np.array([[0], [0]])
    # DCM: iL clamped
    A_dcm = np.array([[0,      0   ],
                      [0, -1/(R*C)]])
    B_dcm = np.array([[0], [0]])
    return A_on, B_on, A_off, B_off, A_dcm, B_dcm, {
        "icc": lambda Vout: Vout / R,          # avg inductor current (CCM)
        "vout": lambda D: D * Vin,             # output voltage (CCM)
        "vout_dcm": lambda D, Tsw: (
            Vin * 2 / (1 + np.sqrt(1 + 8*L/(R*D*D*Tsw)))
        ),
    }

def topology_Boost(Vin, L, C, R):
    """Boost converter state-space matrices."""
    A_on  = np.array([[0,      0   ],
                      [0, -1/(R*C)]])
    B_on  = np.array([[1/L], [0]])
    A_off = np.array([[0,     -1/L ],
                      [1/C, -1/(R*C)]])
    B_off = np.array([[1/L], [0]])
    A_dcm = np.array([[0,      0   ],
                      [0, -1/(R*C)]])
    B_dcm = np.array([[0], [0]])
    return A_on, B_on, A_off, B_off, A_dcm, B_dcm, {
        "icc": lambda Vout: Vout / (R * (1 - 0.5)),
        "vout": lambda D: Vin / (1 - D),
        "vout_dcm": None,
    }

TOPOLOGIES = {"Buck": topology_Buck, "Boost": topology_Boost}


# ── Discretization ──

def discretize_fe(A, B, h):
    """Forward Euler: Ad = I + h*A, Bd = h*B"""
    return np.eye(2) + h * A, h * B

def discretize_be(A, B, h):
    """Backward Euler: Ad = (I - h*A)^{-1}, Bd = Ad * (h*B)"""
    inv = np.linalg.inv(np.eye(2) - h * A)
    B_h = h * B
    return inv, inv @ B_h


# ── Q-format allocation ──

def allocate_q(v_max, total_bits=24):
    """Determine Q-format from expected maximum value."""
    from math import ceil, log2
    int_bits = max(2, ceil(log2(abs(v_max) + 1)) + 1)  # +1 for sign
    frac_bits = total_bits - int_bits
    if frac_bits < 4:
        frac_bits = 4
        int_bits = total_bits - frac_bits
    return int_bits, frac_bits


# ── MNA companion circuits ──

def mna_companions(L, C, R, h, G_Son=1e6, G_Soff=1e-6, G_Don=1e6, G_Doff=1e-6):
    """
    Build ADC+MNA matrices for 2-node topology.
    Returns Y_on, Y_off, Y_dcm matrices and history source update rules.
    """
    G_L = h / L
    G_C = C / h
    G_R = 1.0 / R

    def Y_matrix(G_S, G_D):
        return np.array([
            [G_L + G_S + G_D, -G_D          ],
            [-G_D,             G_D + G_C + G_R]
        ])

    Y_on  = Y_matrix(G_Son, G_Doff)  # S=ON, D=OFF
    Y_off = Y_matrix(G_Soff, G_Don)  # S=OFF, D=ON
    Y_dcm = Y_matrix(G_Soff, G_Doff)  # both off

    return {
        "G_L": G_L, "G_C": G_C, "G_R": G_R,
        "Y_on": Y_on.tolist(), "Y_off": Y_off.tolist(), "Y_dcm": Y_dcm.tolist(),
        "G_Son": G_Son, "G_Soff": G_Soff, "G_Don": G_Don, "G_Doff": G_Doff,
    }


# ── Main generator ──

def generate(topology, Vin, L, C, R, fsw, D, h):
    """Generate golden reference and spec."""
    Tsw = 1.0 / fsw
    Vout_ideal = TOPOLOGIES[topology](Vin, L, C, R)[-1]["vout"](D)

    # Variable range analysis (conservative for DCM startup overshoot)
    if "Buck" in topology:
        Vmax = Vin * 1.2  # Buck can overshoot to nearly Vin during startup
        # Inrush: iL builds up almost linearly at Vin/L during startup
        Imax = max((Vin-Vout_ideal)*D*Tsw/L*2,  # steady-state ripple
                    Vin*Tsw/L * 3)             # startup inrush over ~3 cycles
    else:
        Vmax = max(Vin/(1-D), Vin) * 1.5
        Imax = Vout_ideal / (R*(1-D)) * 3

    q_int, q_frac = allocate_q(max(Vmax, Imax))
    SCALE = 1 << q_frac

    # State-Space matrices
    A_on, B_on, A_off, B_off, A_dcm, B_dcm, info = TOPOLOGIES[topology](Vin, L, C, R)

    spec = {
        "topology": topology,
        "params": {"Vin": Vin, "L": L, "C": C, "R": R, "fsw": fsw, "D": D, "h": h},
        "ideal": {"Vout": Vout_ideal, "Icc": info["icc"](Vout_ideal)},
        "q_format": {"int": q_int, "frac": q_frac, "total": q_int + q_frac, "scale": SCALE},
        "routes": {
            "ss_fe": {
                "Ad_on":  discretize_fe(A_on, B_on, h)[0].tolist(),
                "Bd_on":  discretize_fe(A_on, B_on, h)[1].tolist(),
                "Ad_off": discretize_fe(A_off, B_off, h)[0].tolist(),
                "Bd_off": discretize_fe(A_off, B_off, h)[1].tolist(),
                "Ad_dcm": discretize_fe(A_dcm, B_dcm, h)[0].tolist(),
                "Bd_dcm": discretize_fe(A_dcm, B_dcm, h)[1].tolist(),
            },
            "ss_be": {
                "Ad_on":  discretize_be(A_on, B_on, h)[0].tolist(),
                "Bd_on":  discretize_be(A_on, B_on, h)[1].tolist(),
                "Ad_off": discretize_be(A_off, B_off, h)[0].tolist(),
                "Bd_off": discretize_be(A_off, B_off, h)[1].tolist(),
                "Ad_dcm": discretize_be(A_dcm, B_dcm, h)[0].tolist(),
                "Bd_dcm": discretize_be(A_dcm, B_dcm, h)[1].tolist(),
            },
            "mna": mna_companions(L, C, R, h),
        },
    }

    # Write spec.json
    out_dir = Path.cwd() / "results"
    out_dir.mkdir(exist_ok=True)
    with open(out_dir / "spec.json", "w") as f:
        json.dump(spec, f, indent=2, cls=NumPyEncoder)

    # Write model_fixed.py
    write_model_fixed(out_dir, spec, topology)
    print(f"[golden_ref] Generated {topology} golden reference (Q{q_int}.{q_frac}, {q_int+q_frac}-bit)")
    print(f"             Vout_ideal = {Vout_ideal:.2f} V")
    print(f"             SCALE = {SCALE}")
    print(f"             Output: results/spec.json, results/model_fixed.py")


class NumPyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, (np.integer, np.floating)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)


def write_model_fixed(out_dir, spec, topology):
    """Write the Golden Reference Python file."""
    q = spec["q_format"]
    SCALE = q["scale"]
    params = spec["params"]
    route = spec["routes"]["ss_be"]  # use BE as primary

    lines = f'''"""
model_fixed.py — Auto-generated by golden_ref.py (Agent 1)
Topology: {topology}
Q-format: Q{q["int"]}.{q["frac"]} ({q["total"]}-bit, SCALE={SCALE})
"""

import numpy as np

# ── Parameters ──
Vin = {params["Vin"]}
L   = {params["L"]}
C   = {params["C"]}
R   = {params["R"]}
fsw = {params["fsw"]}
D   = {params["D"]}
h   = {params["h"]}

# ── Q-format ──
INT_BITS  = {q["int"]}
FRAC_BITS = {q["frac"]}
W         = {q["total"]}
SCALE     = {SCALE}
QMAX      = (1 << (W-1)) - 1
QMIN      = -(1 << (W-1))

def q(x):
    """Float → Q-format integer (round, saturate)."""
    v = int(round(float(x) * SCALE))
    return max(min(v, QMAX), QMIN)

def qf(x):
    """De-quantize Q-format integer → float."""
    return float(x) / SCALE

# ── Discrete matrices (State-Space + Backward Euler) ──
Ad_on  = np.array({json.dumps(route["Ad_on"])})
Bd_on  = np.array({json.dumps(route["Bd_on"])})
Ad_off = np.array({json.dumps(route["Ad_off"])})
Bd_off = np.array({json.dumps(route["Bd_off"])})
Ad_dcm = np.array({json.dumps(route["Ad_dcm"])})
Bd_dcm = np.array({json.dumps(route["Bd_dcm"])})

def step_ss_be(iL, vC, pwm_on):
    """Single step: State-Space + Backward Euler."""
    if pwm_on:
        iL_n = float(Ad_on[0,0]*iL + Ad_on[0,1]*vC + Bd_on[0]*Vin)
        vC_n = float(Ad_on[1,0]*iL + Ad_on[1,1]*vC + Bd_on[1]*Vin)
    elif iL > 0:
        iL_n = float(Ad_off[0,0]*iL + Ad_off[0,1]*vC)
        vC_n = float(Ad_off[1,0]*iL + Ad_off[1,1]*vC)
    else:
        iL_n = 0.0
        vC_n = float(Ad_dcm[1,1]*vC)
    return iL_n, vC_n

def step_quantized(iL_q, vC_q, pwm_on):
    """Single step with Q-format quantization (matches Verilog)."""
    if pwm_on:
        iL_n = (q(Ad_on[0,0])*iL_q + q(Ad_on[0,1])*vC_q + (q(Bd_on[0]*Vin)<<FRAC_BITS)) >> FRAC_BITS
        vC_n = (q(Ad_on[1,0])*iL_q + q(Ad_on[1,1])*vC_q + (q(Bd_on[1]*Vin)<<FRAC_BITS)) >> FRAC_BITS
    elif iL_q > 0:
        iL_n = (q(Ad_on[0,0])*iL_q + q(Ad_on[0,1])*vC_q) >> FRAC_BITS
        vC_n = (q(Ad_on[1,0])*iL_q + q(Ad_on[1,1])*vC_q) >> FRAC_BITS
    else:
        iL_n = 0
        vC_n = (q(Ad_dcm[1,1])*vC_q) >> FRAC_BITS
    # Clip
    iL_n = max(min(iL_n, QMAX), QMIN)
    vC_n = max(min(vC_n, QMAX), QMIN)
    return iL_n, vC_n

# ── Simulation runner ──
def simulate(pwm, quantized=False, iL0=0.0, vC0=0.0):
    n = len(pwm)
    iL_arr, vC_arr = np.zeros(n), np.zeros(n)
    if quantized:
        iL_q, vC_q = int(round(iL0*SCALE)), int(round(vC0*SCALE))
        for k in range(n-1):
            iL_q, vC_q = step_quantized(iL_q, vC_q, pwm[k] > 0.5)
            iL_arr[k+1], vC_arr[k+1] = qf(iL_q), qf(vC_q)
    else:
        iL, vC = iL0, vC0
        for k in range(n-1):
            iL, vC = step_ss_be(iL, vC, pwm[k] > 0.5)
            iL_arr[k+1], vC_arr[k+1] = iL, vC
    return iL_arr, vC_arr
'''
    with open(out_dir / "model_fixed.py", "w") as f:
        f.write(lines)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--topology", required=True, choices=list(TOPOLOGIES.keys()))
    p.add_argument("--Vin", type=float, required=True)
    p.add_argument("--L", type=float, required=True)
    p.add_argument("--C", type=float, required=True)
    p.add_argument("--R", type=float, required=True)
    p.add_argument("--fsw", type=float, default=20e3)
    p.add_argument("--D", type=float, default=0.5)
    p.add_argument("--h", type=float, default=50e-9)
    args = p.parse_args()
    generate(args.topology, args.Vin, args.L, args.C, args.R,
             args.fsw, args.D, args.h)
