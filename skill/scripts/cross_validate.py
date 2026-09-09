"""
cross_validate.py — Cross-validate all three routes:
  State-Space + Forward Euler
  State-Space + Backward Euler
  ADC + MNA + Backward Euler

Usage:
  python cross_validate.py --spec spec.json --pwm pwm_data.npy --plc plcs_data.csv
"""

import argparse
import json
import numpy as np
from pathlib import Path


def load_pwm(path):
    """Load PWM signal from CSV or .npy."""
    p = Path(path)
    if p.suffix == ".csv":
        data = np.loadtxt(str(p), delimiter=",", skiprows=1)
        return data[:, 3]  # assume 4th column
    elif p.suffix == ".npy":
        return np.load(str(p))
    raise ValueError(f"Unknown format: {p.suffix}")


def simulate_ss(Ad, Bd, Ad_dcm, Bd_dcm, Vin, pwm):
    """Generic SS simulation, Backward Euler."""
    n = len(pwm)
    iL, vC = np.zeros(n), np.zeros(n)
    for k in range(n - 1):
        s = pwm[k] > 0.5
        if s:
            iL[k+1] = Ad[0,0]*iL[k] + Ad[0,1]*vC[k] + Bd[0]*Vin
            vC[k+1] = Ad[1,0]*iL[k] + Ad[1,1]*vC[k] + Bd[1]*Vin
        elif iL[k] > 0:
            iL[k+1] = Ad[0,0]*iL[k] + Ad[0,1]*vC[k]
            vC[k+1] = Ad[1,0]*iL[k] + Ad[1,1]*vC[k]
        else:
            iL[k+1] = 0.0
            vC[k+1] = Ad_dcm[1,1]*vC[k]
    return iL, vC


def simulate_mna(Y, G_L, G_C, G_R, G_Son, G_Soff, G_Don, G_Doff,
                 Vin, pwm, iL0=0.0, vC0=0.0):
    """Generic MNA simulation."""
    n = len(pwm)
    v_a, v_b = np.zeros(n), np.zeros(n)
    iL, vC = iL0, vC0
    J_L, J_C = 0.0, 0.0

    for k in range(n - 1):
        s = pwm[k] > 0.5
        G_S = G_Son if s else G_Soff
        D_on = (not s) and iL > 0
        G_D = G_Don if D_on else G_Doff

        Y_mat = np.array([[G_L+G_S+G_D, -G_D], [-G_D, G_D+G_C+G_R]])
        rhs = np.array([G_L*Vin + J_L, J_C])
        v = np.linalg.solve(Y_mat, rhs)
        v_a[k+1], v_b[k+1] = v[0], v[1]
        iL = G_L*(Vin - v[0]) + J_L
        vC = v_b[k+1]

        if (not s) and iL <= 0:
            iL = 0.0
            v_b[k+1] = J_C / (G_C + G_R)
        J_L = iL
        J_C = G_C * v_b[k+1]

    return v_b, iL


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--spec", required=True)
    p.add_argument("--pwm", required=True)
    p.add_argument("--plc", default=None, help="PLECS CSV for reference")
    p.add_argument("--plot", action="store_true")
    args = p.parse_args()

    with open(args.spec) as f:
        spec = json.load(f)

    routes = spec["routes"]
    params = spec["params"]
    Vin = params["Vin"]
    pwm = load_pwm(args.pwm)
    n = len(pwm)

    print(f"[cross_validate] Routes: {len(pwm)} steps")
    print(f"  {'Route':>20}  {'vC_mean':>10}  {'iL_mean':>10}  {'vC_MAE':>10}  {'iL_MAE':>10}")
    print("  " + "-" * 66)

    results = {}

    # SS+BE (primary)
    Ad = np.array(routes["ss_be"]["Ad_on"])
    Bd = np.array(routes["ss_be"]["Bd_on"])
    Ad_d = np.array(routes["ss_be"]["Ad_dcm"])
    Bd_d = np.array(routes["ss_be"]["Bd_dcm"])
    iL_be, vC_be = simulate_ss(Ad, Bd, Ad_d, Bd_d, Vin, pwm)
    results["SS+BE"] = (vC_be, iL_be)

    # SS+FE
    Ad = np.array(routes["ss_fe"]["Ad_on"])
    Bd = np.array(routes["ss_fe"]["Bd_on"])
    Ad_d = np.array(routes["ss_fe"]["Ad_dcm"])
    Bd_d = np.array(routes["ss_fe"]["Bd_dcm"])
    iL_fe, vC_fe = simulate_ss(Ad, Bd, Ad_d, Bd_d, Vin, pwm)
    results["SS+FE"] = (vC_fe, iL_fe)

    # MNA+BE
    mna = routes["mna"]
    vC_mn, iL_mn = simulate_mna(
        None, mna["G_L"], mna["G_C"], mna["G_R"],
        mna["G_Son"], mna["G_Soff"], mna["G_Don"], mna["G_Doff"],
        Vin, pwm)
    results["MNA+BE"] = (vC_mn, iL_mn)

    # Print table
    for name, (vc, il) in results.items():
        print(f"  {name:>20}  {np.mean(vc):>10.4f}  {np.mean(il):>10.4f}  {0:>10.4f}  {0:>10.4f}")

    # Pairwise differences
    print()
    ref = "SS+BE"
    for name, (vc, il) in results.items():
        if name == ref:
            continue
        dv = np.abs(vc - results[ref][0])
        di = np.abs(il - results[ref][1])
        print(f"  {name} vs {ref}:  vC MAE={np.mean(dv)*1000:.4f} mV  iL MAE={np.mean(di)*1000:.3f} mA")

    # PLECS comparison (if available)
    if args.plc:
        plc = np.loadtxt(args.plc, delimiter=",", skiprows=1)
        n_plc = min(len(plc), len(vC_be))
        for name, (vc, il) in results.items():
            dv = np.abs(vc[:n_plc] - plc[:n_plc, 1])
            di = np.abs(il[:n_plc] - plc[:n_plc, 2])
            print(f"  {name:>20} vs PLECS:  vC MAE={np.mean(dv)*1000:.4f} mV  iL MAE={np.mean(di)*1000:.3f} mA")

    # Plot
    if args.plot:
        import matplotlib.pyplot as plt
        import scienceplots
        plt.style.use(["science", "ieee", "no-latex"])
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.5, 2.5), sharex=True)
        for name, (vc, il) in results.items():
            ax1.plot(vc[:2000], label=name, lw=0.6)
            ax2.plot(il[:2000], label=name, lw=0.6)
        if args.plc:
            ax1.plot(plc[:2000, 1], 'k--', label="PLECS", lw=0.5, alpha=0.7)
        ax1.legend(fontsize=6); ax1.set_ylabel("vC [V]")
        ax2.legend(fontsize=6); ax2.set_xlabel("Step"); ax2.set_ylabel("iL [A]")
        fig.tight_layout()
        fig.savefig("results/cross_validate.pdf", dpi=300)
        print("\n  Plot saved: results/cross_validate.pdf")


if __name__ == "__main__":
    main()
