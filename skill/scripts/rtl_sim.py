"""
rtl_sim.py — Agent 2: Hardware RTL Agent — verification loop
Compiles Verilog with iverilog, runs simulation, compares VCD
output against Python Golden Reference, computes RMSE.

Usage:
  python rtl_sim.py --rtl solver.v --spec spec.json --golden model_fixed.py
"""

import argparse
import json
import numpy as np
import subprocess
import sys
from pathlib import Path


def parse_vcd(vcd_path, signal_codes):
    """
    Minimal VCD parser. Extracts values at rising edges of strobe signal.

    signal_codes: dict of name → VCD code char, e.g. {"iL": "F", "vC": "G"}
    Returns dict of name → numpy array of sampled values.
    """
    current = {}
    sampled = {name: [] for name in signal_codes}
    prev_strobe = 0
    strobe_code = signal_codes.get("strobe", None)

    with open(vcd_path, 'r') as f:
        for line in f:
            L = line.rstrip()
            if not L or L[0] == '#' or L[0] not in 'b01x':
                continue
            if L[0] == 'b':
                parts = L.split()
                if len(parts) != 2:
                    continue
                bs, code = parts[0][1:], parts[1]
                if not bs or 'x' in bs:
                    continue
                val = int(bs, 2)
                # Sign extend 24-bit
                if len(bs) == 24 and bs[0] == '1':
                    val -= (1 << 24)
                if code in signal_codes.values():
                    for name, c in signal_codes.items():
                        if c == code:
                            current[name] = val
            else:
                if L[0] == 'x':
                    continue
                val = int(L[0])
                code = L[1:]
                if code == strobe_code:
                    if prev_strobe == 0 and val == 1:
                        for name in signal_codes:
                            if name != "strobe" and name in current:
                                sampled[name].append(current.get(name, 0))
                    prev_strobe = val

    return sampled


def run_iverilog(rtl_path, tb_path, vcd_out):
    """Compile and run iverilog simulation."""
    build_dir = Path.cwd() / "build"
    build_dir.mkdir(exist_ok=True)
    sim_file = build_dir / "sim"

    # Find iverilog
    iverilog = Path.home() / "iverilog" / "bin" / "iverilog"
    vvp = Path.home() / "iverilog" / "bin" / "vvp"

    if not iverilog.exists():
        iverilog = Path("iverilog")
        vvp = Path("vvp")

    # Compile
    compile_cmd = [str(iverilog), "-g2012", "-o", str(sim_file),
                   str(rtl_path), str(tb_path)]
    result = subprocess.run(compile_cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("[rtl_sim] iverilog compile FAILED:\n", result.stderr)
        return False

    # Run
    run_cmd = [str(vvp), str(sim_file)]
    result = subprocess.run(run_cmd, capture_output=True, text=True,
                            cwd=build_dir)
    if result.returncode != 0:
        print("[rtl_sim] Simulation FAILED:\n", result.stderr)
        return False

    # Copy VCD
    vcd_src = build_dir / vcd_out.name
    if vcd_src.exists():
        import shutil
        shutil.copy(vcd_src, vcd_out)
        return True
    return False


def compute_rmse(sampled, golden, scale):
    """Compute RMSE between VCD samples and golden reference."""
    min_len = min(len(v) for v in sampled.values())
    errors = {}
    for name in sampled:
        if name in golden:
            vcd_vals = np.array(sampled[name][:min_len], dtype=float) / scale
            gld_vals = golden[name][:min_len]
            err = np.abs(vcd_vals - gld_vals)
            errors[name] = {
                "rmse": float(np.sqrt(np.mean(err**2))),
                "mae": float(np.mean(err)),
                "max": float(np.max(err)),
            }
    return errors


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--rtl", required=True, help="Path to solver.v")
    p.add_argument("--spec", required=True, help="Path to spec.json")
    p.add_argument("--golden", required=True, help="Path to golden reference")
    p.add_argument("--tb", default=None, help="Testbench (auto-generated if absent)")
    args = p.parse_args()

    # Load spec
    with open(args.spec) as f:
        spec = json.load(f)
    scale = spec["q_format"]["scale"]

    # Generate testbench from template if needed
    tb_path = Path(args.rtl).with_name("tb_" + Path(args.rtl).name)
    if not tb_path.exists() and args.tb:
        tb_path = Path(args.tb)

    if not tb_path.exists():
        print("[rtl_sim] No testbench found. Generate one with golden_ref.py first.")
        sys.exit(1)

    # Run simulation
    vcd_out = Path.cwd() / "results" / "sim.vcd"
    vcd_out.parent.mkdir(exist_ok=True)

    print(f"[rtl_sim] Compiling {args.rtl} + {tb_path}...")
    if not run_iverilog(Path(args.rtl), tb_path, vcd_out):
        sys.exit(1)

    # Parse VCD
    signal_codes = {
        "iL": "F", "vC": "G", "pwm": "9", "strobe": "?"
    }
    sampled = parse_vcd(vcd_out, signal_codes)
    n_samples = len(sampled.get("iL", []))
    print(f"[rtl_sim] Parsed {n_samples} samples from VCD")

    if n_samples == 0:
        print("[rtl_sim] ERROR: no samples extracted. Check strobe signal code.")
        sys.exit(1)

    # Compare against golden reference
    import importlib.util
    golden_path = Path(args.golden)
    spec_golden = importlib.util.spec_from_file_location("model_fixed", golden_path)
    golden = importlib.util.module_from_spec(spec_golden)
    spec_golden.loader.exec_module(golden)

    # Generate PWM matching the simulation
    pwm = np.array(sampled.get("pwm", []), dtype=float)
    iL_g, vC_g = golden.simulate(pwm, quantized=True)

    # Align lengths
    min_n = min(n_samples, len(iL_g))
    sampled_iL = np.array(sampled["iL"][:min_n], dtype=float) / scale
    sampled_vC = np.array(sampled["vC"][:min_n], dtype=float) / scale

    err_iL = np.abs(sampled_iL - iL_g[:min_n])
    err_vC = np.abs(sampled_vC - vC_g[:min_n])

    print(f"\n[rtl_sim] ═══════════════════════════════════════")
    print(f"[rtl_sim]   VCD vs Golden Reference (Q{spec['q_format']['int']}.{spec['q_format']['frac']})")
    print(f"[rtl_sim] ═══════════════════════════════════════")
    print(f"[rtl_sim]   iL MAE = {np.mean(err_iL)*1000:.4f} mA")
    print(f"[rtl_sim]   iL max = {np.max(err_iL)*1000:.4f} mA")
    print(f"[rtl_sim]   vC MAE = {np.mean(err_vC)*1000:.4f} mV")
    print(f"[rtl_sim]   vC max = {np.max(err_vC)*1000:.4f} mV")
    print(f"[rtl_sim] ═══════════════════════════════════════")

    # PASS/FAIL
    threshold_mv = 50  # 50mV threshold
    if np.mean(err_vC) * 1000 < threshold_mv:
        print(f"[rtl_sim] ✅ PASS — vC MAE < {threshold_mv} mV")
        return 0
    else:
        print(f"[rtl_sim] ❌ FAIL — vC MAE >= {threshold_mv} mV")
        print(f"[rtl_sim]    Suggestion: check Q-format overflow / saturation logic")
        return 1


if __name__ == "__main__":
    sys.exit(main())
