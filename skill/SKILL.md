---
name: hilcoder
description: "Multi-agent framework for autonomous FPGA-based real-time HIL solver generation. Use when the user provides a power electronic circuit topology and wants an FPGA bitstream for real-time simulation."
---

# HiLCoder

Licensed under PolyForm Noncommercial 1.0.0 (noncommercial use only,
including academic and public research use). See the LICENSE file in
https://github.com/qcsui/HiLCoder

Two coordinated LLM Agents that transform a circuit topology into a verified FPGA-based real-time HIL simulator.

```
User: "Generate a Buck converter HIL solver at 50ns step"
      │
      ▼
┌──────────────────────────────────────────┐
│         Agent 1: Physics Modeler          │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │ SS + FE  │ │ SS + BE  │ │MNA + BE  │  │
│  └──────────┘ └──────────┘ └──────────┘  │
│         ↓ ↓ ↓ cross-validate ↓ ↓ ↓       │
│  Output: model_fixed.py + spec.json      │
└──────────────────────────────────────────┘
      │
      ▼
┌──────────────────────────────────────────┐
│         Agent 2: Hardware RTL Agent       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │  Verilog │ │ iverilog │ │  Vivado  │  │
│  │  RTL     │→│  sim+RMS │→│  synth   │  │
│  └──────────┘ └──────────┘ └──────────┘  │
│         ↓ RMSE vs Golden Reference        │
│  Output: solver.v + top.v + .bit         │
└──────────────────────────────────────────┘
      │
      ▼
         Bitstream ready for Zynq-7020 + ACM9767
```

---

## When to Use

- User provides a DC-DC converter topology (Buck, Boost, Buck-Boost, etc.) and wants an FPGA real-time solver
- User wants to cross-validate multiple numerical methods (State-Space vs MNA, FE vs BE)
- User wants a complete Zynq-7020 bitstream with ACM9767 DAC output

## When NOT to Use

- User wants pure simulation (no FPGA) → use Python switching-level simulation directly
- User has a non-power-electronics circuit (motors, grids) → this skill is power-electronics specific
- User wants HLS-based design → use `vitis-hls` skill instead

---

## Skill Stages

This skill supports three stages. Start from Stage 1 unless user already has intermediate files.

| Stage | Name | Input → Output |
|-------|------|----------------|
| **1** | Fixed-Point Model | Circuit params → `model_fixed.py`, `spec.json` |
| **2** | RTL Generation | `spec.json` → `solver.v`, `top.v`, `sim_pass.vcd` |
| **3** | Bitstream & Verify | `solver.v` → `hil_solver.bit`, `timing.rpt`, `validation.pdf` |

---

## Stage 1: Fixed-Point Model (Agent 1)

Generate three parallel model variants and cross-validate them.

### Step 1.1: Extract circuit parameters

Parse the user's description into structured values:

```json
{
  "topology": "Buck",
  "Vin": 24.0, "L": 100e-6, "C": 100e-6, "R": 10.0,
  "fsw": 20e3, "D": 0.5,
  "h": 50e-9
}
```

### Step 1.2: Generate State-Space matrices (SS route)

```python
# Continuous matrices from KVL/KCL
A_on  = np.array([[0, -1/L], [1/C, -1/(R*C)]])
B_on  = np.array([[1/L], [0]])
A_off = A_on.copy()
B_off = np.array([[0], [0]])
A_dcm = np.array([[0, 0], [0, -1/(R*C)]])
B_dcm = np.array([[0], [0]])

# Discretize (Backward Euler)
def disc_be(A, B, h):
    inv = np.linalg.inv(np.eye(2) - h * A)
    return inv, inv @ (h * B)

# Discretize (Forward Euler)
def disc_fe(A, B, h):
    return np.eye(2) + h * A, h * B
```

### Step 1.3: Generate ADC+MNA matrices (MNA route)

```python
# Companion circuit parameters (Backward Euler)
G_L = h / L    # inductor Norton conductance
G_C = C / h    # capacitor Norton conductance
G_R = 1.0 / R  # load

# MNA matrix for 2-node circuit
Y = np.array([
    [G_L + G_S + G_D,  -G_D        ],
    [-G_D,              G_D + G_C + G_R]
])
# Solve Y * v = i at each step
```

### Step 1.4: Q-format allocation

Determine integer bits based on variable range analysis:

```python
# Max expected values
Vmax = max(Vin, Vin/(1-D), ...)  # topology-dependent
Imax = ...                        # inrush estimate

int_bits = ceil(log2(Vmax)) + 1   # +1 for sign
frac_bits = 24 - int_bits         # 24-bit total
```

Output `spec.json`:

```json
{
  "q_format": {"int": 7, "frac": 17, "total": 24, "scale": 131072},
  "ss_fe": {"Ad": [[...],[...]], "Bd": [[...],[...]]},
  "ss_be": {"Ad": [[...],[...]], "Bd": [[...],[...]]},
  "mna":   {"G_L": 5e-4, "G_C": 2000, ...},
  "pwm":   {"fsw": 20e3, "D": 0.5, "h": 50e-9}
}
```

### Step 1.5: Run

```bash
cd .opencode/skills/agentic-hil/scripts
python golden_ref.py --topology Buck --Vin 24 --L 100e-6 --C 100e-6 --R 10 --fsw 20e3 --D 0.5
```

Output:
- `model_fixed.py` — Golden Reference (all three routes)
- `spec.json` — structured spec for Agent 2

---

## Stage 2: RTL Generation (Agent 2)

Generate Verilog RTL from `spec.json` and verify against golden reference.

### Step 2.1: Generate solver core

Using `templates/solver_template.v`, fill the Q-format placeholders:

| Placeholder | Description | Source |
|-------------|-------------|--------|
| `{QW}` | Total bit width (e.g. 24) | `spec.q_format.total` |
| `{INT_BITS}` | Integer bits (e.g. 7) | `spec.q_format.int` |
| `{FRAC_BITS}` | Fractional bits (e.g. 17) | `spec.q_format.frac` |
| `{SCALE}` | 2^{FRAC_BITS} | computed |
| `{A00}` ... `{B1}` | Matrix coefficients in Q-m. | `spec.ss_be.Ad/Bd` |

### Step 2.2: Generate top level

Using `templates/top_template.v`, configure:
- MMCM divider settings (50 MHz → 180 MHz / 20 MHz)
- DAC output pin mapping
- PWM counter max (fsw × h × SCALE)

### Step 2.3: Simulate and validate

```bash
python scripts/rtl_sim.py --spec spec.json --rtl solver.v --golden model_fixed.py
```

This script:
1. Runs `iverilog` on solver.v + testbench
2. Parses VCD output
3. Computes RMSE vs Golden Reference
4. If RMSE > threshold (e.g. 1% of Vout), traces AST to locate overflow/saturation bugs
5. Iterates until RMSE passes

### Step 2.4: Cross-validate all routes

```bash
python scripts/cross_validate.py --spec spec.json
```

Validates that SS+FE, SS+BE, and MNA+BE all agree within tolerance.

---

## Stage 3: Bitstream & Report

```bash
# In vivado/ directory:
vivado -mode batch -source build_bitstream.tcl
```

Outputs:
- `hil_solver.bit` — download to board
- `timing.rpt` — Vivado timing report
- `utilization.rpt` — resource usage
- `validation.pdf` — IEEE-style comparison plots vs PLECS

---

## File System Layout

```
.opencode/skills/agentic-hil/
├── SKILL.md                     ← This file
├── templates/
│   ├── solver_template.v        ← Parameterized solver core
│   └── top_template.v           ← Board-level top with MMCM + CDC
└── scripts/
    ├── golden_ref.py            ← Stage 1: model generation
    ├── rtl_sim.py               ← Stage 2: iverilog + RMSE
    └── cross_validate.py        ← Stage 2: route comparison
```

## Reference Project Files

The verified reference implementations ship with this repo:

| File | Content |
|------|---------|
| `reference/python/buck_switching.py` | Buck SS+FE+BE golden ref |
| `reference/python/boost_switching.py` | Boost SS+FE+BE golden ref |
| `reference/python/boost_mna.py` | Boost MNA+BE golden ref |
| `reference/rtl/buck_solver.v` | Buck RTL (verified, Q7.17) |
| `reference/rtl/buck_top.v` | Buck top with CDC+DAC |
| `reference/rtl/hil_top.v` | Board-level top with MMCM |
| `reference/vivado/hil_solver.xdc` | Pin constraints |
| `reference/vivado/build_bitstream.tcl` | Bitstream build script |
| `reference/rtl/tb_buck_board.v` | Board-level testbench |
| `reference/vivado/compare_vcd_vs_plecs.py` | VCD vs PLECS comparison |

## Board Pinout Reference

| Signal | FPGA Pin | Notes |
|--------|----------|-------|
| PL_CLK_50M | H16 | 50 MHz reference |
| RST_n (KEY1) | R19 | Active low |
| DAC_CH1[13:0] | P19..Y18 | JP1 pins 1-16 |
| DAC_CH1_CLK | Y16 | JP1 pin 17 |
| DAC_CH1_WRT | Y17 | JP1 pin 18 |
| DAC_CH2[13:0] | U18..W13 | JP1 pins 21-36 |
| DAC_CH2_CLK | W18 | JP1 pin 19 |
| DAC_CH2_WRT | W19 | JP1 pin 20 |
| LED[0] | G14 | Status (active low) |
| LED[1] | C20 | Status (active low) |

## Known Pitfalls

### 1. DAC Encoding Wraparound
The DAC encoding formula `da = 8192 + vC[high:low]` must ensure the bit field does not overflow within the expected voltage range. The safe formula is `vC[{QW}-1 : {QW}-14]` which extracts the top 14 bits. Never use a fixed extraction like `[21:9]` — this wraps at 256V for Q10.14.

### 2. DCM Detection
DCM must trigger when `iL <= 0`, not only when `iL == 0`. Use `iL[{QW}-1] || !(|iL)` to catch negative values that can occur during zero-crossing with Backward Euler discretization.

### 3. Coefficient Format for Small Coupling Terms
When `h/L < 2^{-frac_bits}`, cross-coupling terms quantize to zero in the state Q-format. Use a coefficient format with more fractional bits (e.g., Q1.23 for a Q10.14 state) to preserve small off-diagonal entries. The MAC shift must be adjusted accordingly (`>>> 23` instead of `>>> 14`).

### 4. Pipeline Depth
Select pipeline depth based on the target primitive: DSP48E1 designs inherently require 3-4 pipeline stages for multiply-add; LUT-based multipliers can achieve single-cycle operation even at 180 MHz when the adder tree is well balanced.
