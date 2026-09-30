# HiLCoder

An agent skill that turns a power-electronics circuit topology into a
**verified, FPGA-deployable real-time hardware-in-the-loop (HiL) solver** —
state-space model, fixed-point quantization, Verilog, simulation check against
a floating-point reference, and bitstream.

It is the installable form of the framework described in *Agentic HiL: A
Multi-Agent LLM Framework for Autonomous Generation of FPGA-based Real-Time
Power Electronics Simulators* (accepted, ECCE 2026).

**Interactive walkthrough:** https://qcsui.github.io/agentic-hil/

---

## What it does

You describe a converter. The skill drives two agents:

| Agent | Job | Output |
|---|---|---|
| **Physics Modeler** | Derives state-space (and MNA) equations, discretizes them (Forward/Backward Euler), allocates a fixed-point Q-format from a static range analysis, and cross-validates the routes against each other | `model_fixed.py`, `spec.json` |
| **Hardware RTL** | Emits flattened low-latency Verilog, simulates it with iverilog, checks RMSE against the golden reference, then synthesizes with Vivado | `solver.v`, `top.v`, `.bit` |

On the reference buck converter (24 V → 12 V, 20 kHz, driven into DCM,
Zynq-7020) the generated RTL matches an offline PLECS floating-point model to
**16.77 mV** steady-state error (< 0.15 %), using **4.28 % of the LUTs**, with
a datapath latency under **10 ns** against a 50 ns step.

## Requirements

- An agent runtime that supports skills — [Claude Code](https://claude.com/claude-code) or OpenCode
- Python 3 with NumPy
- [Icarus Verilog](https://steveicarus.github.io/iverilog/) for RTL simulation
- Xilinx Vivado for synthesis and bitstream generation
- A floating-point reference model (PLECS, MATLAB/Simulink, or the included
  Python golden-reference scripts)
- Target board: a Zynq-7020 (the reference pinout is for a Mizar Z7020) plus a
  DAC/ADC daughter board if you want to close the loop with real analog I/O

## Install

**Claude Code**

```bash
git clone https://github.com/qcsui/HiLCoder.git
mkdir -p ~/.claude/skills
cp -r HiLCoder/skill ~/.claude/skills/hilcoder
```

**OpenCode**

```bash
mkdir -p .opencode/skills
cp -r HiLCoder/skill .opencode/skills/hilcoder
```

Project-local installs work too — put it in `.claude/skills/hilcoder` inside
the repo you're working in.

## Use

Once installed, just describe what you want:

```
Generate a Buck converter HiL solver at a 50 ns step.
Vin=24V, L=100uH, C=100uF, R=10ohm, fsw=20kHz, D=0.5, target Zynq-7020.
```

The skill will work through three stages — fixed-point model, RTL generation
and validation, then bitstream and report. `skill/SKILL.md` documents each
stage, the file layout it produces, and the pitfalls it knows about (DAC
encoding wraparound, DCM detection, coefficient formats for small coupling
terms, pipeline depth).

You can also run the scripts directly without an agent:

```bash
python skill/scripts/golden_ref.py       # floating-point + fixed-point reference
python skill/scripts/rtl_sim.py          # iverilog run + RMSE against the reference
python skill/scripts/cross_validate.py   # compare SS/MNA, Forward/Backward Euler routes
```

## Layout

```
skill/                     the skill itself — install this
  SKILL.md                 stage-by-stage instructions the agent follows
  scripts/                 golden reference, RTL simulation, cross-validation
  templates/               Verilog solver and top-level templates
reference/                 verified implementations SKILL.md refers to
  python/                  buck & boost golden references (SS+FE/BE, MNA+BE)
  rtl/                     verified buck solver, top level, board top, testbench
  vivado/                  pin constraints, bitstream script, VCD↔PLECS comparison
```

## Citing

```bibtex
@inproceedings{sui2026agentichil,
  title     = {Agentic HiL: A Multi-Agent LLM Framework for Autonomous
               Generation of FPGA-based Real-Time Power Electronics Simulators},
  author    = {Sui, Qingcheng and Li, Yang and Kong, Jiaze and
               Zuo, Yu and Martinez, Wilmar},
  booktitle = {IEEE Energy Conversion Congress and Exposition (ECCE)},
  year      = {2026}
}
```

## License

[PolyForm Noncommercial 1.0.0](https://polyformproject.org/licenses/noncommercial/1.0.0/), see [LICENSE](LICENSE).

Free for any noncommercial purpose. That explicitly includes universities,
public research organizations and other educational or charitable
institutions, regardless of how they are funded, so academic use needs no
further permission. Commercial use is not granted by this license.
