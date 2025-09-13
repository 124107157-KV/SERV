<img align="right" src="https://svg.wavedrom.com/{signal:[{wave:'0.P...'},{wave:'023450',data:'S E R V'}]}"/>

# SERV

[![Join the chat at https://gitter.im/librecores/serv](https://badges.gitter.im/librecores/serv.svg)](https://gitter.im/librecores/serv?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
[![Compliance tests](https://github.com/olofk/serv/actions/workflows/ci.yml/badge.svg)](https://github.com/olofk/serv/actions/workflows/ci.yml)
[![Documentation Status](https://readthedocs.org/projects/serv/badge/?version=latest)](https://serv.readthedocs.io/en/latest/?badge=latest)

SERV is an award-winning bit-serial RISC-V core

In fact, the award-winning SERV is the world's smallest RISC-V CPU. It's the perfect companion whenever you need a bit of computation and silicon real estate is at a premium.

How small is it then? Synthesizing the latest version of SERV in its most minimal form, yields the following results for some popular FPGA architectures and a typical CMOS process.

| Lattice iCE40 | Intel Cyclone 10LP | AMD Artix-7 | CMOS   |
| ------------- | ------------------ | ----------- | ------ |
| 198 LUT       | 239 LUT            | 125 LUT     | 2.1kGE |
| 164 FF        | 164 FF             | 164 FF      |        |


If you want to know more about SERV, what a bit-serial CPU is and what it's good for, I recommend starting out by watching the fantastic short SERV movies
* [introduction to SERV](https://www.award-winning.me/serv-introduction/)
* [SERV : RISC-V for a fistful of gates](https://www.award-winning.me/serv-for-a-fistful-of-gates/)
* [SERV: 32-bit is the New 8-bit](https://www.award-winning.me/serv-32-bit-is-the-new-8-bit/)
* [Bit by bit - How to fit 8 RISC V cores in a $38 FPGA board (presentation from the Zürich 2019 RISC-V workshop)](https://www.youtube.com/watch?v=xjIxORBRaeQ)

All SERV videos and more can also be found [here](https://www.award-winning.me/videos/).

Apart from being the world's smallest RISC-V CPU, SERV also aims at being the best documented RISC-V CPU. For this there is an official [SERV user manual](https://serv.readthedocs.io/en/latest/#) with block diagrams that are correct to the gate-level, cycle-accurate timing diagrams and an in-depth description of how things work.

# SERV-Stochastic & NN Enhancement: Dual-Mode Bit-Serial RISC-V (RV32I) with Stochastic Computing & NN MAC

[![Compliance tests](https://github.com/olofk/serv/actions/workflows/ci.yml/badge.svg)](https://github.com/olofk/serv/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-User%20Guide-informational)](#documentation)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

**SERV-Stochastic** extends the award-winning, world’s-smallest RISC-V core (SERV) with a **dual-mode ALU** that runs both **deterministic (binary)** and **stochastic** arithmetic on the *same* bit-serial datapath—plus a lightweight **serial MAC path** for neural-network workloads. The design targets **tiny area & power**, preserves **RV32I transparency** in normal mode, and adds **runtime-selectable SC/NN** execution.

> TL;DR
>
> * **Dual-mode ALU:** flip between standard RV32I ops and **stochastic primitives** at runtime.
> * **NN-friendly MAC:** serial multiply–accumulate for dot-products / layers on the bit-serial fabric.
> * **Tiny footprint:** built on SERV’s ultra-compact micro-architecture, validated on **PYNQ-Z2**.
> * **Clean integration:** no ISA breakage; baseline behavior unchanged when SC/NN are disabled.

---

## Table of Contents

* [Why Stochastic on SERV?](#why-stochastic-on-serv)
* [What’s New in This Fork](#whats-new-in-this-fork)
* [Architecture Overview](#architecture-overview)
* [Modules & Interfaces](#modules--interfaces)
* [Build & Run](#build--run)
* [Reproducing FPGA Results (PYNQ-Z2)](#reproducing-fpga-results-pynqz2)
* [Software & Demos](#software--demos)
* [Configuration & Tuning](#configuration--tuning)
* [Verification & Compliance](#verification--compliance)
* [Repository Layout](#repository-layout)
* [Results Snapshot](#results-snapshot)
* [Roadmap](#roadmap)
* [Documentation](#documentation)
* [Acknowledgments](#acknowledgments)
* [Cite This Work](#cite-this-work)
* [License](#license)

---

## Why Stochastic on SERV?

**Stochastic Computing (SC)** represents values as 1-bit streams where probability encodes magnitude. Multiplication becomes **AND** (unipolar) or **XNOR** (bipolar), addition can be done with **MUX-based** schemes, and comparisons become simple **threshold/counter** logic. This trades precision (bit-stream length) for **dramatic hardware simplicity**—a perfect match for **bit-serial** cores like SERV aimed at **ultra-low power edge** scenarios (sensor fusion, always-on classifiers, simple CNN/MLP layers).

---

## What’s New in This Fork

**Dual-Mode ALU (deterministic + stochastic)**

* Add **two runtime controls** to the execute stage:

  * `i_stochastic_mode` — route ops through **stochastic primitives** and SNGs.
  * `i_mac_mode` — enable **serial MAC** path for NN dot-products / reductions.
* When both are **deasserted**, core is **bit-for-bit baseline SERV** (RV32I).

**Stochastic primitives & utilities**

* **Encodings:** unipolar `[0,1]` (AND) and bipolar `[-1,1]` (XNOR).
* **Ops:** stochastic multiply, MUX-based add/average, comparators/threshold, **popcount + normalize**.
* **SNGs:** LFSR-based stochastic number generators (with optional thresholding).
* **Stream control:** length `N`, optional warm-up, and normalization to fixed-point.

**NN-oriented serial MAC**

* Event-driven accumulation of SC products into a **local serial accumulator**.
* Fits the SERV bit-serial ethos (low LUT/FF pressure) and keeps clocks high for IO/Zephyr.

**RTL housekeeping & reuse**

* Reuse of **SERV adder** and shifters where possible.
* Clock-enable and simple gating to limit toggles in SC mode.
* Clean **write-back** into the existing RF without ISA changes.

---

## Architecture Overview

```
              +-------------------------+
 Instr/Decode |      Execute (SERV)     |    Write-back
   ----->---->| +---------------------+ |------> RF
              | |   Dual-Mode  ALU   | |
              | |  +--------------+  | |  i_stochastic_mode
              | |  | Deterministic|  | |  i_mac_mode
              | |  +--------------+  | |
              | |  |  Stochastic  |--|-+---> SNGs, AND/XNOR, MUX-ADD,
              | |  +--------------+  | |      POPCOUNT/NORM, CMP/THR
              | +---------------------+ |
              +-------------------------+
```

* **Deterministic sub-path:** original SERV ALU ops (ADD/SUB/LOGIC/SHIFT/CMP).
* **Stochastic sub-path:** SNG → AND/XNOR → MUX-ADD → POPCOUNT → normalize.
* **NN serial MAC:** consumes SC products into a counter/accumulator; exposes the reduced result through standard write-back.

---

## Modules & Interfaces

### `serv_stoc_alu` (top of SC/NN datapath)

```verilog
module serv_stoc_alu #(
    parameter W = 1,              // bit-serial width (SERV-style)
    parameter B = W - 1
)(
    input  wire        clk,
    input  wire        i_en,
    input  wire        rst,
    input  wire [4:0]  i_alu_ctrl_bus,      // {i_rd_sel[2:0], i_bool_op[1:0]}
    input  wire        i_matrix_op,         // enable MAC / matrix-style ops
    input  wire [2:0]  i_alu_cmp_ctrl_bus,  // {i_sub, i_cmp_eq, i_cmp_sig}
    input  wire [(4*W)-1:0] i_alu_data_bus, // {rs1, rs2, op_a, op_b} bit-serial
    output reg  [B:0]  o_rd,                // result (bit-serial out)
    output wire        o_cmp                // comparator flag
);
```

**Key control semantics**

* `i_stochastic_mode` (top-level): routes execute stage through SC units.
* `i_mac_mode` (top-level): steers SC product into the **serial accumulator**.
* `i_matrix_op`: convenience gate for vector/matrix micro-kernels (e.g., dot products).

**SC building blocks (internal)**

* `sng_lfsr`: LFSR-driven stream with threshold compare.
* `stoc_mul`: AND (unipolar) / XNOR (bipolar) selectable.
* `stoc_add_mux`: MUX-based averaging / scaled add (configurable).
* `popcount_norm`: event counter + normalization to fixed-point Q-format.
* `cmp_thr`: stream-based comparator / threshold detector.

> The ALU exposes only standard write-back semantics; **no ISA changes** are required for baseline software. SC/MAC are enabled via simple control wires mapped in the SoC glue (see below).

---

## Build & Run

> We use **FuseSoC** + Verilator for simulation, and **Vivado** for PYNQ-Z2 synthesis.

### 0) Workspace

```bash
export WORKSPACE=$(pwd)
```

### 1) Install FuseSoC and Verilator

```bash
pip install fusesoc
# Verilator: use your package manager or build from source
```

### 2) Add libraries

```bash
# Upstream standard cores
fusesoc library add fusesoc_cores https://github.com/fusesoc/fusesoc-cores

# Upstream SERV (as a separate lib so you can modify side-by-side)
fusesoc library add serv https://github.com/olofk/serv

# (Optional) MDU library if you want RV32M-style mult/div blocks
fusesoc library add mdu https://github.com/zeeshanrafique23/mdu
```

> This repo (SERV-Stochastic) should sit alongside the libraries above, or you can register it as another FuseSoC library (recommended if you want out-of-tree builds).

### 3) Lint & Elaborate

```bash
fusesoc run --target=lint serv
```

### 4) Run SC/MAC Testbenches (Verilator)

```bash
# Example (adapt to your core/target name if wrapped as a new core):
fusesoc run --target=verilator_tb serv-stochastic --memsize=16384 \
  --firmware=sw/sc_demo.hex --uart_baudrate=57600
```

### 5) Synthesize (Vivado) for PYNQ-Z2

```bash
fusesoc run --tool=vivado serv-stochastic --pnr=none --part=xc7z020clg400-1
```

---

## Reproducing FPGA Results (PYNQ-Z2)

1. Build SoC bitstream with **SC/NN enabled** in the toplevel (glue maps the two mode bits to a simple MMIO control register or fixed top-level generics—choose one flow and keep it consistent).
2. Convert your firmware to HEX (hello, demos, micro-benchmarks):

```bash
python3 $SERV/sw/makehex.py sw/zephyr_hello.bin 4096 > hello.hex
```

3. Pre-load the HEX into BRAM for your board target (example):

```bash
fusesoc run --target=nexys_a7 servant --memfile=/path/to/hello.hex
# For PYNQ-Z2, use your board target or project TCL in boards/pynq-z2/
```

4. Run **SC demo** (stochastic multiply/add/threshold) and **MAC demo** (dot-product on serial path). UART prints confirm enable/disable transitions and normalized outputs.

---

## Software & Demos

* **`sw/sc_demo/`** – shows:

  * toggling `i_stochastic_mode` on/off from SoC MMIO.
  * unipolar vs bipolar stream runs (AND vs XNOR).
  * popcount-to-fixed-point normalization checks.

* **`sw/mac_demo/`** – serial MAC on small vectors / CNN micro-kernels:

  * event-driven accumulation from stochastic products.
  * dot-product sanity and range tests.

* **Zephyr samples** (optional): boot, print, and then toggle SC/MAC for a few cycles before returning to deterministic mode.

---

## Configuration & Tuning

* **Encoding:** `UNIPOLAR` (0..1) or `BIPOLAR` (-1..1).
* **Stream length `N`:** controls precision vs latency (powers of two simplify normalization).
* **Normalization:** shift-right by `log2(N)` for unipolar average; bipolar applies inverse map.
* **SNG seeds/thresholds:** set per test to decorrelate streams and reduce bias.
* **MAC windowing:** set accumulation window (`N`) and number of terms (vector length).

---

## Verification & Compliance

* **RV32I transparency:** with SC/MAC **disabled**, the core remains a compliant SERV instance (passes upstream compliance on our setups).
* **Module-level sims:** SC ops (mul/add/cmp), SNG quality checks, popcount/normalize.
* **SoC-level sims:** mode toggling, mixed deterministic + SC sequences, MAC reductions.

---

## Repository Layout

```
.
├── rtl/
│   ├── serv_stoc_alu.v           # Dual-mode ALU (deterministic + stochastic)
│   ├── sng_lfsr.v                # Stochastic number generator(s)
│   ├── stoc_mul.v                # AND/XNOR selectable
│   ├── stoc_add_mux.v            # MUX-based add/average
│   ├── popcount_norm.v           # Event counter + normalization
│   ├── cmp_thr.v                 # Comparators / thresholding
│   └── serv_nn_mac.v             # Serial MAC accumulator path
├── tb/
│   ├── tb_serv_stoc_alu.sv
│   └── tb_serv_nn_mac.sv
├── sw/
│   ├── sc_demo/
│   ├── mac_demo/
│   └── zephyr_examples/
├── boards/
│   └── pynq-z2/                  # Vivado project TCL / constraints
├── docs/
│   ├── serv_modules_v2.pdf       # Module-by-module notes & interfaces
│   ├── figures/                  # Block diagrams / datapaths
│   └── papers/                   # short/interim/final report PDFs
└── scripts/
    ├── build_fpga.tcl
    └── makehex.py -> $SERV/sw/makehex.py (symlink or copy)
```

> Names above match your project artifacts; adjust to your repo’s exact filenames.

---

## Results Snapshot

* **Area & power:** the dual-mode extensions keep LUT/FF overhead minimal (bit-serial reuse) and sustain **low per-core power** on PYNQ-Z2.
* **Throughput vs precision:** governed by `N` (stream length). Larger `N` improves accuracy while maintaining simple logic.
* **Neural inference:** serial MAC supports dot-products and small layers; activation can be approximated (piecewise or SC-friendly mappings) if desired.

> Tip: Ship a small `results/` README with your measured **LUT/FF/BRAM, Fmax, and power** (Vivado Power) for (a) baseline SERV and (b) SERV-Stochastic, plus a 2–3 line note on synthesis options used (flatten/hierarchy, retiming off/on, etc.). It makes your optimizations immediately visible to readers.

---

## Roadmap

* Optional **compressed streams** (C-extension interplay for code size only; SC is orthogonal).
* **Correlation control** across SNGs for improved variance/accuracy.
* **Approximate activations** in SC (e.g., tanh/sigmoid variants) with quantized baselines.
* **Lite custom opcodes** (if desired) that *alias* to mode toggles without breaking RV32I flow.

---

## Documentation

* **Design notes:** see `docs/serv_modules_v2.pdf` for port definitions, timing notes, and micro-architectural choices.
* **Slides:** the architecture block diagrams and NN/MAC callouts are included in `docs/figures/` (exported from your latest presentation).
* **Reports:** interim, short paper, and dissertation PDFs in `docs/papers/` capture motivation, experiments, and FPGA bring-up.

> Keep the README *implementation-focused*. Put long narrative, literature review, and plots in `docs/` so the front page stays developer-friendly.

---

## Acknowledgments

* **Upstream core:** Olof Kindgren’s [SERV](https://github.com/olofk/serv) (RV32I, bit-serial) — this project builds on SERV’s philosophy and codebase.
* **Board:** PYNQ-Z2 (XC7Z020).
* **Thanks:** supervisors and reviewers for feedback on reducing bullets, clarifying SC ops, and polishing figures.

---

## Cite This Work

If you use SERV-Stochastic in academic work, please cite your short paper/dissertation along with the upstream SERV repository.

```bibtex
@misc{serv-stochastic,
  title  = {SERV-Stochastic: Dual-Mode Bit-Serial RISC-V with Stochastic Computing and NN MAC},
  author = {Keerthivasan Palani and collaborators},
  year   = {2025},
  note   = {Extended SERV core with stochastic ALU and serial MAC on PYNQ-Z2},
  url    = {https://github.com/<your-username>/<your-repo>}
}
```

You can also add a `CITATION.cff` to the repo root for GitHub’s citation UI.

---

## License

This repository inherits SERV’s permissive licensing model. See [LICENSE](LICENSE).
Please retain attribution to upstream SERV where reused.

---

### Quick “What we implemented vs. what we optimized”

**Implemented**

* Runtime **dual-mode ALU** (`i_stochastic_mode`, `i_mac_mode`).
* **SC primitives** (SNG, AND/XNOR, MUX-ADD, popcount/normalize, comparators).
* **Serial MAC** path for NN-style dot-products.
* **SoC glue** to map mode bits (MMIO/CSR) without touching RV32I ISA.
* **Demos & TBs** for SC ops and MAC.

**Optimized**

* **Bit-serial reuse** of existing SERV units (adders/shifters).
* **Clock-enable/gating** to suppress toggling in SC mode.
* **Compact counters/normalizers** sized to stream length `N`.
* **Clean write-back** and minimal muxing so Fmax remains healthy.

