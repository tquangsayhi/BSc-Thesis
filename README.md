# RV32I Pipeline Optimization with Branch Prediction

Bachelor's thesis project by **Quang Le**, Embedded Systems, Tampere University.

This project extends a five-stage, 32-bit RISC-V processor developed in Verilog to study how branch prediction reduces control-hazard penalties and cycles per instruction (CPI). The target platform is the **Zynq-7020 FPGA on the PYNQ-Z1 board**. Current results are from RTL simulation; FPGA timing and resource evaluation are separate implementation steps.

## CPU versions

| Version | Implementation | Main addition |
|---|---|---|
| Baseline | `rtl/baseline/cpu_baseline.v` | Five-stage pipeline with forwarding and load-use stalling |
| BTB | `rtl/btb/cpu_btb.v` | Branch Target Buffer and two-bit direction prediction |
| BTB + RAS | `rtl/ras/cpu_ras.v` | Return Address Stack for function-return prediction |

Each CPU file contains its complete set of modules. **Compile only one CPU version at a time** with the common testbench. The versions share the top-level module name `RISC_V`.

## Architecture

- **Pipeline:** IF, ID, EX, MEM and WB.
- **Data hazards:** operand forwarding and a load-use stall that holds PC/IF-ID and inserts a bubble into ID/EX.
- **BTB:** 32 entries indexed by PC bits `[6:2]`, with tags, target addresses and two-bit saturating direction counters.
- **Prediction:** performed in IF and checked in EX. Mispredictions redirect PC and flush IF/ID and ID/EX.
- **MMIO:** benchmark completion is signalled by a store to `0x40000000`.
- **Measurements:** cycle count, valid EX instruction count, and branch/jump misprediction flushes.

### Return Address Stack

The RAS version uses two independent stacks, each eight entries deep:

| Stack | Role |
|---|---|
| IF RAS | Predicts return targets and updates speculatively when an instruction is accepted |
| EX RAS | Tracks calls and returns that reach EX and supplies recovery state |

`JAL/JALR` with `rd=x1` pushes the instruction's `PC+4`. `JALR` with `rd=x0` and `rs1=x1` uses the IF stack top to predict a return and pops the stack if nonempty. Other register conventions, including `x5`, are outside this initial RAS implementation.

On a misprediction, the IF stack restores all entries and its count from the EX stack's **next state**. This keeps the current EX instruction's push/pop while discarding changes made by younger instructions on the wrong path. Stalls prevent repeated IF updates. An empty stack falls back to BTB prediction; a full push discards the oldest entry.

The selected `predicted_next_pc` travels with the instruction through IF/ID and ID/EX. EX compares this saved prediction with the actual next PC, regardless of whether the prediction came from the BTB or RAS.

## Repository layout

| Path | Contents |
|---|---|
| `rtl/` | Complete baseline, BTB and BTB + RAS implementations |
| `Benchmarks/` | C benchmark sources |
| `Mem/` | Precompiled memory images |
| `Sim/tb_riscv_core.v` | Common simulation testbench |
| `Single Modules/` | Standalone modules retained for reference |

The combined CPU files under `rtl/` are the authoritative implementations. Do not add the reference modules to the same simulation fileset: their module definitions overlap with the combined files.

## Running a benchmark in Vivado

1. Add **one** complete CPU file from `rtl/` as a Design Source.
2. Add `Sim/tb_riscv_core.v` as a Simulation Source and select `tb_riscv_core` as the simulation top.
3. Copy the selected image from `Mem/` to a file named **`firmware.mem`**. Add it to the project and ensure it is available in the simulation working directory. Both instruction memory and data memory load this file through `$readmemh`.
4. Set the testbench's `CPU_VERSION` label to `"BASELINE"`, `"BTB"`, or `"BTB + RAS"`. This changes the printed label only; the selected RTL determines the CPU version.
5. Run behavioral simulation until the benchmark writes to `0x40000000`. The testbench prints the result and performance counters, then stops.

The testbench uses a 20 ns clock period. `CPI (x1000)` is an integer-scaled value: an output of `1314` represents a CPI of approximately `1.314`, with the remaining fraction truncated.

Use the same memory image and testbench settings when comparing CPU versions. The C programs are compiled with a RISC-V GCC toolchain into memory images. Exact build commands, startup code and linker configuration still need to be documented for reproducible image generation; editing a `.c` file does not update its `.mem` file.

## Benchmark findings

The evaluation uses array summation, matrix addition and Quicksort. BTB prediction improves CPI across all three workloads compared with the baseline. Adding RAS provides a further improvement for Quicksort.

For the tested Quicksort image:

| Metric | BTB | BTB + RAS |
|---|---:|---:|
| Result written to MMIO | 107 | 107 |
| Instructions counted | 1357 | 1357 |
| Cycles | 1828 | 1784 |
| CPI | 1.34709 | 1.31466 |
| Branch flushes | 24 | 24 |
| Jump flushes | 34 | 12 |

RAS saves **44 cycles**, a **2.41% reduction** relative to the BTB version for this image. Quicksort's calls and recursion produce returns with changing targets, which benefit from stack-based prediction. The tested array and matrix images execute no returns before MMIO completion, so RAS does not reduce their measured CPI.

## Measurement scope

The common testbench reports results and stops at the MMIO completion store. It currently does not automatically assert an expected result or enforce a timeout. The instruction counter counts valid instructions in EX, rather than retirement at WB, and printed counters are sampled before their nonblocking updates on the completion edge. These conventions must remain consistent across comparisons.

The final benchmark result is a useful check, but is not a complete ISA or algorithm-correctness test. For example, Quicksort's endpoint checksum alone does not verify every sorted element. Broader functional verification and FPGA implementation measurements are further evaluation work.
