# RV32I Pipeline Optimization with Branch Prediction

Bachelor's thesis project by **Quang Le**, Embedded Systems, Tampere University.

This project extends a five-stage RV32I processor implemented in Verilog to evaluate how a Branch Target Buffer (BTB), two-bit direction prediction and a Return Address Stack (RAS) reduce control-hazard penalties and cycles per instruction (CPI).

The target platform is the **Zynq-7020 FPGA on the PYNQ-Z1 board**. The reported results are from RTL simulation. FPGA timing and resource utilization are outside the current evaluation.

## Repository structure

```text
.
├── README.md
├── benchmarks/
├── reference modules/
│   ├── btb_control.v
│   ├── btb.v
│   ├── direction_predictor_fsm.v
│   ├── new_pc_mux.v
│   ├── ras_ex.v
│   ├── ras_if.v
│   └── ras_stack.v
├── result/
│   ├── array.txt
│   ├── matrix.txt
│   └── quicksort.txt
├── rtl/
│   ├── baseline/
│   │   └── cpu_baseline.v
│   ├── btb/
│   │   └── cpu_btb.v
│   └── ras/
│       └── cpu_ras.v
├── sim/
│   ├── tb_array.v
│   ├── tb_matrix.v
│   └── tb_quicksort.v
└── software/
    ├── compile.sh
    ├── link.ld
    ├── start.s
    ├── array/
    ├── matrix_addition/
    └── quicksort/
```

| Directory | Contents |
|---|---|
| `rtl/` | Complete CPU implementations |
| `sim/` | Benchmark-specific verification testbenches |
| `software/` | Shared build files and benchmark-specific source/firmware artifacts |
| `result/` | Simulation logs for the three CPU configurations, grouped by benchmark |
| `benchmarks/` | Benchmark C sources retained for reference |
| `reference modules/` | Individual prediction modules retained for reference |

The source saved alongside each benchmark's firmware in `software/` identifies the program used for that build.

## CPU configurations

| Configuration | RTL file | Description |
|---|---|---|
| Baseline | `rtl/baseline/cpu_baseline.v` | Five-stage pipeline with forwarding and load-use stalling |
| BTB-only | `rtl/btb/cpu_btb.v` | BTB with two-bit direction prediction |
| BTB + RAS | `rtl/ras/cpu_ras.v` | BTB prediction with speculative return-address prediction and recovery |

Each RTL file contains its complete set of modules and uses the top-level module name `RISC_V`.

**Compile only one complete CPU implementation at a time.** Do not add the files in `reference modules/` alongside a complete implementation: their module definitions overlap.

The RAS implementation also exposes `RAS_ENABLE`: setting it to `0` disables the RAS, while `1` enables it.

## Architecture

- **Pipeline:** IF, ID, EX, MEM and WB.
- **Data hazards:** forwarding and load-use stalling. A load-use stall holds PC and IF/ID and inserts a bubble into ID/EX.
- **BTB:** 32 direct-mapped entries indexed by PC `[6:2]`. Each entry contains a valid bit, a 25-bit tag from PC `[31:7]`, a 32-bit target and a two-bit direction counter.
- **Direction states:** `00` strongly taken, `01` weakly taken, `10` weakly not taken and `11` strongly not taken.
- **Prediction:** performed in IF and checked in EX.
- **Recovery:** redirect PC and flush IF/ID and ID/EX.
- **Memory:** separate instruction and data arrays, each containing 4096 words of 32 bits. Both load `firmware.mem`.
- **Completion:** a store to MMIO address `0x40000000`.

### Return Address Stack

The RAS implementation uses two independent stacks, each eight entries deep:

| Stack | Role |
|---|---|
| IF RAS | Predicts return targets and updates speculatively when a fetched instruction is accepted |
| EX RAS | Tracks calls and returns reaching EX and supplies recovery state |

The implemented recognition rules are:

- `JAL` or `JALR` with `rd=x1`: push `PC+4`.
- `JALR` with `rd=x0` and `rs1=x1`: predict from the IF stack top when nonempty and pop on acceptance.

JALR recognition requires `funct3=000`. Other link-register conventions, including `x5`, are outside this implementation.

Prediction uses the stack top before the pop. EX checks the actual target, including cases where a nonzero JALR immediate or a modified `x1` makes the saved address incorrect.

On a pipeline flush, the IF stack restores **all entries and its count from the EX stack's next state**. This retains the resolving instruction's push/pop and removes younger speculative changes.

Stalls prevent repeated IF updates. An empty stack falls back to BTB prediction. A full push discards the oldest entry.

PC selection priority is:

1. EX recovery
2. RAS prediction
3. Taken BTB prediction
4. Sequential `PC+4`

The selected predicted next PC travels through IF/ID and ID/EX for verification.

## Firmware build

The evaluated firmware was built on Linux x86_64 using:

- RISC-V GCC **10.2.0**
- GNU Binutils **2.35.1**
- Python 3 for memory-image conversion

The shared build files are:

| File | Role |
|---|---|
| `software/compile.sh` | Compiles the benchmark and exports firmware and inspection files |
| `software/start.s` | Initializes the stack pointer, clears BSS and calls `main` |
| `software/link.ld` | Places the program in the 16 KiB memory region starting at zero |

The initial stack pointer is `0x4000`; the stack grows downward.

The compiler flags used for evaluation are:

```text
-O0 -march=rv32i -mabi=ilp32 -mno-relax
-T link.ld -nostartfiles -nostdlib
-Wl,-e,_start,-Map=main.map
```

`-mno-relax` disables relaxation because the custom startup does not initialize `gp`.

### Building a benchmark

The current script expects `start.s` and `link.ld` in the working directory. From `software/`, select the benchmark source:

```bash
cd software
bash compile.sh quicksort/main.c
```

For the other benchmarks:

```bash
bash compile.sh array/main.c
```

```bash
bash compile.sh matrix_addition/main.c
```

These commands assume the archived source in each directory is named `main.c`; substitute its actual filename if different.

The script writes its outputs into the current working directory:

| Output | Purpose |
|---|---|
| `main.elf` | Linked executable and symbols |
| `main.map` | Linker memory layout |
| `main.symbols` | Symbol addresses |
| `main.dis` | Disassembly |
| `firmware.bin` | Raw binary image |
| `firmware.mem` | Memory initialization image |

The conversion reads binary bytes as little-endian 32-bit words and zero-pads the image to 4096 words.

**Archive the matching outputs in the selected benchmark directory before building another benchmark.** Each build overwrites the working-directory outputs. Editing a C file does not update an existing `.mem` file.

A benchmark's firmware is shared by all three CPU configurations.

## Functional verification

Each benchmark has a dedicated testbench:

| Benchmark | Testbench | Expected result | Verification |
|---|---|---:|---|
| Array sum | `sim/tb_array.v` | 550 | MMIO scalar result |
| Matrix addition | `sim/tb_matrix.v` | 160 | MMIO result and all 16 output elements |
| Quicksort | `sim/tb_quicksort.v` | 107 | MMIO result and all 10 sorted elements |

Every matrix output element must equal `10`. The expected Quicksort array is:

```text
9, 12, 23, 34, 45, 56, 67, 77, 87, 98
```

The testbenches read output elements directly from data memory:

```verilog
uut.Data_Mem.memory[(DATA_ADDR >> 2) + i]
```

Comparisons use `!==` to detect incorrect values and X/Z values. Verification failures terminate simulation with `$fatal`. A timeout of **100,000 active clock cycles** detects missing completion.

For the evaluated builds:

| Output | Byte address | RAM word indices |
|---|---:|---|
| Quicksort `data` | `0x288` | 162–171 |
| Matrix `matC` | `0x1C8` | 114–129 |

These addresses depend on the firmware build. Check `main.symbols` after rebuilding and update the corresponding testbench if necessary.

## Running in Vivado

1. Add **one** complete CPU file from `rtl/` as a Design Source.
2. Add **one** matching testbench from `sim/` as a Simulation Source.
3. Select `tb_riscv_core` as the simulation top. The benchmark testbench files share this module name, so compile only one at a time.
4. Copy the selected benchmark's memory image to a file named **`firmware.mem`** and ensure it is available in the simulation working directory.
5. Set the testbench's `CPU_VERSION` label to match the selected CPU.
6. Launch behavioral simulation and run until PASS, failure or timeout.

`CPU_VERSION` changes the printed label only. The selected RTL and its parameters determine the CPU configuration.

Close and relaunch simulation after replacing firmware so the memory initialization blocks reload it.

The clock period is **20 ns**, and reset remains active for **100 ns**. Each experiment starts from reset, including cleared BTB valid bits and empty RAS stacks.

Keep the firmware and measurement settings identical across CPU configurations for each benchmark.

## Simulation results

All nine configurations produced the expected MMIO result. Matrix addition and Quicksort also passed every output-element comparison.

Logs are stored in:

- [Array sum results](result/array.txt)
- [Matrix addition results](result/matrix.txt)
- [Quicksort results](result/quicksort.txt)

| Benchmark | CPU | Cycles | Instructions | Branches | CPI | Branch flushes | Jump flushes |
|---|---|---:|---:|---:|---:|---:|---:|
| Array sum | Baseline | 231 | 172 | 12 | 1.343 | 11 | 2 |
| Array sum | BTB | 215 | 172 | 12 | 1.250 | 3 | 2 |
| Array sum | BTB + RAS | 215 | 172 | 12 | 1.250 | 3 | 2 |
| Matrix addition | Baseline | 1135 | 866 | 42 | 1.311 | 21 | 22 |
| Matrix addition | BTB | 1073 | 866 | 42 | 1.239 | 8 | 4 |
| Matrix addition | BTB + RAS | 1073 | 866 | 42 | 1.239 | 8 | 4 |
| Quicksort | Baseline | 2001 | 1398 | 68 | 1.431 | 45 | 79 |
| Quicksort | BTB | 1857 | 1398 | 68 | 1.328 | 24 | 28 |
| Quicksort | BTB + RAS | 1825 | 1398 | 68 | 1.305 | 24 | 12 |

Cycle reductions are calculated from unrounded cycle counts:

| Benchmark | BTB vs baseline | BTB + RAS vs baseline | Additional RAS reduction vs BTB |
|---|---:|---:|---:|
| Array sum | 6.93% | 6.93% | 0.00% |
| Matrix addition | 5.46% | 5.46% | 0.00% |
| Quicksort | 7.20% | 8.80% | 1.72% |

BTB reduces cycles across all three workloads. RAS provides an additional benefit for recursive Quicksort: jump flushes decrease from **28 to 12**, saving **32 cycles** relative to BTB-only.

Array sum and matrix addition signal completion inside `main`, before its return, so RAS provides no additional cycle reduction in their measurement windows.

Across these comparisons, the cycle reduction equals twice the reduction in total flushes, consistent with the pipeline's two-cycle recovery penalty.

## Measurement scope

Counters run after reset release. The testbench reads them before their nonblocking updates on the rising edge that observes the MMIO completion store.

| Metric | Definition |
|---|---|
| Cycles | Active clock cycles counted after reset |
| Instructions | Nonzero instruction words passing through EX; bubbles excluded |
| Branches | Valid conditional branches in EX |
| Branch/jump flushes | Control-flow recovery events associated with branches or jumps |
| CPI | Sampled cycles divided by sampled EX instruction count |

CPI is displayed to three decimal places.

The instruction counter measures EX-stage instructions rather than WB retirement. The measurement includes startup and completion-signalling code. Instructions following the completion store may already have reached EX. The same convention is used for every configuration.

Flush counters measure recovery events, not taken-branch counts. In the baseline, they count the corresponding control-flow redirects.

The results demonstrate correct benchmark outputs and reduced simulated cycle counts for these workloads. They do not establish complete RV32I compliance, exhaustive RAS recovery coverage or achieved FPGA timing. The 20 ns simulation clock does not establish an FPGA maximum operating frequency.