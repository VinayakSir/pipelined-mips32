# Pipelined MIPS32 Processor (Verilog)

A structural/behavioral Verilog implementation of a classic **5-stage pipelined MIPS32** processor — `IF → ID → EX → MEM → WB` — with hazard detection and data forwarding, built and verified in simulation.

## Overview

This project implements a subset of the MIPS32 ISA using the standard inter-stage latch convention (`IF_ID`, `ID_EX`, `EX_MEM`, `MEM_WB`), driven by a two-phase non-overlapping clock (`clk1`, `clk2`) so each stage completes cleanly within its half of the cycle.

**Supported instructions**
| Category | Instructions |
|---|---|
| R-R ALU | ADD, SUB, AND, OR, SLT, MUL |
| R-I ALU | ADDI, SUBI, SLTI |
| Memory | LW, SW |
| Branch | BEQZ, BNEQZ |
| Control | HLT |

**Key features**
- 5-stage pipeline with dedicated inter-stage registers
- EX/MEM and MEM/WB → EX **data forwarding** (`fwdA` / `fwdB` muxes) to resolve RAW hazards without inserting NOPs
- Branch resolution in EX/MEM with pipeline squashing (`TAKEN_BRANCH`)
- Synchronous, active-high reset across all pipeline stages and the register file
- Debug outputs for PC, IR, ALU result, loaded data, halt status, and forwarding-mux selects

## Repository structure

```
pipelined-mips32/
├── rtl/
│   └── pipe_MIPS32.v      # Processor design (synthesizable)
├── sim/
│   └── testbench.v        # Testbench (simulation-only)
└── README.md
```

> **Note:** `testbench.v` uses non-synthesizable constructs (`$display`, `$dumpfile`, hierarchical references). In Vivado, add it under **Simulation Sources** only — keep `pipe_MIPS32` as the synthesis top module and `testbench` as the simulation top module.

## Running the simulation

**Using Icarus Verilog:**
```bash
iverilog -o sim.out rtl/pipe_MIPS32.v sim/testbench.v
vvp sim.out
```

**Using Vivado:** create a project, add `rtl/pipe_MIPS32.v` as a design source and `sim/testbench.v` as a simulation source, then run behavioral simulation.

### Expected output

The testbench pre-loads `R1 = 12`, `R2 = 5`, runs a 14-instruction program exercising every instruction type (including a taken branch and a forwarded RAW hazard chain), and reports:

```
========== Pipeline Halted ==========
R3  (ADD)  = 17 (expect 17)
R4  (SUB)  = 7  (expect 7)
R5  (AND)  = 4  (expect 4)
R6  (OR)   = 13 (expect 13)
R7  (MUL)  = 60 (expect 60)
R8  (ADDI) = 22 (expect 22)
R9  (LW)   = 22 (expect 22)
R10 (post-branch ADD, squash behaviour) = 0
R11 (ADDI after branch) = 13 (expect 13)
Mem[100]   = 22 (expect 22)
======================================
```

## Design challenges & fixes

A few non-obvious issues came up while getting this from "simulates" to "synthesizable and correct":

- **Multiply-driven signal.** `TAKEN_BRANCH` was originally set in one always block (IF stage) and cleared in another (EX stage). Simulators tolerate this, but Vivado's synthesizer rejects it outright. Fixed by consolidating both actions into a single IF-stage always block.
- **Netlist optimized away entirely.** The module had no output ports, so synthesis couldn't find any primary output reachable from internal state and deleted the whole design. Added explicit outputs (`PC_out`, `IR_out`, `ALUOut_out`, `LMD_out`, `HALTED_out`, plus forwarding-mux debug outputs).
- **RAW hazards on back-to-back instructions.** Rather than padding the program with NOPs, added EX/MEM and MEM/WB → EX forwarding logic, verified against a deliberately hazard-heavy instruction sequence (`ADDI R8...` immediately followed by `SW R8...`).
- **Silent pipeline deadlock.** A missing `` `timescale `` directive meant the `#2` intra-assignment delays didn't line up with the testbench's clock timing, and the PC never advanced. Fixed by adding `` `timescale 1ns/1ns `` to match the testbench.
- **Design deleted under synthesis due to undefined reset state.** All state previously relied on the testbench forcing initial values via hierarchical references (`uut.PC = 0`, etc.), which only works in simulation. Added a proper synchronous reset to every pipeline stage and the register file so the design has defined power-up behavior in real hardware.

## Author

Vinayak