 =============================================================
  testbench.v
  Testbench for the 5-stage Pipelined MIPS32 Processor
 =============================================================
  Loads a small hand-assembled program into instruction memory,
  drives the two-phase clock, and dumps register  memory state
  so the pipeline behaviour (including a taken branch) can be
  observed on a waveform (VCD) or in the console.

  IMPORTANT (Vivado project setup)
    This file uses $dumpfile, $display, forever, wait, and
    hierarchical references (uut.Reg[k], uut.Mem[k]) -- none of
    these are synthesizable. Add this file under
    Simulation Sources only, NOT Design Sources. The
    synthesiselaboration top module should be pipe_MIPS32,
    the simulation top module should be testbench.

  Program being executed (R0 is hard-wired to 0)

      Reg[1] = 12, Reg[2] = 5   (pre-loaded before reset)

      0 ADD   R3, R1, R2      ; R3 = 12 + 5  = 17
      1 SUB   R4, R1, R2      ; R4 = 12 - 5  = 7
      2 AND   R5, R1, R2      ; R5 = 12 & 5  = 4
      3 OR    R6, R1, R2      ; R6 = 12  5  = 13
      4 MUL   R7, R1, R2      ; R7 = 12  5  = 60
      5 ADDI  R8, R1, 10       ; R8 = 12 + 10 = 22
      6 SW    R8, 100(R0)      ; Mem[100] = 22
      7 NOP (ADD R0,R0,R0)     ; spacer so SW and BEQZ don't overlap in pipe
      8 NOP (ADD R0,R0,R0)     ; spacer
      9 LW    R9, 100(R0)      ; R9 = Mem[100] = 22
     10 BEQZ  R0, 2            ; R0==0 - branch taken, target = NPC(11)+2 = 13
     11 ADD   R10, R1, R2      ; SHOULD BE SQUASHED (branch redirect in this design)
     12 NOP (ADD R0,R0,R0)     ; spacer
     13 ADDI  R11, R1, 1       ; R11 = 12 + 1 = 13   (branch target)
     14 HLT
 =============================================================

`timescale 1ns1ns

module testbench;

    reg clk1, clk2, rst;
    integer k;

    pipe_MIPS32 uut (.clk1(clk1), .clk2(clk2), .rst(rst));

     -----------------------------------------------------------
     Two-phase non-overlapping clock generation
     -----------------------------------------------------------
    initial begin
        clk1 = 0; clk2 = 0;
        forever begin
            #5  clk1 = 1; #5 clk1 = 0;    clk1 high for 5ns
            #5  clk2 = 1; #5 clk2 = 0;    clk2 high for 5ns
        end
    end

     -----------------------------------------------------------
     Program + register initialization
     -----------------------------------------------------------
    initial begin
        rst = 1;

         instruction memory doesn't need a reset (like a real ROM);
         load it directly regardless of rst.
        for (k = 0; k  1024; k = k + 1) uut.Mem[k] = 0;

         ---- hand assemble the program (see header comment) ----
         R-type  {opcode, rs, rt, rd, 11'b0}
        uut.Mem[0]  = {6'b000000, 5'd1, 5'd2, 5'd3,  11'b0};   ADD  R3,R1,R2
        uut.Mem[1]  = {6'b000001, 5'd1, 5'd2, 5'd4,  11'b0};   SUB  R4,R1,R2
        uut.Mem[2]  = {6'b000010, 5'd1, 5'd2, 5'd5,  11'b0};   AND  R5,R1,R2
        uut.Mem[3]  = {6'b000011, 5'd1, 5'd2, 5'd6,  11'b0};   OR   R6,R1,R2
        uut.Mem[4]  = {6'b000101, 5'd1, 5'd2, 5'd7,  11'b0};   MUL  R7,R1,R2

         I-type  {opcode, rs, rt, imm16}
         NOTE back-to-back dependent instructions below (e.g.
         ADDI R8 immediately followed by SW R8, and SW immediately
         followed by LW from the same address) are intentionally
         NOT padded with NOPs. The DUT now has EXMEM and
         MEMWB-to-EX forwarding, so these RAW hazards are
         resolved by the hardware itself -- this sequence is a
         deliberate test of the forwarding logic.
        uut.Mem[5]  = {6'b001010, 5'd1, 5'd8,  16'd10};         ADDI R8,R1,10        ; R8 = 22
        uut.Mem[6]  = {6'b001001, 5'd0, 5'd8,  16'd100};        SW   R8,100(R0)      ; Mem[100] = 22 (forwarded R8)
        uut.Mem[7]  = {6'b001000, 5'd0, 5'd9,  16'd100};        LW   R9,100(R0)      ; R9 = Mem[100] = 22
        uut.Mem[8]  = {6'b000000, 5'd0, 5'd0, 5'd0,  11'b0};    NOP (memory-access ordering spacer
                                                                      LOAD's address depends on the
                                                                      STORE having actually completed
                                                                      in MEM; forwarding covers register
                                                                      hazards, not same-cycle memory
                                                                      readwrite ordering)
        uut.Mem[9]  = {6'b001110, 5'd0, 5'd0,  16'd2};          BEQZ R0, +2  - target = NPC(10)+2 = 12
        uut.Mem[10] = {6'b000000, 5'd1, 5'd2, 5'd10, 11'b0};    ADD  R10,R1,R2 (squashed)
        uut.Mem[11] = {6'b000000, 5'd0, 5'd0, 5'd0,  11'b0};    NOP (spacer)
        uut.Mem[12] = {6'b001010, 5'd1, 5'd11, 16'd1};          ADDI R11,R1,1  (branch target)
        uut.Mem[13] = {6'b111111, 20'b0, 6'b0};                 HLT

         hold reset for a couple of full clock cycles
        #40;
        rst = 0;

         load operands the cycle after reset deasserts
         (register file is cleared to 0 by rst)
        #1;
        uut.Reg[1] = 12;
        uut.Reg[2] = 5;

        $dumpfile(mips32_pipeline.vcd);
        $dumpvars(0, testbench);
    end

     -----------------------------------------------------------
     Run until HLT reaches WB, then report results
     -----------------------------------------------------------
    initial begin
        wait (uut.HALTED == 1);
        #10;
        $display(n========== Pipeline Halted ==========);
        $display(R3  (ADD)  = %0d (expect 17), uut.Reg[3]);
        $display(R4  (SUB)  = %0d (expect 7),  uut.Reg[4]);
        $display(R5  (AND)  = %0d (expect 4),  uut.Reg[5]);
        $display(R6  (OR)   = %0d (expect 13), uut.Reg[6]);
        $display(R7  (MUL)  = %0d (expect 60), uut.Reg[7]);
        $display(R8  (ADDI) = %0d (expect 22), uut.Reg[8]);
        $display(R9  (LW)   = %0d (expect 22), uut.Reg[9]);
        $display(R10 (post-branch ADD, squash behaviour) = %0d, uut.Reg[10]);
        $display(R11 (ADDI after branch) = %0d (expect 13), uut.Reg[11]);
        $display(Mem[100]   = %0d (expect 22), uut.Mem[100]);
        $display(======================================n);
        $finish;
    end

     Safety timeout in case HALTED never asserts
    initial begin
        #2000;
        $display(TIMEOUT simulation did not halt as expected.);
        $finish;
    end

endmodule