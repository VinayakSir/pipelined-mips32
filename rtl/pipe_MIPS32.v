// =============================================================
//  pipe_MIPS32.v
//  5-Stage Pipelined MIPS32 Processor  (IF -> ID -> EX -> MEM -> WB)
// =============================================================
//  Author : <Your Name>
//  Description:
//    A structural/behavioral Verilog model of a classic 5-stage
//    pipelined RISC processor (MIPS32 subset), implemented using
//    the standard inter-stage latch convention:
//        IF_ID, ID_EX, EX_MEM, MEM_WB
//    Two-phase non-overlapping clocking (clk1, clk2) is used so
//    that each pipeline stage completes cleanly within its half
//    of the clock cycle, avoiding race conditions between reads
//    and writes on the same edge.
//
//  Supported instructions:
//    R-R  ALU : ADD, SUB, AND, OR, SLT, MUL
//    R-I  ALU : ADDI, SUBI, SLTI
//    Memory   : LW, SW
//    Branch   : BEQZ, BNEQZ
//    Misc     : HLT
//
//  FIX LOG:
//    - TAKEN_BRANCH was previously driven from TWO separate always
//      blocks (IF stage set it to 1, EX stage cleared it to 0),
//      both triggered on posedge clk1. Verilog simulators tolerate
//      multiple drivers on a reg (with implementation-defined
//      ordering), but Vivado's synthesizer/elaborator rejects this
//      as a "multiply driven signal" error -- this was blocking RTL
//      generation. Fixed by merging both actions into the single
//      IF-stage always block, so TAKEN_BRANCH now has exactly one
//      driver.
//    - Added output ports (PC_out, IR_out, ALUOut_out, LMD_out,
//      HALTED_out) since the module previously had none, which
//      caused synthesis to optimize the entire design away into
//      an empty netlist (nothing was reachable from a primary
//      output).
//    - Added EX/MEM and MEM/WB-to-EX data forwarding (fwdA/fwdB
//      muxes) to correctly resolve RAW hazards between
//      back-to-back dependent instructions (e.g. ADDI R8,... 
//      immediately followed by SW R8,...), instead of relying on
//      manually inserted NOPs in the test program.
//    - Added missing `timescale directive (was present in testbench.v
//      but absent here) -- without it the #2 intra-assignment delays
//      in this file don't line up with the testbench's 1ns/1ns clock,
//      and the whole pipeline silently deadlocks (PC never advances).
//    - Added a proper synchronous, active-high `rst` to every pipeline
//      stage and the register file. Previously ALL state (PC, HALTED,
//      pipeline latches, Reg[]) relied on the *testbench* forcing
//      initial values via hierarchical references (uut.PC = 0, etc.),
//      which only works in simulation. With no defined power-up state
//      in the RTL itself, Yosys's optimizer could prove every register
//      was permanently unreachable/undefined and deleted the entire
//      design during synthesis (0 cells in the netlist).
// =============================================================

`timescale 1ns/1ns

module pipe_MIPS32 (
    clk1, clk2, rst,
    PC_out,          // current program counter
    IR_out,          // instruction currently in WB stage
    ALUOut_out,      // ALU result currently in WB stage
    LMD_out,         // loaded memory data currently in WB stage
    HALTED_out,      // goes high when HLT completes
    fwdA_out,        // forwarding-mux select for operand A (debug)
    fwdB_out         // forwarding-mux select for operand B (debug)
);

    input  clk1, clk2;                     // Two-phase clock
    input  rst;                            // synchronous, active-high reset

    
    output [31:0] PC_out;
    output [31:0] IR_out;
    output [31:0] ALUOut_out;
    output [31:0] LMD_out;
    output        HALTED_out;
    output [1:0]  fwdA_out;
    output [1:0]  fwdB_out;

    assign PC_out     = PC;
    assign IR_out     = MEM_WB_IR;
    assign ALUOut_out = MEM_WB_ALUOut;
    assign LMD_out    = MEM_WB_LMD;
    assign HALTED_out = HALTED;
    assign fwdA_out   = fwdA;
    assign fwdB_out   = fwdB;

    // ---------------------------------------------------------
    // Pipeline (inter-stage) registers
    // ---------------------------------------------------------
    reg [31:0] PC, IF_ID_IR, IF_ID_NPC;

    reg [31:0] ID_EX_IR, ID_EX_NPC, ID_EX_A, ID_EX_B, ID_EX_Imm;
    reg [2:0]  ID_EX_type, EX_MEM_type, MEM_WB_type;

    reg [31:0] EX_MEM_IR, EX_MEM_ALUOut, EX_MEM_B;
    reg        EX_MEM_cond;

    reg [31:0] MEM_WB_IR, MEM_WB_ALUOut, MEM_WB_LMD;

    // ---------------------------------------------------------
    // Architectural state
    // ---------------------------------------------------------
    reg [31:0] Reg [0:31];        // Register bank (32 x 32-bit)
    reg [31:0] Mem [0:1023];      // Unified 1K x 32-bit memory
                                   // (instructions + data, word-addressed)

    // ---------------------------------------------------------
    // Opcodes
    // ---------------------------------------------------------
    parameter ADD=6'b000000, SUB=6'b000001, AND=6'b000010, OR=6'b000011,
              SLT=6'b000100, MUL=6'b000101, HLT=6'b111111, LW=6'b001000,
              SW=6'b001001, ADDI=6'b001010, SUBI=6'b001011, SLTI=6'b001100,
              BNEQZ=6'b001101, BEQZ=6'b001110;

    // ---------------------------------------------------------
    // Instruction "type" tags used internally by the pipeline
    // ---------------------------------------------------------
    parameter RR_ALU=3'b000, RM_ALU=3'b001, LOAD=3'b010, STORE=3'b011,
              BRANCH=3'b100, HALT=3'b101;

    reg HALTED;
        // Set after HLT instruction is completed (in WB stage)

    reg TAKEN_BRANCH;
        // Required to disable (squash) instructions fetched
        // incorrectly after a taken branch
        // NOTE: single driver only -- see FIX LOG above.

    // ---------------------------------------------------------
    // Forwarding (data hazard) support
    // ---------------------------------------------------------
    // Without forwarding, an instruction that reads a register in
    // ID can get a stale value if an earlier instruction that
    // writes that same register hasn't reached WB yet (classic
    // RAW hazard). Rather than padding the program with NOPs,
    // we detect the hazard combinationally and forward the
    // most recent in-flight result directly into the EX stage.
    //
    // fwdA / fwdB select where the EX stage should actually take
    // its A / B operand from:
    //   00 : no hazard -> use ID_EX_A / ID_EX_B (register file value)
    //   01 : forward from EX_MEM_ALUOut (result one stage ahead, not yet in Reg[])
    //   10 : forward from MEM_WB_ALUOut (result two stages ahead)
    //   11 : forward from MEM_WB_LMD    (load result two stages ahead)
    reg [1:0] fwdA, fwdB;

    // destination register of the instruction currently in EX/MEM
    // and MEM/WB, valid only for instruction types that actually
    // write a register (RR_ALU, RM_ALU, LOAD)
    wire [4:0] EX_MEM_dest = (EX_MEM_type == RR_ALU) ? EX_MEM_IR[15:11] :
                             (EX_MEM_type == RM_ALU) ? EX_MEM_IR[20:16] :
                             (EX_MEM_type == LOAD)   ? EX_MEM_IR[20:16] : 5'd0;

    wire EX_MEM_writes_reg = (EX_MEM_type == RR_ALU) ||
                             (EX_MEM_type == RM_ALU) ||
                             (EX_MEM_type == LOAD);

    wire [4:0] MEM_WB_dest = (MEM_WB_type == RR_ALU) ? MEM_WB_IR[15:11] :
                             (MEM_WB_type == RM_ALU) ? MEM_WB_IR[20:16] :
                             (MEM_WB_type == LOAD)   ? MEM_WB_IR[20:16] : 5'd0;

    wire MEM_WB_writes_reg = (MEM_WB_type == RR_ALU) ||
                             (MEM_WB_type == RM_ALU) ||
                             (MEM_WB_type == LOAD);

    // rs / rt of the instruction currently sitting in ID/EX,
    // i.e. about to be used by the EX stage this cycle
    wire [4:0] ID_EX_rs = ID_EX_IR[25:21];
    wire [4:0] ID_EX_rt = ID_EX_IR[20:16];

    // Forwarding priority: prefer the CLOSEST producer (EX/MEM
    // is one cycle ahead, so it holds the most recent result).
    // Register R0 is hard-wired to 0 and must never be forwarded.
    always @(*) begin
        // operand A (rs)
        if (EX_MEM_writes_reg && (EX_MEM_dest != 5'd0) && (EX_MEM_dest == ID_EX_rs))
            fwdA = 2'b01;                              // forward EX_MEM_ALUOut
        else if (MEM_WB_writes_reg && (MEM_WB_dest != 5'd0) && (MEM_WB_dest == ID_EX_rs))
            fwdA = (MEM_WB_type == LOAD) ? 2'b11 : 2'b10; // forward MEM_WB_LMD / ALUOut
        else
            fwdA = 2'b00;                              // no hazard

        // operand B (rt) -- only meaningful for RR_ALU and STORE,
        // where ID_EX_B is actually used as a source operand
        if (EX_MEM_writes_reg && (EX_MEM_dest != 5'd0) && (EX_MEM_dest == ID_EX_rt))
            fwdB = 2'b01;
        else if (MEM_WB_writes_reg && (MEM_WB_dest != 5'd0) && (MEM_WB_dest == ID_EX_rt))
            fwdB = (MEM_WB_type == LOAD) ? 2'b11 : 2'b10;
        else
            fwdB = 2'b00;
    end

    // Actual operand values used by the EX stage, after forwarding
    wire [31:0] EX_A = (fwdA == 2'b01) ? EX_MEM_ALUOut :
                       (fwdA == 2'b10) ? MEM_WB_ALUOut :
                       (fwdA == 2'b11) ? MEM_WB_LMD    : ID_EX_A;

    wire [31:0] EX_B = (fwdB == 2'b01) ? EX_MEM_ALUOut :
                       (fwdB == 2'b10) ? MEM_WB_ALUOut :
                       (fwdB == 2'b11) ? MEM_WB_LMD    : ID_EX_B;

    // ===========================================================
    // (a) IF Stage : Instruction Fetch
    // ===========================================================
    always @(posedge clk1)
        if (rst)
        begin
            PC           <= #2 0;
            IF_ID_IR     <= #2 0;
            IF_ID_NPC    <= #2 0;
            TAKEN_BRANCH <= #2 0;
        end
        else if (HALTED == 0)
        begin
            // default: clear branch-taken flag every cycle
            // (this used to live in the EX stage always block --
            //  moved here so TAKEN_BRANCH has a single driver)
            TAKEN_BRANCH <= #2 1'b0;

            if (((EX_MEM_IR[31:26] == BEQZ)  && (EX_MEM_cond == 1)) ||
                ((EX_MEM_IR[31:26] == BNEQZ) && (EX_MEM_cond == 0)))
            begin
                // Branch resolved as taken in EX/MEM -> redirect fetch
                IF_ID_IR      <= #2 Mem[EX_MEM_ALUOut];
                TAKEN_BRANCH  <= #2 1'b1;
                IF_ID_NPC     <= #2 EX_MEM_ALUOut + 1;
                PC            <= #2 EX_MEM_ALUOut + 1;
            end
            else
            begin
                IF_ID_IR      <= #2 Mem[PC];
                IF_ID_NPC     <= #2 PC + 1;
                PC            <= #2 PC + 1;
            end
        end

    // ===========================================================
    // (b) ID Stage : Instruction Decode / Register Fetch
    // ===========================================================
    always @(posedge clk2)
        if (rst)
        begin
            ID_EX_A    <= #2 0;
            ID_EX_B    <= #2 0;
            ID_EX_NPC  <= #2 0;
            ID_EX_IR   <= #2 0;
            ID_EX_Imm  <= #2 0;
            ID_EX_type <= #2 RR_ALU;
        end
        else if (HALTED == 0)
        begin
            if (IF_ID_IR[25:21] == 5'b00000) ID_EX_A <= 0;
            else ID_EX_A   <= #2 Reg[IF_ID_IR[25:21]];   // "rs"

            if (IF_ID_IR[20:16] == 5'b00000) ID_EX_B <= 0;
            else ID_EX_B   <= #2 Reg[IF_ID_IR[20:16]];   // "rt"

            ID_EX_NPC  <= #2 IF_ID_NPC;
            ID_EX_IR   <= #2 IF_ID_IR;
            ID_EX_Imm  <= #2 {{16{IF_ID_IR[15]}}, {IF_ID_IR[15:0]}}; // sign extend

            case (IF_ID_IR[31:26])
                ADD,SUB,AND,OR,SLT,MUL : ID_EX_type <= #2 RR_ALU;
                ADDI,SUBI,SLTI         : ID_EX_type <= #2 RM_ALU;
                LW                     : ID_EX_type <= #2 LOAD;
                SW                     : ID_EX_type <= #2 STORE;
                BNEQZ,BEQZ             : ID_EX_type <= #2 BRANCH;
                HLT                    : ID_EX_type <= #2 HALT;
                default                : ID_EX_type <= #2 HALT;  // invalid opcode
            endcase
        end

    // ===========================================================
    // (c) EX Stage : Execution / Effective Address Computation
    // ===========================================================
    always @(posedge clk1)
        if (rst)
        begin
            EX_MEM_type   <= #2 RR_ALU;
            EX_MEM_IR     <= #2 0;
            EX_MEM_ALUOut <= #2 0;
            EX_MEM_B      <= #2 0;
            EX_MEM_cond   <= #2 0;
        end
        else if (HALTED == 0)
        begin
            EX_MEM_type  <= #2 ID_EX_type;
            EX_MEM_IR    <= #2 ID_EX_IR;

            case (ID_EX_type)
                RR_ALU: begin
                    case (ID_EX_IR[31:26])
                        ADD:     EX_MEM_ALUOut <= #2 EX_A + EX_B;
                        SUB:     EX_MEM_ALUOut <= #2 EX_A - EX_B;
                        AND:     EX_MEM_ALUOut <= #2 EX_A & EX_B;
                        OR:      EX_MEM_ALUOut <= #2 EX_A | EX_B;
                        SLT:     EX_MEM_ALUOut <= #2 EX_A < EX_B;
                        MUL:     EX_MEM_ALUOut <= #2 EX_A * EX_B;
                        default: EX_MEM_ALUOut <= #2 32'hxxxxxxxx;
                    endcase
                end

                RM_ALU: begin
                    case (ID_EX_IR[31:26])
                        ADDI:    EX_MEM_ALUOut <= #2 EX_A + ID_EX_Imm;
                        SUBI:    EX_MEM_ALUOut <= #2 EX_A - ID_EX_Imm;
                        SLTI:    EX_MEM_ALUOut <= #2 EX_A < ID_EX_Imm;
                        default: EX_MEM_ALUOut <= #2 32'hxxxxxxxx;
                    endcase
                end

                LOAD, STORE: begin
                    EX_MEM_ALUOut <= #2 EX_A + ID_EX_Imm;
                    EX_MEM_B      <= #2 EX_B;   // forwarded value of the store's data operand
                end

                BRANCH: begin
                    EX_MEM_ALUOut <= #2 ID_EX_NPC + ID_EX_Imm;
                    EX_MEM_cond   <= #2 (EX_A == 0);
                end
            endcase
        end

    // ===========================================================
    // (d) MEM Stage : Memory Access / Branch Completion
    // ===========================================================
    always @(posedge clk2)
        if (rst)
        begin
            MEM_WB_type   <= #2 RR_ALU;
            MEM_WB_IR     <= #2 0;
            MEM_WB_ALUOut <= #2 0;
            MEM_WB_LMD    <= #2 0;
        end
        else if (HALTED == 0)
        begin
            MEM_WB_type <= #2 EX_MEM_type;
            MEM_WB_IR   <= #2 EX_MEM_IR;

            case (EX_MEM_type)
                RR_ALU, RM_ALU:
                    MEM_WB_ALUOut <= #2 EX_MEM_ALUOut;

                LOAD:
                    MEM_WB_LMD    <= #2 Mem[EX_MEM_ALUOut];

                STORE:
                    if (TAKEN_BRANCH == 0)          // disable write if squashed
                        Mem[EX_MEM_ALUOut] <= #2 EX_MEM_B;
            endcase
        end

    // ===========================================================
    // (e) WB Stage : Register Write-back
    // ===========================================================
    integer r;
    always @(posedge clk1)
        if (rst)
        begin
            HALTED <= #2 1'b0;
            for (r = 0; r < 32; r = r + 1)
                Reg[r] <= #2 32'b0;
        end
        else
        begin
            if (TAKEN_BRANCH == 0)   // disable write-back if branch squashed it
            case (MEM_WB_type)
                RR_ALU: Reg[MEM_WB_IR[15:11]] <= #2 MEM_WB_ALUOut;   // "rd"
                RM_ALU: Reg[MEM_WB_IR[20:16]] <= #2 MEM_WB_ALUOut;   // "rt"
                LOAD:   Reg[MEM_WB_IR[20:16]] <= #2 MEM_WB_LMD;      // "rt"
                HALT:   HALTED <= #2 1'b1;
            endcase
        end

endmodule