/******************************************************************************
 * Module: RISC_V
 * Author: Quang Le
 * Project: Bachelor's Thesis - RISC-V Pipeline Performance Optimization
 * File: ras_cpu.v
 *
 * Description:
 *   Five-stage RV32I processor (IF, ID, EX, MEM, WB) with data forwarding,
 *   load-use hazard handling, MMIO, a BTB with 2-bit prediction counters,
 *   and a Return Address Stack (RAS).
 *
 *   This file combines the CPU and all RAS modules for convenient compilation.
 *   The RISC_V top-level interface remains compatible with the existing
 *   simulation testbench.
 *
 * RAS Implementation:
 *
 *   1. Two Separate Stacks
 *      - RAS_IF maintains speculative state for early return prediction.
 *      - RAS_EX maintains state for valid instructions reaching EX.
 *      - Both instantiate independent storage using the ras_stack module.
 *      - Default depth: 8 entries, each holding a 32-bit return address.
 *      - Entry 0 is the top; a count tracks the number of valid entries.
 *
 *   2. Call and Return Recognition
 *      - JAL/JALR with rd=x1: push the instruction's PC+4.
 *      - JALR with rd=x0 and rs1=x1: pop if the stack is not empty.
 *      - JALR recognition also checks funct3=000.
 *      - JALR with rd=x1 and rs1=x1 performs a push only.
 *      - This version supports the x1 calling convention; x5 and coroutine
 *        RAS hints are not implemented.
 *
 *   3. IF Prediction and Speculative Updates
 *      - A recognized return uses the current RAS_IF top as its prediction.
 *      - The top is read before the pop at the accepting clock edge.
 *      - An empty RAS falls back to the existing BTB/PC+4 prediction.
 *      - Call targets continue to use the existing BTB prediction path.
 *      - RAS_IF updates only when IF/ID accepts an instruction:
 *            IF_ID_Write && !flush && !reset
 *      - A stall therefore cannot repeat a push or pop.
 *
 *   4. EX Updates and Misprediction Recovery
 *      - RAS_EX updates once per valid EX instruction.
 *      - Its update is not blocked by ControlStall or flush: the current EX
 *        instruction completes while younger instructions are held or killed.
 *      - On a flush, RAS_IF restores all entries and the count from the NEXT
 *        state of RAS_EX, including the current EX instruction's push/pop.
 *      - This retains the action of the instruction causing the flush while
 *        removing RAS changes made by younger, wrong-path instructions.
 *      - Restoring the full stack also recovers data overwritten by a
 *        speculative push when the stack was full.
 *
 *   5. Prediction Metadata and Target Verification
 *      - IF/ID and ID/EX carry predicted_next_pc and ras_used alongside the
 *        instruction and existing BTB metadata.
 *      - EX compares the actual next PC with the prediction IF really used.
 *      - JALR actual targets are calculated from the forwarded rs1 operand
 *        plus the immediate, with bit 0 cleared.
 *      - BTB training is independent of flush: a stale BTB entry can be
 *        updated even when the RAS prediction was correct.
 *      - ras_used identifies RAS predictions for performance counters.
 *
 *   6. PC Selection and Pipeline Control
 *      - PC selection priority:
 *            EX recovery > RAS return prediction > taken BTB target > PC+4
 *      - predicted_next_pc excludes the EX recovery override.
 *      - Recovery takes priority over the normal PC write-enable stall.
 *      - IF/ID holds prediction metadata during a stall.
 *      - Flushes and ID/EX bubbles clear metadata with the instruction.
 *      - IF/ID asynchronous reset and synchronous flush are separate branches.
 *
 *   7. Stack Boundaries and Configuration
 *      - Empty pop: no change.
 *      - Full push: discard the oldest entry and retain the newest address.
 *      - Reset: clear both stacks and their counts.
 *      - RAS_ENABLE=0 disables RAS prediction and call/return updates.
 *      - RAS_DEPTH sets the stack capacity; COUNT_WIDTH represents 0..DEPTH.
 *
 * Scope:
 *   RAS predicts return targets; architectural control flow is still verified
 *   in EX. Nonzero return immediates or changes to x1 may cause a prediction
 *   miss, which uses the normal flush and redirect mechanism.
 *
 *   Recovery assumes the current in-order pipeline resolves control flow at
 *   EX. RAS_EX is resolved state, not WB-retired state; later-stage exceptions
 *   or replay would require additional recovery handling.
 ******************************************************************************/
module program_counter (
    input wire clk,
    input wire reset,
    input wire write_enable,
    input wire [31:0] next_pc,
    output reg [31:0] pc
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'b0; // Reset PC to 0
        end else begin
            if (write_enable) begin
                pc <= next_pc; // Update PC to next value
            end
        end
    end
endmodule

module pc_adder (
    input [31:0] pc_in,
    output reg [31:0] pc_next
);
    always @(*) begin
        pc_next = pc_in + 4; // Increment PC by 4 for the next instruction
    end
endmodule

// CHANGED (RAS): Separate the IF prediction from the EX recovery override.
module pc_mux (
    input wire [31:0] pc_plus_4,
    input wire btb_hit,
    input wire [1:0] btb_fsm_prediction,
    input wire [31:0] btb_branch_target,
    input wire ras_use,
    input wire [31:0] ras_target_pc,
    input wire [31:0] branch_target,
    input wire misprediction,
    output reg [31:0] predicted_next_pc,
    output reg [31:0] next_pc
);
    always @(*) begin
        predicted_next_pc = pc_plus_4;
        next_pc = pc_plus_4;
        if (ras_use)
            predicted_next_pc = ras_target_pc;
        else if (btb_hit && !btb_fsm_prediction[1])
            predicted_next_pc = btb_branch_target;

        next_pc = predicted_next_pc;
        if (misprediction)
            next_pc = branch_target;
    end
endmodule

module instruction_memory (
    input wire [31:0] read_address,
    output reg [31:0] instruction_out
);
    reg [31:0] instr_ram [0:4095];
    initial begin
        $readmemh("firmware.mem", instr_ram);
    end
    always @(*) begin
        instruction_out = instr_ram[read_address[13:2]]; 
    end
endmodule

// ============================================================================
// IF/ID Pipeline Register
// ============================================================================
module if_id_register (
input wire clk,
input wire reset,
input wire write_enable,
input wire flush,
// Inputs coming FROM the IF stage
input wire [31:0] pc_in,
input wire [31:0] instruction_in,
input wire [31:0] pc_plus_4_in,
input wire btb_hit_in,
input wire [1:0] btb_prediction_in,
input wire [31:0] btb_target_pc_in,
// ADDED (RAS): Actual IF-selected next PC and prediction source.
input wire [31:0] predicted_next_pc_in,
input wire ras_used_in,
// Outputs going TO the ID stage
output reg [31:0] pc_out,
output reg [31:0] instruction_out,
output reg [31:0] pc_plus_4_out,
output reg btb_hit_out,
output reg [1:0] btb_prediction_out,
output reg [31:0] btb_target_pc_out,
// ADDED (RAS): Held/cleared with the associated instruction.
output reg [31:0] predicted_next_pc_out,
output reg ras_used_out
);
    always @(posedge clk or posedge reset) begin
        // CHANGED (RAS): Keep asynchronous reset separate from synchronous
        // flush so both infer unambiguously when adding the new metadata.
        if (reset) begin
            pc_out <= 32'b0;
            instruction_out <= 32'b0;
            pc_plus_4_out <= 32'b0;
            btb_hit_out <= 1'b0;
            btb_prediction_out <= 2'b0;
            btb_target_pc_out <= 32'b0;
            predicted_next_pc_out <= 32'b0;
            ras_used_out <= 1'b0;
        end else if (flush) begin
            pc_out <= 32'b0;
            instruction_out <= 32'b0;
            pc_plus_4_out <= 32'b0;
            btb_hit_out <= 1'b0;
            btb_prediction_out <= 2'b0;
            btb_target_pc_out <= 32'b0;
            // ADDED (RAS): Reset/flush/bubble clears prediction metadata.
            predicted_next_pc_out <= 32'b0;
            ras_used_out <= 1'b0;
        end else begin
            if (write_enable) begin
                pc_out <= pc_in;
                instruction_out <= instruction_in;
                pc_plus_4_out <= pc_plus_4_in;
                btb_hit_out <= btb_hit_in;
                btb_prediction_out <= btb_prediction_in;
                btb_target_pc_out <= btb_target_pc_in;
                // ADDED (RAS): Capture only when the instruction advances.
                predicted_next_pc_out <= predicted_next_pc_in;
                ras_used_out <= ras_used_in;
            end
        end
    end
endmodule

module register_file (
    input wire clk,
    input wire reset,
    input wire [4:0] read_reg1,
    input wire [4:0] read_reg2,
    input wire [4:0] write_reg,
    input wire [31:0] write_data,
    input wire reg_write,
    output reg [31:0] read_data1,
    output reg [31:0] read_data2
);
    reg [31:0] registers [0:31]; 
    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0; 
            end
        end else if (reg_write && write_reg != 5'b00000) begin
            registers[write_reg] <= write_data; 
        end
    end
    always @(*) begin
        if (reg_write && write_reg == read_reg1 && write_reg != 5'b0) begin
            read_data1 = write_data;
        end else begin
            read_data1 = registers[read_reg1]; 
        end
        if (reg_write && write_reg == read_reg2 && write_reg != 5'b0) begin
            read_data2 = write_data;
        end else begin
            read_data2 = registers[read_reg2]; 
    end
    end
endmodule

module main_control_unit (
    input wire [6:0] opcode,
    output reg RegWrite,
    output reg ALUSrc,
    output reg MemToReg,
    output reg MemRead,
    output reg MemWrite,
    output reg Branch,
    output reg Jump, // <-- ADDED: Jump control signal
    output reg [1:0] ALUOp,
    output reg ALUSrcA,
    output reg JALR_signal
);
    always @(*) begin
        // Default values to prevent latches
        RegWrite = 1'b0; ALUSrc = 1'b0; MemToReg = 1'b0;
        MemRead = 1'b0; MemWrite = 1'b0; Branch = 1'b0;
        Jump = 1'b0; ALUOp = 2'b00; ALUSrcA = 1'b0; JALR_signal = 1'b0;
        case (opcode)
            7'b0110011: begin // R-Type (ADD, SUB, XOR, etc.)
                RegWrite = 1'b1;
                ALUOp    = 2'b10;
            end
            7'b0010011: begin // I-Type Math (ADDI, XORI, etc.)
                ALUSrc   = 1'b1;
                RegWrite = 1'b1;
                ALUOp    = 2'b10;
            end
            7'b0000011: begin // I-Type Loads (LW, LH, LB)
                ALUSrc   = 1'b1;
                MemToReg = 1'b1;
                RegWrite = 1'b1;
                MemRead  = 1'b1;
                ALUOp    = 2'b00; 
            end
            7'b0100011: begin // S-Type Stores (SW, SH, SB)
                ALUSrc   = 1'b1;
                MemWrite = 1'b1;
                ALUOp    = 2'b00; 
            end
            7'b1100011: begin // B-Type Branches (BEQ, BNE, etc.)
                Branch   = 1'b1;
                ALUOp    = 2'b01; 
            end
            7'b1101111: begin // J-Type Jump and Link (JAL)
                RegWrite = 1'b1; 
                Jump     = 1'b1;
                ALUOp    = 2'b00; 
            end
            7'b0110111: begin 
                RegWrite = 1'b1;
                ALUSrc   = 1'b1;
                ALUOp    = 2'b11; // ALU will just add 0 + Immediate
            end
            7'b0010111: begin 
                ALUSrc   = 1'b1;
                RegWrite = 1'b1;
                ALUSrcA   = 1'b1;
                ALUOp    = 2'b00; // ALU will add PC + Immediate
            end
            7'b1100111: begin
                RegWrite = 1'b1; 
                Jump     = 1'b1;
                ALUSrc   = 1'b1;
                ALUOp    = 2'b00;
                JALR_signal = 1'b1;
            end
            default: begin end
        endcase
    end
endmodule

module immediate_generator (
    input wire [31:0] instruction,
    output reg [31:0] immediate
);
    always @(*) begin
        case (instruction[6:0])
            7'b0010011, // I-Type (ADDI, XORI, etc.)
            7'b0000011: // I-Type Loads (LW, LH, LB)
                immediate = {{20{instruction[31]}}, instruction[31:20]}; 
            7'b0100011: // S-Type Stores (SW, SH, SB)
                immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; 
            7'b1100011: // B-Type Branches (BEQ, BNE, etc.)
                immediate = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0}; 
            7'b0110111: // U-Type (LUI)
                immediate = { instruction[31:12], 12'b0 };
            7'b1101111: // J-Type Jump and Link (JAL)
                immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0}; 
            7'b0010111: // U-Type (AUIPC)
                immediate = { instruction[31:12], 12'b0 };
            7'b1100111: // I-Type Jump and Link Register (JALR)
                immediate = {{20{instruction[31]}}, instruction[31:20]};
            default:
                immediate = 32'b0; 
        endcase
    end
endmodule

// ============================================================================
// ID/EX Pipeline Register
// ============================================================================
module id_ex_register (
input wire clk,
input wire reset,
input wire stall,  
input wire flush,  
// Inputs coming FROM the ID stage
input wire [31:0] pc_in,
input wire [31:0] instruction_in,
input wire [31:0] read_data1_in,
input wire [31:0] reg_to_mux_in,
input wire [31:0] immgen_wire_in,
input wire ALUSrc_in,
input wire MemToReg_in,
input wire MemRead_in,
input wire MemWrite_in,
input wire Branch_in,
input wire RegWrite_in,
input wire Jump_in,
input wire ALUSrcA_in,
input wire JALR_signal_in,
input wire [1:0] ALUOp_in,
input wire [4:0] rd_in, // Destination register for write-back
input wire [31:0] pc_plus_4_in, // <-- ADDED: For JAL return address
input wire btb_hit_in,
input wire [1:0] btb_prediction_in,
input wire [31:0] btb_target_pc_in,
// ADDED (RAS): Actual IF-selected next PC and prediction source.
input wire [31:0] predicted_next_pc_in,
input wire ras_used_in,
// Outputs going TO the EX stage
output reg [31:0] pc_out,
output reg [31:0] instruction_out,
output reg [31:0] read_data1_out,
output reg [31:0] reg_to_mux_out,
output reg [31:0] immgen_wire_out,
output reg ALUSrc_out,
output reg MemToReg_out,
output reg MemRead_out,
output reg MemWrite_out,
output reg Branch_out,
output reg RegWrite_out,
output reg Jump_out,
output reg ALUSrcA_out,
output reg JALR_signal_out,
output reg [1:0] ALUOp_out,
output reg [4:0] rd_out,
output reg [31:0] pc_plus_4_out,
output reg btb_hit_out,
output reg [1:0] btb_prediction_out,
output reg [31:0] btb_target_pc_out,
// ADDED (RAS): Held/cleared with the associated instruction.
output reg [31:0] predicted_next_pc_out,
output reg ras_used_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 32'b0;
            instruction_out <= 32'b0;
            read_data1_out <= 32'b0;
            reg_to_mux_out <= 32'b0;
            immgen_wire_out <= 32'b0;
            ALUSrc_out <= 1'b0;
            MemToReg_out <= 1'b0;
            MemRead_out <= 1'b0;
            MemWrite_out <= 1'b0;
            Branch_out <= 1'b0;
            RegWrite_out <= 1'b0;
            Jump_out <= 1'b0;
            ALUOp_out <= 2'b00;
            rd_out <= 5'b0;
            pc_plus_4_out <= 32'b0;
            ALUSrcA_out <= 1'b0;
            JALR_signal_out <= 1'b0;
            btb_hit_out <= 1'b0;
            btb_prediction_out <= 2'b0;
            btb_target_pc_out <= 32'b0;
            // ADDED (RAS): Reset/flush/bubble clears prediction metadata.
            predicted_next_pc_out <= 32'b0;
            ras_used_out <= 1'b0;
        end else if (flush||stall) begin
            pc_out <= 32'b0;
            instruction_out <= 32'b0;
            read_data1_out <= 32'b0;
            reg_to_mux_out <= 32'b0;
            immgen_wire_out <= 32'b0;
            ALUSrc_out <= 1'b0;
            MemToReg_out <= 1'b0;
            MemRead_out <= 1'b0;
            MemWrite_out <= 1'b0;
            Branch_out <= 1'b0;
            RegWrite_out <= 1'b0;
            Jump_out <= 1'b0;
            ALUOp_out <= 2'b00;
            rd_out <= 5'b0;
            pc_plus_4_out <= 32'b0;
            ALUSrcA_out <= 1'b0;
            JALR_signal_out <= 1'b0;
            btb_hit_out <= 1'b0;
            btb_prediction_out <= 2'b0;
            btb_target_pc_out <= 32'b0;
            // ADDED (RAS): Reset/flush/bubble clears prediction metadata.
            predicted_next_pc_out <= 32'b0;
            ras_used_out <= 1'b0;
        end else begin
            pc_out <= pc_in;
            instruction_out <= instruction_in;
            read_data1_out <= read_data1_in;
            reg_to_mux_out <= reg_to_mux_in;
            immgen_wire_out <= immgen_wire_in;
            ALUSrc_out <= ALUSrc_in;
            MemToReg_out <= MemToReg_in;    
            MemRead_out <= MemRead_in;
            MemWrite_out <= MemWrite_in;
            Branch_out <= Branch_in;
            RegWrite_out <= RegWrite_in;
            Jump_out <= Jump_in;
            ALUOp_out <= ALUOp_in;
            rd_out <= rd_in;
            pc_plus_4_out <= pc_plus_4_in;
            ALUSrcA_out <= ALUSrcA_in;
            JALR_signal_out <= JALR_signal_in;
            btb_hit_out <= btb_hit_in;
            btb_prediction_out <= btb_prediction_in;
            btb_target_pc_out <= btb_target_pc_in;
            // ADDED (RAS): Capture only when the instruction advances.
            predicted_next_pc_out <= predicted_next_pc_in;
            ras_used_out <= ras_used_in;
        end
    end
endmodule

module alu (
    input [31:0] operand_a,
    input [31:0] operand_b,
    input [3:0] alu_control,
    output reg [31:0] alu_result
);
    always @(*) begin
        case (alu_control)
            4'b0000: alu_result = operand_a & operand_b; // AND
            4'b0001: alu_result = operand_a | operand_b; // OR
            4'b0010: alu_result = operand_a + operand_b; // ADD
            4'b0011: alu_result = operand_a ^ operand_b; // XOR
            4'b0100: alu_result = operand_a << operand_b[4:0]; // Shift Left
            4'b0101: alu_result = operand_a >> operand_b[4:0]; // Shift Right Logical
            4'b0110: alu_result = operand_a - operand_b; // SUBTRACT
            4'b0111: alu_result = $unsigned($signed(operand_a) >>> operand_b[4:0]); // Shift Right Arith.
            4'b1000: alu_result = ($signed(operand_a) < $signed(operand_b)) ? 32'd1 : 32'd0; // SLT
            4'b1001: alu_result = (operand_a < operand_b) ? 32'd1 : 32'd0; // SLTU  
            4'b1111: alu_result = operand_b; // PASS B (Fixes LUI garbage data)
            default: alu_result = 32'b0;
        endcase
    end
endmodule

module alu_control (
    input [1:0] alu_op,
    input [2:0] funct3,
    input       funct7_bit5, 
    input       opcode5,
    output reg [3:0] alu_control_out
);
    always @(*) begin
        alu_control_out = 4'b0000;
        case (alu_op)
            2'b00: alu_control_out = 4'b0010; // Load/Store/Jump/LUI: ADD
            2'b01: alu_control_out = 4'b0110; // Branch: SUBTRACT
            2'b10: begin // R-Type or I-Type Math
                case (funct3)
                    3'b000: begin
                        if (funct7_bit5 == 1'b1 && opcode5 == 1'b1)
                            alu_control_out = 4'b0110; // SUB
                        else
                            alu_control_out = 4'b0010; // ADD
                    end
                    3'b111: alu_control_out = 4'b0000; // AND
                    3'b110: alu_control_out = 4'b0001; // OR
                    3'b100: alu_control_out = 4'b0011; // XOR
                    3'b001: alu_control_out = 4'b0100; // SLL
                    3'b101: begin
                        if (funct7_bit5 == 1'b1)
                            alu_control_out = 4'b0111; // SRA
                        else
                            alu_control_out = 4'b0101; // SRL
                    end
                    3'b010: alu_control_out = 4'b1000; // SLT
                    3'b011: alu_control_out = 4'b1001; // SLTU
                    default: alu_control_out = 4'b0000;
                endcase
            end
            2'b11: alu_control_out = 4'b1111; // LUI -> Pass Operand B
            default: alu_control_out = 4'b0000;
        endcase
    end
endmodule

module ALU_MUX (
    input wire [31:0] read_data2,
    input wire [31:0] immediate,
    input wire ALUSrc,
    output reg [31:0] alu_operand_b
);
    always @(*) begin
        if (ALUSrc) begin
            alu_operand_b = immediate;
        end else begin
            alu_operand_b = read_data2; 
        end
    end
endmodule

module Branch_Adder (
    input wire [31:0] pc,
    input wire [31:0] immediate,
    output reg [31:0] branch_target
);
    always @(*) begin
        branch_target = pc + immediate;
    end
endmodule

// ============================================================================
// EX/MEM Pipeline Register
// ============================================================================
module ex_mem_register (
input wire clk,
input wire reset,
// Inputs coming FROM the IF stage
input wire [31:0] alu_result_in,
input wire [31:0] reg_to_mux_in,
input wire [31:0] pc_plus_4_in, 
input wire MemToReg_in,
input wire MemRead_in,
input wire MemWrite_in,
input wire Branch_in,
input wire RegWrite_in,
input wire Jump_in,
input wire [4:0] rd_in, // Destination register for write-back
// Outputs going TO the ID stage
output reg [31:0] pc_plus_4_out,
output reg [31:0] alu_result_out,
output reg [31:0] reg_to_mux_out,
output reg MemToReg_out,
output reg MemRead_out,
output reg MemWrite_out,
output reg Branch_out,
output reg RegWrite_out,
output reg Jump_out,
output reg [4:0] rd_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_plus_4_out <= 32'b0;
            alu_result_out <= 32'b0;
            reg_to_mux_out <= 32'b0;
            MemToReg_out <= 1'b0;
            MemRead_out <= 1'b0;
            MemWrite_out <= 1'b0;
            Branch_out <= 1'b0;
            RegWrite_out <= 1'b0;
            Jump_out <= 1'b0;
            rd_out <= 5'b0;
        end else begin
            pc_plus_4_out <= pc_plus_4_in;
            alu_result_out <= alu_result_in;
            reg_to_mux_out <= reg_to_mux_in;
            MemToReg_out <= MemToReg_in;
            MemRead_out <= MemRead_in;
            MemWrite_out <= MemWrite_in;
            Branch_out <= Branch_in;
            RegWrite_out <= RegWrite_in;
            Jump_out <= Jump_in;
            rd_out <= rd_in;
        end
    end
endmodule

module data_memory (
    input clk,
    input MemRead,
    input MemWrite,
    input [31:0] address,
    input [31:0] write_data,
    output reg [31:0] read_data
);
    reg [31:0] memory [0:4095]; // Your 16 KB memory block
    // ADD THIS INITIAL BLOCK
    initial begin
        $readmemh("firmware.mem", memory);
    end
    always @(*) begin
        if (MemRead == 1'b1) begin
            read_data = memory[address[13:2]];
        end else begin
            read_data = 32'b0;
        end
    end
    always @(posedge clk) begin
        if (MemWrite == 1'b1) begin
            memory[address[13:2]] <= write_data;
        end
    end
endmodule

// ============================================================================
// MEM/WB Pipeline Register
// ============================================================================
module mem_wb_register (
input wire clk,
input wire reset,
// Inputs coming FROM the IF stage
input wire [31:0] pc_plus_4_in, 
input wire [31:0] read_data_memory_in,
input wire [31:0] alu_result_in,
input wire MemToReg_in,
input wire RegWrite_in,
input wire Jump_in, 
input wire [4:0] rd_in, 
// Outputs going TO the ID stage
output reg [31:0] pc_plus_4_out,
output reg [31:0] read_data_memory_out,
output reg [31:0] alu_result_out,
output reg MemToReg_out,
output reg RegWrite_out,
output reg Jump_out, 
output reg [4:0] rd_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            read_data_memory_out <= 32'b0;
            alu_result_out <= 32'b0;
            MemToReg_out <= 1'b0;
            RegWrite_out <= 1'b0;
            Jump_out <= 1'b0;
            rd_out <= 5'b0;
            pc_plus_4_out <= 32'b0;
        end else begin
            read_data_memory_out <= read_data_memory_in;
            alu_result_out <= alu_result_in;
            MemToReg_out <= MemToReg_in;
            RegWrite_out <= RegWrite_in;
            Jump_out <= Jump_in;
            rd_out <= rd_in;
            pc_plus_4_out <= pc_plus_4_in;
        end
    end
endmodule

module MUX_DATA_MEMORY (
    input wire [31:0] alu_result,
    input wire [31:0] read_data_memory,
    input wire [31:0] pc_plus_4, 
    input wire MemToReg,
    input wire Jump,             
    output reg [31:0] write_back_data
);
    always @(*) begin
        if (Jump) begin
            write_back_data = pc_plus_4;      // <-- ADDED: JAL saves PC+4
        end else if (MemToReg) begin
            write_back_data = read_data_memory; 
        end else begin
            write_back_data = alu_result; 
        end
    end
endmodule

// ============================================================================
// FORWARDING UNIT
// ============================================================================
module forwarding_unit (
    input wire [4:0] ID_EX_rs1,
    input wire [4:0] ID_EX_rs2,
    // Data from the instruction in the MEM stage
    input wire [4:0] EX_MEM_rd,
    input wire       EX_MEM_RegWrite,
    // Data from the instruction in the WB stage
    input wire [4:0] MEM_WB_rd,
    input wire       MEM_WB_RegWrite,
    // Outputs to the MUXes in the EX stage
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB
);
    always @(*) begin
        // Default values (no forwarding)
        ForwardA = 2'b00;
        ForwardB = 2'b00;
         // Check for forwarding from WB stage
        if (MEM_WB_RegWrite && (MEM_WB_rd != 5'b0) &&
            (MEM_WB_rd == ID_EX_rs1)) begin
            ForwardA = 2'b01; // Forward from WB stage to EX stage for rs1
        end
        if (MEM_WB_RegWrite && (MEM_WB_rd != 5'b0) &&
            (MEM_WB_rd == ID_EX_rs2)) begin
            ForwardB = 2'b01; // Forward from WB stage to EX stage for rs2
        end
        // Check for forwarding from MEM stage
        if (EX_MEM_RegWrite && (EX_MEM_rd != 5'b0) && (EX_MEM_rd == ID_EX_rs1)) begin
            ForwardA = 2'b10; // Forward from MEM stage to EX stage for rs1
        end
        if (EX_MEM_RegWrite && (EX_MEM_rd != 5'b0) && (EX_MEM_rd == ID_EX_rs2)) begin
            ForwardB = 2'b10; // Forward from MEM stage to EX stage for rs2
        end
    end
endmodule

module mux_3_1 (
    input wire [31:0] input0,
    input wire [31:0] input1,
    input wire [31:0] input2,
    input wire [1:0] select,
    output reg [31:0] mux_out
);
    always @(*) begin
        case (select)
            2'b00: mux_out = input0; // No forwarding
            2'b01: mux_out = input1; // Forward from WB stage
            2'b10: mux_out = input2; // Forward from MEM stage
            default: mux_out = input0; // Default to no forwarding
        endcase
    end
endmodule

// ============================================================================
// HAZARD DETECTION UNIT
// ============================================================================
module hazard_detection_unit (
    input wire [4:0] ID_EX_rd,
    input wire ID_EX_MemRead,
    input wire [4:0] IF_ID_rs1,
    input wire [4:0] IF_ID_rs2,
    output reg PCWrite,
    output reg IF_ID_Write,
    output reg ControlStall
);
    always @(*) begin
        // Default values
        PCWrite = 1'b1;
        IF_ID_Write = 1'b1;
        ControlStall = 1'b0;
        // Check for load-use hazard
        if (ID_EX_MemRead && ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2)) && (ID_EX_rd != 5'b0)) begin
            PCWrite = 1'b0;       // Stall the PC
            IF_ID_Write = 1'b0;   // Stall the IF/ID pipeline register
            ControlStall = 1'b1;  // Stall the control signals
        end
    end
endmodule

module control_hazard (
    input wire Branch,
    input wire Zero,
    input wire Jump,
    output reg flush
);
    always @(*) begin
        if (Jump || (Branch && Zero)) begin
            flush = 1'b1; // Flush the pipeline
        end else begin
            flush = 1'b0;       // Do not flush 
        end
    end
endmodule

module branch_comparator(
    input wire [31:0] rs1_data,
    input wire [31:0] rs2_data,
    input wire [2:0]  funct3,
    output reg        branch_taken
);
    always @(*) begin
        case(funct3)
            3'b000: branch_taken = (rs1_data == rs2_data);                     // BEQ
            3'b001: branch_taken = (rs1_data != rs2_data);                     // BNE
            3'b100: branch_taken = ($signed(rs1_data) < $signed(rs2_data));    // BLT
            3'b101: branch_taken = ($signed(rs1_data) >= $signed(rs2_data));   // BGE
            3'b110: branch_taken = (rs1_data < rs2_data);                      // BLTU
            3'b111: branch_taken = (rs1_data >= rs2_data);                     // BGEU
            default: branch_taken = 1'b0;
        endcase
    end
endmodule

module performance_counters (
    input wire clk,
    input wire reset,
    input wire branch_in_ex,   
    input wire branch_taken,   
    input wire jump_taken,     
    input wire stall_signal,   
    input wire valid_inst,
    output reg [63:0] cycle_count,
    output reg [63:0] instruction_count,
    output reg [63:0] branch_count,
    output reg [63:0] branch_flush_count, 
    output reg [63:0] jump_flush_count
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            cycle_count        <= 64'b0;
            instruction_count  <= 64'b0;
            branch_count       <= 64'b0;
            branch_flush_count <= 64'b0;
            jump_flush_count   <= 64'b0;
        end else begin
            cycle_count <= cycle_count + 1;
            if (branch_taken) begin
                branch_flush_count <= branch_flush_count + 1;
            end
            if (jump_taken) begin
                jump_flush_count <= jump_flush_count + 1;
            end
            if (branch_in_ex && !stall_signal && valid_inst) begin
                branch_count <= branch_count + 1;
            end
            // THE PERFECT INSTRUCTION COUNTER
            if (valid_inst) begin
                instruction_count <= instruction_count + 64'd1;
            end
        end
    end
endmodule

module btb #(
    parameter ENTRIES = 32,
    parameter INDEX_BITS = 5
)(
    input wire clk,
    input wire rst,
    // ----------------------------------------------------
    // IF Stage: Asynchronous Read Ports (The Guesser)
    // ----------------------------------------------------
    input wire [31:0] if_pc,             // Current PC being fetched
    output wire btb_hit,                 // High if Valid == 1 AND Tags match
    output wire [31:0] btb_target_pc,    // Stored destination address
    output wire [1:0] btb_prediction,    // Current 2-bit FSM state
    // ----------------------------------------------------
    // EX Stage: Synchronous Write Ports (The Checker)
    // ----------------------------------------------------
    input wire ex_we,                    // Write Enable (High when branch resolves)
    input wire [31:0] ex_pc,             // PC of the branch in EX stage
    input wire [31:0] ex_target_pc,      // Calculated actual target address
    input wire [1:0] ex_new_fsm          // Updated FSM state from EX logic
);
    // ----------------------------------------------------
    // Memory Arrays (Total 60 bits per row)
    // ----------------------------------------------------
    reg valid_array  [0:ENTRIES-1];      // 1-bit Valid flag
    reg [24:0] tag_array [0:ENTRIES-1];  // 25-bit Tag
    reg [1:0] fsm_array  [0:ENTRIES-1];  // 2-bit Prediction State
    reg [31:0] target_array [0:ENTRIES-1]; // 32-bit Target PC
    // ----------------------------------------------------
    // Internal Index & Tag Wires
    // ----------------------------------------------------
    // [1:0] are ignored due to 4-byte alignment
    wire [INDEX_BITS-1:0] if_index = if_pc[6:2];
    wire [24:0]           if_tag   = if_pc[31:7];
    wire [INDEX_BITS-1:0] ex_index = ex_pc[6:2];
    wire [24:0]           ex_tag   = ex_pc[31:7];
    integer i;
    // ----------------------------------------------------
    // IF Stage: Asynchronous Read Logic
    // ----------------------------------------------------
    assign btb_hit = (valid_array[if_index] == 1'b1) && (tag_array[if_index] == if_tag);
    assign btb_target_pc = target_array[if_index];
    assign btb_prediction = fsm_array[if_index];
    // ----------------------------------------------------
    // EX Stage: Synchronous Write & Reset Logic
    // ----------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            // Clear all valid bits on startup
            for (i = 0; i < ENTRIES; i = i + 1) begin
                valid_array[i] <= 1'b0;
            end
        end else if (ex_we) begin
            valid_array[ex_index]  <= 1'b1; //stay at 1 forever
            tag_array[ex_index]    <= ex_tag;
            fsm_array[ex_index]    <= ex_new_fsm;
            target_array[ex_index] <= ex_target_pc;
        end
    end
endmodule

module predictor_fsm(
    input wire [1:0] currentState,
    input wire taken, 
    output reg [1:0] nextState
);
    // 2'b00 : Strongly taken
    // 2'b01 : Weakly taken
    // 2'b10 : Weakly not taken
    // 2'b11 : Strongly not taken
    always @(*) begin
        case(currentState)
            2'b00 : nextState = (taken) ? 2'b00 : 2'b01;
            2'b01 : nextState = (taken) ? 2'b00 : 2'b10;
            2'b10 : nextState = (taken) ? 2'b01 : 2'b11;
            2'b11 : nextState = (taken) ? 2'b10 : 2'b11;
            default: nextState = 2'b01;
        endcase
    end
endmodule

// CHANGED (RAS): Compare the actual next PC against the prediction IF used.
// BTB training is independent of flush: a stale BTB may coexist with a
// correct RAS prediction. Priority remains JALR > branch > JAL.
module btb_control (
    input wire btb_hit,
    input wire [31:0] btb_target_pc,
    input wire [1:0] btb_prediction,
    input wire id_ex_Branch,
    input wire Zero,
    input wire [31:0] branch_adder_result,
    input wire [31:0] alu_result,
    input wire id_ex_JALR_signal,
    input wire id_ex_Jump,
    input wire [31:0] id_ex_pc,
    input wire [31:0] id_ex_pc_plus_4,
    input wire valid_ex,
    input wire [31:0] predicted_next_pc,
    output reg flush,
    output reg write_enable,
    output reg [31:0] ex_target_pc,
    output reg [1:0] ex_new_fsm,
    output reg [31:0] ex_pc,
    output reg [31:0] branch_target
);
    wire [1:0] next_prediction;
    reg [31:0] jalr_target;
    predictor_fsm Predictor (
        .currentState(btb_prediction), .taken(Zero), .nextState(next_prediction)
    );
    always @(*) begin
        flush         = 1'b0;
        write_enable  = 1'b0;
        ex_pc         = id_ex_pc;
        ex_target_pc  = branch_adder_result;
        ex_new_fsm    = 2'b00;
        branch_target = id_ex_pc_plus_4;
        jalr_target   = {alu_result[31:1], 1'b0};

        if (valid_ex) begin
            if (id_ex_JALR_signal) begin
                branch_target = jalr_target;
                ex_target_pc  = jalr_target;
                if (!btb_hit || (btb_target_pc != jalr_target) ||
                    (btb_prediction != 2'b00))
                    write_enable = 1'b1;
            end else if (id_ex_Branch) begin
                branch_target = Zero ? branch_adder_result : id_ex_pc_plus_4;
                // CHANGED: Always train the mathematical branch target.
                ex_target_pc = branch_adder_result;
                if (btb_hit) begin
                    write_enable = 1'b1;
                    ex_new_fsm = next_prediction;
                end else if (Zero) begin
                    write_enable = 1'b1;
                    ex_new_fsm = 2'b01; // Preserve initial weakly-taken policy.
                end
            end else if (id_ex_Jump) begin
                branch_target = branch_adder_result;
                ex_target_pc  = branch_adder_result;
                if (!btb_hit || (btb_target_pc != branch_adder_result) ||
                    (btb_prediction != 2'b00))
                    write_enable = 1'b1;
            end
            // CHANGED: A BTB miss alone does not require a flush anymore.
            flush = (predicted_next_pc != branch_target);
        end
    end
endmodule
module ras_stack #(
    parameter DEPTH = 8,
    parameter COUNT_WIDTH = $clog2(DEPTH + 1)
)(
    input wire clk,
    input wire reset,
    input wire update_enable,
    input wire push,
    input wire pop,
    input wire [31:0] push_address,
    input wire restore,
    input wire [DEPTH*32-1:0] restore_entries,
    input wire [COUNT_WIDTH-1:0] restore_count,
    output reg [DEPTH*32-1:0] entries,
    output reg [COUNT_WIDTH-1:0] count,
    output reg [DEPTH*32-1:0] next_entries,
    output reg [COUNT_WIDTH-1:0] next_count,
    output reg [31:0] top_address,
    output reg not_empty
);
    localparam [COUNT_WIDTH-1:0] CAPACITY = DEPTH;
    integer i;

    always @(*) begin
        i            = 0;
        next_entries = entries;
        next_count   = count;
        top_address  = 32'b0;
        not_empty    = (count != {COUNT_WIDTH{1'b0}});
        if (not_empty)
            top_address = entries[31:0];

        // ADDED: Restore all data, not only the count; pushes may overwrite data.
        if (restore) begin
            next_entries = restore_entries;
            next_count   = restore_count;
        end else if (update_enable) begin
            // Version 1 decode makes push and pop mutually exclusive.
            if (push) begin
                for (i = 1; i < DEPTH; i = i + 1)
                    next_entries[i*32 +: 32] = entries[(i-1)*32 +: 32];
                next_entries[31:0] = push_address;
                if (count < CAPACITY)
                    next_count = count + 1'b1;
            end else if (pop && not_empty) begin
                for (i = 0; i < DEPTH-1; i = i + 1)
                    next_entries[i*32 +: 32] = entries[(i+1)*32 +: 32];
                next_entries[(DEPTH-1)*32 +: 32] = 32'b0;
                next_count = count - 1'b1;
            end
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            entries <= {DEPTH*32{1'b0}};
            count   <= {COUNT_WIDTH{1'b0}};
        end else begin
            entries <= next_entries;
            count   <= next_count;
        end
    end
endmodule

module ras_if #(
    parameter DEPTH = 8,
    parameter COUNT_WIDTH = $clog2(DEPTH + 1)
)(
    input wire clk,
    input wire reset,
    input wire enable,
    input wire accept_if,
    input wire flush,
    input wire [31:0] instruction,
    input wire [31:0] pc_plus_4,
    input wire [DEPTH*32-1:0] resolved_next_entries,
    input wire [COUNT_WIDTH-1:0] resolved_next_count,
    output reg use_ras,
    output wire [31:0] target_pc,
    output wire [DEPTH*32-1:0] entries,
    output wire [COUNT_WIDTH-1:0] count
);
    reg is_jal, is_jalr, is_call, is_return;
    reg update_enable;
    wire not_empty;

    always @(*) begin
        is_jal   = (instruction[6:0] == 7'b1101111);
        is_jalr  = (instruction[6:0] == 7'b1100111) &&
                   (instruction[14:12] == 3'b000);
        is_call  = (is_jal || is_jalr) && (instruction[11:7] == 5'd1);
        is_return = is_jalr && (instruction[11:7] == 5'd0) &&
                    (instruction[19:15] == 5'd1);
        use_ras = 1'b0;
        update_enable = enable && accept_if && !flush && !reset;
        // Prediction remains combinational while stalled; no push/pop repeats.
        if (enable && !reset && is_return && not_empty)
            use_ras = 1'b1;
        // Version 1 predicts the saved PC+4 directly. Nonzero JALR immediate
        // or a changed x1 may mispredict; EX always checks the actual target.
    end

    ras_stack #(.DEPTH(DEPTH), .COUNT_WIDTH(COUNT_WIDTH)) Stack (
        .clk(clk), .reset(reset), .update_enable(update_enable),
        .push(is_call), .pop(is_return), .push_address(pc_plus_4),
        // ADDED: flush overrides an IF call/return or stall on the same edge.
        .restore(flush), .restore_entries(resolved_next_entries),
        .restore_count(resolved_next_count), .entries(entries), .count(count),
        .next_entries(), .next_count(), .top_address(target_pc),
        .not_empty(not_empty)
    );
endmodule

module ras_ex #(
    parameter DEPTH = 8,
    parameter COUNT_WIDTH = $clog2(DEPTH + 1)
)(
    input wire clk,
    input wire reset,
    input wire enable,
    input wire valid_ex,
    input wire [31:0] instruction,
    input wire [31:0] pc_plus_4,
    output reg is_return,
    output wire [DEPTH*32-1:0] entries,
    output wire [COUNT_WIDTH-1:0] count,
    output wire [DEPTH*32-1:0] next_entries,
    output wire [COUNT_WIDTH-1:0] next_count
);
    reg is_jal, is_jalr, is_call;
    reg update_enable;

    always @(*) begin
        is_jal   = (instruction[6:0] == 7'b1101111);
        is_jalr  = (instruction[6:0] == 7'b1100111) &&
                   (instruction[14:12] == 3'b000);
        is_call  = (is_jal || is_jalr) && (instruction[11:7] == 5'd1);
        is_return = is_jalr && (instruction[11:7] == 5'd0) &&
                    (instruction[19:15] == 5'd1);
        // ADDED: Do NOT gate by ControlStall or flush. The current EX
        // instruction completes even when younger instructions are discarded.
        update_enable = enable && valid_ex && !reset;
    end

    ras_stack #(.DEPTH(DEPTH), .COUNT_WIDTH(COUNT_WIDTH)) Stack (
        .clk(clk), .reset(reset), .update_enable(update_enable),
        .push(is_call), .pop(is_return), .push_address(pc_plus_4),
        .restore(1'b0), .restore_entries({DEPTH*32{1'b0}}),
        .restore_count({COUNT_WIDTH{1'b0}}), .entries(entries), .count(count),
        // ADDED: IF rollback uses these combinational NEXT outputs so the
        // EX call/return responsible for the flush is retained exactly once.
        .next_entries(next_entries), .next_count(next_count),
        .top_address(), .not_empty()
    );
endmodule


// ============================================================================
// RISC-V TOP MODULE (The Full CPU)
// ============================================================================
// CHANGED (RAS): Set RAS_ENABLE=0 for BTB-only comparison using the same RTL.
module RISC_V #(
    parameter RAS_ENABLE = 1,
    parameter RAS_DEPTH = 8
)(input clk, input aresetn,
    output wire [31:0] mmio_address,
    output wire [31:0] mmio_write_data,
    output wire        mmio_write_enable
);
    wire rst = ~aresetn; // Active high reset for internal modules
    wire [31:0] pc_wire, pc_out_wire, pc_next_wire, branch_target, WB_data_wire, read_data1, reg_to_mux, immgen_wire, WB_wire, mux_to_ALU, read_data_wire, fetch_instruction_wire, ALU_operand_a, forwarded_to_MUX_B, alu_operand_a_final;
    // IF_ID Pipeline Register Wires
    wire [31:0] if_id_pc, if_id_instruction, if_id_pc_plus_4;
    // ID_EX Pipeline Register Wires
    wire [31:0] id_ex_pc, id_ex_instruction, id_ex_read_data1, id_ex_reg_to_mux, id_ex_immgen_wire, id_ex_pc_plus_4;
    wire [4:0] id_ex_rd;
    // EX_MEM Pipeline Register Wires
    wire [31:0] ex_mem_alu_result, ex_mem_reg_to_mux, ex_mem_pc_plus_4, ex_mem_forward_data;
    wire [4:0] ex_mem_rd;
    // MEM_WB Pipeline Register Wires
    wire [31:0] mem_wb_pc_plus_4, mem_wb_read_data_memory, mem_wb_alu_result;
    wire [4:0] mem_wb_rd;
    wire [1:0] ForwardA, ForwardB; // Forwarding control signals
    wire  ControlStall, PCWrite, IF_ID_Write, flush;
    //Control Wires
    wire id_ex_ALUSrc, id_ex_MemToReg, id_ex_MemRead, id_ex_MemWrite, id_ex_Branch, id_ex_RegWrite, id_ex_Jump, id_ex_ALUSrcA , id_ex_JALR_signal;
    wire ex_mem_MemToReg, ex_mem_RegWrite, ex_mem_Branch, ex_mem_Jump, ex_mem_MemRead, ex_mem_MemWrite;
    wire mem_wb_MemToReg, mem_wb_RegWrite, mem_wb_Jump;
    wire [1:0] id_ex_ALUOp;
    wire [3:0] alu_control_wire;
    wire RegWrite, ALUSrc, MemToReg, MemRead, MemWrite, Branch, Zero, Jump, ALUSrcA, JALR_signal; // <-- Added Jump and ALUSrcA wires
    wire [1:0] ALUOp;
    wire [31:0] final_branch_target;
    // IF Stage Prediction Wires
    wire if_btb_hit;
    wire [1:0] if_btb_prediction;
    wire [31:0] if_btb_target_pc;
    //Forwarded from IF/ID registers
    wire if_id_btb_hit;
    wire [1:0] if_id_btb_prediction;
    wire [31:0] if_id_btb_target_pc;
    // EX Stage Forwarded Prediction Wires (from ID/EX register)
    wire id_ex_btb_hit;
    wire [1:0] id_ex_btb_prediction;
    wire [31:0] id_ex_btb_target_pc;
    // EX Stage Control Outputs (fed back to IF and BTB)
    wire ex_we;
    wire [1:0] ex_new_fsm;
    wire [31:0] ex_update_pc;
    wire [31:0] ex_target_pc_update;
    wire [31:0] ex_recovery_pc;
    // ADDED (RAS): Two physically separate stacks; EX NEXT state feeds rollback.
    localparam RAS_COUNT_WIDTH = $clog2(RAS_DEPTH + 1);
    wire [RAS_DEPTH*32-1:0] ras_if_entries, ras_ex_entries, ras_ex_next_entries;
    wire [RAS_COUNT_WIDTH-1:0] ras_if_count, ras_ex_count, ras_ex_next_count;
    wire [31:0] ras_target_pc;
    wire if_ras_used, if_id_ras_used, id_ex_ras_used, ex_is_return;
    wire [31:0] if_predicted_next_pc, if_id_predicted_next_pc;
    wire [31:0] id_ex_predicted_next_pc;
    reg valid_ex, if_accept, pc_write_enable;

    always @(*) begin
        // Existing all-zero instruction represents a bubble in this CPU.
        valid_ex = (id_ex_instruction != 32'b0) && !rst;
        if_accept = IF_ID_Write && !flush && !rst;
        // CHANGED (RAS): Recovery must not be blocked by a front-end stall.
        pc_write_enable = PCWrite || flush;
    end

    ras_if #(.DEPTH(RAS_DEPTH), .COUNT_WIDTH(RAS_COUNT_WIDTH)) RAS_IF (
        .clk(clk), .reset(rst), .enable(RAS_ENABLE != 0),
        .accept_if(if_accept), .flush(flush),
        .instruction(fetch_instruction_wire), .pc_plus_4(pc_next_wire),
        .resolved_next_entries(ras_ex_next_entries),
        .resolved_next_count(ras_ex_next_count),
        .use_ras(if_ras_used), .target_pc(ras_target_pc),
        .entries(ras_if_entries), .count(ras_if_count)
    );

    ras_ex #(.DEPTH(RAS_DEPTH), .COUNT_WIDTH(RAS_COUNT_WIDTH)) RAS_EX (
        .clk(clk), .reset(rst), .enable(RAS_ENABLE != 0), .valid_ex(valid_ex),
        .instruction(id_ex_instruction), .pc_plus_4(id_ex_pc_plus_4),
        .is_return(ex_is_return), .entries(ras_ex_entries), .count(ras_ex_count),
        .next_entries(ras_ex_next_entries), .next_count(ras_ex_next_count)
    );

    program_counter PC (
        .clk(clk), .reset(rst), .next_pc(pc_wire), .pc(pc_out_wire), .write_enable(pc_write_enable) // CHANGED (RAS): redirect priority
    );
    pc_adder PC_Adder (
        .pc_in(pc_out_wire), .pc_next(pc_next_wire)
    );
    pc_mux PC_MUX (
        .pc_plus_4(pc_next_wire), .btb_hit(if_btb_hit),
        .btb_fsm_prediction(if_btb_prediction), 
        .btb_branch_target(if_btb_target_pc),
        // ADDED (RAS): Prediction source plus pre-recovery prediction output.
        .ras_use(if_ras_used), .ras_target_pc(ras_target_pc),
        .predicted_next_pc(if_predicted_next_pc),
        .branch_target(ex_recovery_pc),
        .misprediction(flush),
        .next_pc(pc_wire)
    );
    btb #(
    .ENTRIES(32), 
    .INDEX_BITS(5)
    ) Branch_Target_Buffer (
        .clk(clk),
        .rst(rst),
        // IF Stage Reads
        .if_pc(pc_out_wire),
        .btb_hit(if_btb_hit),
        .btb_target_pc(if_btb_target_pc),
        .btb_prediction(if_btb_prediction),
        // EX Stage Writes
        .ex_we(ex_we),
        .ex_pc(ex_update_pc),
        .ex_target_pc(ex_target_pc_update),
        .ex_new_fsm(ex_new_fsm)
    );
    instruction_memory instruction_memory(
        .read_address(pc_out_wire), .instruction_out(fetch_instruction_wire)
    );
    // Address Decoder for MMIO 
    wire is_mmio = (ex_mem_alu_result[31:16] == 16'h4000);
    assign mmio_address      = ex_mem_alu_result;
    assign mmio_write_data   = ex_mem_reg_to_mux;
    assign mmio_write_enable = ex_mem_MemWrite && (is_mmio);
    if_id_register IF_ID (
    .clk(clk), .reset(rst),
    .pc_in(pc_out_wire), 
    .instruction_in(fetch_instruction_wire),
    .pc_plus_4_in(pc_next_wire), 
    .pc_out(if_id_pc), 
    .instruction_out(if_id_instruction),
    .pc_plus_4_out(if_id_pc_plus_4), .write_enable(IF_ID_Write), .flush(flush),
    .btb_hit_in(if_btb_hit), .btb_prediction_in(if_btb_prediction),
    .btb_target_pc_in(if_btb_target_pc),
    .btb_hit_out(if_id_btb_hit), .btb_prediction_out(if_id_btb_prediction),
    .btb_target_pc_out(if_id_btb_target_pc),
    // ADDED (RAS): prediction and source advance with IF/ID.
    .predicted_next_pc_in(if_predicted_next_pc), .ras_used_in(if_ras_used),
    .predicted_next_pc_out(if_id_predicted_next_pc), .ras_used_out(if_id_ras_used)
);
    register_file Reg_File (
        .clk(clk), .reset(rst),
        .read_reg1(if_id_instruction[19:15]), .read_reg2(if_id_instruction[24:20]), .write_reg(mem_wb_rd),
        .write_data(WB_data_wire), .reg_write(mem_wb_RegWrite),
        .read_data1(read_data1), .read_data2(reg_to_mux)
    );
    main_control_unit Control (
        .opcode(if_id_instruction[6:0]),
        .RegWrite(RegWrite), .ALUSrc(ALUSrc), .MemToReg(MemToReg),
        .MemRead(MemRead), .MemWrite(MemWrite), .Branch(Branch),
        .Jump(Jump), 
        .ALUOp(ALUOp), .ALUSrcA(ALUSrcA), .JALR_signal(JALR_signal)
    );
    immediate_generator Imm_Gen (
        .instruction(if_id_instruction), .immediate(immgen_wire) 
    );
    id_ex_register ID_EX (
        .clk(clk), .reset(rst),
        .pc_in(if_id_pc), .instruction_in(if_id_instruction),
        .read_data1_in(read_data1), .reg_to_mux_in(reg_to_mux),
        .immgen_wire_in(immgen_wire), .ALUSrc_in(ALUSrc),
        .MemToReg_in(MemToReg), .MemRead_in(MemRead),
        .MemWrite_in(MemWrite), .Branch_in(Branch),
        .RegWrite_in(RegWrite), .Jump_in(Jump), .ALUSrcA_in(ALUSrcA), .JALR_signal_in(JALR_signal),
        .ALUOp_in(ALUOp), .rd_in(if_id_instruction[11:7]),
        .pc_plus_4_in(if_id_pc_plus_4),
        .pc_out(id_ex_pc), .instruction_out(id_ex_instruction),
        .read_data1_out(id_ex_read_data1), 
        .reg_to_mux_out(id_ex_reg_to_mux),
        .immgen_wire_out(id_ex_immgen_wire), 
        .ALUSrc_out(id_ex_ALUSrc),
        .MemToReg_out(id_ex_MemToReg), 
        .MemRead_out(id_ex_MemRead),
        .MemWrite_out(id_ex_MemWrite), 
        .Branch_out(id_ex_Branch),
        .RegWrite_out(id_ex_RegWrite), 
        .Jump_out(id_ex_Jump),
        .ALUOp_out(id_ex_ALUOp), 
        .rd_out(id_ex_rd),
        .pc_plus_4_out(id_ex_pc_plus_4), 
        .stall(ControlStall), .flush(flush), 
        .ALUSrcA_out(id_ex_ALUSrcA),
        .JALR_signal_out(id_ex_JALR_signal),
        .btb_hit_in(if_id_btb_hit), .btb_prediction_in(if_id_btb_prediction),
        .btb_target_pc_in(if_id_btb_target_pc), .btb_hit_out(id_ex_btb_hit),
        .btb_prediction_out(id_ex_btb_prediction), .btb_target_pc_out(id_ex_btb_target_pc),
        // ADDED (RAS): prediction and source advance with ID/EX.
        .predicted_next_pc_in(if_id_predicted_next_pc), .ras_used_in(if_id_ras_used),
        .predicted_next_pc_out(id_ex_predicted_next_pc), .ras_used_out(id_ex_ras_used)
    );
    assign alu_operand_a_final = (id_ex_ALUSrcA) ? id_ex_pc : ALU_operand_a;
    alu ALU (
        .operand_a(alu_operand_a_final), .operand_b(mux_to_ALU),
        .alu_control(alu_control_wire), .alu_result(WB_wire)
    );
    alu_control ALU_Control (
        .alu_op(id_ex_ALUOp), .funct3(id_ex_instruction[14:12]),
        .funct7_bit5(id_ex_instruction[30]), .opcode5(id_ex_instruction[5]),
        .alu_control_out(alu_control_wire)
    );
    ALU_MUX ALU_MUX (
        .read_data2(forwarded_to_MUX_B), .immediate(id_ex_immgen_wire),
        .ALUSrc(id_ex_ALUSrc), .alu_operand_b(mux_to_ALU)
    );
    Branch_Adder Branch_Adder (
        .pc(id_ex_pc), .immediate(id_ex_immgen_wire), .branch_target(branch_target)
    );
    ex_mem_register EX_MEM (
        .clk(clk), .reset(rst),
        .alu_result_in(WB_wire),
        .reg_to_mux_in(forwarded_to_MUX_B),
        .pc_plus_4_in(id_ex_pc_plus_4), 
        .MemToReg_in(id_ex_MemToReg),
        .MemRead_in(id_ex_MemRead), .MemWrite_in(id_ex_MemWrite),
        .Branch_in(id_ex_Branch), .RegWrite_in(id_ex_RegWrite),
        .Jump_in(id_ex_Jump), .rd_in(id_ex_rd),
        .pc_plus_4_out(ex_mem_pc_plus_4),
        .alu_result_out(ex_mem_alu_result),
        .reg_to_mux_out(ex_mem_reg_to_mux), 
        .MemToReg_out(ex_mem_MemToReg), 
        .MemRead_out(ex_mem_MemRead),
        .MemWrite_out(ex_mem_MemWrite), 
        .Branch_out(ex_mem_Branch),
        .RegWrite_out(ex_mem_RegWrite), 
        .Jump_out(ex_mem_Jump),
        .rd_out(ex_mem_rd)
    );
    data_memory Data_Mem (
        .clk(clk), .MemRead(ex_mem_MemRead & (~is_mmio)), .MemWrite(ex_mem_MemWrite & (~is_mmio)),
        .address(ex_mem_alu_result), .write_data(ex_mem_reg_to_mux), .read_data(read_data_wire) 
    );
    mem_wb_register MEM_WB (
        .clk(clk), .reset(rst),
        .pc_plus_4_in(ex_mem_pc_plus_4), 
        .read_data_memory_in(read_data_wire), 
        .alu_result_in(ex_mem_alu_result),
        .MemToReg_in(ex_mem_MemToReg), 
        .RegWrite_in(ex_mem_RegWrite),
        .Jump_in(ex_mem_Jump), 
        .rd_in(ex_mem_rd),
        .pc_plus_4_out(mem_wb_pc_plus_4),
        .read_data_memory_out(mem_wb_read_data_memory), 
        .alu_result_out(mem_wb_alu_result),
        .MemToReg_out(mem_wb_MemToReg), 
        .RegWrite_out(mem_wb_RegWrite),
        .Jump_out(mem_wb_Jump), 
        .rd_out(mem_wb_rd)
    );
    MUX_DATA_MEMORY MUX_Data_Mem (
        .alu_result(mem_wb_alu_result), .read_data_memory(mem_wb_read_data_memory),
        .pc_plus_4(mem_wb_pc_plus_4), 
        .Jump(mem_wb_Jump),             
        .MemToReg(mem_wb_MemToReg), .write_back_data(WB_data_wire)
    );
    forwarding_unit Forwarding_Unit (
        .ID_EX_rs1(id_ex_instruction[19:15]), .ID_EX_rs2(id_ex_instruction[24:20]),
        .EX_MEM_rd(ex_mem_rd), .EX_MEM_RegWrite(ex_mem_RegWrite),
        .MEM_WB_rd(mem_wb_rd), .MEM_WB_RegWrite(mem_wb_RegWrite),
        .ForwardA(ForwardA), .ForwardB(ForwardB)
    );
    assign ex_mem_forward_data =
    ex_mem_Jump ? ex_mem_pc_plus_4 : ex_mem_alu_result;
    mux_3_1 MUX_ForwardA (
        .input0(id_ex_read_data1), 
        .input1(WB_data_wire), 
        .input2(ex_mem_forward_data), 
        .select(ForwardA), 
        .mux_out(ALU_operand_a) // Forwarded data for ALU operand A
    );
    mux_3_1 MUX_ForwardB (
        .input0(id_ex_reg_to_mux), 
        .input1(WB_data_wire), 
        .input2(ex_mem_forward_data), 
        .select(ForwardB), 
        .mux_out(forwarded_to_MUX_B) // Forwarded data for ALU operand B
    );
    // ========================================================================
    // NEW BRANCH COMPARATOR
    // ========================================================================
    branch_comparator Branch_Comparator (
        .rs1_data(ALU_operand_a), 
        .rs2_data(forwarded_to_MUX_B), 
        .funct3(id_ex_instruction[14:12]), 
        .branch_taken(Zero) // <-- Using Zero to indicate branch taken
    );
    hazard_detection_unit Hazard_Unit (
        .ID_EX_rd(id_ex_rd), .ID_EX_MemRead(id_ex_MemRead),
        .IF_ID_rs1(if_id_instruction[19:15]), .IF_ID_rs2(if_id_instruction[24:20]),
        .PCWrite(PCWrite), .IF_ID_Write(IF_ID_Write), .ControlStall(ControlStall)
    );
    /*control_hazard Control_Hazard (
        .Branch(id_ex_Branch), .Zero(Zero), .Jump(id_ex_Jump),
        .flush(flush) 
    );*/
    btb_control Branch_Controller (
    // ADDED (RAS): Resolve the actual prediction, regardless of its source.
    .valid_ex(valid_ex), .predicted_next_pc(id_ex_predicted_next_pc),
    .btb_hit(id_ex_btb_hit),                 // Forwarded from ID/EX
    .btb_target_pc(id_ex_btb_target_pc),     // Forwarded from ID/EX
    .btb_prediction(id_ex_btb_prediction),   // Forwarded from ID/EX
    .id_ex_Branch(id_ex_Branch),
    .Zero(Zero),                 
    .branch_adder_result(branch_target),
    .alu_result(WB_wire),
    .id_ex_JALR_signal(id_ex_JALR_signal),
    .id_ex_Jump(id_ex_Jump),
    .id_ex_pc(id_ex_pc),
    .id_ex_pc_plus_4(id_ex_pc_plus_4),
    // Outputs routing back to IF/BTB
    .flush(flush),
    .write_enable(ex_we),
    .ex_target_pc(ex_target_pc_update),
    .ex_new_fsm(ex_new_fsm),
    .ex_pc(ex_update_pc),
    .branch_target(ex_recovery_pc)
);
    wire [63:0] sim_cycles, sim_instructions, sim_branches, sim_branch_flushes, sim_jump_flushes;
    performance_counters Perf_Counters (
        .clk(clk),
        .reset(rst),
        .branch_in_ex(id_ex_Branch), 
        .branch_taken(flush && id_ex_Branch), //branch_misprediction
        .jump_taken(flush && id_ex_Jump),  //jump_misprediction         
        .stall_signal(ControlStall), 
        .valid_inst(id_ex_instruction != 32'b0), // Only count if not a bubble
        .cycle_count(sim_cycles),
        .instruction_count(sim_instructions),
        .branch_count(sim_branches),
        .branch_flush_count(sim_branch_flushes),
        .jump_flush_count(sim_jump_flushes)
    );
    // ADDED (RAS): Debug counters visible hierarchically in the testbench.
    wire [63:0] sim_returns, sim_ras_predictions, sim_ras_mispredictions;
    wire [63:0] sim_return_mispredictions;
    ras_performance_counters RAS_Counters (
        .clk(clk), .reset(rst), .valid_ex(valid_ex),
        .return_in_ex(ex_is_return), .ras_used_in_ex(id_ex_ras_used),
        .misprediction(flush), .return_count(sim_returns),
        .ras_prediction_count(sim_ras_predictions),
        .ras_misprediction_count(sim_ras_mispredictions),
        .return_misprediction_count(sim_return_mispredictions)
    );
endmodule

