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

module pc_mux (
    input wire [31:0] pc_plus_4,
    input wire [31:0] branch_target,
    input wire branch_taken,
    output reg [31:0] next_pc
);
    always @(*) begin
        if (branch_taken) begin 
            next_pc = branch_target;
        end else begin
            next_pc = pc_plus_4;
        end
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

// Outputs going TO the ID stage
output reg [31:0] pc_out,
output reg [31:0] instruction_out,
output reg [31:0] pc_plus_4_out


);
    always @(posedge clk or posedge reset) begin
        if (reset || flush) begin
            pc_out <= 32'b0;
            instruction_out <= 32'b0;
            pc_plus_4_out <= 32'b0;
        end else begin
            if (write_enable) begin
                pc_out <= pc_in;
                instruction_out <= instruction_in;
                pc_plus_4_out <= pc_plus_4_in;
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
                ALUSrc   = 1'b1;
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
output reg [31:0] pc_plus_4_out

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

// ============================================================================
// RISC-V TOP MODULE (The Full CPU)
// ============================================================================
module RISC_V (input clk, input aresetn,

    // Instruction Memory Interface
    //output wire [31:0] imem_address,
    //input  wire [31:0] imem_instruction,

    // Data Memory Interface
    //output wire [31:0] dmem_address,
    //output wire [31:0] dmem_write_data,
    //input  wire [31:0] dmem_read_data,
    //output wire dmem_mem_read,
    //output wire dmem_mem_write,

    // MMIO Interface
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
    wire [31:0] ex_mem_alu_result, ex_mem_reg_to_mux, ex_mem_pc_plus_4;
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
    program_counter PC (
        .clk(clk), .reset(rst), .next_pc(pc_wire), .pc(pc_out_wire), .write_enable(PCWrite)
    );

    pc_adder PC_Adder (
        .pc_in(pc_out_wire), .pc_next(pc_next_wire)
    );

    assign final_branch_target = (id_ex_JALR_signal)? WB_wire : branch_target; // Use ALU result for JALR, branch target for branches

    pc_mux PC_MUX (
        .pc_plus_4(pc_next_wire), .branch_target(final_branch_target),
        .branch_taken(flush),
        .next_pc(pc_wire)
    );

    instruction_memory instruction_memory(
        .read_address(pc_out_wire), .instruction_out(fetch_instruction_wire)
    );

    // ========================================================================
    // MEMORY INTERFACE ROUTING
    // ========================================================================
    // Connect the internal CPU wires directly to the new external memory ports!

    // 1. Instruction Memory
    // If stalled, hold the current PC. Otherwise, fetch the next PC!
    //assign imem_address = (ControlStall) ? pc_out_wire : pc_wire;
    //assign fetch_instruction_wire = imem_instruction;

    // 2. Address Decoder for MMIO 
    wire is_mmio = (ex_mem_alu_result[31:16] == 16'h4000);

    // 3. Data Memory
    //assign dmem_address    = WB_wire;
    //assign dmem_write_data = forwarded_to_MUX_B;
    // Gate the read signal so BRAM stays asleep during MMIO reads
    //assign dmem_mem_read   = id_ex_MemRead & (~is_mmio);
    // Gate the BRAM write enable so it safely ignores MMIO addresses
    //assign dmem_mem_write  = id_ex_MemWrite & (~is_mmio);
    //assign read_data_wire  = dmem_read_data;

    // 4. MMIO Hardware Interface

    //assign mmio_address      = 32'h40000000; 
    //assign mmio_write_data   = 32'hFFFFFFFF; 
    //assign mmio_write_enable = 1'b1;
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
    .pc_plus_4_out(if_id_pc_plus_4), .write_enable(IF_ID_Write), .flush(flush)
    
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
        .JALR_signal_out(id_ex_JALR_signal)
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
    mux_3_1 MUX_ForwardA (
        .input0(id_ex_read_data1), 
        .input1(WB_data_wire), 
        .input2(ex_mem_alu_result), 
        .select(ForwardA), 
        .mux_out(ALU_operand_a) // Forwarded data for ALU operand A
    );
    
    mux_3_1 MUX_ForwardB (
        .input0(id_ex_reg_to_mux), 
        .input1(WB_data_wire), 
        .input2(ex_mem_alu_result), 
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

    control_hazard Control_Hazard (
        .Branch(id_ex_Branch), .Zero(Zero), .Jump(id_ex_Jump),
        .flush(flush) 
    );

    wire [63:0] sim_cycles, sim_instructions, sim_branches, sim_branch_flushes, sim_jump_flushes;

    performance_counters Perf_Counters (
        .clk(clk),
        .reset(rst),
        .branch_in_ex(id_ex_Branch), 
        .branch_taken(id_ex_Branch & Zero), 
        .jump_taken(id_ex_Jump),            
        .stall_signal(ControlStall), 
        .valid_inst(id_ex_instruction != 32'b0), // Only count if not a bubble
        .cycle_count(sim_cycles),
        .instruction_count(sim_instructions),
        .branch_count(sim_branches),
        .branch_flush_count(sim_branch_flushes),
        .jump_flush_count(sim_jump_flushes)
    );
endmodule
