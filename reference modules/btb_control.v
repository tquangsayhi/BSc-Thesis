module btb_control(
    input wire btb_hit,
    input wire [31:0] btb_target_pc,
    input wire [1:0] btb_prediction,
    input wire id_ex_Branch,
    input wire Zero,                  // If the branch is actually taken (rs1 == rs2)
    input wire [31:0] branch_adder_result,
    input wire [31:0] alu_result,
    input wire id_ex_JALR_signal,
    input wire id_ex_Jump,
    input wire [31:0] id_ex_pc,     
    input wire [31:0] id_ex_pc_plus_4,
    output reg flush,                 // Also acts as the misprediction signal for the pipeline
    output reg write_enable,          // Used to trigger a BTB write
    output reg [31:0] ex_target_pc,   // Target PC to store in the BTB
    output reg [1:0] ex_new_fsm,      // New FSM state to store in the BTB
    output reg [31:0] ex_pc,          // PC of the branch used to index the BTB
    output reg [31:0] branch_target   // Used for misprediction recovery in the IF stage
);

    wire [1:0] next_prediction;       // Use for the branch instruction only

    // Instantiate the FSM outside the procedural block
    predictor_fsm Predictor (
        .currentState(btb_prediction),
        .taken(Zero),
        .nextState(next_prediction)
    );

    always @(*) begin
        flush         = 1'b0;
        write_enable  = 1'b0;
        ex_pc         = id_ex_pc;
        ex_target_pc  = btb_target_pc;
        branch_target = btb_target_pc;
        ex_new_fsm    = next_prediction; 

        // First Priority: BTB_HIT
        if (btb_hit) begin
            
            // 2nd Priority: JALR or normal branch instruction
            if (id_ex_JALR_signal) begin
                ex_new_fsm = 2'b00; // Jumps are always Strongly Taken
                if ((btb_target_pc == alu_result) && (!btb_prediction[1])) begin
                    write_enable = 1'b0;
                    flush = 1'b0;
                end else begin
                    write_enable = 1'b1;
                    flush = 1'b1;
                    branch_target = alu_result;
                    ex_target_pc  = alu_result;
                end
            end
            
            else if (id_ex_Branch) begin
                write_enable = 1'b1; 
                ex_new_fsm = next_prediction;
                
                if ((!btb_prediction[1]) == Zero) begin
                    flush = 1'b0;
                end else begin
                    flush = 1'b1;
                    // Correction for misprediction: route based on actual mathematical outcome
                    branch_target = (Zero) ? branch_adder_result : id_ex_pc_plus_4;
                end
            end
            
            else if (id_ex_Jump) begin
                ex_new_fsm = 2'b00;
                // Predict Jump taken
                if (!btb_prediction[1]) begin
                    flush = 1'b0;
                    write_enable = 1'b0;
                end else begin
                    flush = 1'b1;
                    write_enable = 1'b1;
                end
            end
            
        end else begin // BTB_MISS
            
            if (id_ex_JALR_signal) begin
                write_enable = 1'b1;
                ex_new_fsm = 2'b00;
                flush = 1'b1;
                branch_target = alu_result;
                ex_target_pc  = alu_result;
            end
            else if (id_ex_Jump) begin
                write_enable = 1'b1;
                ex_new_fsm = 2'b00;
                flush = 1'b1;
                branch_target = branch_adder_result;
                ex_target_pc  = branch_adder_result;
            end
            else if (id_ex_Branch) begin
                if (Zero) begin
                    flush = 1'b1;
                    write_enable = 1'b1;
                    ex_new_fsm = 2'b01; // Hardcode initial Weakly Taken state for new allocations
                    branch_target = branch_adder_result;
                    ex_target_pc  = branch_adder_result;
                end else begin
                    flush = 1'b0;
                    write_enable = 1'b0; // Ignore Not Taken misses to save BTB memory
                end
            end
        end
    end

endmodule