module pc_mux (
    input wire [31:0] pc_plus_4,
    input wire btb_hit,
    input wire [1:0] btb_fsm_prediction,
    input wire [31:0] btb_branch_target, //the branch target collected from the btb
    input wire [31:0] branch_target, // the correct branch target got from the branch adder or alu result after misprediction
    input wire misprediction,
    output reg [31:0] next_pc
);
    always @(*) begin
        if (misprediction) begin 
            // Highest priority
            next_pc = branch_target;
        end else begin
            // If MSB is 0 (01 or 00), it is predicted Taken
            if (btb_hit && (btb_fsm_prediction[1] == 1'b0)) begin
                next_pc = btb_branch_target;
            end else begin
               next_pc = pc_plus_4; 
            end
        end
    end
endmodule