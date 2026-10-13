// ADDED (RAS): Speculative RAS in IF. Decode the currently fetched instruction.
// Read the OLD top for prediction; update once at the accepting clock edge.
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
