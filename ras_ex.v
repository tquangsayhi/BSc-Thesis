// ADDED (RAS): Resolved RAS in EX, separate from the speculative IF stack.
// Assumes this CPU resolves control flow in order at EX, with no later traps.
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
