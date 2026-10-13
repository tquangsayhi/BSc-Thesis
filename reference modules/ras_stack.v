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
