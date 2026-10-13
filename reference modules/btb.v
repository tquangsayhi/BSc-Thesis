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