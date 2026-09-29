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