`timescale 1ns / 1ps

module tb_riscv_core();
    reg clk;
    reg aresetn;
    
    wire [31:0] mmio_address;
    wire [31:0] mmio_write_data;
    wire mmio_write_enable;
    
    RISC_V uut (
        .clk(clk),
        .aresetn(aresetn),
        .mmio_address(mmio_address),
        .mmio_write_data(mmio_write_data),
        .mmio_write_enable(mmio_write_enable)
    );

    always #10 clk = ~clk;

    initial begin
        clk = 0;
        aresetn = 0; 
        #100;
        aresetn = 1; 
    end

    // Auto-stop when the C code writes to MMIO
    always @(posedge clk) begin
        if (mmio_write_enable && mmio_address == 32'h40000000) begin
            $display("========================================");
            $display("       THESIS BASELINE METRICS          ");
            $display("========================================");
            $display("FINAL C RESULT:     %d", mmio_write_data);
            
            $display("Total Cycles:       %d", uut.Perf_Counters.cycle_count);
            $display("Total Instructions: %d", uut.Perf_Counters.instruction_count);
            $display("Total Branches:     %d", uut.Perf_Counters.branch_count);
            $display("Branch Flushes:     %d", uut.Perf_Counters.branch_flush_count);
            $display("Jump Flushes:       %d", uut.Perf_Counters.jump_flush_count);
            $display("Baseline CPI (x1000): %d", (uut.Perf_Counters.cycle_count * 1000) / uut.Perf_Counters.instruction_count);
            $display("========================================");
            $finish;
        end
    end
endmodule