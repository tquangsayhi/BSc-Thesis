`timescale 1ns / 1ps

module tb_riscv_core();

    // Display label only; select the matching CPU source/configuration.
    parameter CPU_VERSION = "BTB + RAS Version";
    parameter [31:0] EXPECTED_RESULT = 32'd550;
    parameter integer MAX_CYCLES = 100000;

    reg clk;
    reg aresetn;
    integer elapsed_cycles;

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
        elapsed_cycles = 0;

        #100;
        aresetn = 1;
    end

    always @(posedge clk) begin
        if (!aresetn) begin
            elapsed_cycles = 0;
        end else begin
            elapsed_cycles = elapsed_cycles + 1;

            if (mmio_write_enable &&
                mmio_address == 32'h40000000) begin

                $display("========================================");
                $display("         ARRAY SUM VERIFICATION         ");
                $display("CPU Version: %0s", CPU_VERSION);

                // Detect incorrect results, including X/Z values.
                if (mmio_write_data !== EXPECTED_RESULT) begin
                    $fatal(1,
                           "FAIL: MMIO result = %0d, expected %0d",
                           mmio_write_data, EXPECTED_RESULT);
                end

                $display("PASS: array sum matches expected value 550");

                if ((^uut.Perf_Counters.instruction_count === 1'bx) ||
                    (uut.Perf_Counters.instruction_count == 64'd0)) begin
                    $fatal(1, "Invalid instruction count");
                end

                // Same counter sampling convention as the other benchmarks.
                $display("========================================");
                $display("         THESIS CPU METRICS             ");
                $display("FINAL C RESULT:     %0d", mmio_write_data);
                $display("Total Cycles:       %0d",
                         uut.Perf_Counters.cycle_count);
                $display("Total Instructions: %0d",
                         uut.Perf_Counters.instruction_count);
                $display("Total Branches:     %0d",
                         uut.Perf_Counters.branch_count);
                $display("Branch Flushes:     %0d",
                         uut.Perf_Counters.branch_flush_count);
                $display("Jump Flushes:       %0d",
                         uut.Perf_Counters.jump_flush_count);
                $display("CPI:                %0.3f",
                         $itor(uut.Perf_Counters.cycle_count) /
                         $itor(uut.Perf_Counters.instruction_count));
                $display("========================================");

                $finish;

            end else if (elapsed_cycles >= MAX_CYCLES) begin
                $fatal(1, "TIMEOUT: no completion after %0d cycles",
                       MAX_CYCLES);
            end
        end
    end

endmodule