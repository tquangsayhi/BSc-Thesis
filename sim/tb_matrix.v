`timescale 1ns / 1ps

module tb_riscv_core();

    // Display label only; select the matching CPU source in the project.
    parameter CPU_VERSION = "BTB + RAS Version";

    // These values apply to the current matrix addition firmware.
    parameter [31:0] DATA_ADDR = 32'h000001C8;
    parameter [31:0] EXPECTED_RESULT = 32'd160;
    parameter integer MAX_CYCLES = 100000;

    localparam integer DATA_BASE = DATA_ADDR >> 2;

    reg clk;
    reg aresetn;

    wire [31:0] mmio_address;
    wire [31:0] mmio_write_data;
    wire mmio_write_enable;

    reg [31:0] expected [0:15];
    integer i;
    integer errors;
    integer elapsed_cycles;

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
        errors = 0;

        for (i = 0; i < 16; i = i + 1)
            expected[i] = 32'd10;

        if ((DATA_ADDR[1:0] != 2'b00) ||
            (DATA_ADDR > 32'd16320)) begin
            $fatal(1, "Invalid matrix address for the 16 KiB RAM");
        end

        #100;
        aresetn = 1;
    end

    always @(posedge clk) begin
        if (!aresetn) begin
            elapsed_cycles = 0;
        end else begin
            elapsed_cycles = elapsed_cycles + 1;

            // Completion is checked before timeout on the same edge.
            if (mmio_write_enable &&
                mmio_address == 32'h40000000) begin

                errors = 0;

                $display("========================================");
                $display("       MATRIX ADDITION VERIFICATION     ");
                $display("CPU Version: %0s", CPU_VERSION);

                // Case inequality also detects X/Z values.
                if (mmio_write_data !== EXPECTED_RESULT) begin
                    $display("FAIL: MMIO result = %0d, expected %0d",
                             mmio_write_data, EXPECTED_RESULT);
                    errors = errors + 1;
                end

                // Compare every output element with the expected array.
                for (i = 0; i < 16; i = i + 1) begin
                    if (uut.Data_Mem.memory[DATA_BASE + i] !== expected[i]) begin
                        $display(
                            "FAIL: matC[%0d][%0d] = %0d (hex %08h), expected %0d",
                            i / 4, i % 4,
                            uut.Data_Mem.memory[DATA_BASE + i],
                            uut.Data_Mem.memory[DATA_BASE + i],
                            expected[i]
                        );
                        errors = errors + 1;
                    end
                end

                if (errors != 0) begin
                   $fatal(1, "Matrix verification failed: %0d errors", errors);
                end

                $display("PASS: MMIO result matches");
                $display("PASS: all 16 matrix elements match");

                if ((^uut.Perf_Counters.instruction_count === 1'bx) ||
                    (uut.Perf_Counters.instruction_count == 64'd0)) begin
                    $fatal(1, "Invalid instruction count; cannot calculate CPI");
                end

                // Read counters before this edge's nonblocking updates,
                // matching the original testbench measurement convention.
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