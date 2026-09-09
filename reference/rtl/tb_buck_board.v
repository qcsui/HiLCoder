// tb_buck_board.v — Board-level testbench for Buck on Mizar Z7020 + ACM9767
// Drives hil_top with 50MHz clock, dumps all clock domains for GTKWave.
// 50MHz in → MMCM → 180MHz solver + 20MHz DAC

`timescale 1ns / 10ps

module tb_buck_board;

    reg clk_50m = 0;
    reg rst_n = 0;

    wire [13:0] da_ch1, da_ch2;
    wire        da_clk1, da_wrt1, da_clk2, da_wrt2;
    wire [1:0]  led;

    hil_top dut (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .da_ch1(da_ch1),
        .da_ch2(da_ch2),
        .da_clk1(da_clk1),
        .da_wrt1(da_wrt1),
        .da_clk2(da_clk2),
        .da_wrt2(da_wrt2),
        .led   (led)
    );

    // 50 MHz clock (20ns period)
    always #10 clk_50m = ~clk_50m;

    initial begin
        $dumpfile("buck_board_sim.vcd");
        $dumpvars(0, tb_buck_board);

        #30  rst_n = 1;      // release reset
        #200;                 // wait for MMCM lock

        $display("--- Board simulation started ---");
        $display("clk_50m period = %0d ns", 20);
        $display("MMCM locked = %d", dut.mmcm_locked);

        #2000000;             // run for 2ms

        $display("Done at t=%0t", $time);
        $finish;
    end

endmodule
