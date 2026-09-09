// clk_gen_sim.v — Simulation-only clock generation (replaces MMCME2_BASE)
// 50 MHz → 180 MHz solver, 20 MHz DAC clock
`timescale 1ns / 10ps

module clk_gen (
    input  wire clk_in,       // 50 MHz
    input  wire rst,          // reset (ignored in sim)
    output wire clk_solver,   // 180 MHz
    output wire clk_dac,      // 20 MHz
    output wire locked
);

    // 180 MHz: period = 5.556ns = 55.56 * 100ps
    // But timescale is 1ns, so we need finer resolution
    // Use phase-shifted oscillators
    reg clk_s = 0;
    reg clk_d = 0;
    reg lock = 0;

    initial begin
        // Wait a bit for reset to settle
        #25 lock = 1;
    end

    // 180 MHz: period = 1/180e6 = 5.556ns → toggle every 2.778ns
    always #2.778 clk_s = ~clk_s;

    // 20 MHz: period = 50ns → toggle every 25ns
    always #25 clk_d = ~clk_d;

    assign clk_solver = clk_s;
    assign clk_dac    = clk_d;
    assign locked     = lock;

endmodule
