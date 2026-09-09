// clk_gen.v — Synthesizable clock generation using MMCME2_BASE
// 50 MHz reference → 180 MHz solver + 20 MHz DAC
// VCO = 50 MHz × 18 = 900 MHz
//   CLKOUT0 = 900 / 5   = 180 MHz
//   CLKOUT1 = 900 / 45  = 20 MHz

module clk_gen (
    input  wire clk_in,       // 50 MHz
    input  wire rst,          // async reset (active high)
    output wire clk_solver,   // 180 MHz
    output wire clk_dac,      // 20 MHz
    output wire locked
);

    wire clkfb;

    MMCME2_BASE #(
        .CLKIN1_PERIOD(20.0),
        .CLKFBOUT_MULT_F(18.0),
        .CLKOUT0_DIVIDE_F(5.0),
        .CLKOUT1_DIVIDE(45),
        .DIVCLK_DIVIDE(1)
    ) mmcm_inst (
        .CLKIN1(clk_in),
        .RST    (rst),
        .CLKFBIN(clkfb),
        .CLKFBOUT(clkfb),
        .CLKOUT0(clk_solver),
        .CLKOUT1(clk_dac),
        .LOCKED (locked)
    );

endmodule
