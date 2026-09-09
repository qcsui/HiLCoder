// hil_top.v — Board-level top for Mizar Z7020 + ACM9767 dual 14-bit DAC
// HIL solver (Buck) with MMCM clock generation:
//   50 MHz ref → 180 MHz solver + 20 MHz DAC strobe
// Zero external dependencies.

module hil_top (
    input  wire        clk,          // 50 MHz PL clock (H16)
    input  wire        rst_n,        // KEY1 (R19), active low

    output wire [13:0] da_ch1,       // ACM9767 CH1 data (vC)
    output wire [13:0] da_ch2,       // ACM9767 CH2 data (iL)
    output wire        da_clk1,      // CH1 clock
    output wire        da_wrt1,      // CH1 write enable
    output wire        da_clk2,      // CH2 clock
    output wire        da_wrt2,      // CH2 write enable

    output wire [1:0]  led           // status LEDs
);

    //=================================================================
    // MMCM: 50 MHz → 180 MHz (solver) + 20 MHz (DAC base)
    // VCO = 50 × 18 = 900 MHz
    //   CLKOUT0 = 900/5  = 180 MHz  (solver_clk)
    //   CLKOUT1 = 900/45 = 20 MHz   (DAC reference)
    //=================================================================
    wire        clk_solver;
    wire        clk_dac;
    wire        mmcm_locked;

    clk_gen clk_inst (
        .clk_in(clk),
        .rst    (!rst_n),
        .clk_solver(clk_solver),
        .clk_dac(clk_dac),
        .locked(mmcm_locked)
    );

    wire rst_sys = !rst_n || !mmcm_locked;

    //=================================================================
    // Solver instance
    //=================================================================
    wire [13:0] ch1, ch2;

    buck_top solver_board (
        .ref_clk (clk_solver),
        .rst_n   (!rst_sys),
        .pwm_duty(16'd500),           // fixed 50% duty @ 20kHz
        .da_ch1  (ch1),
        .da_ch2  (ch2),
        .da_clk1 (da_clk1),
        .da_wrt1 (da_wrt1),
        .da_clk2 (da_clk2),
        .da_wrt2 (da_wrt2)
    );

    assign da_ch1 = ch1;
    assign da_ch2 = ch2;

    //=================================================================
    // Status LED: locked = ON
    //=================================================================
    assign led[0] = !mmcm_locked;  // OFF when locked (LED is active low)
    assign led[1] = !rst_n;        // ON during reset

endmodule
