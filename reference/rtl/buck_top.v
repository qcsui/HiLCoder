// buck_top.v — Dual-clock Buck solver with dual 14-bit DAC outputs
// solver @ 180 MHz, DAC I/O @ 20 MHz (solver_clk/9)

module buck_top (
    input  wire        ref_clk,       // 180 MHz
    input  wire        rst_n,
    input  wire [15:0] pwm_duty,
    output wire [13:0] da_ch1,        // vC (14-bit unsigned, 0=-5V, 8192=0V, 16383=+5V)
    output wire [13:0] da_ch2,        // iL
    output wire        da_clk1,       // CH1 clock (= 20 MHz)
    output wire        da_wrt1,       // CH1 write enable
    output wire        da_clk2,       // CH2 clock
    output wire        da_wrt2        // CH2 write enable
);

    // Clock divider: solver_clk/9 → 20 MHz base rate
    reg [3:0] div_cnt;
    wire      pulse_1t = (div_cnt == 0);  // 1-cycle pulse every 9

    always @(posedge ref_clk or negedge rst_n) begin
        if (!rst_n) div_cnt <= 0;
        else div_cnt <= (div_cnt >= 8) ? 0 : div_cnt + 1;
    end

    // Pipeline enable: stretch pulse_1t to 4 cycles (depth of solver pipeline)
    reg [2:0] pipe_shift;
    wire      io_en = (pipe_shift != 0);  // high for 4 cycles

    always @(posedge ref_clk or negedge rst_n) begin
        if (!rst_n) pipe_shift <= 0;
        else if (pulse_1t) pipe_shift <= 4'd4;
        else if (io_en)    pipe_shift <= pipe_shift - 1'd1;
    end

    // PWM generator (20 kHz at 180 MHz / 9 × 4-cycle en)
    reg [15:0] pwm_cnt;
    wire       pwm_out;

    always @(posedge ref_clk or negedge rst_n) begin
        if (!rst_n) pwm_cnt <= 0;
        else if (pulse_1t) pwm_cnt <= (pwm_cnt >= 999) ? 0 : pwm_cnt + 1;
    end

    assign pwm_out = (pwm_cnt < pwm_duty);

    // Solver core
    wire [23:0] iL_q, vC_q;

    buck_solver solver_inst (
        .clk   (ref_clk),
        .rst_n (rst_n),
        .en    (io_en),
        .pwm   (pwm_out),
        .iL_out(iL_q),
        .vC_out(vC_q)
    );

    // CDC synchronizers for both channels
    reg [23:0] vC_s1, vC_s2;
    reg [23:0] iL_s1, iL_s2;

    always @(posedge ref_clk or negedge rst_n) begin
        if (!rst_n) begin
            vC_s1 <= 0; vC_s2 <= 0;
            iL_s1 <= 0; iL_s2 <= 0;
        end else if (io_en) begin
            vC_s1 <= vC_q; vC_s2 <= vC_s1;
            iL_s1 <= iL_q; iL_s2 <= iL_s1;
        end
    end

    // 14-bit DAC encoding: unsigned, mid-scale 8192 = 0V
    // Clamp negative values to 0 first (Q7.17 signed → non-negative)
    wire [23:0] vC_pos = vC_s2[23] ? 24'd0 : vC_s2;
    wire [23:0] iL_pos = iL_s2[23] ? 24'd0 : iL_s2;
    // vC in Q7.17: bits [21:9] = 13-bit → + offset 8192 = 8192-16383
    assign da_ch1 = 14'd8192 + vC_pos[22:10];
    assign da_ch2 = 14'd8192 + iL_pos[22:10];

    // DAC strobe: 1-cycle pulse at 20 MHz rate (use pulse_1t, not io_en)
    reg da_strobe;
    always @(posedge ref_clk or negedge rst_n) begin
        if (!rst_n) da_strobe <= 0;
        else da_strobe <= pulse_1t;
    end

    assign da_clk1 = da_strobe;
    assign da_wrt1 = da_strobe;
    assign da_clk2 = da_strobe;
    assign da_wrt2 = da_strobe;

endmodule
