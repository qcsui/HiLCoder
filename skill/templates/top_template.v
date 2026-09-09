// top_template.v — Board-level top with MMCM clock gen + dual DAC
// Fill placeholders: {BOARD_CLK}, {SOLVER_MHZ}, {DAC_MHZ}, {PWM_MAX}

module hil_top (
    input  wire        clk,            // {BOARD_CLK} MHz board clock
    input  wire        rst_n,          // reset button (active low)
    output wire [13:0] da_ch1,         // DAC CH1 data (vC)
    output wire [13:0] da_ch2,         // DAC CH2 data (iL)
    output wire        da_clk1, da_wrt1,  // DAC CH1 clock
    output wire        da_clk2, da_wrt2,  // DAC CH2 clock
    output wire [1:0]  led
);

    // ── MMCM: {BOARD_CLK} MHz → {SOLVER_MHZ} MHz solver + {DAC_MHZ} MHz DAC ──
    // VCO = {BOARD_CLK} MHz × {VCO_MULT}
    wire        clk_solver, clk_dac, mmcm_locked;

    clk_gen clk_inst (
        .clk_in(clk),
        .rst    (!rst_n),
        .clk_solver(clk_solver),
        .clk_dac(clk_dac),
        .locked(mmcm_locked)
    );

    wire rst_sys = !rst_n || !mmcm_locked;

    // ── Clock divider: solver_clk/{DIV_RATIO} → DAC I/O enable ──
    reg [3:0] div_cnt;
    wire      pulse_1t = (div_cnt == 0);

    always @(posedge clk_solver or negedge rst_n) begin
        if (!rst_n) div_cnt <= 0;
        else div_cnt <= (div_cnt >= {DIV_RATIO}-1) ? 0 : div_cnt + 1;
    end

    // Pipeline enable: stretch to 4 cycles for 4-stage solver
    reg [2:0] pipe_shift;
    wire      io_en = (pipe_shift != 0);

    always @(posedge clk_solver or negedge rst_n) begin
        if (!rst_n) pipe_shift <= 0;
        else if (pulse_1t) pipe_shift <= 4'd4;
        else if (io_en)    pipe_shift <= pipe_shift - 1'd1;
    end

    // ── PWM generator ──
    reg [15:0] pwm_cnt;
    wire       pwm_out;

    always @(posedge clk_solver or negedge rst_n) begin
        if (!rst_n) pwm_cnt <= 0;
        else if (pulse_1t) pwm_cnt <= (pwm_cnt >= {PWM_MAX}) ? 0 : pwm_cnt + 1;
    end

    assign pwm_out = (pwm_cnt < {PWM_DUTY});

    // ── Solver core ──
    wire [{QW}-1:0] iL_q, vC_q;

    solver_core solver_inst (
        .clk   (clk_solver),
        .rst_n (!rst_sys),
        .en    (io_en),
        .pwm   (pwm_out),
        .iL_out(iL_q),
        .vC_out(vC_q)
    );

    // ── CDC synchronizer (solver → DAC) ──
    reg [{QW}-1:0] vC_s1, vC_s2;
    reg [{QW}-1:0] iL_s1, iL_s2;

    always @(posedge clk_solver or negedge rst_n) begin
        if (!rst_n) {vC_s1, vC_s2, iL_s1, iL_s2} <= 0;
        else if (io_en) begin
            vC_s1 <= vC_q; vC_s2 <= vC_s1;
            iL_s1 <= iL_q; iL_s2 <= iL_s1;
        end
    end

    // ── DAC encoding (14-bit unsigned, mid-scale 8192 = 0V) ──
    // Safe formula: extract top DAC_BITS bits = [QW-1 : QW-DAC_BITS]
    // Never wraps within the signed Q-format range (max positive < 2^{QW-1})
    wire signed [{QW}-1:0] vC_pos = vC_s2[{QW}-1] ? {QW}{1'b0} : vC_s2;
    wire signed [{QW}-1:0] iL_pos = iL_s2[{QW}-1] ? {QW}{1'b0} : iL_s2;
    assign da_ch1 = 14'd8192 + vC_pos[{QW}-1 : {QW}-14];
    assign da_ch2 = 14'd8192 + iL_pos[{QW}-1 : {QW}-14];

    // ── DAC strobe ──
    reg da_strobe;
    always @(posedge clk_solver or negedge rst_n) begin
        if (!rst_n) da_strobe <= 0;
        else da_strobe <= pulse_1t;
    end
    assign da_clk1 = da_strobe; assign da_wrt1 = da_strobe;
    assign da_clk2 = da_strobe; assign da_wrt2 = da_strobe;

    // ── Status LED ──
    assign led[0] = !mmcm_locked;
    assign led[1] = !rst_n;

endmodule
