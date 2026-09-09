// buck_solver.v — Q7.17 fixed-point, 4-stage pipeline
// Integer bits = 7 (range ±64), fractional bits = 17
// Step size h = 50ns, Backward Euler discretization

module buck_solver (
    input  wire        clk, rst_n,
    input  wire        pwm, en,
    output wire [23:0] iL_out,
    output wire [23:0] vC_out
);

    localparam QMAX = (1 << 23) - 1;
    localparam QMIN = -(1 << 23);

    // Coefficients in Q7.17 (SCALE = 131072)
    // Mode ON (S=ON, D=OFF)
    localparam signed [23:0] A00_ON  =  24'sd131072;  //  1.000000
    localparam signed [23:0] A01_ON  = -24'sd66;      // -0.000500
    localparam signed [23:0] A10_ON  =  24'sd66;      //  0.000500
    localparam signed [23:0] A11_ON  =  24'sd131065;  //  0.999950
    localparam signed [23:0] BV0_ON  =  24'sd1573;    //  0.012000
    localparam signed [23:0] BV1_ON  =  24'sd1;       //  6e-6

    // Mode OFF: same A, BV = 0
    localparam signed [23:0] A00_OFF =  24'sd131072;
    localparam signed [23:0] A01_OFF = -24'sd66;
    localparam signed [23:0] A10_OFF =  24'sd66;
    localparam signed [23:0] A11_OFF =  24'sd131065;
    localparam signed [23:0] BV0_OFF =  24'sd0;
    localparam signed [23:0] BV1_OFF =  24'sd0;

    // Mode DCM
    localparam signed [23:0] A00_DCM =  24'sd131072;  //  1.0
    localparam signed [23:0] A01_DCM =  24'sd0;
    localparam signed [23:0] A10_DCM =  24'sd0;
    localparam signed [23:0] A11_DCM =  24'sd131065;  //  0.99995
    localparam signed [23:0] BV0_DCM =  24'sd0;
    localparam signed [23:0] BV1_DCM =  24'sd0;

    reg signed [23:0] iL, vC;
    wire dcm = !pwm && (iL[23] || !(|iL));  // iL ≤ 0: sign bit OR all-zero

    // Coefficient selection wires (combinational)
    wire signed [23:0] a0 = dcm ? A00_DCM : (pwm ? A00_ON : A00_OFF);
    wire signed [23:0] a1 = dcm ? A01_DCM : (pwm ? A01_ON : A01_OFF);
    wire signed [23:0] a2 = dcm ? A10_DCM : (pwm ? A10_ON : A10_OFF);
    wire signed [23:0] a3 = dcm ? A11_DCM : (pwm ? A11_ON : A11_OFF);
    wire signed [23:0] b0 = dcm ? BV0_DCM : (pwm ? BV0_ON : BV0_OFF);
    wire signed [23:0] b1 = dcm ? BV1_DCM : (pwm ? BV1_ON : BV1_OFF);

    // Stage 1: input registers
    reg signed [23:0] iL_s1, vC_s1;
    reg dcm_s1, pwm_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin iL_s1 <= 0; vC_s1 <= 0; dcm_s1 <= 0; pwm_s1 <= 0; end
        else begin iL_s1 <= iL; vC_s1 <= vC; dcm_s1 <= dcm; pwm_s1 <= pwm; end
    end

    // Stage 2: multiply (DSP48E1)
    reg signed [47:0] p00_s2, p01_s2, p10_s2, p11_s2, pb0_s2, pb1_s2;
    reg dcm_s2, pwm_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p00_s2 <= 0; p01_s2 <= 0; p10_s2 <= 0; p11_s2 <= 0;
            pb0_s2 <= 0; pb1_s2 <= 0; dcm_s2 <= 0; pwm_s2 <= 0;
        end else begin
            dcm_s2 <= dcm_s1; pwm_s2 <= pwm_s1;
            p00_s2 <= a0 * iL_s1;
            p01_s2 <= a1 * vC_s1;
            p10_s2 <= a2 * iL_s1;
            p11_s2 <= a3 * vC_s1;
            pb0_s2 <= dcm_s1 ? 48'd0 : (pwm_s1 ? $signed({{24{b0[23]}}, b0}) <<< 17 : 48'd0);
            pb1_s2 <= dcm_s1 ? 48'd0 : (pwm_s1 ? $signed({{24{b1[23]}}, b1}) <<< 17 : 48'd0);
        end
    end

    // Stage 3: 48-bit add
    reg signed [47:0] sum0_s3, sum1_s3;
    reg dcm_s3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sum0_s3 <= 0; sum1_s3 <= 0; dcm_s3 <= 0; end
        else begin
            sum0_s3 <= p00_s2 + p01_s2 + pb0_s2;
            sum1_s3 <= p10_s2 + p11_s2 + pb1_s2;
            dcm_s3 <= dcm_s2;
        end
    end

    // Stage 4: saturate + state update (Q7.17 → bits [40:17])
    // Overflow: bits [47:41] should all match bit 40
    wire [23:0] iL_n = dcm_s3 ? 24'd0 : (
        (|sum0_s3[47:41] != sum0_s3[40]) ? (sum0_s3[47] ? QMIN : QMAX) : sum0_s3[40:17]);
    wire [23:0] vC_n = (
        (|sum1_s3[47:41] != sum1_s3[40]) ? (sum1_s3[47] ? QMIN : QMAX) : sum1_s3[40:17]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin iL <= 0; vC <= 0; end
        else if (en) begin iL <= iL_n; vC <= vC_n; end
    end

    assign iL_out = iL;
    assign vC_out = vC;

endmodule
