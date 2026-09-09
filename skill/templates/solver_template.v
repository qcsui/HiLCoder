// solver_template.v — Parameterized fixed-point solver
// Fill placeholders {QW}, {INT_BITS}, {FRAC_BITS}, etc. before synthesis.
// See SKILL.md Stage 2.1 for placeholder documentation.
//
// Pipeline depth: {PIPELINE} stages (4 for DSP-based >150MHz, 1 for LUT-based)
// Coefficient format: same Q-format as states for well-conditioned systems;
//   use extended fractional format (e.g., Q1.23) when h/L < 2^{-frac_bits}
// Q-format: Q{INT_BITS}.{FRAC_BITS} ({QW}-bit signed)

module solver_core #(
    parameter COEFF_PATH = ""   // ignored in synthesis; for documentation only
)(
    input  wire        clk, rst_n,
    input  wire        en, pwm,
    output wire [{QW}-1:0] iL_out,
    output wire [{QW}-1:0] vC_out
);

    // ── Q-format constants ──
    localparam QMAX = (1 << ({QW}-1)) - 1;
    localparam QMIN = -(1 << ({QW}-1));
    localparam FRAC = {FRAC_BITS};

    // ── Precomputed coefficients in Q-format ──
    // Generated from golden_ref.py → spec.json → coeff_table.py

    // Mode ON (S=ON, D=OFF)
    localparam signed [{QW}-1:0] A00_ON  = {QW}'sd{A00_ON_Q};
    localparam signed [{QW}-1:0] A01_ON  = {QW}'sd{A01_ON_Q};
    localparam signed [{QW}-1:0] A10_ON  = {QW}'sd{A10_ON_Q};
    localparam signed [{QW}-1:0] A11_ON  = {QW}'sd{A11_ON_Q};
    localparam signed [{QW}-1:0] BV0_ON  = {QW}'sd{BV0_ON_Q};  // Bd[0]*Vin
    localparam signed [{QW}-1:0] BV1_ON  = {QW}'sd{BV1_ON_Q};

    // Mode OFF (S=OFF, D=ON)
    localparam signed [{QW}-1:0] A00_OFF = {QW}'sd{A00_OFF_Q};
    localparam signed [{QW}-1:0} A01_OFF = {QW}'sd{A01_OFF_Q};
    localparam signed [{QW}-1:0] A10_OFF = {QW}'sd{A10_OFF_Q};
    localparam signed [{QW}-1:0] A11_OFF = {QW}'sd{A11_OFF_Q};
    localparam signed [{QW}-1:0] BV0_OFF = {QW}'sd{BV0_OFF_Q};
    localparam signed [{QW}-1:0] BV1_OFF = {QW}'sd{BV1_OFF_Q};

    // Mode DCM
    localparam signed [{QW}-1:0] A00_DCM = {QW}'sd{A00_DCM_Q};
    localparam signed [{QW}-1:0] A01_DCM = {QW}'sd{A01_DCM_Q};
    localparam signed [{QW}-1:0] A10_DCM = {QW}'sd{A10_DCM_Q};
    localparam signed [{QW}-1:0] A11_DCM = {QW}'sd{A11_DCM_Q};
    localparam signed [{QW}-1:0] BV0_DCM = {QW}'sd{BV0_DCM_Q};
    localparam signed [{QW}-1:0] BV1_DCM = {QW}'sd{BV1_DCM_Q};

    // ── State registers ──
    reg signed [{QW}-1:0] iL, vC;

    // DCM detection: iL ≤ 0 → clamp to zero
    wire dcm = !pwm && (iL[{QW}-1] || !(|iL));

    // ── Coefficient mux (combinational) ──
    wire signed [{QW}-1:0] a0 = dcm ? A00_DCM : (pwm ? A00_ON : A00_OFF);
    wire signed [{QW}-1:0] a1 = dcm ? A01_DCM : (pwm ? A01_ON : A01_OFF);
    wire signed [{QW}-1:0] a2 = dcm ? A10_DCM : (pwm ? A10_ON : A10_OFF);
    wire signed [{QW}-1:0] a3 = dcm ? A11_DCM : (pwm ? A11_ON : A11_OFF);
    wire signed [{QW}-1:0] b0 = dcm ? BV0_DCM : (pwm ? BV0_ON : BV0_OFF);
    wire signed [{QW}-1:0] b1 = dcm ? BV1_DCM : (pwm ? BV1_ON : BV1_OFF);

    {PIPELINE_SECTION}

    assign iL_out = iL;
    assign vC_out = vC;

endmodule
