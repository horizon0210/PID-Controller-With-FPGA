`timescale 1ns/1ps
module motor_control_top #(
    // 클록/주파수 설정
    parameter integer CLK_HZ  = 100_000_000,
    parameter integer PWM_HZ  = 20_000,
    parameter integer GATE_HZ = 200,

    // 인코더 카운트→물리량 변환계수 (FP32, 필요시 변경)
    // 예: 0.94059658 (rad/ct) = 32'h3F70CAF0
    parameter [31:0] INT_TO_RADS_FACTOR = 32'h3F70CAF0
)(
    input  wire aclk,
    input  wire rst_n,

    // Encoder 입력
    input  wire enc_a_in,
    input  wire enc_b_in,

    // 목표값 (FP32, PID 설계 단위와 일치: 예 rad/s)
    input  wire [31:0] w_target_fp_in,

    // === PID 계수/포화값: 상위에서 입력으로 제공 ===
    input  wire [31:0] a0_in,
    input  wire [31:0] c1_in, input wire [31:0] c2_in, input wire [31:0] c3_in,
    input  wire [31:0] c4_in, input wire [31:0] c5_in, input wire [31:0] c6_in, input wire [31:0] c7_in, input wire[31:0] c8_in,
    input  wire [31:0] ysat_in,              // 예: 12.0f

    // PWM 스케일 일치용: 1/YSAT (FP32) 를 함께 입력
    //  (pwm_generator가 런타임 입력으로 역수 사용한다고 가정)
    input  wire [31:0] recip_ysat_in,        // 예: (1/12) = 0x3DAAAAAB

    // 드라이버 인터페이스
    output wire rpwm,   // RPWM
    output wire lpwm,   // LPWM
    output wire r_en,   // R_EN (active-high)
    output wire l_en,    // L_EN (active-high)
    output wire [31:0] spdcnt_32bit
);
    // ---------------- Encoder ----------------
    
    
    wire signed [15:0] spdcnt;
    wire delta_valid;
    wire enc_dir;
    wire [15:0] enc_err_illegal;
    
    assign spdcnt_32bit = {{16{spdcnt[15]}},spdcnt };
    

    enc_pulse #(
        .CLK_HZ   (CLK_HZ),
        .GATE_HZ  (GATE_HZ),
        .MINPW_CYC(50)
    ) u_enc (
        .clk         (aclk),
        .rst_n       (rst_n),
        .enc_a_in    (enc_a_in),
        .enc_b_in    (enc_b_in),
        .spdcnt      (spdcnt),
        .delta_valid (delta_valid),
        .dir         (enc_dir),
        .err_illegal (enc_err_illegal)
      
    );



    // ---------------- PID ----------------
    wire [31:0] y_out;
    wire        out_valid;
    wire        busy;

    pid_controller_axi #(
        .INT_TO_RADS_FACTOR (INT_TO_RADS_FACTOR)
    ) u_pid (
        .aclk          (aclk),
        .rst_n         (rst_n),
        .w_target_fp_in(w_target_fp_in),
        .x_spdcnt_in   (spdcnt),
        .data_valid_in (delta_valid),

        // ★ 상위 입력 계수/포화 값 전달
        .a0_in(a0_in),
        .c1_in(c1_in), .c2_in(c2_in), .c3_in(c3_in),
        .c4_in(c4_in), .c5_in(c5_in), .c6_in(c6_in), .c7a_in(c7_in), .c7b_in(c8_in),
        .ysat_in(ysat_in),

        .y_out   (y_out),
        .busy    (busy),
        .out_valid(out_valid)
    );


    // ---------------- PWM ----------------
    // 주의: pwm_generator가 런타임 입력으로 역수(rec 1/YSAT)를 받는 개정형이라고 가정
    
    wire pwm_core;
    wire dir_core;
    pwm_generator #(
        .CLK_HZ   (CLK_HZ),
        .PWM_FREQ (PWM_HZ)
    ) u_pwm (
        .aclk                 (aclk),
        .rst_n                (rst_n),
        .voltage_in           (y_out),          // FP32, ±YSAT
        .voltage_valid        (out_valid),      // PID 결과 갱신 시 반영
        .recip_max_voltage_fp (recip_ysat_in),  // = 1/YSAT
        .pwm_out              (pwm_core),
        .dir_out              (dir_core)
    );

    // --------- 드라이버 보완/킬 ---------
    // EN 핀은 항상 활성
    assign r_en = 1'b1;
    assign l_en = 1'b1;

    // 방향별 한쪽 PWM만 사용
    assign rpwm = (dir_core ? pwm_core : 1'b0);
    assign lpwm = (dir_core ? 1'b0 : pwm_core);
    


endmodule
