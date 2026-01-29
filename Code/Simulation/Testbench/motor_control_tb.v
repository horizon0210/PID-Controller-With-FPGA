`timescale 1ns / 1ps

module motor_control_tb;

    // ─────────────────────────────────────────
    // 보드/게이트/ PWM 파라미터
    // ─────────────────────────────────────────
    localparam integer CLK_HZ      = 100_000_000;
    localparam integer PWM_HZ      = 20_000;
    localparam integer GATE_HZ     = 200;              // ⇒ 5 ms
    localparam integer GATE_CLOCKS = CLK_HZ / GATE_HZ; // = 500_000

    // ─────────────────────────────────────────
    // PID Δ-형 계수 (NEW 세트, 사용자 지정)
    // ─────────────────────────────────────────
    localparam [31:0] A0_FP_NEW  = 32'h3C864B8B; // c0 = 0.016393443
    localparam [31:0] C1_FP_NEW  = 32'h3DE21965; // c1 = 0.110400000
    localparam [31:0] C2_FP_NEW  = 32'hBDE4FC8E; //  c2 = -0.254104918
    localparam [31:0] C3_FP_NEW  = 32'h3AEC5C01; //  c3 =  0.004098361
    localparam [31:0] C4_FP_NEW  = 32'hBEA75178; //  c4 = -0.742203279
    localparam [31:0] C5_FP_NEW  = 32'h3F0B6AB1; //  c5 =  1.237711475
    localparam [31:0] C6_FP_NEW  = 32'hBE5F6EF6; //  c6 = -0.495901639
    // Anti-Windup 2-tap
    localparam [31:0] C7A_FP_NEW = 32'h3B9D4952;  // c7a =  0.040000000
    localparam [31:0] C7B_FP_NEW = 32'hB8A505D6; // c7b = -0.000655738

    // 시스템/목표/포화
    localparam [31:0] YSAT_FP          = 32'h41400000; // 12.0
    localparam [31:0] RECIP_YSAT_FP    = 32'h3DAAAAAB; // 1/12
    localparam [31:0] W_TGT_FP         = 32'h42C80000; // 100.0 (rad/s)

    // 인코더 변환(5 ms, 1336 CPR(4x) 기반)
    localparam [31:0] INT_TO_RADS_FACTOR_FP = 32'h3F70CAF0; // 0.94059658
    localparam real   INT_TO_RADS_REAL      = 0.94059658;   // rad/s per count

    // ─────────────────────────────────────────
    // DUT 신호
    // ─────────────────────────────────────────
    reg  aclk, rst_n;
    reg  enc_a_in, enc_b_in;
    reg  [31:0] w_target_fp_in;
    reg  [31:0] a0_in, c1_in, c2_in, c3_in, c4_in, c5_in, c6_in, c7a_in, c7b_in;
    reg  [31:0] ysat_in, recip_ysat_in;

    wire rpwm, lpwm, r_en, l_en;

    // 관측용(계층 접근)
    wire        dut_delta_valid;
    wire signed [15:0] dut_spdcnt;

    // PWM 내부 관측
    wire        pwm_voltage_valid;
    wire [31:0] pwm_voltage_fp;

    // ─────────────────────────────────────────
    // 테스트 변수
    // ─────────────────────────────────────────
    integer i;

    real    duty_cycle;
    real    equiv_voltage;

    // 1차 식물(게이트당 1회 업데이트)
    real    motor_speed_rad_s;
    real    ku_real, lam_real;

    // 로깅 스냅샷
    integer last_edges_abs;
    real    last_cmd_v;
    real    last_omega;

    // ─────────────────────────────────────────
    // DUT 인스턴스
    // ─────────────────────────────────────────
    motor_control_top #(
        .CLK_HZ             (CLK_HZ),
        .PWM_HZ             (PWM_HZ),
        .GATE_HZ            (GATE_HZ),
        .INT_TO_RADS_FACTOR (INT_TO_RADS_FACTOR_FP)
    ) dut (
        .aclk            (aclk),
        .rst_n           (rst_n),
        .enc_a_in        (enc_a_in),
        .enc_b_in        (enc_b_in),
        .w_target_fp_in  (w_target_fp_in),
        .a0_in           (a0_in),
        .c1_in           (c1_in), .c2_in(c2_in), .c3_in(c3_in),
        .c4_in           (c4_in), .c5_in(c5_in), .c6_in(c6_in),
        .c7_in           (c7a_in), .c8_in(c7b_in),
        .ysat_in         (ysat_in),
        .recip_ysat_in   (recip_ysat_in),
        .rpwm            (rpwm),
        .lpwm            (lpwm),
        .r_en            (r_en),
        .l_en            (l_en)
        // dbg는 생략
    );

    // 내부 신호 접근(계층)
    assign dut_delta_valid   = dut.u_pid.out_valid;
    assign dut_spdcnt        = dut.spdcnt;
    assign pwm_voltage_valid = dut.u_pwm.voltage_valid;
    assign pwm_voltage_fp    = dut.u_pwm.voltage_in;

    // 100 MHz 클록
    always #5 aclk = ~aclk;

    // ─────────────────────────────────────────
    // float32 → real
    // ─────────────────────────────────────────
    function real fp32_to_real;
        input [31:0] x;
        reg     sign;
        integer exp_field;
        integer frac_field;
        real    frac_real, mant;
        integer e;
        begin
            sign       = x[31];
            exp_field  = x[30:23];
            frac_field = x[22:0];
            frac_real  = frac_field / 8388608.0; // 2^23
            if (exp_field == 0) begin
                mant = frac_real; e = -126;
                fp32_to_real = (sign ? -1.0 : 1.0) * mant * (2.0 ** e);
            end else if (exp_field == 255) begin
                fp32_to_real = (sign ? -1.0 : 1.0) * 1.0e30;
            end else begin
                mant = 1.0 + frac_real; e = exp_field - 127;
                fp32_to_real = (sign ? -1.0 : 1.0) * mant * (2.0 ** e);
            end
        end
    endfunction

    // ─────────────────────────────────────────
    // Front-burst 쿼드러처 생성기 (Verilog-2001)
    //  - 정방향(+1): 00→01→11→10→00
    //  - 역방향(-1): 00→10→11→01→00
    //  - enc_pulse(MINPW_CYC=50) 통과 보장: 간격 >= 54 clk
    //  - 버림(floor) 누적으로 정수 카운트 생성
    //  - 5 ms 게이트 내 앞쪽에 몰아서(Front-burst) 에지 발생
    // ─────────────────────────────────────────
    localparam integer MINPW_CYC_TB    = 50;   // enc_pulse와 동일
    localparam integer SYNC_MARGIN     = 4;
    localparam integer EDGE_GAP_CYC    = MINPW_CYC_TB + SYNC_MARGIN; // >= 54
    localparam integer HEAD_DELAY_CYC  = EDGE_GAP_CYC;                // 게이트 초 머리 여유
    localparam integer TAIL_SAFETY_CYC = EDGE_GAP_CYC;                // 게이트 말미 여유
    localparam real    TS_GATE_S       = 1.0 / GATE_HZ;

    // PWM 유효 ↑ 검출(게이트 경계와 동기)
    reg pwm_vv_d;
    wire pwm_vv_rose = pwm_voltage_valid & ~pwm_vv_d;
    always @(posedge aclk or negedge rst_n) begin
        if (!rst_n) pwm_vv_d <= 1'b0;
        else        pwm_vv_d <= pwm_voltage_valid;
    end

    // 상태/카운트
    reg        burst_active;
    integer    edges_left;        // 남은 4x 에지 수(= 이번 게이트 카운트)
    integer    edge_gap_cnt;
    reg        dir_fwd;           // 1=정방향
    real       C_accum_real;      // 누적 카운트(실수, 4x 기준)
    integer    C_floor_prev, C_floor_now;

    // Verilog-2001용 floor (버림)
    function integer floor_int;
        input real r;
        integer itmp;
        begin
            if (r >= 0.0) floor_int = $rtoi(r);
            else begin
                itmp = $rtoi(-r);
                if ((-1.0*itmp) == r) floor_int = -itmp;
                else floor_int = -(itmp+1);
            end
        end
    endfunction
    real vcmd_real;
    integer capacity;
    // enc_a_in / enc_b_in 생성
    always @(posedge aclk or negedge rst_n) begin
        if (!rst_n) begin
            enc_a_in        <= 1'b0;
            enc_b_in        <= 1'b0;
            burst_active    <= 1'b0;
            edges_left      <= 0;
            edge_gap_cnt    <= 0;
            dir_fwd         <= 1'b1;
            C_accum_real    <= 0.0;
            C_floor_prev    <= 0;
            C_floor_now     <= 0;
            motor_speed_rad_s <= 0.0;
            last_edges_abs  <= 0;
            last_cmd_v      <= 0.0;
            last_omega      <= 0.0;
        end else begin
            // 게이트 시작( PID y_out 유효 )
            if (pwm_vv_rose) begin
                

                // 1) 명령전압(+클램프)
                vcmd_real = fp32_to_real(pwm_voltage_fp);
                if (vcmd_real >  12.0) vcmd_real =  12.0;
                if (vcmd_real < -12.0) vcmd_real = -12.0;

                // 2) 1차 식물 업데이트 (게이트당 1회)
                motor_speed_rad_s = motor_speed_rad_s
                                  + (ku_real * vcmd_real - lam_real * motor_speed_rad_s) * TS_GATE_S;

                // 3) 이번 게이트 카운트(버림) : counts = ω / (rad/s per count)
                C_accum_real = C_accum_real + (motor_speed_rad_s / INT_TO_RADS_REAL);
                C_floor_now  = floor_int(C_accum_real);

                edges_left   = C_floor_now - C_floor_prev;
                C_floor_prev = C_floor_now;

                dir_fwd <= (edges_left >= 0);
                if (edges_left < 0) edges_left = -edges_left;

                // 용량 제한(필터 여유 포함)
                capacity = (GATE_CLOCKS - HEAD_DELAY_CYC - TAIL_SAFETY_CYC) / EDGE_GAP_CYC;
                if (edges_left > capacity) edges_left = capacity;

                // 프런트-번치 시작
                if (edges_left == 0) begin
                    burst_active <= 1'b0;
                    edge_gap_cnt <= 0;
                end else begin
                    burst_active <= 1'b1;
                    edge_gap_cnt <= HEAD_DELAY_CYC;
                end

                // 로깅 스냅샷
                last_cmd_v     <= vcmd_real;
                last_omega     <= motor_speed_rad_s;
                last_edges_abs <= edges_left;
            end
            // 버스트 실행: 상태 의존 단일-비트 토글
            else if (burst_active) begin
                if (edge_gap_cnt != 0) begin
                    edge_gap_cnt <= edge_gap_cnt - 1;
                end else if (edges_left > 0) begin
                    if (dir_fwd) begin
                        // +1: 00→01(B↑), 01→11(A↑), 11→10(B↓), 10→00(A↓)
                        case ({enc_a_in, enc_b_in})
                            2'b00: enc_b_in <= ~enc_b_in; // 00->01
                            2'b01: enc_a_in <= ~enc_a_in; // 01->11
                            2'b11: enc_b_in <= ~enc_b_in; // 11->10
                            2'b10: enc_a_in <= ~enc_a_in; // 10->00
                            default: enc_b_in <= ~enc_b_in; // 안전망
                        endcase
                    end else begin
                        // -1: 00→10(A↑), 10→11(B↑), 11→01(A↓), 01→00(B↓)
                        case ({enc_a_in, enc_b_in})
                            2'b00: enc_a_in <= ~enc_a_in; // 00->10
                            2'b10: enc_b_in <= ~enc_b_in; // 10->11
                            2'b11: enc_a_in <= ~enc_a_in; // 11->01
                            2'b01: enc_b_in <= ~enc_b_in; // 01->00
                            default: enc_a_in <= ~enc_a_in; // 안전망
                        endcase
                    end

                    edge_gap_cnt <= EDGE_GAP_CYC;

                    if (edges_left == 1) begin
                        edges_left   <= 0;
                        burst_active <= 1'b0;
                    end else begin
                        edges_left   <= edges_left - 1;
                    end
                end
            end
        end
    end

    // ─────────────────────────────────────────
    // 초기화 & 시나리오
    // ─────────────────────────────────────────
    initial begin
        aclk = 1'b0; rst_n = 1'b0;
        enc_a_in = 1'b0; enc_b_in = 1'b0;

        // PID 계수 주입
        w_target_fp_in = W_TGT_FP;
        a0_in  = A0_FP_NEW;
        c1_in  = C1_FP_NEW; c2_in = C2_FP_NEW; c3_in = C3_FP_NEW;
        c4_in  = C4_FP_NEW; c5_in = C5_FP_NEW; c6_in = C6_FP_NEW;
        c7a_in = C7A_FP_NEW; c7b_in = C7B_FP_NEW;
        ysat_in = YSAT_FP;  recip_ysat_in = RECIP_YSAT_FP;

        // 식물 파라미터(게이트 모델)
        ku_real  = 50;
        lam_real = 5;

        // 리셋
        #(20); rst_n = 1'b1;

        $display("Step | target(spdcnt) | measured spdcnt |   y[V]   |  w[rad/s]");
        $display("-------------------------------------------------------------");

        for (i = 0; i <= 200; i = i + 1) begin
            // 게이트 하나 대기
            repeat (GATE_CLOCKS) @(posedge aclk);
            // enc_pulse의 delta_valid(=PID out_valid) 대기
            @(posedge dut_delta_valid);

            // 로깅(옵션): PWM compare에서 등가전압 추정
            duty_cycle    = dut.u_pwm.compare_value / (CLK_HZ / PWM_HZ + 0.0);
            equiv_voltage = (dut.u_pwm.dir_out) ? (duty_cycle * 12.0) : -(duty_cycle * 12.0);

            $display("%4d | %15d | %16d | %7.9f | %10.6f",
                     i, last_edges_abs, dut_spdcnt, last_cmd_v, last_omega);
        end

        #100;
        $finish;
    end

endmodule
