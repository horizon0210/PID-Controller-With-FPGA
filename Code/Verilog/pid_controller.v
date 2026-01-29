`timescale 1ns / 1ps

module pid_controller_axi #(
    // 예: 1.8812 (주석만 예시, 실제 값은 설계에 맞춰 주입)
    parameter [31:0] INT_TO_RADS_FACTOR = 32'h3F70CAF0
)(
    input  wire         aclk,
    input  wire         rst_n,
    input  wire [31:0]  w_target_fp_in,
    input  wire signed [15:0]  x_spdcnt_in,
    input  wire         data_valid_in,

    // PID/Delta-form 계수
    input  wire [31:0]  a0_in,
    input  wire [31:0]  c1_in, input wire [31:0] c2_in, input wire [31:0] c3_in,
    input  wire [31:0]  c4_in, input wire [31:0] c5_in, input wire [31:0] c6_in,
    // === 변경: c7을 2-tap으로 분해 ===
    input  wire [31:0]  c7a_in,  // = Kb*Ts
    input  wire [31:0]  c7b_in,  // = -Kb*Ts*(aTd/(Ts+aTd))  (1-tap이면 0으로)
    // 포화 한계
    input  wire [31:0]  ysat_in,

    output wire [31:0]  y_out,
    output wire         busy,
    output wire         out_valid
);

    // --- FSM 상태 정의 ---
    localparam S_IDLE                 = 6'd0;
    localparam S_LATCH_INPUTS         = 6'd1; 
    localparam S_X_CONV_SETUP         = 6'd2;  localparam S_X_CONV_WAIT         = 6'd3;
    localparam S_XN_CALC_SETUP        = 6'd4;  localparam S_XN_CALC_WAIT        = 6'd5;
    localparam S_MAC1_SETUP           = 6'd6;  localparam S_MAC1_WAIT           = 6'd7;
    localparam S_MAC2_SETUP           = 6'd8;  localparam S_MAC2_WAIT           = 6'd9;
    localparam S_MAC3_SETUP           = 6'd10; localparam S_MAC3_WAIT           = 6'd11;
    localparam S_MAC4_SETUP           = 6'd12; localparam S_MAC4_WAIT           = 6'd13;
    localparam S_MAC5_SETUP           = 6'd14; localparam S_MAC5_WAIT           = 6'd15;
    localparam S_MAC6_SETUP           = 6'd16; localparam S_MAC6_WAIT           = 6'd17;
    localparam S_MAC_C6_SETUP         = 6'd18; localparam S_MAC_C6_WAIT         = 6'd19;

    // === 신규: 분기 없는 2-tap AW 경로(감산 2회 + 누적 2회) ===
    localparam S_AW_E1_SUB_SETUP      = 6'd20; localparam S_AW_E1_SUB_WAIT      = 6'd21; // eaw1 = ysat_d1 - y_d1
    localparam S_AW_ACC1_SETUP        = 6'd22; localparam S_AW_ACC1_WAIT        = 6'd23; // sum = c7a*eaw1 + sum_mac
    localparam S_AW_E2_SUB_SETUP      = 6'd24; localparam S_AW_E2_SUB_WAIT      = 6'd25; // eaw2 = ysat_d2 - y_d2
    localparam S_AW_ACC2_SETUP        = 6'd26; localparam S_AW_ACC2_WAIT        = 6'd27; // delta_y = c7b*eaw2 + sum

    localparam S_ADD_Y_SETUP          = 6'd28; localparam S_ADD_Y_WAIT          = 6'd29;

    // 출력 포화 체크(현재 y_n)
    localparam S_SAT_CHECK_GT_SETUP   = 6'd30; localparam S_SAT_CHECK_GT_WAIT   = 6'd31;
    localparam S_SAT_CHECK_LT_SETUP   = 6'd32; localparam S_SAT_CHECK_LT_WAIT   = 6'd33;
    localparam S_SAT_FINALIZE         = 6'd34;
    localparam S_UPDATE               = 6'd35;

    reg [5:0] state, next_state;

    // --- Opcode/상수 ---
    // Xilinx FP FMA core: 0x00 (a*b + c), 0x01 (a*b - c)
    localparam OP_FMA = 8'h00;
    localparam OP_SUB = 8'h01;

    // Comparator 결과: bit0=EQ(1), bit1=LT(2), bit2=GT(4) ... (포화 체크용)
    localparam COND_LT = 8'h02; 
    localparam COND_GT = 8'h04; 

    localparam [31:0] FP_ZERO = 32'h00000000; // 0.0
    localparam [31:0] FP_ONE  = 32'h3F800000; // 1.0

    // --- 데이터 레지스터 ---
    reg  signed [15:0] x_spdcnt_reg;
    reg  [31:0] w_n_fp, x_n_fp, x_spdcnt_fp_temp;
    reg  [31:0] delta_y_d1, w_d1, w_d2, x_d1, x_d2, y_d1, y_d2;
    reg  [31:0] sum_mac, sub_result, delta_y, y_n, y_out_reg;

    // 포화 출력 이력(2-tap AW용)
    reg  [31:0] y_sat_d1, y_sat_d2;

    // Comparator(출력 포화 판단용)
    reg  [7:0]  comp_gt_code, comp_lt_code;

    // --- AXI-Stream 신호 ---
    wire s_fma_a_tready, s_fma_b_tready, s_fma_c_tready, s_fma_op_tready, m_fma_result_tvalid;
    wire [31:0] m_fma_result_tdata;
    reg  s_fma_a_tvalid, s_fma_b_tvalid, s_fma_c_tvalid, s_fma_op_tvalid, m_fma_result_tready;
    reg  [31:0] s_fma_a_tdata, s_fma_b_tdata, s_fma_c_tdata;
    reg  [7:0]  s_fma_op_tdata;
    
    wire s_comp_a_tready, s_comp_b_tready, m_comp_result_tvalid;
    wire [7:0] m_comp_result_tdata;
    reg  s_comp_a_tvalid, s_comp_b_tvalid, m_comp_result_tready;
    reg  [31:0] s_comp_a_tdata, s_comp_b_tdata;

    wire s_i2f_tready, m_i2f_tvalid;
    wire [31:0] m_i2f_tdata;
    reg  s_i2f_tvalid, m_i2f_tready;
    reg  signed [15:0] s_i2f_tdata;

    // ============================================================
    // 1) 순차 로직
    // ============================================================
    always @(posedge aclk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            y_d1 <= 32'h0; y_out_reg <= 32'h0; y_d2 <= 32'h0; 
            y_sat_d1 <= 32'h0; y_sat_d2 <= 32'h0;
            delta_y_d1 <= 32'h0; w_d1 <= 32'h0; w_d2 <= 32'h0; x_d1 <= 32'h0; x_d2 <= 32'h0;
            w_n_fp <= 32'h0; x_n_fp <= 32'h0; x_spdcnt_fp_temp <= 32'h0;
            sum_mac <= 32'h0; sub_result <= 32'h0; delta_y <= 32'h0; y_n <= 32'h0;
            comp_gt_code <= 8'h0; comp_lt_code <= 8'h0;
            x_spdcnt_reg <= 16'd0;
        end else begin
            state <= next_state;

            // 입력 래치
            if (state == S_IDLE && data_valid_in) begin
                x_spdcnt_reg <= x_spdcnt_in;
                w_n_fp       <= w_target_fp_in;
            end

            // int->float 결과
            if (m_i2f_tvalid && m_i2f_tready) begin
                if (state == S_X_CONV_WAIT)
                    x_spdcnt_fp_temp <= m_i2f_tdata;
            end

            // FMA 결과 래치
            if (m_fma_result_tvalid && m_fma_result_tready) begin
                case(state)
                    S_XN_CALC_WAIT: x_n_fp <= m_fma_result_tdata;

                    // 누적 합(sum_mac) 갱신 지점들
                    S_MAC1_WAIT, S_MAC2_WAIT, S_MAC3_WAIT, S_MAC4_WAIT,
                    S_MAC5_WAIT, S_MAC6_WAIT, S_MAC_C6_WAIT,
                    S_AW_ACC1_WAIT:
                        sum_mac <= m_fma_result_tdata;

                    // 감산 결과(ysat - y) 보관
                    S_AW_E1_SUB_WAIT,
                    S_AW_E2_SUB_WAIT:
                        sub_result <= m_fma_result_tdata;

                    // 최종 delta_y
                    S_AW_ACC2_WAIT:  delta_y <= m_fma_result_tdata;

                    // y_n
                    S_ADD_Y_WAIT:    y_n     <= m_fma_result_tdata;
                endcase
            end

            // Comparator 결과 (출력 포화 판단)
            if (m_comp_result_tvalid && m_comp_result_tready) begin
                case (state)
                    S_SAT_CHECK_GT_WAIT: comp_gt_code  <= m_comp_result_tdata;
                    S_SAT_CHECK_LT_WAIT: comp_lt_code  <= m_comp_result_tdata;
                endcase
            end

            // 출력 포화 최종 결정
            if (state == S_SAT_FINALIZE) begin
                if (comp_gt_code == COND_GT)
                    y_out_reg <= ysat_in;
                else if (comp_lt_code == COND_LT)
                    y_out_reg <= {~ysat_in[31], ysat_in[30:0]}; // -ysat
                else
                    y_out_reg <= y_n;
            end

            // 파이프/지연 레지스터 업데이트
            if (state == S_UPDATE) begin
                delta_y_d1 <= delta_y;
                w_d2 <= w_d1;       w_d1 <= w_n_fp;
                x_d2 <= x_d1;       x_d1 <= x_n_fp;
                y_d2 <= y_d1;      // ★ 추가: y[n-2] <= y[n-1]
                y_d1 <= y_n;       // 비포화 y[n] 저장 (AW back-calc용)
                y_sat_d2 <= y_sat_d1;             // 포화 출력 이력
                y_sat_d1 <= y_out_reg;            // 포화 y[n]
            end
        end
    end

    // ============================================================
    // 2) 조합 로직 (상태/핸드셰이크)
    // ============================================================
    always @* begin
        next_state = state;

        // 기본값
        s_fma_a_tvalid=0; s_fma_b_tvalid=0; s_fma_c_tvalid=0; s_fma_op_tvalid=0; m_fma_result_tready=0;
        s_comp_a_tvalid=0; s_comp_b_tvalid=0; m_comp_result_tready=0;
        s_i2f_tvalid=0; m_i2f_tready=0;

        s_i2f_tdata=0; s_fma_a_tdata=0; s_fma_b_tdata=0; s_fma_c_tdata=0; s_fma_op_tdata=0; s_comp_a_tdata=0; s_comp_b_tdata=0;
        
        case (state)
            S_IDLE: if (data_valid_in) next_state = S_LATCH_INPUTS;

            S_LATCH_INPUTS: next_state = S_X_CONV_SETUP;

            // int -> float
            S_X_CONV_SETUP: begin
                s_i2f_tvalid = 1'b1; s_i2f_tdata = x_spdcnt_reg;
                if (s_i2f_tready) next_state = S_X_CONV_WAIT;
            end
            S_X_CONV_WAIT: begin
                m_i2f_tready = 1'b1;
                if (m_i2f_tvalid) next_state = S_XN_CALC_SETUP;
            end
            
            // x[n] = x_cnt * factor
            S_XN_CALC_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = x_spdcnt_fp_temp;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = INT_TO_RADS_FACTOR;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = FP_ZERO;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_XN_CALC_WAIT;
            end
            S_XN_CALC_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_MAC1_SETUP;
            end

            // sum_mac = c1*w[n]
            S_MAC1_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = c1_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = w_n_fp;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = FP_ZERO;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_MAC1_WAIT;
            end
            S_MAC1_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_MAC2_SETUP;
            end

            // sum_mac += a0*Δy[n-1]
            S_MAC2_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = a0_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = delta_y_d1;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = sum_mac;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_MAC2_WAIT;
            end
            S_MAC2_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_MAC3_SETUP;
            end
            
            // sum_mac += c2*w[n-1]
            S_MAC3_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = c2_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = w_d1;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = sum_mac;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_MAC3_WAIT;
            end
            S_MAC3_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_MAC4_SETUP;
            end
            
            // sum_mac += c3*w[n-2]
            S_MAC4_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = c3_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = w_d2;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = sum_mac;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_MAC4_WAIT;
            end
            S_MAC4_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_MAC5_SETUP;
            end
            
            // sum_mac += c4*x[n]
            S_MAC5_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = c4_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = x_n_fp;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = sum_mac;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_MAC5_WAIT;
            end
            S_MAC5_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_MAC6_SETUP;
            end

            // sum_mac += c5*x[n-1]
            S_MAC6_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = c5_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = x_d1;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = sum_mac;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_MAC6_WAIT;
            end
            S_MAC6_WAIT: begin
                m_fma_result_tready = 1'b1;
                if(m_fma_result_tvalid) next_state = S_MAC_C6_SETUP;
            end

            // sum_mac += c6*x[n-2]
            S_MAC_C6_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = c6_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = x_d2;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = sum_mac;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_MAC_C6_WAIT;
            end
            S_MAC_C6_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_AW_E1_SUB_SETUP;
            end

            // === 2-tap AW: eaw1 = ysat_d1 - y_d1 ===
            S_AW_E1_SUB_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_SUB;  // a*b - c
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = FP_ONE;   // 1.0 * ysat_d1 - y_d1
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = y_sat_d1;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = y_d1;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_AW_E1_SUB_WAIT;
            end
            S_AW_E1_SUB_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_AW_ACC1_SETUP;
            end

            // sum_mac = c7a*eaw1 + sum_mac
            S_AW_ACC1_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;  // a*b + c
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = c7a_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = sub_result; // eaw1
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = sum_mac;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_AW_ACC1_WAIT;
            end
            S_AW_ACC1_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_AW_E2_SUB_SETUP;
            end

            // eaw2 = ysat_d2 - y_d2
            S_AW_E2_SUB_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_SUB;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = FP_ONE;    // 1.0 * ysat_d2 - y_d2
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = y_sat_d2;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = y_d2;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_AW_E2_SUB_WAIT;
            end
            S_AW_E2_SUB_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_AW_ACC2_SETUP;
            end

            // delta_y = c7b*eaw2 + sum_mac
            S_AW_ACC2_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = c7b_in;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = sub_result; // eaw2
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = sum_mac;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_AW_ACC2_WAIT;
            end
            S_AW_ACC2_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_ADD_Y_SETUP;
            end

            // y_n = y[n-1] + delta_y
            S_ADD_Y_SETUP: begin
                s_fma_op_tvalid = 1'b1; s_fma_op_tdata = OP_FMA;
                s_fma_a_tvalid = 1'b1;  s_fma_a_tdata = FP_ONE;
                s_fma_b_tvalid = 1'b1;  s_fma_b_tdata = y_d1;
                s_fma_c_tvalid = 1'b1;  s_fma_c_tdata = delta_y;
                if (s_fma_a_tready && s_fma_b_tready && s_fma_c_tready && s_fma_op_tready) next_state = S_ADD_Y_WAIT;
            end
            S_ADD_Y_WAIT: begin
                m_fma_result_tready = 1'b1;
                if (m_fma_result_tvalid) next_state = S_SAT_CHECK_GT_SETUP;
            end

            // 출력 포화(현재 y_n) 체크
            S_SAT_CHECK_GT_SETUP: begin
                s_comp_a_tvalid = 1'b1; s_comp_a_tdata = y_n;
                s_comp_b_tvalid = 1'b1; s_comp_b_tdata = ysat_in;
                if (s_comp_a_tready && s_comp_b_tready) next_state = S_SAT_CHECK_GT_WAIT;
            end
            S_SAT_CHECK_GT_WAIT: begin
                m_comp_result_tready = 1'b1;
                if (m_comp_result_tvalid) next_state = S_SAT_CHECK_LT_SETUP;
            end

            S_SAT_CHECK_LT_SETUP: begin
                s_comp_a_tvalid = 1'b1; s_comp_a_tdata = y_n;
                s_comp_b_tvalid = 1'b1; s_comp_b_tdata = {~ysat_in[31], ysat_in[30:0]}; // -ysat
                if (s_comp_a_tready && s_comp_b_tready) next_state = S_SAT_CHECK_LT_WAIT;
            end
            S_SAT_CHECK_LT_WAIT: begin
                m_comp_result_tready = 1'b1;
                if (m_comp_result_tvalid) next_state = S_SAT_FINALIZE;
            end

            S_SAT_FINALIZE: begin
                next_state = S_UPDATE;
            end
            
            S_UPDATE: begin
                next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end
    
    assign y_out     = y_out_reg;
    assign out_valid = (state == S_UPDATE) ? 1'b1 : 1'b0;
    assign busy      = (state != S_IDLE);

    // --- IP Inst ---
    floating_point_0 fma_ip (
        .aclk(aclk), 
        .s_axis_a_tvalid(s_fma_a_tvalid), .s_axis_a_tready(s_fma_a_tready), .s_axis_a_tdata(s_fma_a_tdata),
        .s_axis_b_tvalid(s_fma_b_tvalid), .s_axis_b_tready(s_fma_b_tready), .s_axis_b_tdata(s_fma_b_tdata),
        .s_axis_c_tvalid(s_fma_c_tvalid), .s_axis_c_tready(s_fma_c_tready), .s_axis_c_tdata(s_fma_c_tdata),
        .s_axis_operation_tvalid(s_fma_op_tvalid), .s_axis_operation_tready(s_fma_op_tready), .s_axis_operation_tdata(s_fma_op_tdata),
        .m_axis_result_tvalid(m_fma_result_tvalid), .m_axis_result_tready(m_fma_result_tready), .m_axis_result_tdata(m_fma_result_tdata)
    );
    
    floating_point_1 comp_ip (
        .aclk(aclk), 
        .s_axis_a_tvalid(s_comp_a_tvalid), .s_axis_a_tready(s_comp_a_tready), .s_axis_a_tdata(s_comp_a_tdata),
        .s_axis_b_tvalid(s_comp_b_tvalid), .s_axis_b_tready(s_comp_b_tready), .s_axis_b_tdata(s_comp_b_tdata),
        .m_axis_result_tvalid(m_comp_result_tvalid), .m_axis_result_tready(m_comp_result_tready), .m_axis_result_tdata(m_comp_result_tdata)
    );

    floating_point_2 i2f_ip (
        .aclk(aclk), 
        .s_axis_a_tvalid(s_i2f_tvalid), .s_axis_a_tready(s_i2f_tready), .s_axis_a_tdata(s_i2f_tdata),
        .m_axis_result_tvalid(m_i2f_tvalid), .m_axis_result_tready(m_i2f_tready), .m_axis_result_tdata(m_i2f_tdata)
    );

endmodule
