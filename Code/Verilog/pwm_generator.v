`timescale 1ns / 1ps

module pwm_generator #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer PWM_FREQ = 20_000
)(
    input  wire         aclk,
    input  wire         rst_n,
    input  wire [31:0]  recip_max_voltage_fp,
    input  wire [31:0]  voltage_in,
    input  wire         voltage_valid,
    output reg          pwm_out,
    output reg          dir_out
);

    // --- FSM 상태 정의 (단순화) ---
    localparam S_IDLE              = 3'd0;
    localparam S_SCALE_MUL_SETUP   = 3'd1;  localparam S_SCALE_MUL_WAIT    = 3'd2;
    localparam S_FINAL_MUL_SETUP   = 3'd3;  localparam S_FINAL_MUL_WAIT    = 3'd4;
    localparam S_F2F_SETUP         = 3'd5;  localparam S_F2F_WAIT          = 3'd6;

    reg [2:0] state, next_state;

    // --- PWM 관련 파라미터 및 레지스터 ---
    localparam integer PWM_PERIOD = CLK_HZ / PWM_FREQ;
    localparam [31:0] PWM_PERIOD_FP = 32'h459C4000; // 5000.0f
    reg [$clog2(PWM_PERIOD)-1:0] pwm_counter;
    reg [$clog2(PWM_PERIOD)-1:0] compare_value;

    // --- 중간 계산값 저장 레지스터 ---
    reg [31:0] voltage_reg;
    wire [31:0] abs_voltage_fp;
    reg [31:0] scaled_val_fp;
    reg [31:0] final_val_fp;

    // --- AXI-Stream 신호 ---
    reg  s_mul_a_tvalid; wire s_mul_a_tready; reg [31:0] s_mul_a_tdata;
    reg  s_mul_b_tvalid; wire s_mul_b_tready; reg [31:0] s_mul_b_tdata;
    wire [31:0] m_mul_tdata;  wire m_mul_tvalid; reg m_mul_tready;
    
    reg  s_f2f_tvalid;   wire s_f2f_tready;   reg [31:0] s_f2f_tdata;
    wire [31:0] m_f2f_tdata;  wire m_f2f_tvalid; reg m_f2f_tready;

    // --- 1. PWM 생성부 (Datapath) ---
    always @(posedge aclk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 0;
            pwm_out <= 1'b0;
        end else begin
            if (pwm_counter >= (PWM_PERIOD - 1)) pwm_counter <= 0;
            else pwm_counter <= pwm_counter + 1;

            if (pwm_counter < compare_value) pwm_out <= 1'b1;
            else pwm_out <= 1'b0;
        end
    end
    
    // --- 2. 절대값 계산 (조합 논리) ---
    assign abs_voltage_fp = {1'b0, voltage_reg[30:0]};

    // --- 3. 듀티 사이클 계산부 (Control Path - FSM) ---
    always @(posedge aclk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            dir_out <= 1'b0;
            compare_value <= 0;
            voltage_reg <= 32'h0;
        end else begin
            state <= next_state;
            
            if (voltage_valid && state == S_IDLE) begin
                voltage_reg <= voltage_in;
                dir_out <= ~voltage_in[31];
            end

            if (state == S_SCALE_MUL_WAIT && m_mul_tvalid && m_mul_tready) scaled_val_fp <= m_mul_tdata;
            if (state == S_FINAL_MUL_WAIT && m_mul_tvalid && m_mul_tready) final_val_fp <= m_mul_tdata;
            if (state == S_F2F_WAIT && m_f2f_tvalid && m_f2f_tready) begin
                if (m_f2f_tdata >= PWM_PERIOD) compare_value <= PWM_PERIOD - 1;
                else                           compare_value <= m_f2f_tdata[15:0]; // 정수 변환 결과 사용
            end
        end
    end

    always @* begin
        next_state = state;
        s_mul_a_tvalid = 0; s_mul_a_tdata = 0;
        s_mul_b_tvalid = 0; s_mul_b_tdata = 0;
        m_mul_tready = 0;
        s_f2f_tvalid = 0; s_f2f_tdata = 0;
        m_f2f_tready = 0;

        case (state)
            S_IDLE: if (voltage_valid) next_state = S_SCALE_MUL_SETUP;
            
            S_SCALE_MUL_SETUP: begin
                s_mul_a_tvalid = 1'b1; s_mul_a_tdata = abs_voltage_fp;
                s_mul_b_tvalid = 1'b1; s_mul_b_tdata = recip_max_voltage_fp;
                if (s_mul_a_tready && s_mul_b_tready) next_state = S_SCALE_MUL_WAIT;
            end
            S_SCALE_MUL_WAIT: begin
                m_mul_tready = 1'b1;
                if (m_mul_tvalid) next_state = S_FINAL_MUL_SETUP;
            end
            
            S_FINAL_MUL_SETUP: begin
                s_mul_a_tvalid = 1'b1; s_mul_a_tdata = scaled_val_fp;
                s_mul_b_tvalid = 1'b1; s_mul_b_tdata = PWM_PERIOD_FP;
                if (s_mul_a_tready && s_mul_b_tready) next_state = S_FINAL_MUL_WAIT;
            end
            S_FINAL_MUL_WAIT: begin
                m_mul_tready = 1'b1;
                if (m_mul_tvalid) next_state = S_F2F_SETUP;
            end
            
            S_F2F_SETUP: begin
                s_f2f_tvalid = 1'b1; s_f2f_tdata = final_val_fp;
                if (s_f2f_tready) next_state = S_F2F_WAIT;
            end
            S_F2F_WAIT: begin
                m_f2f_tready = 1'b1;
                if (m_f2f_tvalid) next_state = S_IDLE;
            end
            
            default: next_state = S_IDLE;
        endcase
    end

    // ================================================================
    // IP Core Instantiation
    // ================================================================
    // IP 이름은 Vivado에서 생성한 실제 이름으로 변경해야 합니다.

    // 1. Multiplier (2-input IP, 2번 재사용됨)
    floating_point_3 mul_inst (
        .aclk(aclk),
        .s_axis_a_tdata(s_mul_a_tdata),
        .s_axis_b_tdata(s_mul_b_tdata),
        .s_axis_a_tvalid(s_mul_a_tvalid),
        .s_axis_a_tready(s_mul_a_tready),
        .s_axis_b_tvalid(s_mul_b_tvalid),
        .s_axis_b_tready(s_mul_b_tready),
        .m_axis_result_tdata(m_mul_tdata),
        .m_axis_result_tvalid(m_mul_tvalid),
        .m_axis_result_tready(m_mul_tready)
    );

    // 2. Float to Fixed (1-input IP)
    floating_point_4 f2f_inst (
        .aclk(aclk),
        .s_axis_a_tdata(s_f2f_tdata),
        .s_axis_a_tvalid(s_f2f_tvalid),
        .s_axis_a_tready(s_f2f_tready),
        .m_axis_result_tdata(m_f2f_tdata),
        .m_axis_result_tvalid(m_f2f_tvalid),
        .m_axis_result_tready(m_f2f_tready)
    );

endmodule