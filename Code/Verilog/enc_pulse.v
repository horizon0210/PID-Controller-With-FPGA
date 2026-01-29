`timescale 1ns/1ps
// ================================================================
// Encoder Pulse Processor (Quad 4x, 2.5ms gate, 8-bit up/down)
// - Verilog-2001
// - 2-FF 동기화 + 최소 펄스폭 필터(>= MINPW_CYC)
// - 4x 디코드(모든 에지에서 ±1), 불법 전이 집계
// - 2.5 ms(=400 Hz)마다 8-bit up/down 누적값 래치 → SPDCNT
// - 경계 클록의 마지막 step까지 포함하여 래치
// - acc8은 단일 always 블록에서만 구동(안전)
// ================================================================
module enc_pulse #(
    parameter integer CLK_HZ     = 100_000_000, // 보드 기준 클록(Hz)
    parameter integer GATE_HZ    = 200,         // 2.5 ms 고정(원하면 수정)
    parameter integer MINPW_CYC  = 50            // 허용 최소 펄스폭(클록 사이클)
)(
    input  wire clk,
    input  wire rst_n,
    input  wire enc_a_in,   // 비동기 A
    input  wire enc_b_in,   // 비동기 B

    output reg  signed [15:0] spdcnt,     // 5 ms당 4x Δcount(부호 있음)
    output reg               delta_valid,// spdcnt 유효(1clk 펄스)
    output reg               dir,        // 최근 전이 기준 방향(1=정방향)
    output reg  [15:0]       err_illegal
);

    // ------------------------------
    // 2-FF 동기화
    // ------------------------------
    // (* ASYNC_REG = "TRUE" *)
    reg [1:0] sync_a, sync_b;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_a <= 2'b00;
            sync_b <= 2'b00;
        end else begin
            sync_a <= {sync_a[0], enc_a_in};
            sync_b <= {sync_b[0], enc_b_in};
        end
    end

    // ------------------------------
    // 최소 펄스폭 필터 (>= MINPW_CYC 유지 시에만 상태 반영)
    // ------------------------------
    reg filt_a, filt_b;
    reg [5:0] a_cnt, b_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            filt_a <= 1'b0; a_cnt <= 6'd0;
            filt_b <= 1'b0; b_cnt <= 6'd0;
            
        end else begin
            // A 채널
            if (sync_a[1] != filt_a) begin
                if (a_cnt >= (MINPW_CYC-1)) begin
                    filt_a <= sync_a[1];
                    a_cnt  <= 6'd0;
                end else begin
                    a_cnt  <= a_cnt + 6'd1;
                end
            end else begin
                a_cnt <= 6'd0;
            end
            // B 채널
            if (sync_b[1] != filt_b) begin
                if (b_cnt >= (MINPW_CYC-1)) begin
                    filt_b <= sync_b[1];
                    b_cnt  <= 6'd0;
                end else begin
                    b_cnt  <= b_cnt + 6'd1;
                end
            end else begin
                b_cnt <= 6'd0;
            end
        end
    end

    // ------------------------------
    // 4x 쿼드러처 디코더 (상태 전이 표) → step: -1/0/+1
    // ------------------------------
    reg [1:0] prev_ab;
    wire [1:0] curr_ab = {filt_a, filt_b};
    reg  signed [1:0] step; // -1,0,+1

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_ab     <= 2'b00;
            step        <= 2'sd0;
            err_illegal <= 16'd0;
            dir         <= 1'b1;
        end else begin
            case ({prev_ab, curr_ab})
                // 정방향(+1): 00→01, 01→11, 11→10, 10→00
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: begin step <=  2'sd1; dir <= 1'b1; end
                // 역방향(-1): 00→10, 10→11, 11→01, 01→00
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: begin step <= -2'sd1; dir <= 1'b0; end
                default: begin
                    // 같은 상태 or 불법 전이(두 비트 동시변화 등)
                    if ((prev_ab != curr_ab) &&
                        ((prev_ab[1]^curr_ab[1]) & (prev_ab[0]^curr_ab[0]))) begin
                        if (err_illegal != 16'hFFFF) err_illegal <= err_illegal + 16'd1;
                    end
                    step <= 2'sd0;
                end
            endcase
            prev_ab <= curr_ab;
        end
    end

    // ------------------------------
    // 게이트 타이머 (2.5 ms) - 펄스 생성
    // ------------------------------
    localparam integer GATE_CYCLES = (CLK_HZ / GATE_HZ); // 정수로 딱 떨어지게 설정 권장
    reg [31:0] gate_cnt;
    wire gate_pulse = (gate_cnt == (GATE_CYCLES - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_cnt <= 32'd0;
        end else begin
            if (gate_pulse)
                gate_cnt <= 32'd0;
            else
                gate_cnt <= gate_cnt + 32'd1;
        end
    end

    // ------------------------------
    // 8-bit up/down 누적 (포화) - 단일 always에서만 구동
    // 경계 클록(step 포함) → spdcnt에 acc8_next 래치 후 acc8 클리어
    // ------------------------------
    reg  signed [15:0] acc8;
    reg  signed [15:0] acc8_next;

    // 포화 더하기(조합)
    always @* begin
        acc8_next = acc8;
        case (step)
            2'sd1:  acc8_next = (acc8 ==  16'sh7FFF)  ? acc8 : acc8 + 16'sd1; // 
           -2'sd1:  acc8_next = (acc8 == -16'sh8000) ? acc8 : acc8 - 16'sd1; // 
            default: /* no change */ ;
        endcase
    end

    // 누적/클리어(순차)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc8 <= 16'sd0;
        end 
        else begin
            if (gate_pulse) begin
                acc8 <= 16'sd0;       // 윈도우 경계에서 클리어
            end else begin
                acc8 <= acc8_next;            // 평소에는 누적
            end
        end
    end

    // ------------------------------
    // 래치 & 유효 펄스 (경계 클록의 마지막 step까지 포함)
    // ------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spdcnt      <= 16'sd0;
            delta_valid <= 1'b0;
        end else begin
            delta_valid <= 1'b0;
            if (gate_pulse) begin
                spdcnt      <= acc8_next; // 마지막 step 포함한 값으로 래치
                delta_valid <= 1'b1;      // 1clk 유효 펄스
            end
        end
    end

endmodule
