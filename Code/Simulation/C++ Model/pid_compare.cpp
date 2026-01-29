#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

// ============================================================
// FP32 HEX -> float (Verilog localparam 그대로 쓰기 위함)
// ============================================================
static inline float f32_from_hex(uint32_t u){
    float f;
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

// ============================================================
// Verilog와 동일한 FP32 상수(HEX)
// ============================================================
static const float C0  = f32_from_hex(0x3C864B8B); // 0.016393443
static const float C1  = f32_from_hex(0x3DE21965); // 0.110400000
static const float C2  = f32_from_hex(0xBDE4FC8E); // -0.254104918
static const float C3  = f32_from_hex(0x3AEC5C01); // 0.004098361
static const float C4  = f32_from_hex(0xBEA75178); // -0.742203279
static const float C5  = f32_from_hex(0x3F0B6AB1); // 1.237711475
static const float C6  = f32_from_hex(0xBE5F6EF6); // -0.495901639
static const float C7A = f32_from_hex(0x3B9D4952); // 0.040000000
static const float C7B = f32_from_hex(0xB8A505D6); // -0.000655738

static const float YSAT       = f32_from_hex(0x41400000); // 12.0
static const float RECIP_YSAT = f32_from_hex(0x3DAAAAAB); // 1/12
static const float W_TGT      = f32_from_hex(0x42C80000); // 100.0
static const float INT2RADS   = f32_from_hex(0x3F70CAF0); // 0.94059658 rad/s per count

// ============================================================
// Encoder: floor + carry (spdcnt 정수화)
//  - Verilog과 동일하게 INT2RADS를 HEX 상수로 고정
//  - rad_per_cnt = INT2RADS * Ts 로 계산하여 PI/CPR 경로 제거(재현성↑)
// ============================================================
struct EncoderFloor {
    float Ts;
    float rad_per_cnt;   // rad/count
    float int2radfac;    // rad/s per count  (= INT2RADS)
    float theta_rad;
    long  C_prev;

    EncoderFloor(float Ts_)
        : Ts(Ts_), rad_per_cnt(0.0f), int2radfac(INT2RADS), theta_rad(0.0f), C_prev(0)
    {
        rad_per_cnt = int2radfac * Ts; // (rad/s per count)*Ts = rad/count
    }

    void sample(float x_true, int& spdcnt, float& x_meas) {
        theta_rad += x_true * Ts;
        const float C_real = theta_rad / rad_per_cnt;
        const long  C_now  = (long)std::floor(C_real); // floor 고정

        spdcnt = (int)(C_now - C_prev);
        C_prev = C_now;

        x_meas = (float)spdcnt * int2radfac; // rad/s
    }
};

// ============================================================
// General PID (FP32) : P/I/D 분리형 + 1차 D필터 + back-calculation AW(1-tap)
//  - 이 부분은 "수식 동치" 비교용이므로 기존 형태 유지
// ============================================================
class GeneralPidControllerF32 {
public:
    GeneralPidControllerF32(float kp, float ki, float kd,
                            float a, float b, float c,
                            float kb,
                            float dt, float out_min, float out_max)
        : Kp(kp), Ki(ki), Kd(kd),
          a_param(a), b_weight(b), c_weight(c),
          Kb(kb),
          dt(dt), output_min(out_min), output_max(out_max)
    { reset(); }

    void reset() {
        integral_term = 0.0f;
        derivative_term_prev = 0.0f;
        setpoint_prev = 0.0f;
        measurement_prev = 0.0f;
        unsaturated_output_prev = 0.0f;
        saturated_output_prev   = 0.0f;
    }

    float calculate(float setpoint, float measurement) {
        const float error = setpoint - measurement;

        const float p_term = Kp * (b_weight * setpoint - measurement);

        const float Td = (Kp > 1e-12f) ? (Kd / Kp) : 0.0f;
        const float common_denominator = a_param * Td + dt; // (a*Td + Ts)
        const float coeff_feedback     = (a_param * Td) / common_denominator;
        const float coeff_gain         = (Kp * Td) / common_denominator;

        const float derivative_input_change =
            (c_weight * (setpoint - setpoint_prev)) - (measurement - measurement_prev);

        const float d_term = coeff_feedback * derivative_term_prev
                           + coeff_gain * derivative_input_change;

        const float saturation_error = saturated_output_prev - unsaturated_output_prev; // e_sat[n-1]
        integral_term += Ki * (error + Kb * saturation_error) * dt;

        const float unsaturated_output = p_term + integral_term + d_term;
        const float saturated_output   = std::max(output_min, std::min(unsaturated_output, output_max));

        setpoint_prev           = setpoint;
        measurement_prev        = measurement;
        derivative_term_prev    = d_term;
        unsaturated_output_prev = unsaturated_output;
        saturated_output_prev   = saturated_output;

        return saturated_output;
    }

private:
    float Kp, Ki, Kd;
    float a_param, b_weight, c_weight;
    float Kb;
    float dt, output_min, output_max;

    float integral_term;
    float derivative_term_prev;
    float setpoint_prev, measurement_prev;
    float unsaturated_output_prev, saturated_output_prev;
};

// ============================================================
// Δ-form PID (FP32) : Verilog HEX 계수 기반 + 2-tap AW
//  - compute_delta_coeffs() 제거: 계수는 HEX 상수 그대로 사용
// ============================================================
class DeltaPid2TapAw {
public:
    explicit DeltaPid2TapAw(float y_sat_limit)
        : YSAT_(y_sat_limit) { reset(); }

    void reset() {
        dy1 = 0.0f;
        w1 = w2 = 0.0f;
        x1 = x2 = 0.0f;
        y_unsat_1 = 0.0f;  y_unsat_2 = 0.0f;
        y_sat_1   = 0.0f;  y_sat_2   = 0.0f;
    }

    float step(float w, float x) {
        const float e_sat_1 = y_sat_1 - y_unsat_1;
        const float e_sat_2 = y_sat_2 - y_unsat_2;

        // RTL이 adder-tree일 수도 있어서, 일단 원래 식(직렬 누산) 그대로 둡니다.
        // (정확히 맞추려면 RTL의 누산 순서/파이프라인과 동일하게 바꾸는 게 핵심)
        const float dy =
              C0 * dy1
            + C1 * w  + C2 * w1 + C3 * w2
            + C4 * x  + C5 * x1 + C6 * x2
            + C7A * e_sat_1 + C7B * e_sat_2;

        const float y_unsat = y_unsat_1 + dy;
        const float y_sat   = std::clamp(y_unsat, -YSAT_, +YSAT_);

        dy1 = dy;
        w2 = w1; w1 = w;
        x2 = x1; x1 = x;
        y_unsat_2 = y_unsat_1;  y_unsat_1 = y_unsat;
        y_sat_2   = y_sat_1;    y_sat_1   = y_sat;

        return y_sat;
    }

private:
    float YSAT_;

    float dy1;
    float w1, w2;
    float x1, x2;
    float y_unsat_1, y_unsat_2;
    float y_sat_1,   y_sat_2;
};

// ============================================================
// Compare driver (허용오차 1e-3, 200 samples)
// ============================================================
int main() {
    // ---- 공통 파라미터 ----
    const float Kp = 0.11f;
    const float Ki = 0.08f;
    const float TD = 0.010f;
    const float Kd = Kp * TD;
    const float N  = 120.0f;
    const float a  = 1.0f / N;
    const float b  = 1.0f;
    const float c  = 0.0f;
    const float Ts = 0.005f;
    const float Kb = 12.0f;
    const float Tt = 1.0f / Kb; (void)Tt; // 여기서는 일반 PID용 파라미터로만 쓰고, delta는 HEX 고정

    const float UMIN = -YSAT, UMAX = YSAT;

    // ---- 컨트롤러 2개 생성 ----
    GeneralPidControllerF32 pid_general(Kp, Ki, Kd, a, b, c, Kb, Ts, UMIN, UMAX);
    DeltaPid2TapAw          pid_delta(YSAT);

    // ---- 식물(1차) & 엔코더 ----
    const float Ku  = 50.0f;
    const float lam = 5.0f;

    float x_true = 0.0f;
    EncoderFloor enc(Ts);

    // ---- 비교 설정 ----
    const int   SAMPLES = 100;
    const float ABS_TOL = 1e-3f;
    const float REL_TOL = 1e-3f;

    float max_abs_err = 0.0f;
    float max_rel_err = 0.0f;
    int   max_err_n   = -1;
    int mismatch_cnt  = 0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "# INT_TO_RADS_FACTOR(FP32 hex) = " << std::setprecision(9) << (double)INT2RADS << "\n";
    std::cout << std::setprecision(6);
    std::cout << "n | t[s]  | spdcnt | x_meas  | y_general   | y_delta     | abs_err    | rel_err\n";
    std::cout << "-------------------------------------------------------------------------------------\n";

    for (int n = 0; n < SAMPLES; ++n) {
        const float t = (float)n * Ts;

        int spdcnt = 0;
        float x_meas = 0.0f;
        enc.sample(x_true, spdcnt, x_meas);

        // 같은 입력(동일 x_meas, 동일 setpoint)
        const float y_g = pid_general.calculate(W_TGT, x_meas);
        const float y_d = pid_delta.step(W_TGT, x_meas);

        const float abs_err = std::fabs(y_g - y_d);
        const float denom   = std::max(1e-12f, std::fabs(y_d));
        const float rel_err = abs_err / denom;

        if (abs_err > max_abs_err) {
            max_abs_err = abs_err;
            max_rel_err = rel_err;
            max_err_n   = n;
        }

        const bool ok = (abs_err <= ABS_TOL) || (rel_err <= REL_TOL);
        if (!ok) mismatch_cnt++;

        std::cout << std::setw(3) << n << " | "
                  << std::setw(6) << (double)t << " | "
                  << std::setw(6) << spdcnt << " | "
                  << std::setw(7) << (double)x_meas << " | "
                  << std::setw(11) << std::setprecision(9) << (double)y_g << " | "
                  << std::setw(11) << std::setprecision(9) << (double)y_d << " | "
                  << std::setw(9)  << std::setprecision(9) << (double)abs_err << " | "
                  << std::setw(9)  << std::setprecision(9) << (double)rel_err
                  << (ok ? "" : "  <-- mismatch")
                  << "\n";

        // 식물 업데이트는 기준으로 y_delta 사용
        x_true += Ts * (Ku * y_d - lam * x_true);

        std::cout << std::setprecision(6);
    }

    std::cout << "\n=== SUMMARY ===\n";
    std::cout << "Samples         : " << SAMPLES << "\n";
    std::cout << "Mismatch count  : " << mismatch_cnt
              << " (ABS_TOL=" << (double)ABS_TOL << ", REL_TOL=" << (double)REL_TOL << ")\n";

    return (mismatch_cnt == 0) ? 0 : 1;
}
