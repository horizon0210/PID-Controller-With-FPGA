#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>
#include <string>

// ============================================================
//  FP32 bit-accurate constants (Verilog localparam HEX 그대로 사용)
// ============================================================
static inline float f32_from_hex(uint32_t u){
    float f;
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

// ---- Verilog coeffs (IEEE-754 FP32 bit pattern) ----
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

// 인코더 변환 상수도 Verilog HEX 그대로
static const float INT2RADS   = f32_from_hex(0x3F70CAF0); // 0.94059658 rad/s per count

// ============================================================
//  RTL 쪽 MUL->ADD와 더 비슷하게 만들기 위한 헬퍼
// ============================================================
static inline float mul_rn(float a, float b) { volatile float r = a * b; return r; }
static inline float add_rn(float a, float b) { volatile float r = a + b; return r; }

// ============================================================
// Δ-form PID (2-tap AW) : Verilog과 동일 계수/누적 순서
// ============================================================
class DeltaPid2TapAw {
public:
    DeltaPid2TapAw(float y_sat_limit) : YSAT_(y_sat_limit) { reset(); }

    void reset() {
        dy1 = 0.0f;
        w1 = w2 = 0.0f;
        x1 = x2 = 0.0f;
        y_unsat_1 = 0.0f;  y_unsat_2 = 0.0f;
        y_sat_1   = 0.0f;  y_sat_2   = 0.0f;
    }

    float step(float w, float x) {
        const float e_sat_1 = add_rn(y_sat_1,   -y_unsat_1);
        const float e_sat_2 = add_rn(y_sat_2,   -y_unsat_2);

        float acc = 0.0f;
        acc = add_rn(acc, mul_rn(C0,  dy1));
        acc = add_rn(acc, mul_rn(C1,  w));
        acc = add_rn(acc, mul_rn(C2,  w1));
        acc = add_rn(acc, mul_rn(C3,  w2));
        acc = add_rn(acc, mul_rn(C4,  x));
        acc = add_rn(acc, mul_rn(C5,  x1));
        acc = add_rn(acc, mul_rn(C6,  x2));
        acc = add_rn(acc, mul_rn(C7A, e_sat_1));
        acc = add_rn(acc, mul_rn(C7B, e_sat_2));
        const float dy = acc;

        const float y_unsat = add_rn(y_unsat_1, dy);
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
// Encoder: floor + carry
// ============================================================
struct EncoderFloor {
    float Ts;
    float rad_per_cnt;
    float int2radfac;
    float theta_rad;
    long  C_prev;

    EncoderFloor(float Ts_)
        : Ts(Ts_), theta_rad(0.0f), C_prev(0)
    {
        int2radfac  = INT2RADS;
        rad_per_cnt = mul_rn(int2radfac, Ts);
    }

    void sample(float x_true, int& spdcnt, float& x_meas) {
        theta_rad = std::fmaf(x_true, Ts, theta_rad);

        const float C_real = theta_rad / rad_per_cnt;
        const long  C_now  = (long)std::floor(C_real);

        spdcnt = (int)(C_now - C_prev);
        C_prev = C_now;

        x_meas = mul_rn((float)spdcnt, int2radfac);
    }
};

// ============================================================
// 아주 단순한 txt 로더 (공백/줄바꿈 구분 float 전부 읽기)
// ============================================================
static std::vector<float> read_y_txt(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "파일을 열 수 없습니다: " << path << "\n";
        return {};
    }
    std::vector<float> v;
    float x;
    while (ifs >> x) v.push_back(x);
    return v;
}

int main() {
    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(6);

    const float Ts = 0.005f;

    DeltaPid2TapAw ctrl(YSAT);

    const float Ku  = 50.0f;
    const float lam = 5.0f;

    const float w_true = W_TGT;
    float x_true = 0.0f;

    EncoderFloor enc(Ts);

    std::cout << "# INT_TO_RADS_FACTOR = " << std::setprecision(9) << enc.int2radfac << "\n";
    std::cout << std::setprecision(6);
    std::cout << "   t[s] |   w(Tgt) |  x_true | x_meas | spdcnt |    y[V] | Duty[%]\n";

    const int STEPS = 100;

    // ===== y 저장 (비교용) =====
    std::vector<float> y_sim;
    y_sim.reserve(STEPS );

    for (int n = 0; n <= STEPS; ++n) {
        const float t = (float)n * Ts;

        int spdcnt = 0;
        float x_meas = 0.0f;
        enc.sample(x_true, spdcnt, x_meas);

        const float y = ctrl.step(w_true, x_meas);
        y_sim.push_back(y);

        const float duty = mul_rn(mul_rn(std::fabs(y), RECIP_YSAT), 100.0f);

        std::cout << std::setw(8) << t      << " | "
                  << std::setw(9) << w_true << " | "
                  << std::setw(7) << x_true << " | "
                  << std::setw(7) << x_meas << " | "
                  << std::setw(7) << spdcnt << " | "
                  << std::setw(8) << std::setprecision(9) << y << " | "
                  << std::setw(7) << std::setprecision(6) << duty << "\n";

        x_true = add_rn(x_true, mul_rn(Ts, add_rn(mul_rn(Ku, y), -mul_rn(lam, x_true))));
    }

    // ============================================================
    // txt와 단순 비교 (1e-3 허용오차)
    // ============================================================
    const std::string txt_path = "y_values_step1_to_100.txt";
    const float tol = 1e-3f;

    std::vector<float> y_ref = read_y_txt(txt_path);
    if (y_ref.empty()) {
        std::cerr << "참조 y 값이 비어있습니다. txt 내용을 확인하세요.\n";
        return 1;
    }

    const int N = (int)std::min(y_ref.size(), y_sim.size());

    int pass = 0, fail = 0;
    float max_err = 0.0f;
    int max_i = -1;

    std::cout << "\n=== Compare (abs tol = " << tol << ") ===\n";
    std::cout << "ref_count=" << y_ref.size() << ", sim_count=" << y_sim.size()
              << ", compare_count=" << N << "\n";

    for (int i = 0; i < N; ++i) {
        float err = std::fabs(y_sim[i] - y_ref[i]);
        if (err <= tol) {
            pass++;
        } else {
            fail++;
            if (err > max_err) { max_err = err; max_i = i; }

            // FAIL 몇 개만 출력
            if (fail <= 10) {
                std::cout << "FAIL i=" << i
                          << " sim=" << std::setprecision(10) << y_sim[i]
                          << " ref=" << std::setprecision(10) << y_ref[i]
                          << " |err|=" << std::setprecision(10) << err
                          << std::setprecision(6) << "\n";
            }
        }
    }

    std::cout << "\nSummary: PASS=" << pass << " FAIL=" << fail
              << " max_err=" << std::setprecision(10) << max_err
              << " at i=" << max_i << std::setprecision(6) << "\n";

    if (fail == 0) std::cout << "==> ALL PASS (|err| <= 1e-3)\n";
    else           std::cout << "==> FAIL EXISTS\n";

    return 0;
}
