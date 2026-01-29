/********************  motor_control_driver.c  ********************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "xil_io.h"
#include "xparameters.h"
#include "sleep.h"
#include <float.h>

/* === 하드웨어 베이스 주소 (플랫폼에 맞게 수정) === */
#ifndef MOTOR_CTRL_BASE
#define MOTOR_CTRL_BASE XPAR_PID_CONTROLLER_INTER_0_BASEADDR
#endif

/* === 레지스터 오프셋 (32-bit) === */
#define REG_A0          0x00  // a0 (=c0)
#define REG_C1          0x04  // c1
#define REG_C2          0x08  // c2
#define REG_C3          0x0C  // c3
#define REG_C4          0x10  // c4
#define REG_C5          0x14  // c5
#define REG_C6          0x18  // c6
#define REG_C7          0x1C  // c7a (AW tap1)
#define REG_C8          0x20  // c7b (AW tap2)
#define REG_YSAT        0x24  // voltage saturation (e.g., 12.0 V)
#define REG_RECIP_YSAT  0x28  // 1/YSAT
#define REG_W_TARGET    0x2C  // target speed [rad/s]
#define REG_STATUS13    0x30  // measured spdcnt/status (RO) — 하위 16비트 spdcnt

/* === 인코더/게이트 설정: 보드와 동일하게 맞추세요 === */
#define CPR_QUAD   1336          /* 쿼드(4x) 기준 rev당 카운트 */
#define GATE_HZ    200            /* 게이트 빈도(예: 200Hz → 5ms) */

/* === 파생 상수 === */
#define TWO_PI     (6.28318530717958647692)
#define Ts_sec     (1.0/(double)(GATE_HZ))
#define GATE_US    ((unsigned)(Ts_sec*1e6))  /* 게이트 시간(마이크로초) */

/* spdcnt → 물리량 환산 (게이트/해상도에 종속, 자동 계산) */
#define SPDC_TO_RADPS_FACTOR  ((float)(TWO_PI * (double)GATE_HZ / (double)CPR_QUAD))  /* rad/s per count */
#define SPDC_TO_RPM_FACTOR    ((float)(60.0       * (double)GATE_HZ / (double)CPR_QUAD)) /* RPM per count */

/* 출력 포화값 */
#define YSAT_VOLT     (12.0f)
#define RE_YSAT_VOLT  (1.0f/12.0f)

/* 유틸 */
static void setup_stdio_unbuffered(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    setvbuf(stdin,  NULL, _IONBF, 0);
}

/* 한 줄 입력 + 에코 + 백스페이스 */
static int read_line_echo(char *buf, int maxlen) {
    int n = 0;
    for (;;) {
        int ch = getchar();
        if (ch == '\r' || ch == '\n') {
            putchar('\r'); putchar('\n');
            buf[n] = '\0'; return n;
        } else if (ch == 8 || ch == 127) { /* BS/DEL */
            if (n > 0) { n--; putchar('\b'); putchar(' '); putchar('\b'); }
        } else if (isprint(ch)) {
            if (n < maxlen - 1) { buf[n++] = (char)ch; putchar(ch); }
        }
    }
}

static double ask_double(const char *prompt) {
    char line[128]; double v; char *endp;
    for (;;) {
        printf("%s", prompt);
        if (read_line_echo(line, sizeof(line)) <= 0) continue;
        v = strtod(line, &endp);
        if (endp != line) return v;
        printf("  (숫자를 다시 입력하세요)\r\n");
    }
}

/* float ↔ u32 비트재해석 */
static inline uint32_t f2u(float x){ union{float f; uint32_t u;}v; v.f=x; return v.u; }
static inline float    u2f(uint32_t u){ union{uint32_t u; float f;}v; v.u=u; return v.f; }

/* 연속계 → 시간상수/필터 파라미터 변환 */
static void compute_time_constants(double Kp, double Ki, double Kd, double N,
                                   double *Ti, double *Td, double *a)
{
    *Ti = (Ki > 0.0 && Kp > 0.0) ? (Kp / Ki) : 1e30; /* Ki=0 → effectively ∞ */
    *Td = (Kp > 0.0) ? (Kd / Kp) : 0.0;
    *a  = (N  > 0.0) ? (1.0 / N) : 0.0;              /* ★ a = 1/N */
}

/* Δ-형(증분형) 2-DOF PID + D-필터(a=1/N) + 2-tap AW(c7a,c7b) 계수 계산 */
static void compute_coeffs(double Kp, double Ki, double Kd,
                           double N, double b, double c, double Kb,
                           /* out */
                           float *a0, float *c1, float *c2, float *c3,
                           float *c4, float *c5, float *c6,
                           float *c7a, float *c7b)
{
    const double Ts = Ts_sec;
    double Ti, Td, a;
    compute_time_constants(Kp, Ki, Kd, N, &Ti, &Td, &a);

    const double den        = Ts + a*Td;                 /* Ts + a*Td */
    const double Ts_over_Ti = (Ti < 1e20) ? (Ts/Ti) : 0; /* Ki=0이면 0 */

    /* c0 (=a0): D-필터 피드백 계수 */
    const double C0 = (den > 0.0) ? ((a*Td)/den) : 0.0;

    /* 본문 계수 (골든과 동일식) */
    const double C1 =  Kp * ( b + Ts_over_Ti + (Td*c)/den );

    const double C2 = -Kp * (  b*(Ts + 2.0*a*Td)
                             + (a*Td*Ts_over_Ti)
                             + (2.0*Td*c) ) / den;

    const double C3 =  (Kp*Td*(a*b + c)) / den;

    const double C4 = -Kp * ( 1.0 + Ts_over_Ti + (Td/den) );

    const double C5 =  Kp * (  Ts + 2.0*a*Td
                             + (a*Td*Ts_over_Ti)
                             + (2.0*Td) ) / den;

    const double C6 = -Kp * ( Td*(a + 1.0) ) / den;

    /* 2-tap AW: Δy += c7a*e_sat[n-1] + c7b*e_sat[n-2]
       e_sat = y_sat - y_unsat
       c7a = Kb*Ts,  c7b = -c7a * c0    (★ 하드웨어/골든과 동일) */
    const double C7A = Ki * Kb * Ts;
    const double C7B = -C7A * C0;

    *a0  = (float)C0;
    *c1  = (float)C1;  *c2  = (float)C2;  *c3  = (float)C3;
    *c4  = (float)C4;  *c5  = (float)C5;  *c6  = (float)C6;
    *c7a = (float)C7A; *c7b = (float)C7B;
}

/* RPM ↔ rad/s */
static inline float rpm_to_radps(float rpm){ return rpm * (float)(TWO_PI/60.0); }

static inline void wr_f32(uint32_t off, float v){ Xil_Out32(MOTOR_CTRL_BASE + off, f2u(v)); }

int main(void)
{
    double Kp, Ki, Kd, N, b, c, Kb;
    double rpm_target;

    setup_stdio_unbuffered();

    printf("=== Motor Control Driver (Vitis) ===\n");
    printf("게이트=%.3f ms, CPR(quad)=%d → spdcnt: %.6f rad/s, %.6f RPM per count\r\n",
           (double)Ts_sec*1e3, CPR_QUAD,
           (double)SPDC_TO_RADPS_FACTOR, (double)SPDC_TO_RPM_FACTOR);

    /* 파라미터 입력 */
    Kp  = ask_double("Kp: ");
    Ki  = ask_double("Ki: ");
    Kd  = ask_double("Kd: ");
    N   = ask_double("N (D-filter, a=1/N): ");
    b   = ask_double("b (P setpoint weight): ");
    c   = ask_double("c (D setpoint weight): ");
    Kb  = ask_double("Kb (anti-windup 1/s): ");
    rpm_target = ask_double("Target RPM: ");

    /* 계수 계산 */
    float a0, c1, c2, c3, c4, c5, c6, c7a, c7b;
    compute_coeffs(Kp, Ki, Kd, N, b, c, Kb,
                   &a0,&c1,&c2,&c3,&c4,&c5,&c6,&c7a,&c7b);

    const float w_target = rpm_to_radps((float)rpm_target);

    printf("\r\n--- Coeffs to write (Δ-form + 2-tap AW) ---\r\n");
    printf("a0=%g\r\nc1=%g\r\nc2=%g\r\nc3=%g\r\nc4=%g\r\nc5=%g\r\nc6=%g\r\nc7a=%g\r\nc7b=%g\r\n",
           a0,c1,c2,c3,c4,c5,c6,c7a,c7b);
    printf("W_target(rad/s)=%.6f  (from %.3f RPM)\r\n", w_target, rpm_target);
    printf("YSAT=%.3f  1/YSAT=%.6f\r\n", (float)YSAT_VOLT, RE_YSAT_VOLT);

    usleep(100000);

    /* 하드웨어로 전송 */
    wr_f32(REG_A0,  a0);
    wr_f32(REG_C1,  c1);
    wr_f32(REG_C2,  c2);
    wr_f32(REG_C3,  c3);
    wr_f32(REG_C4,  c4);
    wr_f32(REG_C5,  c5);
    wr_f32(REG_C6,  c6);
    wr_f32(REG_C7,  c7a);   /* ★ tap1 */
    wr_f32(REG_C8,  c7b);   /* ★ tap2 */

    wr_f32(REG_YSAT,       YSAT_VOLT);
    wr_f32(REG_RECIP_YSAT, RE_YSAT_VOLT);
    wr_f32(REG_W_TARGET,   w_target);

    uint32_t raw_T = Xil_In32(MOTOR_CTRL_BASE + REG_W_TARGET);
    printf("W_target readback: %f\r\n", u2f(raw_T));

    

    /* 실시간 모니터링 (spdcnt → RPM) */
    for (int i=0;i<15000;i++){
        uint32_t raw = Xil_In32(MOTOR_CTRL_BASE + REG_STATUS13);
        int16_t  sp  = (int16_t)(raw & 0xFFFF);
        double   rpm = (double)sp * SPDC_TO_RPM_FACTOR;
        printf("reg13=0x%08lX  spdcnt=%d  RPM=%.2f\r\n",
               (unsigned long)raw, (int)sp, rpm);
    }
    /* return 0; */
}
