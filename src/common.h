#pragma once

#ifdef PI
#undef PI
#endif

// Dasar dengan presisi tinggi (literal langsung, aman constexpr)
constexpr float PI       = 3.14159265358979323846f;  // π
constexpr float TAU      = 6.28318530717958647692f;  // 2*π

constexpr float SQRT_2   = 1.41421356237309504880f;  // √2
constexpr float SQRT_3   = 1.73205080756887729352f;  // √3

// Kalau kamu butuh FRAC_2_SQRT_PI (2/√π) → hardcode aja
constexpr float FRAC_2_SQRT_PI = 1.12837916709551257390f;  // ≈ 2 / sqrt(π)

// Fraksi π / τ (semua dari literal atau hitung sederhana)
constexpr float FRAC_PI_2   = PI / 2.0f;
constexpr float FRAC_PI_3   = PI / 3.0f;
constexpr float FRAC_PI_4   = PI / 4.0f;
constexpr float FRAC_PI_6   = PI / 6.0f;
constexpr float FRAC_PI_8   = PI / 8.0f;

// Tau fraksi
constexpr float FRAC_TAU_2  = TAU / 2.0f;   // = PI
constexpr float FRAC_TAU_3  = TAU / 3.0f;
constexpr float FRAC_TAU_4  = TAU / 4.0f;   // = FRAC_PI_2
constexpr float FRAC_TAU_6  = TAU / 6.0f;
constexpr float FRAC_TAU_8  = TAU / 8.0f;
constexpr float FRAC_TAU_12 = TAU / 12.0f;

// Konversi derajat-radian (FIX utama!)
constexpr float DEG_TO_RAD  = 0.0174532925199432957692f;   // π / 180
constexpr float RAD_TO_DEG  = 57.2957795130823208768f;     // 180 / π

// Alias umum
constexpr float HALF_PI     = FRAC_PI_2;
constexpr float PI_6        = FRAC_PI_6;
constexpr float TWO_PI      = TAU;
constexpr float TWO_PI_OVER_256 = TAU / 256.0f;

// Lainnya dari kode asli kamu (sudah pakai literal presisi)
constexpr float E        = 2.71828182845904523536f;
constexpr float E_CONST  = 0.57721566490153286060f;  // Euler-Mascheroni
constexpr float FRAC_1_TAU = 1.0f / TAU;
constexpr float FRAC_1_PI  = 1.0f / PI;
constexpr float FRAC_2_TAU = 2.0f / TAU;   // = FRAC_1_PI
constexpr float FRAC_2_PI  = 2.0f / PI;
constexpr float FRAC_1_SQRT_2 = 1.0f / SQRT_2;
constexpr float FRAC_4_TAU = 4.0f / TAU;   // = FRAC_2_PI

constexpr float LN_2     = 0.693147180559945309417f;
constexpr float LN_10    = 2.30258509299404568402f;
constexpr float LOG2_E   = 1.44269504088896340736f;
constexpr float LOG10_E  = 0.43429448190325182765f;
constexpr float LOG10_2  = 0.30102999566398119521f;
constexpr float LOG2_10  = 3.32192809488736234787f;

constexpr float COS_45   = SQRT_2 / 2.0f;

// Helper
template<typename T>
__forceinline constexpr T sq(T x) { return x * x; }

constexpr float DegreesToRadians(float angleInDegrees) {
    return angleInDegrees * DEG_TO_RAD;
}

constexpr float RadiansToDegrees(float angleInRadians) {
    return angleInRadians * RAD_TO_DEG;
}