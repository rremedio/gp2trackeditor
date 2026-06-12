#pragma once
#include <array>
#include <cstdint>
#include "gp2cos_reference.hpp"   // golden oracle: GP2's extracted t_Sinus bytes

// ---------------------------------------------------------------------------
// GP2 geometry — the cosine table expressed as its generating formula.
//
// GP2 represents angles in "binary radians": 65536 units per full turn, so a
// quarter turn is 0x4000. The engine keeps a cosine table of amplitude 0x4000
// (16384), sampled every 8 units, and for the position step it does a RAW,
// non-interpolated lookup COS[angle >> 3]. That lookup is therefore the cosine
// of the angle quantised to the 8-unit grid:
//
//       COS[i] = round( 16384 * cos( i * pi / 4096 ) )          (i = angle>>3)
//
// ...with exactly ONE exception. At i = 1992 the exact value is 703.5004 —
// dead on a .5 rounding boundary — and GP2's own table generator resolved it
// to 703 (round() gives 704). That single 1-unit entry is load-bearing: the
// racing-line solver is a chaotic feedback system, and a track whose heading
// passes through this angle (e.g. F1CT09) drops from 100% to 29% correct
// without it. So we override that one entry and PROVE the whole table equals
// GP2's bytes at compile time (see the static_assert at the bottom).
//
// Why a formula instead of an embedded blob: the formula documents intent
// (this IS a cosine), and computing it at build time keeps the runtime fully
// deterministic (no per-call float). The float boundary entries aren't
// portable, so we don't trust them blindly — the static_assert is the
// guardrail: if any toolchain rounds a boundary entry differently, the BUILD
// fails loudly instead of the racing line drifting silently.
// ---------------------------------------------------------------------------

namespace gp2geom {

inline constexpr int   kGrid       = 4098;    // indices 0..4097  (cos over [0, pi])
inline constexpr short kAmplitude  = 16384;   // 0x4000
inline constexpr int   kQuirkIndex = 1992;    // the lone .5-boundary entry GP2 rounds down
inline constexpr short kQuirkValue = 703;
inline constexpr double kPi = 3.14159265358979323846;

// constexpr cosine (std::cos isn't portably constexpr before C++26): fold the
// argument into [0, pi/2] via cos(x) = -cos(pi - x), then a Taylor series.
// Accurate to ~1e-15 over [0, pi] — far inside the rounding margin (the closest
// non-quirk entry sits >1e-4 from a .5 boundary).
constexpr double cosine(double x) {
    bool negate = false;
    if (x > kPi * 0.5) { x = kPi - x; negate = true; }
    double term = 1.0, sum = 1.0, x2 = x * x;
    for (int n = 1; n < 16; ++n) {
        term *= -x2 / static_cast<double>((2 * n - 1) * (2 * n));
        sum  += term;
    }
    return negate ? -sum : sum;
}

// round half away from zero, matching the game's table.
constexpr short roundToShort(double v) {
    long r = (v >= 0.0) ? static_cast<long>(v + 0.5) : static_cast<long>(v - 0.5);
    return static_cast<short>(r);
}

// Build the table at compile time from the formula + the one documented quirk.
constexpr std::array<short, kGrid> makeCosineTable() {
    std::array<short, kGrid> t{};
    for (int i = 0; i < kGrid; ++i) {
        t[i] = (i == kQuirkIndex)
                 ? kQuirkValue
                 : roundToShort(kAmplitude * cosine(i * (kPi / 4096.0)));
    }
    return t;
}

inline constexpr std::array<short, kGrid> kCosine = makeCosineTable();

// Raw (non-interpolated) cosine of an angle in binary-radian units: cos is
// even, so fold the sign, then index the 8-unit grid. This is the readable
// equivalent of GP2's COS[((a>>2)&0xFFFE)>>1].
constexpr short scos(int angle) {
    int a = static_cast<std::int16_t>(angle & 0xFFFF);   // wrap to signed 16-bit
    if (a < 0) a = -a;
    return kCosine[a >> 3];
}

// ---- compile-time guardrails -------------------------------------------------
constexpr bool matchesReference() {
    for (int i = 0; i < kGrid; ++i)
        if (kCosine[i] != kCosRef[i]) return false;
    return true;
}
static_assert(matchesReference(),
    "gp2geom cosine table != GP2 reference bytes: a float rounding boundary "
    "flipped on this toolchain. Inspect the diff and add a documented override "
    "before shipping — do NOT relax this assert.");

static_assert(kCosine[0]    ==  16384, "cos(0)");
static_assert(kCosine[2048] ==      0, "cos(pi/2)");
static_assert(kCosine[4096] == -16384, "cos(pi)");
static_assert(kCosine[1992] ==    703, "GP2 .5-boundary quirk");

} // namespace gp2geom
