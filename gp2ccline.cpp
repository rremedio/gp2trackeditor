#include "gp2ccline.hpp"
#include "gp2cos.hpp"    // gp2geom::kCosine — the verified cosine table (COS)
#include "gp2atan.hpp"   // gp2geom::kArctan — the verified atan table (ATAN1)
#include <cstring>

namespace gp2geom {
namespace {

// Table aliases so the transcription below reads like the original kernel.
inline int COS (int i)  { return kCosine[i]; }   // amplitude 0x4000 cosine table
inline int ATAN(int i)  { return kArctan[i]; }    // atan table, 0x10000/turn

#define ONE60  (((int64_t)1) << 60)
constexpr int CBB0C = 20861;          // data const @0xCBB0C: segment-frame theta' tilt

inline int s16(int v)   { v &= 0xFFFF; return (v & 0x8000) ? v - 0x10000 : v; }
inline int abs16(int v) { v = s16(v); return v < 0 ? -v : v; }
inline int rdw(const std::uint8_t* d, int o) { return d[o] | (d[o + 1] << 8); }

// GetSinusVal (0x104B9): interpolated cosine, amplitude 0x4000.
inline int gv(int ax) {
    ax = s16(ax);
    if (ax < 0) ax = (-ax) & 0xFFFF;
    int frac = ax & 7;
    int widx = ((ax >> 2) & 0xFFFE) >> 1;
    int base = COS(widx), nxt = COS(widx + 1);
    int d = s16(nxt - base);
    return s16(base + ((d * frac) >> 3));
}

// sub_10240: 32-bit integer sqrt, FIXED 3-iteration 16-bit Newton (not floor).
std::uint32_t sqrt32(std::uint32_t V) {
    if (V == 0) return 0;
    int shift = 0;
    while (V < 0x40000000u) { shift++; V <<= 2; if (V == 0) return 0; }
    std::uint32_t half = V >> 1;
    std::uint32_t di = ((V >> 17) + 0x8000) & 0xFFFF;
    for (int it = 0; it < 3; it++) {
        std::uint32_t hi = half >> 16, q;
        if (hi >= di) q = half & 0xFFFF;
        else          q = half / di;
        di = ((di >> 1) + q) & 0xFFFF;
    }
    return (di >> shift) & 0xFFFF;
}

// sub_102F0: 64-bit integer sqrt, FIXED 5-iteration Newton.
std::uint32_t sqrt64(std::uint64_t V) {
    if (V == 0) return 0;
    if ((std::uint32_t)(V >> 32) == 0) return sqrt32((std::uint32_t)V);
    int shift = 0;
    while ((V >> 32) < 0x40000000ull) { shift++; V <<= 2; }
    std::uint64_t half = V >> 1;
    std::uint32_t est = (std::uint32_t)((half >> 32) + 0x80000000ull);
    for (int it = 0; it < 5; it++) {
        std::uint32_t q = (std::uint32_t)(half / est);
        est = (est >> 1) + q;
    }
    return est >> shift;
}

// sub_10502: cos(|angle|) in Q30 (amp 0x40000000) via the cos table + interp.
std::int32_t cos30(int angle) {
    int a = abs16(angle);
    int widx = a >> 3, frac = a & 7;
    int base  = COS(widx);
    int delta = COS(widx + 1) - COS(widx);
    return ((std::int32_t)base << 16) + ((std::int32_t)(delta * frac) << 13);
}

// sub_7827D: Q30 cos/sin; larger component via sqrt(ONE60-other^2), smaller via table.
void q30_cossin(int angle, std::int32_t* pcos, std::int32_t* psin) {
    int a = s16(angle);
    int coarse_sin = COS(abs16(s16(0x4000 - a)) >> 3);
    int coarse_sin_abs = coarse_sin < 0 ? -coarse_sin : coarse_sin;
    std::int32_t c30, s30;
    if (coarse_sin_abs >= 0x2000) {
        c30 = cos30(a);
        std::int64_t smag = (std::int64_t)sqrt64((std::uint64_t)(ONE60 - (std::int64_t)c30 * c30));
        s30 = coarse_sin < 0 ? (std::int32_t)(-smag) : (std::int32_t)smag;
    } else {
        s30 = cos30(s16(0x4000 - a));
        std::int64_t cmag = (std::int64_t)sqrt64((std::uint64_t)(ONE60 - (std::int64_t)s30 * s30));
        int coarse_cos = COS(abs16(a) >> 3);
        c30 = coarse_cos < 0 ? (std::int32_t)(-cmag) : (std::int32_t)cmag;
    }
    *pcos = c30; *psin = s30;
}

// sub_10798: atan2(y,x) in 0x10000 units (full circle), 16-bit args.
int atan2u(int y, int x) {
    y = s16(y); x = s16(x);
    int dy = y < 0 ? -y : y;
    int dx = x < 0 ? -x : x;
    int signs_differ = (y < 0) != (x < 0);
    int a;
    if (dy >= dx) {
        if (dy == 0) return 0;
        int idx = (dx << 11) / dy;
        a = 0x4000 - ATAN(idx);
    } else {
        int idx = (dy << 11) / dx;
        a = ATAN(idx);
    }
    if (signs_differ) a = -a;
    if (x < 0) a += 0x8000;
    return s16(a);
}

// sub_78492: build the world arc-centre W for the first segment of a command.
void build_centre(const gp2cc_ccgeo* g, int idx, int P, int H,
                  std::int32_t arg2, int arg1mul4, std::int32_t* pWx, std::int32_t* pWy) {
    int seg = idx;
    int f14 = (int)g->f14[seg];
    int latP = (int)(((std::int32_t)P * f14) >> 15);
    int ang  = s16(g->segAngle[seg]);
    int cosA = gv(ang);
    int sinA = gv(s16(0x4000 - ang));
    std::int32_t RX = (std::int16_t)(((std::int32_t)P * cosA + (std::int32_t)latP * sinA) >> 14);
    std::int32_t RY = (std::int16_t)(((std::int32_t)latP * cosA - (std::int32_t)P * sinA) >> 14);
    RX += g->cx8[seg];
    RY += g->cy8[seg];
    int sinH = gv(s16(0x4000 - H));
    int cosH = gv(s16(H));
    std::int16_t a1m4 = (std::int16_t)arg1mul4;
    RX += (std::int16_t)(((std::int32_t)sinH * a1m4) >> 14);
    RY += (std::int16_t)(((std::int32_t)cosH * a1m4) >> 14);
    std::int32_t Wx = RX, Wy = RY;
    if (arg2 != 0) {
        std::int32_t mag = arg2 < 0 ? -arg2 : arg2;
        int perp = (arg2 < 0) ? s16(H - 0x4000) : s16(H + 0x4000);
        std::int32_t c32, s32; q30_cossin(perp, &c32, &s32);
        Wx += (std::int32_t)(((std::int64_t)s32 * mag) >> 30);
        Wy += (std::int32_t)(((std::int64_t)c32 * mag) >> 30);
    }
    *pWx = Wx; *pWy = Wy;
}

// sub_787E7: reproject world point W onto seg idx -> P (and H on curves).
void reproject(const gp2cc_ccgeo* g, int idx, std::int32_t Wx, std::int32_t Wy,
               std::int32_t arg2, int* pP, int* pH) {
    int seg = idx;
    int f14 = (int)g->f14[seg];
    int ang = s16(g->segAngle[seg]);
    int thetaP = s16(ang - (int)((((std::int32_t)(f14 >> 1) * CBB0C) << 1) >> 16));
    std::int32_t relx = Wx - g->cx8[seg];
    std::int32_t rely = Wy - g->cy8[seg];
    std::int32_t c32, s32; q30_cossin(thetaP, &c32, &s32);
    std::int64_t lat64 = (std::int64_t)c32 * relx - (std::int64_t)s32 * rely;
    std::int64_t lon64 = (std::int64_t)s32 * relx + (std::int64_t)c32 * rely;
    std::int32_t lat = (std::int32_t)(lat64 >> 30);
    std::int32_t lon = (std::int32_t)(lon64 >> 30);
    if (arg2 == 0) {
        int d = s16(*pH - thetaP);
        std::int32_t sn = cos30(s16(0x4000 - d));
        std::int32_t cs = cos30(d);
        std::int32_t tanterm = 0;
        if (cs != 0) tanterm = (std::int32_t)(((std::int64_t)sn * lon) / cs);
        *pP = s16(lat - tanterm);
        return;
    }
    std::int64_t a2 = arg2;
    std::int64_t disc = a2 * a2 - (std::int64_t)lon * lon;
    std::int64_t T = (std::int64_t)sqrt64((std::uint64_t)disc);
    std::int64_t Tsigned = (arg2 < 0) ? -T : T;
    *pP = s16((std::int32_t)(lat - (std::int32_t)Tsigned));
    std::int32_t Ts = (std::int32_t)Tsigned, lonv = lon;
    std::uint32_t aa = (std::uint32_t)(arg2 < 0 ? -arg2 : arg2);
    while (((aa >> 16) > 0) || ((aa >> 16) == 0 && (aa & 0xFFFF) >= 0x7F00)) {
        Ts >>= 1; lonv >>= 1; aa >>= 1;
    }
    int at = atan2u((int)(std::int16_t)(Ts & 0xFFFF), (int)(std::int16_t)(lonv & 0xFFFF));
    at = (arg2 < 0) ? s16(at + 0x4000) : s16(at - 0x4000);
    *pH = s16(at + thetaP);
}

// sub_78CEC: per-seg curvature-ramp nudge of W (slope!=0 commands).
void nudge(std::int32_t* pWx, std::int32_t* pWy, int H, std::int32_t arg2, std::int32_t slope) {
    std::int32_t ebp = slope;
    int a;
    if (arg2 < 0) { ebp = -ebp; a = s16(H - 0x4000); }
    else            a = s16(H + 0x4000);
    int sinA = gv(s16(0x4000 - a));
    int cosA = gv(s16(a));
    *pWx += (std::int32_t)(((std::int64_t)sinA * ebp) >> 14);
    *pWy += (std::int32_t)(((std::int64_t)cosA * ebp) >> 14);
}

// GetSingleCmdArgs (0x78D4C): parse one cc command.
struct cc_cmd {
    int end, word, N, a1, a2;
    std::int32_t arg2, slope;
    int arg1mul4, arg1if2is0;
};
void parse_cmd(const std::uint8_t* cc, int* po, cc_cmd* c) {
    int o = *po;
    std::memset(c, 0, sizeof *c);
    int word = rdw(cc, o); o += 2;
    c->word = word;
    c->N = word & 0x7FF;
    if (word == 0) { c->end = 1; *po = o; return; }
    if (word & 0x8000) { c->a1 = rdw(cc, o); o += 2; c->a2 = rdw(cc, o); o += 2; }
    int cx = rdw(cc, o); o += 2;
    std::int32_t arg2 = (std::int32_t)(std::int16_t)rdw(cc, o); o += 2;
    if (word & 0x4000) { arg2 = (std::int32_t)((std::uint32_t)arg2 << 16) | rdw(cc, o); o += 2; }
    if (!(word & 0x1000)) arg2 <<= 3;
    c->arg2 = arg2;
    if (word & 0x2000) {
        std::int32_t arg3 = (std::int32_t)(std::int16_t)rdw(cc, o); o += 2;
        if (word & 0x4000) { arg3 = (std::int32_t)((std::uint32_t)arg3 << 16) | rdw(cc, o); o += 2; }
        if (!(word & 0x1000)) arg3 <<= 3;
        c->slope = (arg3 - arg2) / c->N;
    }
    if (arg2 != 0) { c->arg1if2is0 = 0; c->arg1mul4 = (std::int16_t)(cx << 2); }
    else           { c->arg1mul4 = 0;   c->arg1if2is0 = cx; }
    *po = o;
}

} // anonymous namespace

// UACalcBestLine (0x78EB2): walk the command stream; seed P/H; per segment store
// bestLine/angle18 then reproject the carried world point; nudge on ramps.
int computeCcLine(const gp2cc_ccgeo& gref, const std::uint8_t* cc, int cc_len, int cc_off,
                  std::int16_t* bestLine, std::int16_t* angle18,
                  int* cmdStartSeg, int* pNumCmds) {
    const gp2cc_ccgeo* g = &gref;
    int n = g->n;
    if (n <= 0) return -1;
    int cur = 0, cmdIdx = 0;
    int H = s16(g->segAngle[0]);
    int P = 0;
    int w0 = rdw(cc, cc_off);
    // seed P (segPosX_0A) is a SIGNED 16-bit word; sign-extend it.
    if (!(w0 & 0x800) && (w0 & 0x8000)) P = s16(rdw(cc, cc_off + 2));
    int o = cc_off;
    for (;;) {
        cc_cmd c;
        parse_cmd(cc, &o, &c);
        if (c.end) break;
        if (o > cc_len) return -2;
        if ((c.word & 0x800) && (c.word & 0x8000)) { P = s16(c.a1); H = s16(c.a2); }
        if (c.arg2 == 0) H = s16(H + s16(c.arg1if2is0));
        if (cmdStartSeg) cmdStartSeg[cmdIdx] = cur;
        cmdIdx++;
        std::int32_t Wx, Wy;
        build_centre(g, cur, P, H, c.arg2, c.arg1mul4, &Wx, &Wy);
        std::int32_t arg2 = c.arg2;
        int left = c.N;
        while (left > 0) {
            angle18[cur]  = (std::int16_t)s16(H - s16(g->segAngle[cur]));
            bestLine[cur] = (std::int16_t)s16(P);
            cur = (cur + 1) % n;
            reproject(g, cur, Wx, Wy, arg2, &P, &H);
            left--;
            if (left == 0) break;
            if (arg2 != 0 && c.slope != 0) {
                arg2 += c.slope;
                nudge(&Wx, &Wy, H, arg2, c.slope);
            }
        }
    }
    if (pNumCmds) *pNumCmds = cmdIdx;
    return 0;
}

} // namespace gp2geom
