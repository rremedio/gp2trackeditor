/* gp2cc — geometry compiler (see gp2cc.h). Ported verbatim from datparse.py,
 * which is bit-validated against the GP2Lap dumps. */
#include "gp2cc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The game's trig tables are EMBEDDED (gp2cc_tables.c, generated from GP2.EXE) so
 * gp2cc needs no GP2.EXE at runtime — important for embedding in the editor.
 *   COS   = t_Sinus     (amp 0x4000, one entry/8 angle units; gv interp + raw)
 *   ATAN1 = t_ArithTab1  (atan, indexed by the curved-branch atan2 / finer cos) */
extern const short gp2cc_COS[];
extern const short gp2cc_ATAN1[];
static const short *const COS   = gp2cc_COS;
static const short *const ATAN1 = gp2cc_ATAN1;

/* Tables are embedded; nothing to load. Kept for API compatibility (the exe path
 * is ignored). Always succeeds. */
int gp2cc_load_tables(const char *gp2exe_path) {
    (void)gp2exe_path;
    return 0;
}

static inline int s16(int v) { v &= 0xFFFF; return (v & 0x8000) ? v - 0x10000 : v; }

/* GetSinusVal (0x104B9): cos, interpolated. */
int gp2cc_gv(int ax) {
    ax = s16(ax);
    if (ax < 0) ax = (-ax) & 0xFFFF;
    int frac = ax & 7;
    int widx = ((ax >> 2) & 0xFFFE) >> 1;
    int base = COS[widx], nxt = COS[widx + 1];
    int d = s16(nxt - base);
    return s16(base + ((d * frac) >> 3));
}
/* raw (non-interpolated) cosine for the position step (InitTrackSegs 0x79A9F). */
static int scos(int a) {
    a = s16(a);
    if (a < 0) a = -a;
    return COS[((a >> 2) & 0xFFFE) >> 1];
}

/* ---- track-command arg sizes (datparse TRK_ARGSZ). opcode 0x45 is conditional
 * on w_C5Code bit9 (handled inline). -1 = unknown (desync). */
static const int TRK_ARGSZ[0x66] = {
    2,2,2,0,0,4,0,0,2,2, 10,10,2,2,4,4,2,2,2,2, 2,2,0,0,2,2,4,0,0,0,
    0,0,0,0,0,0,0,0,4,4, 0,2,6,4,8,4,2,4,2,2, 2,2,4,4,2,2,26,2,4,8,
    14,4,4,4,2,2,4,4,6,14, 2,4,14,16,8,8,4,6,2,2, 4,0,2,2,0,2,0,0,0,2,
    2,0,2,4,6,6,2,0,0,0, 0,0 /* 0x64,0x65 */
};

/* read little-endian words from a buffer */
static int     rd16 (const uint8_t *d, int o) { return d[o] | (d[o+1] << 8); }
static int     rds16(const uint8_t *d, int o) { return s16(rd16(d, o)); }
static int32_t rd32 (const uint8_t *d, int o) { return (int32_t)(d[o] | (d[o+1]<<8) | (d[o+2]<<16) | (d[o+3]<<24)); }

/* Buffer-based core: compile geometry from a .dat image already in memory (the
 * caller owns `d`; no allocation/free here). The editor calls this with the
 * track it serialized via WriteTrack. Returns 0 on success. */
int gp2cc_compile_geometry_buf(const uint8_t *d, int len, gp2cc_track *t) {
    (void)len;
    int base = 0x1020 + rd32(d, 0x1010);          /* MainTrackData */
    t->hdr_angle = rds16(d, base + 0);
    int hdr_climb = rds16(d, base + 2);
    t->hdr_Y     = rds16(d, base + 4);
    t->hdr_X     = rds16(d, base + 8);
    t->hdr_width = rd16(d, base + 0xA);
    t->hdr_c5    = rd16(d, base + 0xE);
    int unknownNumber = rd16(d, base + 0x12);

    int o = base + 0x14;                            /* Handle4Unknowns */
    for (int i = 0; i < unknownNumber; i++) {
        int ax = rd16(d, o); o += 2;
        if (ax != 0x1F) o += 2;
    }

    /* ---- walk the command stream, integrating heading/startAngle (Q16) ---- */
    int32_t TA = (t->hdr_angle & 0xFFFF) << 16;     /* d_TrkStrtAngle */
    int32_t SA = TA;                                /* d_StartAngle   */
    int c5b12 = (t->hdr_c5 & 0x1000) != 0;
    int c5b9  = (t->hdr_c5 & 0x200)  != 0;
    int n = 0, ccoff = -1;

    /* ---- road-width state (tseg+0x60 left / +0x62 right), with transition ramps.
     * Width cmds: op 0x05 both (TrkWidthChange), 0x34 left (sub_75ACA), 0x35 right
     * (sub_75BCD); each: arg1=transition length, arg2=target. len==0 = instant;
     * len>0 = ramp over `len` segs, delta=(target-cur)/len (trunc). Per-seg:
     * store cur THEN ramp (cur+=delta; len--). Init both = header width. */
    int curL = t->hdr_width, curR = t->hdr_width;
    int deltaL = 0, deltaR = 0, transitL = 0, transitR = 0;

    for (;;) {
        int w = rd16(d, o);
        if (w == 0xFFFF) { ccoff = o + 2; break; }
        if (w & 0x8000) {
            int op = (w >> 8) & 0x7F;
            int szc;
            if (op == 0x45) szc = c5b9 ? 14 : 12;   /* C5sub_75D97 conditional */
            else if (op < 0x66) szc = TRK_ARGSZ[op];
            else szc = -1;
            if (szc < 0) { return -2; }     /* desync / unknown opcode */
            if (op == 0x34 || op == 0x05) {          /* left (or both) width change */
                int len = rds16(d, o + 2), target = rds16(d, o + 4);
                if (len == 0) { curL = target; deltaL = 0; transitL = 0; }
                else          { transitL = len; deltaL = (target - curL) / len; }
            }
            if (op == 0x35 || op == 0x05) {          /* right (or both) width change */
                int len = rds16(d, o + 2), target = rds16(d, o + 4);
                if (len == 0) { curR = target; deltaR = 0; transitR = 0; }
                else          { transitR = len; deltaR = (target - curR) / len; }
            }
            o += 2 + szc;
            continue;
        }
        /* geometry command: w = N segments */
        int N = w;
        int d1w  = rds16(d, o + 2);                 /* d_1stArg per-seg angle word */
        int32_t d1 = d1w << 16;                      /* ...as Q16 for the integration */
        /* tseg+0x14 (θ' tilt source) = (d1word * 0x6488) >> 14, 0x6488=round(pi/2*2^14);
         * constant for every seg in this command. Validated bit-exact vs CCREP f14. */
        int32_t f14 = ((int32_t)d1w * 0x6488) >> 14;
        o += 10;
        /* half-step (sub_79166: TrkStrtAngle += d1/2). The bit12 gate is `xor ax,ax`,
         * which zeros ONLY the low 16 bits of the 32-bit half-step (NOT the whole
         * value): d1=d1w<<16, so the integer part of d1/2 survives when bit12 clear.
         * Earlier `:0` wrongly killed the whole half-step -> command-boundary stalls
         * on bit12-clear tracks (e.g. f1ct02a, c5=0xD80). */
        int32_t half_a = d1 >> 1;
        if (!c5b12) half_a &= ~0xFFFF;
        for (int k = 0; k < N; k++) {
            if (n >= GP2CC_MAXSEG) { return -3; }
            if (k == 0) { SA = TA; TA += half_a; }
            else        { TA += d1; SA += d1; }
            t->fAngle[n]     = TA;
            t->angle[n]      = (int16_t)(TA >> 16);
            t->startAngle[n] = (int16_t)(SA >> 16);
            t->f14[n]        = f14;
            t->widthL[n]     = curL;        /* store width, THEN ramp (sub_77AD0) */
            t->widthR[n]     = curR;
            if (transitL > 0) { curL += deltaL; transitL--; }
            if (transitR > 0) { curR += deltaR; transitR--; }
            n++;
        }
        TA += half_a;                               /* command-end half-step */
    }
    /* the walk over-produces by 1 (a wrap seg the game doesn't count:
     * w_NumTrackSegs = w_ActSegCount-1). Drop it so the warp uses the real N. */
    n -= 1;
    t->n = n;
    t->ccoff = ccoff;

    /* ---- side vectors (tseg+0x0C/0E right, +0x4C/4E left): one component =
     * ((scos(startAngle)*width)>>17)<<6, matching InitTrackSegs 0x796C3/0x79735. */
    for (int i = 0; i < n; i++) {
        int sa = s16(t->startAngle[i]);
        int csa = scos(sa), ssa = scos(0x4000 - sa);   /* cos/sin via raw t_Sinus */
        #define SIDEVEC(cosv, wid) ((int16_t)(((int16_t)((int)(cosv)*(int)(wid) >> 16) >> 1) << 6))
        t->sideRX[i] = SIDEVEC(csa, t->widthR[i]);
        t->sideRY[i] = SIDEVEC(ssa, t->widthR[i]);
        t->sideLX[i] = SIDEVEC(csa, t->widthL[i]);
        t->sideLY[i] = SIDEVEC(ssa, t->widthL[i]);
        #undef SIDEVEC
    }

    /* ---- exact integer position (t_Sinus accumulation) + linear end-warp ---- */
    int32_t WX = t->hdr_X << 3, WY = t->hdr_Y << 3;
    int32_t *rx8 = t->rawX8, *ry8 = t->rawY8;
    for (int i = 0; i < n; i++) {
        int h = t->fAngle[i] >> 16;
        rx8[i] = WX; ry8[i] = WY;
        WX += (scos(h)        * 0x400) >> 14;       /* X step = cos(h)*128 */
        WY += (scos(0x4000-h) * 0x400) >> 14;       /* Y step = sin(h)*128 */
    }
    int32_t gx8 = WX - rx8[0], gy8 = WY - ry8[0];   /* closure gap (1/8 units) */
    t->gapX8 = gx8; t->gapY8 = gy8;
    /* EXACT integer warp (sub_0_76D39): a Bresenham gap-distributor. Verbatim
     * behaviour: the routine SKIPS segment 0 (loop entry jumps to edi+=0x6C first),
     * so segment i has accumulated |gap| exactly i times -> correction =
     * sign(gap)*floor(i*|gap|/M), with M = w_ActSegCount = n+1 (the over-produced
     * wrap seg the game counts but we drop). Validated bit-exact (full 1/8-unit incl
     * sub-bits) vs the BESTLN dumps on F1CT01/02a/03/09/11/16. Subtract to close the
     * loop. Earlier float `gap*i/n` was wrong on both M (n vs n+1) and the truncate-
     * vs-floor of the negative quotient -> ~1u error that the cc-line amplified. */
    {
        int64_t M = (int64_t)n + 1;
        int64_t agx = gx8 < 0 ? -(int64_t)gx8 : gx8;
        int64_t agy = gy8 < 0 ? -(int64_t)gy8 : gy8;
        for (int i = 0; i < n; i++) {
            int32_t sx = (int32_t)((agx * i) / M);   /* floor, operand >= 0 */
            int32_t sy = (int32_t)((agy * i) / M);
            t->X8[i] = rx8[i] - (gx8 < 0 ? -sx : sx);
            t->Y8[i] = ry8[i] - (gy8 < 0 ? -sy : sy);
        }
    }
    (void)hdr_climb;
    return 0;
}

/* File wrapper: read the whole .dat into a buffer, then compile from it. */
int gp2cc_compile_geometry(const char *dat_path, gp2cc_track *t) {
    FILE *f = fopen(dat_path, "rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *d = (uint8_t*)malloc(sz);
    if (!d) { fclose(f); return -1; }
    if (fread(d, 1, sz, f) != (size_t)sz) { fclose(f); free(d); return -1; }
    fclose(f);
    int rc = gp2cc_compile_geometry_buf(d, (int)sz, t);
    free(d);
    return rc;
}

/* ======================================================================== *
 *  cc-line (racing line) — verbatim asm->C port of UACalcBestLine (0x78EB2)
 *  and its helper set. The cc-line is a chaotic feedback system; every step
 *  is transcribed instruction-faithfully so it is bit-exact by construction.
 *  IDA addrs cited per routine; disasm in new-export-from-selection-annotated.lst.
 * ======================================================================== */

#define ONE60  (((int64_t)1) << 60)
#define CBB0C  20861          /* data const @0xCBB0C, segment-frame θ' tilt */

static int abs16(int v) { v = s16(v); return v < 0 ? -v : v; }

/* sub_10240 (0x10240): integer sqrt of a 32-bit value via a FIXED 3-iteration
 * 16-bit Newton (NOT exact floor — must be replicated bit-for-bit). */
static uint32_t sqrt32_10240(uint32_t V) {
    if (V == 0) return 0;
    int shift = 0;
    while (V < 0x40000000u) { shift++; V <<= 2; if (V == 0) return 0; }
    uint32_t half = V >> 1;                 /* bx:cx */
    uint32_t di = ((V >> 17) + 0x8000) & 0xFFFF;   /* 16-bit seed */
    for (int it = 0; it < 3; it++) {
        uint32_t hi = half >> 16, q;
        if (hi >= di) q = half & 0xFFFF;    /* overflow guard (div skipped) */
        else          q = half / di;        /* 32/16 -> 16-bit */
        di = ((di >> 1) + q) & 0xFFFF;
    }
    return (di >> shift) & 0xFFFF;
}

/* sub_102F0 (0x102F0): integer sqrt of a 64-bit value via a FIXED 5-iteration
 * Newton (exact division inside, but fixed iters -> lands floor±1; NOT math.isqrt).
 * Bit-exactness of the whole cc-line hinges on this matching the game exactly. */
static uint32_t sqrt64_102F0(uint64_t V) {
    if (V == 0) return 0;
    if ((uint32_t)(V >> 32) == 0) return sqrt32_10240((uint32_t)V);
    int shift = 0;
    while ((V >> 32) < 0x40000000ull) { shift++; V <<= 2; }
    uint64_t half = V >> 1;
    uint32_t est = (uint32_t)((half >> 32) + 0x80000000ull);   /* 32-bit seed */
    for (int it = 0; it < 5; it++) {
        uint32_t q = (uint32_t)(half / est);                   /* exact unsigned div */
        est = (est >> 1) + q;                                  /* 32-bit, wraps as asm */
    }
    return est >> shift;
}

/* sub_10502 (0x10502): cos(|angle|) in Q30 (amp 0x40000000) via the cosine
 * table with high-precision interp. result32 = (COS[widx]<<16)+(delta*frac)<<13.
 * Sign is carried by the extended COS table (negative for widx>2048). */
static int32_t cos30(int angle) {
    int a = abs16(angle);
    int widx = a >> 3, frac = a & 7;
    int base  = COS[widx];
    int delta = COS[widx + 1] - COS[widx];
    return ((int32_t)base << 16) + ((int32_t)(delta * frac) << 13);
}

/* sub_7827D (0x7827D): Q30 cos/sin of `angle`. Larger-magnitude component via
 * sqrt(ONE60-other^2), smaller via the cos table; decided by coarse |sin|>=0x2000.
 * Signs re-applied from the coarse table values. */
static void q30_cossin(int angle, int32_t *pcos, int32_t *psin) {
    int a = s16(angle);
    int coarse_sin = COS[abs16(s16(0x4000 - a)) >> 3];   /* COS[..]=sin(a) coarse */
    int coarse_sin_abs = coarse_sin < 0 ? -coarse_sin : coarse_sin;
    int32_t c30, s30;
    if (coarse_sin_abs >= 0x2000) {            /* sin large: cos via table, sin via sqrt */
        c30 = cos30(a);
        int64_t smag = (int64_t)sqrt64_102F0((uint64_t)(ONE60 - (int64_t)c30 * c30));
        s30 = coarse_sin < 0 ? (int32_t)(-smag) : (int32_t)smag;
    } else {                                   /* cos large: sin via table, cos via sqrt */
        s30 = cos30(s16(0x4000 - a));
        int64_t cmag = (int64_t)sqrt64_102F0((uint64_t)(ONE60 - (int64_t)s30 * s30));
        int coarse_cos = COS[abs16(a) >> 3];
        c30 = coarse_cos < 0 ? (int32_t)(-cmag) : (int32_t)cmag;
    }
    *pcos = c30; *psin = s30;
}

/* sub_10798 (0x10798): atan2(y,x) in 0x10000 units (full circle). 16-bit args. */
static int atan2_u(int y, int x) {
    y = s16(y); x = s16(x);
    int dy = y < 0 ? -y : y;
    int dx = x < 0 ? -x : x;
    int signs_differ = (y < 0) != (x < 0);
    int a;
    if (dy >= dx) {
        if (dy == 0) return 0;                 /* both zero */
        int idx = (dx << 11) / dy;             /* <=2048 since dx<=dy */
        a = 0x4000 - ATAN1[idx];
    } else {
        int idx = (dy << 11) / dx;
        a = ATAN1[idx];
    }
    if (signs_differ) a = -a;
    if (x < 0) a += 0x8000;
    return s16(a);
}

/* sub_78492 (0x78492): build the world arc-centre point W=(C94BC,C94C0) for the
 * first segment of a command, from (seg, P, H, arg2, arg1mul4). 1/8 world units.
 *   latP = (P*f14)>>15 ; base = rotate(P,latP) by PLAIN segAngle + segCentre
 *   + arg1mul4 along H ; + |arg2| along perp(H) (Q30 arc offset) when arg2!=0. */
static void build_centre(const gp2cc_ccgeo *g, int idx, int P, int H,
                         int32_t arg2, int arg1mul4, int32_t *pWx, int32_t *pWy) {
    int seg = idx;
    int f14 = (int)g->f14[seg];
    int latP = (int)(((int32_t)P * f14) >> 15);   /* (P*f14)<<2 then >>16 then >>1 */
    int ang  = s16(g->segAngle[seg]);
    int cosA = gp2cc_gv(ang);                      /* gv(segAngle)   */
    int sinA = gp2cc_gv(s16(0x4000 - ang));        /* gv(0x4000-seg) */
    /* RX = P*cos + latP*sin ; RY = latP*cos - P*sin  (each >>14, s16) */
    int32_t RX = (int16_t)(((int32_t)P * cosA + (int32_t)latP * sinA) >> 14);
    int32_t RY = (int16_t)(((int32_t)latP * cosA - (int32_t)P * sinA) >> 14);
    RX += g->cx8[seg];
    RY += g->cy8[seg];
    /* arg1mul4 component along H: gv(0x4000-H)=sin(H), gv(H)=cos(H) */
    int sinH = gp2cc_gv(s16(0x4000 - H));
    int cosH = gp2cc_gv(s16(H));
    int16_t a1m4 = (int16_t)arg1mul4;
    RX += (int16_t)(((int32_t)sinH * a1m4) >> 14);
    RY += (int16_t)(((int32_t)cosH * a1m4) >> 14);
    int32_t Wx = RX, Wy = RY;
    if (arg2 != 0) {
        int32_t mag = arg2 < 0 ? -arg2 : arg2;
        int perp = (arg2 < 0) ? s16(H - 0x4000) : s16(H + 0x4000);
        int32_t c32, s32; q30_cossin(perp, &c32, &s32);
        Wx += (int32_t)(((int64_t)s32 * mag) >> 30);   /* x gets sin */
        Wy += (int32_t)(((int64_t)c32 * mag) >> 30);   /* y gets cos */
    }
    *pWx = Wx; *pWy = Wy;
}

/* sub_787E7 (0x787E7): reproject world point W onto seg `idx` -> P (and H on
 * curves). Uses the θ' tilt and reads the sub-bit-exact segment centre. */
static void reproject(const gp2cc_ccgeo *g, int idx, int32_t Wx, int32_t Wy,
                      int32_t arg2, int *pP, int *pH) {
    int seg = idx;
    int f14 = (int)g->f14[seg];
    int ang = s16(g->segAngle[seg]);
    /* θ' = segAngle - ((f14/2 * CBB0C)>>15) */
    int thetaP = s16(ang - (int)((((int32_t)(f14 >> 1) * CBB0C) << 1) >> 16));
    int32_t relx = Wx - g->cx8[seg];
    int32_t rely = Wy - g->cy8[seg];
    int32_t c32, s32; q30_cossin(thetaP, &c32, &s32);
    /* lat = cos*relx - sin*rely ; lon = sin*relx + cos*rely  (Q30, >>30) */
    int64_t lat64 = (int64_t)c32 * relx - (int64_t)s32 * rely;
    int64_t lon64 = (int64_t)s32 * relx + (int64_t)c32 * rely;
    int32_t lat = (int32_t)(lat64 >> 30);
    int32_t lon = (int32_t)(lon64 >> 30);
    if (arg2 == 0) {
        /* straight (loc_78A54): P = lat - lon*tan(H-θ'). The asm uses sub_10502
         * (raw cos table, NOT the sqrt-normalised q30 pair): sn=cos30(0x4000-d),
         * cs=cos30(d); tanterm = (sn*lon)/cs (signed 64-bit divide). H unchanged. */
        int d = s16(*pH - thetaP);
        int32_t sn = cos30(s16(0x4000 - d));        /* sub_10502(0x4000-d) = sin(d) */
        int32_t cs = cos30(d);                       /* sub_10502(d)        = cos(d) */
        int32_t tanterm = 0;
        if (cs != 0) tanterm = (int32_t)(((int64_t)sn * lon) / cs);
        *pP = s16(lat - tanterm);
        return;
    }
    int64_t a2 = arg2;
    int64_t disc = a2 * a2 - (int64_t)lon * lon;
    int64_t T = (int64_t)sqrt64_102F0((uint64_t)disc);
    int64_t Tsigned = (arg2 < 0) ? -T : T;
    *pP = s16((int32_t)(lat - (int32_t)Tsigned));
    /* heading: scale (Tsigned, lon, |arg2|) down until |arg2|<0x7F00, then atan2 */
    int32_t Ts = (int32_t)Tsigned, lonv = lon;
    uint32_t aa = (uint32_t)(arg2 < 0 ? -arg2 : arg2);
    while (((aa >> 16) > 0) || ((aa >> 16) == 0 && (aa & 0xFFFF) >= 0x7F00)) {
        Ts >>= 1; lonv >>= 1; aa >>= 1;
    }
    int at = atan2_u((int)(int16_t)(Ts & 0xFFFF), (int)(int16_t)(lonv & 0xFFFF));
    at = (arg2 < 0) ? s16(at + 0x4000) : s16(at - 0x4000);
    *pH = s16(at + thetaP);
}

/* sub_78CEC (0x78CEC): per-seg curvature-ramp nudge of W (slope!=0 commands).
 * a = H ± 0x4000 (sign by arg2) ; C94BC += sin(a)*slope>>14, C94C0 += cos(a)*slope>>14. */
static void nudge(int32_t *pWx, int32_t *pWy, int H, int32_t arg2, int32_t slope) {
    int32_t ebp = slope;
    int a;
    if (arg2 < 0) { ebp = -ebp; a = s16(H - 0x4000); }
    else            a = s16(H + 0x4000);
    int sinA = gp2cc_gv(s16(0x4000 - a));
    int cosA = gp2cc_gv(s16(a));
    *pWx += (int32_t)(((int64_t)sinA * ebp) >> 14);
    *pWy += (int32_t)(((int64_t)cosA * ebp) >> 14);
}

/* GetSingleCmdArgs (0x78D4C): parse one cc command from cc[*po]. */
typedef struct {
    int      end;          /* word==0 (last command) */
    int      word;         /* the command word (bp) */
    int      N;            /* word & 0x7FF (wActCCLCookedCmd) */
    int      a1, a2;       /* wCCL1stCmdArg1/2 (bit15 header words) */
    int32_t  arg2;         /* dActCCLineArg2 (×8 unless bit12, dword if bit14) */
    int32_t  slope;        /* dArg3mArg2divLen = (arg3-arg2)/N */
    int      arg1mul4;     /* wActCCLArg1mul4  (arg1<<2 when arg2!=0) */
    int      arg1if2is0;   /* wActCLArg1If2is0 (arg1 when arg2==0)   */
} cc_cmd;

static int rdw(const uint8_t *d, int o) { return d[o] | (d[o + 1] << 8); }

static void parse_cmd(const uint8_t *cc, int *po, cc_cmd *c) {
    int o = *po;
    memset(c, 0, sizeof *c);
    int word = rdw(cc, o); o += 2;
    c->word = word;
    c->N = word & 0x7FF;
    if (word == 0) { c->end = 1; *po = o; return; }
    if (word & 0x8000) { c->a1 = rdw(cc, o); o += 2; c->a2 = rdw(cc, o); o += 2; }
    int cx = rdw(cc, o); o += 2;                 /* arg1 */
    int32_t arg2 = (int32_t)(int16_t)rdw(cc, o); o += 2;
    if (word & 0x4000) { arg2 = (int32_t)((uint32_t)arg2 << 16) | rdw(cc, o); o += 2; }
    if (!(word & 0x1000)) arg2 <<= 3;
    c->arg2 = arg2;
    if (word & 0x2000) {                          /* arg3 → slope */
        int32_t arg3 = (int32_t)(int16_t)rdw(cc, o); o += 2;
        if (word & 0x4000) { arg3 = (int32_t)((uint32_t)arg3 << 16) | rdw(cc, o); o += 2; }
        if (!(word & 0x1000)) arg3 <<= 3;
        int32_t diff = arg3 - arg2;
        c->slope = diff / c->N;                    /* idiv (trunc toward zero) */
    }
    if (arg2 != 0) { c->arg1if2is0 = 0; c->arg1mul4 = (int16_t)(cx << 2); }
    else           { c->arg1mul4 = 0;   c->arg1if2is0 = cx; }
    *po = o;
}

int gp2cc_reproject_dbg(int segAngle, int32_t cx8, int32_t cy8, int f14,
                        int32_t Wx, int32_t Wy, int32_t arg2, int Hin, int *pH) {
    gp2cc_ccgeo g; g.n = 1;
    g.segAngle[0] = (int16_t)segAngle; g.cx8[0] = cx8; g.cy8[0] = cy8; g.f14[0] = f14;
    int P = 0, H = Hin;
    reproject(&g, 0, Wx, Wy, arg2, &P, &H);
    if (pH) *pH = H;
    return P;
}

/* Front half of sub_787E7 exposed for stage validation: θ', q30 cos/sin, lat, lon. */
void gp2cc_reproject_stages(int segAngle, int32_t cx8, int32_t cy8, int f14,
                            int32_t Wx, int32_t Wy,
                            int32_t *pCos32, int32_t *pSin32,
                            int32_t *pLat, int32_t *pLon, int *pTheta) {
    int ang = s16(segAngle);
    int thetaP = s16(ang - (int)((((int32_t)(f14 >> 1) * CBB0C) << 1) >> 16));
    int32_t relx = Wx - cx8;
    int32_t rely = Wy - cy8;
    int32_t c32, s32; q30_cossin(thetaP, &c32, &s32);
    int64_t lat64 = (int64_t)c32 * relx - (int64_t)s32 * rely;
    int64_t lon64 = (int64_t)s32 * relx + (int64_t)c32 * rely;
    if (pCos32) *pCos32 = c32;
    if (pSin32) *pSin32 = s32;
    if (pLat) *pLat = (int32_t)(lat64 >> 30);
    if (pLon) *pLon = (int32_t)(lon64 >> 30);
    if (pTheta) *pTheta = thetaP;
}

void gp2cc_build_dbg(int segAngle, int32_t cx8, int32_t cy8, int f14,
                     int P, int H, int32_t arg2, int arg1mul4,
                     int32_t *pWx, int32_t *pWy) {
    gp2cc_ccgeo g; g.n = 1;
    g.segAngle[0] = (int16_t)segAngle; g.cx8[0] = cx8; g.cy8[0] = cy8; g.f14[0] = f14;
    build_centre(&g, 0, P, H, arg2, arg1mul4, pWx, pWy);
}

int gp2cc_compute_ccline_dbg(const gp2cc_ccgeo *g, const uint8_t *cc, int cc_len,
                             int cc_off, int16_t *bestLine, int16_t *angle18,
                             int32_t *capWx, int32_t *capWy, int32_t *capArg2,
                             int *cmdStartSeg, int *pNumCmds) {
    int n = g->n;
    if (n <= 0) return -1;
    int cur = 0, cmdIdx = 0;
    /* seed (0x78EB2): H = segAngle[0]; P = ccdata[cc_off+2] if bit15 && !bit11 */
    int H = s16(g->segAngle[0]);
    int P = 0;
    int w0 = rdw(cc, cc_off);
    /* seed segPosX_0A (=P) is a SIGNED 16-bit word ([ebx+2], used by a signed
     * imul in sub_78492); sign-extend or build_centre blows up on negative seeds. */
    if (!(w0 & 0x800) && (w0 & 0x8000)) P = s16(rdw(cc, cc_off + 2));
    int o = cc_off;
    for (;;) {
        cc_cmd c;
        parse_cmd(cc, &o, &c);
        if (c.end) break;
        if (o > cc_len) return -2;
        if ((c.word & 0x800) && (c.word & 0x8000)) { P = s16(c.a1); H = s16(c.a2); }
        if (c.arg2 == 0) H = s16(H + s16(c.arg1if2is0));
        if (cmdStartSeg) cmdStartSeg[cmdIdx] = cur;   /* this command's first segment */
        cmdIdx++;
        int32_t Wx, Wy;
        build_centre(g, cur, P, H, c.arg2, c.arg1mul4, &Wx, &Wy);
        int32_t arg2 = c.arg2;
        int left = c.N;
        while (left > 0) {
            angle18[cur]  = (int16_t)s16(H - s16(g->segAngle[cur]));
            bestLine[cur] = (int16_t)s16(P);
            if (capWx) capWx[cur] = Wx;
            if (capWy) capWy[cur] = Wy;
            if (capArg2) capArg2[cur] = arg2;
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

int gp2cc_compute_ccline(const gp2cc_ccgeo *g, const uint8_t *cc, int cc_len,
                         int cc_off, int16_t *bestLine, int16_t *angle18,
                         int *cmdStartSeg, int *pNumCmds) {
    return gp2cc_compute_ccline_dbg(g, cc, cc_len, cc_off, bestLine, angle18,
                                    0, 0, 0, cmdStartSeg, pNumCmds);
}
