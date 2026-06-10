/*
 * gp2cc — standalone bit-exact reproduction of GP2's compiled track geometry
 * and cc-line (racing line), ported verbatim from the disassembly.
 *
 * Goal: given a track .dat (and the t_Sinus table from GP2.EXE), reproduce the
 * game's per-segment compiled geometry + cc-line bit-for-bit, validated against
 * the GP2Lap BESTLNN.TXT dumps. The editor's new compiled-track view will call
 * this module and just draw the arrays — it never touches the fixed point.
 *
 * Status: geometry compiler ported + validated (heading/startAngle exact,
 * position via integer t_Sinus accumulation + warp). cc-line (build/reproject/
 * nudge + the Q30/atan2 helpers) to follow.
 */
#ifndef GP2CC_H
#define GP2CC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GP2CC_MAXSEG 4096

typedef struct {
    int     n;                  /* number of track segments */
    int32_t fAngle[GP2CC_MAXSEG];   /* d_TrkStrtAngle per seg, Q16 (full precision) */
    int16_t angle[GP2CC_MAXSEG];    /* heading = fAngle>>16 (tseg+0)   */
    int16_t startAngle[GP2CC_MAXSEG]; /* d_StartAngle>>16 (tseg+0xA, side-vec) */
    int32_t f14[GP2CC_MAXSEG];      /* tseg+0x14 = (cmd d1 * 0x6488)>>14 (θ' tilt) */
    int32_t widthL[GP2CC_MAXSEG];   /* tseg+0x60 left road width (w_TrkBeginWidth)  */
    int32_t widthR[GP2CC_MAXSEG];   /* tseg+0x62 right road width (w_TrkBeginWidth2) */
    /* side vectors (tseg+0x4C/0x4E left, +0x0C/0x0E right): edge = centre ± side/64.
     * side = ((scos(startAngle)*width)>>17)<<6 components. (sideRX/RY validated == dump xSide/ySide.) */
    int16_t sideLX[GP2CC_MAXSEG], sideLY[GP2CC_MAXSEG];
    int16_t sideRX[GP2CC_MAXSEG], sideRY[GP2CC_MAXSEG];
    /* exact 1/8-world-unit position (the game's w_TrkStrtX/Y, warped) */
    int32_t X8[GP2CC_MAXSEG];   /* world X (= dump col4) * 8, warped */
    int32_t Y8[GP2CC_MAXSEG];   /* world Y (= dump col3) * 8, warped */
    int32_t rawX8[GP2CC_MAXSEG]; /* world X * 8, BEFORE warp (raw integration) */
    int32_t rawY8[GP2CC_MAXSEG]; /* world Y * 8, BEFORE warp */
    int32_t gapX8, gapY8;        /* raw closure gap (rawEnd - start), 1/8 units */
    int     ccoff;              /* file offset of the cc-line command stream */
    /* header */
    int16_t hdr_angle, hdr_X, hdr_Y;
    uint16_t hdr_width, hdr_c5;
} gp2cc_track;

/* Per-segment geometry the cc-line compiler consumes. The cc-line internal
 * convention pairs C94BC with tseg+4 (=dump col3 =world Y) and C94C0 with tseg+8
 * (=dump col4 =world X), so cx8/cy8 follow that pairing (NOT the X8/Y8 naming). */
typedef struct {
    int     n;
    int16_t segAngle[GP2CC_MAXSEG];  /* tseg+0    heading                       */
    int32_t cx8[GP2CC_MAXSEG];       /* tseg+4 full 1/8-unit (C94BC-paired)      */
    int32_t cy8[GP2CC_MAXSEG];       /* tseg+8 full 1/8-unit (C94C0-paired)      */
    int32_t f14[GP2CC_MAXSEG];       /* tseg+0x14 (width-derived, drives θ' tilt) */
} gp2cc_ccgeo;

/* Load the t_Sinus cosine table + t_ArithTab1 atan table from GP2.EXE
 * (file offs 0x1DDAEC / 0x1DFAF0). Returns 0 ok. */
int  gp2cc_load_tables(const char *gp2exe_path);

/* GetSinusVal (0x104B9): cosine, amplitude 0x4000, 8-unit table + lerp. */
int  gp2cc_gv(int ax);

/* Compile the track geometry from a .dat file into `t`. Returns 0 on success. */
int  gp2cc_compile_geometry(const char *dat_path, gp2cc_track *t);

/* Same, from a .dat image already in memory (caller owns `dat`). For the editor,
 * which serializes its in-memory track to a .dat buffer. Returns 0 on success. */
int  gp2cc_compile_geometry_buf(const uint8_t *dat, int len, gp2cc_track *t);

/* Run UACalcBestLine (0x78EB2) over the cc-command stream `cc` (the raw .dat
 * bytes) starting at byte offset `cc_off`, using geometry `g`. Writes per-seg
 * bestLine (tseg+0x16) and angle18 (tseg+0x18). Returns 0 on success.
 * Verbatim asm->C port of the cc-line helpers (bit-exact by construction). */
/* Run the cc-line. `cmdStartSeg` (NULL ok) is filled with each cc-command's first
 * segment index (cmdStartSeg[0..*pNumCmds-1]); the editor's CCLineSections map 1:1
 * to cc-commands in order, so this places the per-sector indicators. */
int  gp2cc_compute_ccline(const gp2cc_ccgeo *g, const uint8_t *cc, int cc_len,
                          int cc_off, int16_t *bestLine, int16_t *angle18,
                          int *cmdStartSeg, int *pNumCmds);

/* Same, but also captures the carried world point W (=C94BC/C94C0) and arg2
 * per segment (for stage-by-stage validation vs CCREP). Any out ptr may be NULL. */
int  gp2cc_compute_ccline_dbg(const gp2cc_ccgeo *g, const uint8_t *cc, int cc_len,
                              int cc_off, int16_t *bestLine, int16_t *angle18,
                              int32_t *capWx, int32_t *capWy, int32_t *capArg2,
                              int *cmdStartSeg, int *pNumCmds);

/* Debug: reproject a SINGLE world point W onto one segment (isolates sub_787E7
 * from the carried-W feedback). Returns P; writes *pH (curve heading). */
int  gp2cc_reproject_dbg(int segAngle, int32_t cx8, int32_t cy8, int f14,
                         int32_t Wx, int32_t Wy, int32_t arg2, int Hin, int *pH);

/* Debug: build the world point W for one segment (isolates sub_78492). */
void gp2cc_build_dbg(int segAngle, int32_t cx8, int32_t cy8, int f14,
                     int P, int H, int32_t arg2, int arg1mul4,
                     int32_t *pWx, int32_t *pWy);

/* Debug: expose the reproject intermediates (q30 cos/sin of θ', and lat/lon)
 * for stage-by-stage validation against the extended CCREP dump. */
void gp2cc_reproject_stages(int segAngle, int32_t cx8, int32_t cy8, int f14,
                            int32_t Wx, int32_t Wy,
                            int32_t *pCos32, int32_t *pSin32,
                            int32_t *pLat, int32_t *pLon, int *pTheta);

#ifdef __cplusplus
}
#endif

#endif
