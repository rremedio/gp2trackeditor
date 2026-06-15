#pragma once
#include <cstdint>
#include "gp2cc.h"   // gp2cc_ccgeo input struct

// ---------------------------------------------------------------------------
// gp2ccline — GP2's racing-line (cc-line) solver, ported to C++ and fed by the
// formula-generated cosine/atan tables (gp2cos.hpp / gp2atan.hpp).
//
// This is the CHAOTIC kernel: a reproject/nudge feedback loop whose output
// diverges on any rounding difference. It is therefore a FAITHFUL transcription
// of the engine's fixed-point math (UACalcBestLine 0x78EB2 + helpers), not a
// re-derivation — kept legible with comments, but the integers are load-bearing
// and must not be "cleaned up". Validated bit-exact vs the original C kernel.
// ---------------------------------------------------------------------------
namespace gp2geom {

// Drop-in for gp2cc_compute_ccline. Writes per-segment bestLine (tseg+0x16) and
// angle18 (tseg+0x18); cmdStartSeg/pNumCmds (may be null) record each cc-command's
// first segment. Returns 0 on success.
int computeCcLine(const gp2cc_ccgeo& g, const std::uint8_t* cc, int cc_len, int cc_off,
                  std::int16_t* bestLine, std::int16_t* angle18,
                  int* cmdStartSeg, int* pNumCmds);

} // namespace gp2geom
