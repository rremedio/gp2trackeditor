#pragma once
#include <cstdint>
#include "gp2cc.h"   // shared output format: gp2cc_track + GP2CC_MAXSEG

// ---------------------------------------------------------------------------
// gp2geom — GP2 track-geometry compiler, written as the geometric process it
// actually is (a fixed-point "turtle" that integrates a curve from its
// curvature), rather than as a transcription of the disassembly.
//
// Same bit-exact integers as the engine, same output struct as the original
// gp2cc compiler (so it is a drop-in for gp2cc_compile_geometry_buf), but the
// steps are named — turn, advance one chord, midpoint half-step, ramp width,
// close the loop — and every "magic" constant is derived in a comment.
//
// The cosine it uses is the build-time-verified table from gp2cos.hpp.
// ---------------------------------------------------------------------------
namespace gp2geom {

// Compile a track .dat image already in memory into `out`. Returns 0 on success,
// negative on a command-stream desync. Bit-exact with the game.
int compileGeometry(const std::uint8_t* dat, int len, gp2cc_track& out);

} // namespace gp2geom
