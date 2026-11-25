// SPDX-FileCopyrightText: 2026 Jinyu Yang
// SPDX-License-Identifier: 0BSD
//
// onze/core/rng.h -- counter-based random number generator (CBRNG) that 
// is order-independent and thread-safe
//
// Every random draw is a function of a five-tuple coordinate:
// 
//          (seed, stream, entity, tick, index) -> uint64_t
//
// Any draw can be reproduced in isolation regardless of execution order or
// thread scheduling. Two different coordinates never give Philox the same
// (key, counter) input. Math functions (sqrt, log, cos) routes through 
// onze::dmath instead of <cmath> for cross-platform reproducibility.
//
// Backed by Philox4x32-10 (Salmon et al., "Parallel Random Numbers: As Easy
// as 1, 2, 3", 2011) with SplitMix64 key whitening.

#pragma once

#include <array>
#include <cstdint>

namespace onze {

/// Independent substreams encoded in the Philox counter. 0 is reserved.
/// Append-only: do not renumber or reuse IDs, or replay determinism breaks.
enum class Stream : std::uint32_t {
    Decision = 1,
    Keeper = 2,
    Perception = 3,
    Team = 4,
    Execution = 5,
    Timing = 6,
    Contest = 7,
    Balance = 8,
    Referee = 9,
    Form = 10,
    Scenario = 11,
};

/// Returns 64 deterministic pseudorandom bits for the given coordinate.
std::uint64_t rng(
    std::uint64_t seed,
    Stream stream,
    std::uint32_t entity,
    std::uint32_t tick,
    std::uint32_t index
);

/// Packs a (block, k) pair to a flat draw index for rng().
/// Precondition: block < 65536 and k < 65536.
constexpr std::uint32_t rng_index(
    std::uint32_t block, 
    std::uint32_t k
) {
    return 65536u * block + k; 
}

// -----------------------------------------------------------------------------
// Distributions
// -----------------------------------------------------------------------------

/// Midpoint-quantized uniform float strictly inside (0, 1): [2^-24, 1 - 2^-24].
float uniform01f(std::uint64_t u);

/// Midpoint-quantized uniform double strictly inside (0, 1): [2^-53, 1 - 2^-53].
double uniform01(std::uint64_t u);

/// Standard normal variate N(0, 1). Box-Muller. Guaranteed finite (within ±6.76).
double normal(std::uint64_t u);

/// Standard Gumbel variate Gumbel(0, 1). Guaranteed finite.
double gumbel(std::uint64_t u);

/// Uniform variate scaled to [lo, hi] (precondition: lo <= hi; finite hi - lo).
double uniform(double lo, double hi, std::uint64_t u);

/// Bernoulli trial with success probability p in [0, 1].
/// Guaranteed false for p <= 0 and true for p >= 1.
inline bool bernoulli(double p, std::uint64_t u) {
    return uniform01(u) < p;
}

// -----------------------------------------------------------------------------
// Low-level primitives
// -----------------------------------------------------------------------------

/// 10-round Philox4x32 bijection mapping a 128-bit counter and 64-bit key
/// to four 32-bit pseudorandom words.
std::array<std::uint32_t, 4> philox4x32_10(
    std::array<std::uint32_t, 4> ctr,
    std::array<std::uint32_t, 2> key
);

/// Single-step SplitMix64 hash (Weyl increment + Stafford Mix13 finalizer).
std::uint64_t splitmix64(std::uint64_t x);

}  // namespace onze
