// SPDX-FileCopyrightText: 2026 Jinyu Yang
// SPDX-License-Identifier: 0BSD
//
// onze/core/rng.cpp -- Philox4x32-10, SplitMix64, and deterministic variates.

#include "onze/core/rng.h"

#include "onze/core/dmath.h"

namespace onze {

std::array<std::uint32_t, 4> philox4x32_10(std::array<std::uint32_t, 4> c,
                                           std::array<std::uint32_t, 2> k) {
    // Philox4x32 multipliers. See Salmon et al. (2011).
    constexpr std::uint64_t M0 = 0xD2511F53u, M1 = 0xCD9E8D57u;
    // Weyl key bumps: floor(2^32 * (phi - 1)) and floor(2^32 * (sqrt(3) - 1)).
    constexpr std::uint32_t W0 = 0x9E3779B9u, W1 = 0xBB67AE85u;

    // Round 0 uses the unbumped key; bump before rounds 1..9.
    for (int round = 0; round < 10; ++round) {
        if (round > 0) {
            k[0] += W0;
            k[1] += W1;
        }
        const std::uint64_t p0 = M0 * c[0];
        const std::uint64_t p1 = M1 * c[2];

        c = {
            static_cast<std::uint32_t>(p1 >> 32) ^ c[1] ^ k[0],
            static_cast<std::uint32_t>(p1),
            static_cast<std::uint32_t>(p0 >> 32) ^ c[3] ^ k[1],
            static_cast<std::uint32_t>(p0),
        };
    }
    return c;
}

std::uint64_t splitmix64(std::uint64_t x) {
    // Weyl step floor(2^64 / phi) followed by David Stafford's Mix13 finalizer.
    std::uint64_t z = x + 0x9E3779B97F4A7C15ull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

std::uint64_t rng(
    std::uint64_t seed,
    Stream stream,
    std::uint32_t entity,
    std::uint32_t tick,
    std::uint32_t index
) {
    // Whiten seed into an uncorrelated 64-bit Philox key.
    const std::uint64_t key = splitmix64(seed);
    const auto out = philox4x32_10(
        {entity, tick, index, static_cast<std::uint32_t>(stream)},
        {
            static_cast<std::uint32_t>(key >> 32),
            static_cast<std::uint32_t>(key)
        }
    );

    // Return lanes 0 and 1 only; rng() returns 64 bits.
    return (static_cast<std::uint64_t>(out[0]) << 32) | out[1];
}

// Midpoint quantization: extract top b bits (b = 23 for float, b = 52 for
// double) and compute (bits + 0.5) * 2^-b. At bits = 2^b - 1, the IEEE-754
// ULP is 0.5, so both the addition and power-of-two scaling are exact with no
// rounding up to 1.0. Output is symmetric and strictly inside (0, 1).

float uniform01f(std::uint64_t u) {
    return (static_cast<float>(u >> 41) + 0.5f) * (1.0f / 8388608.0f);
}

double uniform01(std::uint64_t u) {
    return (static_cast<double>(u >> 12) + 0.5) * (1.0 / 4503599627370496.0);
}

double normal(std::uint64_t u) {
    // Box-Muller on two independent 32-bit midpoint-quantized uniforms in
    // [2^-33, 1 - 2^-33]. Discards the sine twin to remain stateless.
    // Lower bound u1 >= 2^-33 caps magnitude at sqrt(66 * ln(2)) ~= 6.76 sigma.
    const double u1 = (static_cast<double>(u >> 32) + 0.5) * (1.0 / 4294967296.0);
    const double u2 = (static_cast<double>(u & 0xFFFFFFFFull) + 0.5) * (1.0 / 4294967296.0);
    return dmath::sqrt(-2.0 * dmath::log(u1)) * dmath::cos(6.283185307179586476925286766559 * u2);
}

double uniform(double lo, double hi, std::uint64_t u) {
    return lo + (hi - lo) * uniform01(u);
}

double gumbel(std::uint64_t u) {
    // Inverse-CDF Gumbel; uniform01() stays strictly in (0, 1), avoiding
    // log(0) singularities at both endpoints.
    return -dmath::log(-dmath::log(uniform01(u)));
}

}  // namespace onze
