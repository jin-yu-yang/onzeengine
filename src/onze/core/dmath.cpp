// SPDX-License-Identifier: SunPro AND MIT AND 0BSD
//
// onze::dmath implementation. See dmath.h.
//
// Origin: musl libc v1.1.22, src/math
//         (from FreeBSD msun, from Sun fdlibm).
//   k_sin, k_cos  <- __sin.c, __cos.c
//   rem_pio2      <- __rem_pio2.c (medium-size path only)
//   atan, atan2   <- atan.c, atan2.c
//   exp, log      <- exp.c, log.c
// musl's changes are under the MIT License. See THIRD_PARTY_NOTICES.md.
//
// Changes: Copyright (C) 2026 Jinyu Yang. BSD Zero Clause License, see LICENSE.
//   - Ported to C++. Bit access with std::memcpy. std::ldexp in exp, 
// std::frexp in log.
//   - rem_pio2: folds |x| > 1.6e6 with std::remainder by the double 2*pi.
//     fdlibm uses Payne-Hanek reduction here.
//
// Our own code, not from fdlibm:
//   - asin, acos: atan2(x, sqrt((1-x)*(1+x))).
//   - pow: exp(y*log(x)). The error increases with |y*log(x)|.
//   - erf: power series for |x| < 3, continued fraction for 3 <= |x| < 6.

/* Notice from __rem_pio2.c, unchanged: */
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunSoft, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 *
 * Optimized by Bruce D. Evans.
 */

/* Notice from atan.c, unchanged: */
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

/* Notice from exp.c, unchanged: */
/*
 * ====================================================
 * Copyright (C) 2004 by Sun Microsystems, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#include "onze/core/dmath.h"

#include <cstdint>
#include <cstring>
#include <limits>

namespace onze::dmath {
namespace {

inline std::uint64_t bits(double x) {
  std::uint64_t u;
  std::memcpy(&u, &x, sizeof u);
  return u;
}
inline int exponent(double x) { return static_cast<int>((bits(x) >> 52) & 0x7ff); }

// --- sin / cos kernels on [-pi/4, pi/4] ---
constexpr double S1 = -1.66666666666666324348e-01, S2 = 8.33333333332248946124e-03,
                 S3 = -1.98412698298579493134e-04, S4 = 2.75573137070700676789e-06,
                 S5 = -2.50507602534068634195e-08, S6 = 1.58969099521155010221e-10;
constexpr double C1 = 4.16666666666666019037e-02, C2 = -1.38888888888741095749e-03,
                 C3 = 2.48015872894767294178e-05, C4 = -2.75573143513906633035e-07,
                 C5 = 2.08757232129817482790e-09, C6 = -1.13596475577881948265e-11;

double k_sin(double x, double y, bool iy) {
  const double z = x * x;
  const double w = z * z;
  const double r = S2 + z * (S3 + z * S4) + z * w * (S5 + z * S6);
  const double v = z * x;
  if (!iy) return x + v * (S1 + z * r);
  return x - ((z * (0.5 * y - v * r) - y) - v * S1);
}

double k_cos(double x, double y) {
  const double z = x * x;
  double w = z * z;
  const double r = z * (C1 + z * (C2 + z * C3)) + w * w * (C4 + z * (C5 + z * C6));
  const double hz = 0.5 * z;
  w = 1.0 - hz;
  return w + (((1.0 - w) - hz) + (z * r - x * y));
}

// Argument reduction: x = n*pi/2 + y[0] + y[1], |y[0]| <= pi/4. Cody-Waite with three rounds
// (good to 151 bits) for |x| below about 1.6e6. Larger arguments do not occur in the simulation;
// they are first folded with the exact IEEE remainder by the double 2*pi.
constexpr double kInvPio2 = 6.36619772367581382433e-01, kPio2_1 = 1.57079632673412561417e+00,
                 kPio2_1t = 6.07710050650619224932e-11, kPio2_2 = 6.07710050630396597660e-11,
                 kPio2_2t = 2.02226624879595063154e-21, kPio2_3 = 2.02226624871116645580e-21,
                 kPio2_3t = 8.47842766036889956997e-32;
constexpr double kToInt = 6755399441055744.0;   // 1.5 * 2^52

int rem_pio2(double x, double y[2]) {
  if (std::fabs(x) > 1.6e6) x = std::remainder(x, 6.283185307179586476925286766559);
  const double fn = (x * kInvPio2 + kToInt) - kToInt;   // round to nearest
  const int n = static_cast<int>(fn);
  double r = x - fn * kPio2_1;
  double w = fn * kPio2_1t;
  y[0] = r - w;
  const int ex = exponent(x);
  if (ex - exponent(y[0]) > 16) {
    double t = r;
    w = fn * kPio2_2;
    r = t - w;
    w = fn * kPio2_2t - ((t - r) - w);
    y[0] = r - w;
    if (ex - exponent(y[0]) > 49) {
      t = r;
      w = fn * kPio2_3;
      r = t - w;
      w = fn * kPio2_3t - ((t - r) - w);
      y[0] = r - w;
    }
  }
  y[1] = (r - y[0]) - w;
  return n;
}

constexpr double kPio4 = 7.85398163397448278999e-01;

// --- atan ---
constexpr double kAtanHi[4] = {4.63647609000806093515e-01, 7.85398163397448278999e-01,
                               9.82793723247329054082e-01, 1.57079632679489655800e+00};
constexpr double kAtanLo[4] = {2.26987774529616870924e-17, 3.06161699786838301793e-17,
                               1.39033110312309984516e-17, 6.12323399573676603587e-17};
constexpr double kAT[11] = {
    3.33333333333329318027e-01,  -1.99999999998764832476e-01, 1.42857142725034663711e-01,
    -1.11111104054623557880e-01, 9.09088713343650656196e-02,  -7.69187620504482999495e-02,
    6.66107313738753120669e-02,  -5.83357013379057348645e-02, 4.97687799461593236017e-02,
    -3.65315727442169155270e-02, 1.62858201153657823623e-02};

constexpr double kPi = 3.1415926535897931160e+00, kPiLo = 1.2246467991473531772e-16;
constexpr double kHalfPi = 1.57079632679489655800e+00;

// --- exp / log ---
constexpr double kLn2Hi = 6.93147180369123816490e-01, kLn2Lo = 1.90821492927058770002e-10,
                 kInvLn2 = 1.44269504088896338700e+00;
constexpr double P1 = 1.66666666666666019037e-01, P2 = -2.77777777770155933842e-03,
                 P3 = 6.61375632143793436117e-05, P4 = -1.65339022054652515390e-06,
                 P5 = 4.13813679705723846039e-08;
constexpr double Lg1 = 6.666666666666735130e-01, Lg2 = 3.999999999940941908e-01,
                 Lg3 = 2.857142874366239149e-01, Lg4 = 2.222219843214978396e-01,
                 Lg5 = 1.818357216161805012e-01, Lg6 = 1.531383769920937332e-01,
                 Lg7 = 1.479819860511658591e-01;

}  // namespace

double sin(double x) {
  if (!std::isfinite(x)) return std::numeric_limits<double>::quiet_NaN();
  const double ax = std::fabs(x);
  if (ax <= kPio4) {
    if (ax < 7.450580596923828125e-09) return x;   // |x| < 2^-27
    return k_sin(x, 0.0, false);
  }
  double y[2];
  switch (rem_pio2(x, y) & 3) {
    case 0: return k_sin(y[0], y[1], true);
    case 1: return k_cos(y[0], y[1]);
    case 2: return -k_sin(y[0], y[1], true);
    default: return -k_cos(y[0], y[1]);
  }
}

double cos(double x) {
  if (!std::isfinite(x)) return std::numeric_limits<double>::quiet_NaN();
  const double ax = std::fabs(x);
  if (ax <= kPio4) {
    if (ax < 7.450580596923828125e-09) return 1.0;
    return k_cos(x, 0.0);
  }
  double y[2];
  switch (rem_pio2(x, y) & 3) {
    case 0: return k_cos(y[0], y[1]);
    case 1: return -k_sin(y[0], y[1], true);
    case 2: return -k_cos(y[0], y[1]);
    default: return k_sin(y[0], y[1], true);
  }
}

double atan(double x) {
  if (std::isnan(x)) return x;
  const bool neg = std::signbit(x);
  double ax = std::fabs(x);
  if (ax >= 7.378697629483820646e19) {   // |x| >= 2^66
    return neg ? -kAtanHi[3] : kAtanHi[3];
  }
  int id;
  if (ax < 0.4375) {
    if (ax < 7.450580596923828125e-09) return x;
    id = -1;
  } else if (ax < 1.1875) {
    if (ax < 0.6875) {
      id = 0;
      ax = (2.0 * ax - 1.0) / (2.0 + ax);
    } else {
      id = 1;
      ax = (ax - 1.0) / (ax + 1.0);
    }
  } else if (ax < 2.4375) {
    id = 2;
    ax = (ax - 1.5) / (1.0 + 1.5 * ax);
  } else {
    id = 3;
    ax = -1.0 / ax;
  }
  const double xr = (id < 0) ? x : ax;
  const double z = xr * xr;
  const double w = z * z;
  const double s1 = z * (kAT[0] + w * (kAT[2] + w * (kAT[4] + w * (kAT[6] + w * (kAT[8] + w * kAT[10])))));
  const double s2 = w * (kAT[1] + w * (kAT[3] + w * (kAT[5] + w * (kAT[7] + w * kAT[9]))));
  if (id < 0) return xr - xr * (s1 + s2);
  const double r = kAtanHi[id] - ((xr * (s1 + s2) - kAtanLo[id]) - xr);
  return neg ? -r : r;
}

double atan2(double y, double x) {
  if (std::isnan(x) || std::isnan(y)) return x + y;
  if (x == 1.0) return atan(y);
  const int m = (std::signbit(y) ? 1 : 0) | (std::signbit(x) ? 2 : 0);
  if (y == 0.0) {
    switch (m) {
      case 0:
      case 1: return y;
      case 2: return kPi;
      default: return -kPi;
    }
  }
  if (x == 0.0) return (m & 1) ? -kHalfPi : kHalfPi;
  if (std::isinf(x)) {
    if (std::isinf(y)) {
      switch (m) {
        case 0: return kPio4;
        case 1: return -kPio4;
        case 2: return 3.0 * kPio4;
        default: return -3.0 * kPio4;
      }
    }
    switch (m) {
      case 0: return 0.0;
      case 1: return -0.0;
      case 2: return kPi;
      default: return -kPi;
    }
  }
  if (std::isinf(y)) return (m & 1) ? -kHalfPi : kHalfPi;
  const int ey = exponent(y), ex = exponent(x);
  if (ey - ex > 64) return (m & 1) ? -kHalfPi : kHalfPi;   // |y/x| > 2^64
  double z;
  if ((m & 2) && ex - ey > 64) {
    z = 0.0;                                               // |y/x| < 2^-64, x < 0
  } else {
    z = atan(std::fabs(y / x));
  }
  switch (m) {
    case 0: return z;
    case 1: return -z;
    case 2: return kPi - (z - kPiLo);
    default: return (z - kPiLo) - kPi;
  }
}

double asin(double x) {
  if (!(x >= -1.0 && x <= 1.0)) return std::numeric_limits<double>::quiet_NaN();
  return atan2(x, std::sqrt((1.0 - x) * (1.0 + x)));
}

double acos(double x) {
  if (!(x >= -1.0 && x <= 1.0)) return std::numeric_limits<double>::quiet_NaN();
  return atan2(std::sqrt((1.0 - x) * (1.0 + x)), x);
}

double exp(double x) {
  if (std::isnan(x)) return x;
  if (x > 7.09782712893383973096e+02) return std::numeric_limits<double>::infinity();
  if (x < -7.45133219101941108420e+02) return 0.0;
  const double ax = std::fabs(x);
  double hi, lo;
  int k;
  if (ax > 0.34657359027997264) {              // |x| > 0.5 ln2
    if (ax >= 1.0397207708399179) {            // |x| >= 1.5 ln2
      k = static_cast<int>(kInvLn2 * x + (x < 0.0 ? -0.5 : 0.5));
    } else {
      k = x < 0.0 ? -1 : 1;
    }
    hi = x - k * kLn2Hi;
    lo = k * kLn2Lo;
    x = hi - lo;
  } else if (ax > 3.7252902984619140625e-09) { // |x| > 2^-28
    k = 0;
    hi = x;
    lo = 0.0;
  } else {
    return 1.0 + x;
  }
  const double xx = x * x;
  const double c = x - xx * (P1 + xx * (P2 + xx * (P3 + xx * (P4 + xx * P5))));
  const double y = 1.0 + (x * c / (2.0 - c) - lo + hi);
  if (k == 0) return y;
  return std::ldexp(y, k);
}

double log(double x) {
  if (std::isnan(x) || x < 0.0) return std::numeric_limits<double>::quiet_NaN();
  if (x == 0.0) return -std::numeric_limits<double>::infinity();
  if (std::isinf(x)) return x;
  if (x == 1.0) return 0.0;
  int k;
  double m = std::frexp(x, &k);                // x = m * 2^k, m in [0.5, 1)
  if (m < 0.70710678118654752440) {
    m *= 2.0;
    k -= 1;
  }
  const double f = m - 1.0;                    // m in [sqrt(2)/2, sqrt(2))
  const double hfsq = 0.5 * f * f;
  const double s = f / (2.0 + f);
  const double z = s * s;
  const double w = z * z;
  const double t1 = w * (Lg2 + w * (Lg4 + w * Lg6));
  const double t2 = z * (Lg1 + w * (Lg3 + w * (Lg5 + w * Lg7)));
  const double r = t2 + t1;
  const double dk = k;
  return s * (hfsq + r) + dk * kLn2Lo - hfsq + f + dk * kLn2Hi;
}

double pow(double x, double y) {
  if (y == 0.0) return 1.0;
  if (x == 0.0) return 0.0;
  if (y == 1.0) return x;
  if (y == 2.0) return x * x;
  if (y == 0.5) return std::sqrt(x);
  if (y == 1.5) return x * std::sqrt(x);
  return exp(y * log(x));
}

double erf(double x) {
  if (std::isnan(x)) return x;
  const double ax = std::fabs(x);
  if (ax >= 6.0) return x < 0.0 ? -1.0 : 1.0;
  double r;
  if (ax < 3.0) {
    // erf(x) = 2/sqrt(pi) * exp(-x^2) * sum_{n>=0} 2^n x^(2n+1) / (1*3*...*(2n+1)): all terms
    // positive, so no cancellation.
    double term = ax, sum = ax;
    const double x2 = ax * ax;
    for (int n = 1; n < 200; ++n) {
      term *= 2.0 * x2 / (2.0 * n + 1.0);
      sum += term;
      if (term < sum * 1e-17) break;
    }
    r = 1.12837916709551257390 * exp(-x2) * sum;
  } else {
    // erfc by the continued fraction x/(x^2 + 1/2 /(1 + 1/(x^2 + 3/2 /(1 + ...)))), bottom up.
    const double x2 = ax * ax;
    double f = 0.0;
    for (int n = 60; n >= 1; --n) f = (0.5 * n) / (ax + f);
    (void)x2;
    r = 1.0 - exp(-ax * ax) / ((ax + f) * 1.77245385090551602730);
  }
  return x < 0.0 ? -r : r;
}

}  // namespace onze::dmath