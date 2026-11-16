// SPDX-License-Identifier: 0BSD
// Copyright (C) 2026 Jinyu Yang
//
// onze::dmath: our own transcendental functions.
//
// The system libm is not stable, even for one executable: C runtimes pick FMA or non-FMA code by
// CPU at run time, and an OS update can replace the library. All simulation code calls these
// functions instead. They use plain IEEE-754 double operations. With -ffp-contract=off they give
// the same bits on every run of one build.
//
// sin, cos, atan, atan2, exp and log come from fdlibm (Sun Microsystems), through musl libc.
// asin, acos, pow and erf are our own code and do not have fdlibm's error bounds. See dmath.cpp
// for the license notices and the changes.
//
// sqrt, fabs, floor, ceil, remainder, ldexp and frexp are exact under IEEE 754, so the std
// versions may stay.

#pragma once

#include <cmath>

namespace onze::dmath {

double sin(double x);
double cos(double x);
double atan(double x);
double atan2(double y, double x);
double asin(double x);
double acos(double x);
double exp(double x);
double log(double x);
double pow(double x, double y);   // x > 0, or x == 0 with y > 0
double erf(double x);

inline double sqrt(double x) { return std::sqrt(x); }
inline double hypot(double x, double y) { return std::sqrt(x * x + y * y); }
inline double hypot(double x, double y, double z) { return std::sqrt(x * x + y * y + z * z); }

// Angle to [-pi, pi] (IEEE remainder, exact).
inline double wrap(double a) { return std::remainder(a, 6.283185307179586476925286766559); }
inline double radians(double deg) { return deg * (3.14159265358979323846 / 180.0); }
inline double degrees(double rad) { return rad * (180.0 / 3.14159265358979323846); }

// Guarded tick form of spec 12 R12: ceil(60*x - 1e-9), in double.
inline int ticks_ceil(double seconds) { return static_cast<int>(std::ceil(60.0 * seconds - 1e-9)); }

}  // namespace onze::dmath