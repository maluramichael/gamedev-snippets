// Gamma and linear light: the logic of the demo, no SDL, so test.cpp can check it.
//
// A pixel value in a PNG or on screen is an sRGB code value s in 0..1. It is NOT an amount of light: the code was bent
// by a curve so the 256 steps spread evenly over what the eye sees. Light adds up in linear units l, so every
// calculation that means light (mixing two lamps, a lighting term, a blur) belongs in linear space:
//
//     decode(s) -> l     (sRGB code to linear light)         encode(l) -> s     (linear light to sRGB code)
//
// The curve is the exact piecewise sRGB function (a short straight piece near black, then a power of 2.4 with an
// offset), not the common shortcut pow(x, 2.2).
//
// Pipeline lists the four ways the demo lights a textured sphere with the same light L (linear units):
//   Naive         out = a * L                    everything in code values, no curve at all
//   Correct       out = enc(dec(a) * L)          decode the texture, light in linear space, encode once at the end
//   RawAlbedo     out = enc(a * L)               the texture was never decoded, only the output is corrected
//   EncodedTwice  out = enc(enc(dec(a) * L))     the output is corrected two times (e.g. by hand and by the framebuffer)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gam {

struct Rgb {
    double r, g, b;
};

struct Vec3 {
    double x, y, z;
};

enum class Pipeline { Naive, Correct, RawAlbedo, EncodedTwice };

/// Clamps to 0..1; NaN becomes 0.
inline double clamp01(double x) {
    if (!(x > 0.0)) return 0.0;
    return x > 1.0 ? 1.0 : x;
}

/// sRGB code value to linear light. Input is clamped to 0..1.
inline double decode(double s) {
    s = clamp01(s);
    return s <= 0.04045 ? s / 12.92 : std::pow((s + 0.055) / 1.055, 2.4);
}

/// Linear light to sRGB code value. Input is clamped to 0..1.
inline double encode(double l) {
    l = clamp01(l);
    if (l >= 1.0) return 1.0;  // 1.055 - 0.055 is not exactly 1 in floating point
    return clamp01(l <= 0.0031308 ? l * 12.92 : 1.055 * std::pow(l, 1.0 / 2.4) - 0.055);
}

inline Rgb decode(Rgb c) { return {decode(c.r), decode(c.g), decode(c.b)}; }
inline Rgb encode(Rgb c) { return {encode(c.r), encode(c.g), encode(c.b)}; }

/// 0..1 to 0..255, rounded to nearest.
inline std::uint8_t toByte(double s) { return static_cast<std::uint8_t>(std::lround(clamp01(s) * 255.0)); }

/// The mistake: interpolate the code values as if they were light. t is clamped to 0..1.
inline Rgb mixSrgb(Rgb a, Rgb b, double t) {
    t = clamp01(t);
    return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t};
}

/// Interpolate the light, then encode the result. t is clamped to 0..1.
inline Rgb mixLinear(Rgb a, Rgb b, double t) {
    t = clamp01(t);
    const Rgb la = decode(a);
    const Rgb lb = decode(b);
    return encode(Rgb{la.r + (lb.r - la.r) * t, la.g + (lb.g - la.g) * t, la.b + (lb.b - la.b) * t});
}

/// Relative luminance (Rec. 709 weights) of a LINEAR color: how much light it really carries, 0..1.
inline double luminance(Rgb linear) { return 0.2126 * linear.r + 0.7152 * linear.g + 0.0722 * linear.b; }

/// One channel of a texture value a (sRGB code, 0..1) lit by L (linear units, may be above 1) through one pipeline.
/// The result is the code value that goes to the screen, clamped to 0..1.
inline double shade(Pipeline p, double a, double light) {
    a = clamp01(a);
    light = std::max(0.0, light);
    switch (p) {
        case Pipeline::Naive: return clamp01(a * light);
        case Pipeline::Correct: return encode(decode(a) * light);
        case Pipeline::RawAlbedo: return encode(a * light);
        case Pipeline::EncodedTwice: return encode(encode(decode(a) * light));
    }
    return 0.0;
}

inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/// Unit vector, (0, 0, 1) for a vector without length.
inline Vec3 normalize(Vec3 v) {
    const double len = std::sqrt(dot(v, v));
    if (len < 1e-12) return {0.0, 0.0, 1.0};
    return {v.x / len, v.y / len, v.z / len};
}

/// Diffuse light in linear units for unit vectors n (surface) and l (towards the light), plus a constant ambient.
inline double lambert(Vec3 n, Vec3 l, double ambient) {
    ambient = clamp01(ambient);
    return ambient + (1.0 - ambient) * std::max(0.0, dot(n, l));
}

/// Normal of a sphere with the given radius at (px, py) from its center, y pointing up, seen from +z.
/// False outside the sphere (and for a radius of 0 or less); the rim itself still counts as inside.
inline bool sphereNormal(double px, double py, double radius, Vec3& n) {
    if (!(radius > 0.0)) return false;
    const double r2 = px * px + py * py;
    const double radius2 = radius * radius;
    if (r2 > radius2) return false;
    n = {px / radius, py / radius, std::sqrt(std::max(0.0, 1.0 - r2 / radius2))};
    return true;
}

/// Anti-aliasing at the rim: 1 well inside, 0 well outside, 0.5 exactly on the circle.
inline double coverage(double dist, double radius) { return clamp01(radius + 0.5 - dist); }

/// Checker pattern on a sphere by longitude and latitude, `cells` cells per half turn (30 degrees at 6).
/// True for the even cells. Without cells (0 or less) the surface is plain.
inline bool checkerCell(Vec3 n, int cells) {
    if (cells <= 0) return true;
    constexpr double kPi = 3.14159265358979323846;
    const double step = kPi / static_cast<double>(std::min(cells, 1024));
    const double u = std::atan2(n.x, n.z);
    const double v = std::asin(std::clamp(n.y, -1.0, 1.0));
    const int iu = static_cast<int>(std::floor(u / step));
    const int iv = static_cast<int>(std::floor(v / step));
    return (iu + iv) % 2 == 0;
}

/// Light direction (unit, always in front of the sphere) from a mouse position on the 960x540 canvas.
inline Vec3 lightFromMouse(double mx, double my) { return normalize({(mx - 480.0) / 300.0, -(my - 340.0) / 180.0, 0.6}); }

/// Position 0..1 of x on a bar that starts at x0 and is `width` wide; 0 for a bar without width.
inline double sliderT(double x, double x0, double width) {
    if (!(width > 0.0)) return 0.0;
    return clamp01((x - x0) / width);
}

}  // namespace gam
