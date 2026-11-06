// Ordered dithering without SDL: Bayer threshold maps, quantizing against a small fixed palette in sRGB or in
// linear light, a procedurally generated test image and an error measure. main.cpp only draws the results.
//
// The idea in one line: a pixel does not pick its nearest palette color. It picks the lower or the upper neighbour,
// and the position inside the gap (0..1) is compared with a threshold that depends on the pixel's place in a tiled
// matrix. Over a tile the share of upper pixels then equals that position.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace od {

/// Color with channels in 0..1, gamma encoded (what a screen file holds), unless a function says linear.
struct Rgb {
    double r, g, b;
};

enum class Space { Srgb, Linear };
enum class Pattern { Off, Bayer2, Bayer4, Bayer8, Noise };

/// `levels` evenly spaced values per channel in encoded space (0, 1/(L-1), ..., 1). `gray` = one channel for all three.
struct Palette {
    int levels;
    bool gray;
};

struct Image {
    int w = 0;
    int h = 0;
    std::vector<Rgb> px;  // row-major, w * h entries
};

// ---- transfer curve -----------------------------------------------------------------------------------------------

inline double clamp01(double v) {
    if (!(v > 0.0)) return 0.0;  // also catches NaN
    return v > 1.0 ? 1.0 : v;
}

inline double srgbToLinear(double v) {
    v = clamp01(v);
    return v <= 0.04045 ? v / 12.92 : std::pow((v + 0.055) / 1.055, 2.4);
}

inline double linearToSrgb(double l) {
    l = clamp01(l);
    return l <= 0.0031308 ? l * 12.92 : 1.055 * std::pow(l, 1.0 / 2.4) - 0.055;
}

/// Linear luminance (Rec. 709 weights) of a gamma encoded color.
inline double luminance(Rgb c) { return 0.2126 * srgbToLinear(c.r) + 0.7152 * srgbToLinear(c.g) + 0.0722 * srgbToLinear(c.b); }

// ---- threshold maps -----------------------------------------------------------------------------------------------

/// Index 0..n*n-1 of the Bayer matrix at (x, y); the matrix tiles, negative coordinates wrap. n must be a power of two,
/// anything else returns 0. M(2n) = [4M, 4M+2; 4M+3, 4M+1], so the top level of the recursion is the lowest "digit".
inline int bayerIndex(int n, int x, int y) {
    if (n < 1 || (n & (n - 1)) != 0) return 0;
    x = ((x % n) + n) % n;
    y = ((y % n) + n) % n;
    int value = 0;
    int weight = 1;
    for (int size = n; size > 1; size /= 2) {
        const int half = size / 2;
        const bool right = x >= half;
        const bool lower = y >= half;
        value += (right ? (lower ? 1 : 2) : (lower ? 3 : 0)) * weight;
        weight *= 4;
        x %= half;
        y %= half;
    }
    return value;
}

inline int patternSize(Pattern p) {
    switch (p) {
        case Pattern::Bayer2: return 2;
        case Pattern::Bayer4: return 4;
        case Pattern::Bayer8: return 8;
        default: return 0;
    }
}

/// White noise that stays put: the same pixel always gets the same value, strictly between 0 and 1.
inline double noiseThreshold(int x, int y) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 0x9E3779B1U ^ static_cast<std::uint32_t>(y) * 0x85EBCA77U;
    h ^= h >> 16;
    h *= 0x7FEB352DU;
    h ^= h >> 15;
    h *= 0x846CA68BU;
    h ^= h >> 16;
    return (static_cast<double>(h) + 0.5) / 4294967296.0;
}

/// Threshold in (0, 1) for a pixel. Bayer cells are centered, (index + 0.5) / n^2, so a position of k / n^2 beats
/// exactly k cells of a tile. Off is a constant 0.5, which is plain rounding.
inline double threshold(Pattern p, int x, int y) {
    const int n = patternSize(p);
    if (n > 0) return (static_cast<double>(bayerIndex(n, x, y)) + 0.5) / static_cast<double>(n * n);
    if (p == Pattern::Noise) return noiseThreshold(x, y);
    return 0.5;
}

// ---- quantizing ---------------------------------------------------------------------------------------------------

/// One channel (v encoded, 0..1) against `levels` palette entries. The lower and upper neighbour are found in encoded
/// space (the palette is fixed), the position between them is measured in `space`. Returns an encoded palette value.
/// In linear space the share of upper pixels matches the *light* of the original, in sRGB space it matches the code value.
inline double quantizeChannel(double v, int levels, double th, Space space) {
    if (levels < 2) return 0.0;
    v = clamp01(v);
    const int steps = levels - 1;
    const int k = std::min(static_cast<int>(std::floor(v * static_cast<double>(steps))), steps - 1);
    const double loEnc = static_cast<double>(k) / static_cast<double>(steps);
    const double hiEnc = static_cast<double>(k + 1) / static_cast<double>(steps);
    const double x = space == Space::Linear ? srgbToLinear(v) : v;
    const double lo = space == Space::Linear ? srgbToLinear(loEnc) : loEnc;
    const double hi = space == Space::Linear ? srgbToLinear(hiEnc) : hiEnc;
    const double gap = hi - lo;
    const double t = gap > 0.0 ? (x - lo) / gap : 0.0;
    return t > th ? hiEnc : loEnc;
}

inline Rgb ditherPixel(Rgb c, Palette palette, Pattern pattern, Space space, int x, int y) {
    const double th = threshold(pattern, x, y);
    if (palette.gray) {
        const double q = quantizeChannel(linearToSrgb(luminance(c)), palette.levels, th, space);
        return {q, q, q};
    }
    return {quantizeChannel(c.r, palette.levels, th, space), quantizeChannel(c.g, palette.levels, th, space),
            quantizeChannel(c.b, palette.levels, th, space)};
}

inline Image dither(const Image& src, Palette palette, Pattern pattern, Space space) {
    Image out;
    out.w = src.w;
    out.h = src.h;
    out.px.resize(src.px.size());
    for (int y = 0; y < src.h; ++y) {
        for (int x = 0; x < src.w; ++x) {
            const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(src.w) + static_cast<std::size_t>(x);
            if (i < src.px.size()) out.px[i] = ditherPixel(src.px[i], palette, pattern, space, x, y);
        }
    }
    return out;
}

inline int paletteSize(Palette p) {
    if (p.levels < 1) return 0;
    return p.gray ? p.levels : p.levels * p.levels * p.levels;
}

/// Distinct tones per channel the combination can show: (levels - 1) * n^2 + 1 for a Bayer map, -1 for noise.
inline int tones(Pattern p, int levels) {
    if (levels < 2) return 1;
    if (p == Pattern::Noise) return -1;
    const int n = patternSize(p);
    return n == 0 ? levels : (levels - 1) * n * n + 1;
}

// ---- test image ---------------------------------------------------------------------------------------------------

inline Rgb hsvToRgb(double hue, double s, double v) {
    double h = std::fmod(hue, 360.0);
    if (h < 0.0) h += 360.0;
    const double c = v * s;
    const double sector = h / 60.0;
    const double x = c * (1.0 - std::fabs(std::fmod(sector, 2.0) - 1.0));
    const double m = v - c;
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    switch (static_cast<int>(sector)) {
        case 0: r = c; g = x; break;
        case 1: r = x; g = c; break;
        case 2: g = c; b = x; break;
        case 3: g = x; b = c; break;
        case 4: r = x; b = c; break;
        default: r = c; b = x; break;
    }
    return {r + m, g + m, b + m};
}

/// Top fifth: a gray ramp (the clearest place to see the sRGB mistake). Below, left: a lit sphere on a gradient, shaded
/// in linear light like a renderer would. Right: a hue chart that fades towards dark.
inline Image makeTestImage(int w, int h) {
    Image img;
    if (w <= 0 || h <= 0) return img;
    img.w = w;
    img.h = h;
    img.px.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

    const int band = h / 5;
    const double sceneH = static_cast<double>(h - band);
    const double split = 0.62 * static_cast<double>(w);
    const double cx = split / 2.0;
    const double cy = static_cast<double>(band) + sceneH / 2.0;
    const double radius = 0.42 * std::min(split, sceneH);
    const double lx = -0.45;
    const double ly = -0.55;
    const double lz = 0.70;
    const double lnorm = std::sqrt(lx * lx + ly * ly + lz * lz);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            Rgb c{};
            if (y < band) {
                const double v = static_cast<double>(x) / static_cast<double>(std::max(1, w - 1));
                c = {v, v, v};
            } else {
                const double sy = (static_cast<double>(y - band) + 0.5) / sceneH;
                if (static_cast<double>(x) < split) {
                    const double bg = 0.62 - 0.42 * sy;
                    c = {bg, bg, bg};
                    const double dx = (static_cast<double>(x) + 0.5 - cx) / radius;
                    const double dy = (static_cast<double>(y) + 0.5 - cy) / radius;
                    const double d2 = dx * dx + dy * dy;
                    if (d2 < 1.0) {
                        const double nz = std::sqrt(1.0 - d2);
                        const double diffuse = std::max(0.0, (dx * lx + dy * ly + nz * lz) / lnorm);
                        const double light = 0.05 + 0.95 * diffuse;
                        // Blinn highlight: half vector between the light and the viewer (0, 0, 1).
                        const double hx = lx / lnorm;
                        const double hy = ly / lnorm;
                        const double hz = lz / lnorm + 1.0;
                        const double hlen = std::sqrt(hx * hx + hy * hy + hz * hz);
                        const double spec = 0.5 * std::pow(std::max(0.0, (dx * hx + dy * hy + nz * hz) / hlen), 48.0);
                        c = {linearToSrgb(1.0 * light + spec), linearToSrgb(0.62 * light + spec), linearToSrgb(0.30 * light + spec)};
                    }
                } else {
                    const double u = (static_cast<double>(x) - split) / std::max(1.0, static_cast<double>(w) - split);
                    c = hsvToRgb(330.0 * u, 0.75, 1.0 - 0.6 * sy);
                }
            }
            img.px[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = c;
        }
    }
    return img;
}

// ---- output -------------------------------------------------------------------------------------------------------

inline std::uint8_t toByte(double v) { return static_cast<std::uint8_t>(std::lround(clamp01(v) * 255.0)); }

/// Writes RGBA bytes row by row, `pitch` is the distance between row starts in bytes (padding stays untouched). A pitch
/// too small for a row writes nothing.
inline void toRgba(const Image& img, std::uint8_t* dst, int pitch) {
    if (dst == nullptr || img.w <= 0 || img.h <= 0 || pitch < img.w * 4) return;
    if (img.px.size() < static_cast<std::size_t>(img.w) * static_cast<std::size_t>(img.h)) return;
    for (int y = 0; y < img.h; ++y) {
        std::uint8_t* row = dst + static_cast<std::size_t>(y) * static_cast<std::size_t>(pitch);
        for (int x = 0; x < img.w; ++x) {
            const Rgb& p = img.px[static_cast<std::size_t>(y) * static_cast<std::size_t>(img.w) + static_cast<std::size_t>(x)];
            std::uint8_t* out = row + static_cast<std::size_t>(x) * 4U;
            out[0] = toByte(p.r);
            out[1] = toByte(p.g);
            out[2] = toByte(p.b);
            out[3] = 255;
        }
    }
}

// ---- error --------------------------------------------------------------------------------------------------------

/// Mean absolute difference of the block-averaged linear luminance, blocks of block x block pixels (full blocks only).
/// That is what the eye does when it stands back: it averages. 0 when there is nothing to compare.
inline double blockError(const Image& a, const Image& b, int block) {
    if (block < 1 || a.w != b.w || a.h != b.h || a.w <= 0 || a.h <= 0) return 0.0;
    const std::size_t count = static_cast<std::size_t>(a.w) * static_cast<std::size_t>(a.h);
    if (a.px.size() < count || b.px.size() < count) return 0.0;
    const int bw = a.w / block;
    const int bh = a.h / block;
    if (bw == 0 || bh == 0) return 0.0;
    double sum = 0.0;
    for (int by = 0; by < bh; ++by) {
        for (int bx = 0; bx < bw; ++bx) {
            double la = 0.0;
            double lb = 0.0;
            for (int y = by * block; y < (by + 1) * block; ++y) {
                for (int x = bx * block; x < (bx + 1) * block; ++x) {
                    const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(a.w) + static_cast<std::size_t>(x);
                    la += luminance(a.px[i]);
                    lb += luminance(b.px[i]);
                }
            }
            sum += std::fabs(la - lb) / static_cast<double>(block * block);
        }
    }
    return sum / static_cast<double>(bw * bh);
}

}  // namespace od
