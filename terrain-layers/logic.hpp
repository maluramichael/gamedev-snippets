// Logic of the terrain-layers demo, no SDL: gradient noise, fBm, three terrain layers, the four ways to blend them,
// the mask, relief statistics, hillshade and colors. Everything that can be wrong lives here and is covered by test.cpp.
//
// The noise is written from scratch for this demo (nothing is shared with the other demos): Perlin gradient noise on an
// integer hash, summed up as fBm.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace tl {

inline constexpr float kSea = 0.30F;          // heights below this are water
inline constexpr float kViewSpan = 3.0F;      // world units across the map
inline constexpr float kMaxPan = 200.0F;      // keeps noise coordinates small enough for float precision and the int cast
inline constexpr std::uint32_t kDefaultSeed = 3;
inline constexpr float kDefaultThreshold = 0.68F;  // where the mask is half on, in continent value
inline constexpr float kDefaultSoftness = 0.12F;   // half width of the mask's ramp

// Blend constants. Layers c (continent), m (mountains) and d (detail) all live in [0, 1].
inline constexpr float kBase = 0.10F;            // height of the sea floor under a continent value of 0
inline constexpr float kContinentWeight = 0.5F;  // base = kBase + kContinentWeight * c
inline constexpr float kMountainWeight = 0.4F;   // the most the mountain layer can add
inline constexpr float kDetailAmp = 0.08F;       // detail moves the height by at most +- kDetailAmp / 2
inline constexpr float kMultBase = 0.8F;         // multiply: base * (kMultBase + kMultGain * m)
inline constexpr float kMultGain = 0.8F;
inline constexpr float kPlainsBase = 0.26F;      // lerp: the flat terrain of the lowlands
inline constexpr float kPlainsGain = 0.25F;

inline float clamp01(float v) { return std::clamp(v, 0.0F, 1.0F); }
inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

/// 0 below e0, 1 above e1, smooth S-curve in between. Zero or negative width is a hard step.
inline float smoothstep(float e0, float e1, float x) {
    if (!(e1 > e0)) return x >= e0 ? 1.0F : 0.0F;
    const float t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3.0F - 2.0F * t);
}

/// Perlin's fade curve 6t^5 - 15t^4 + 10t^3: slope 0 at both ends, so neighbouring lattice cells join without a seam.
inline float fade(float t) { return t * t * t * (t * (t * 6.0F - 15.0F) + 10.0F); }

inline std::uint32_t hash(std::int32_t x, std::int32_t y, std::uint32_t seed) {
    std::uint32_t h = seed * 0x9E3779B1u;
    h ^= static_cast<std::uint32_t>(x) * 0x85EBCA6Bu;
    h = ((h << 13) | (h >> 19)) * 5u + 0xE6546B64u;
    h ^= static_cast<std::uint32_t>(y) * 0xC2B2AE35u;
    h ^= h >> 16;
    h *= 0x85EBCA6Bu;
    h ^= h >> 13;
    h *= 0xC2B2AE35u;
    h ^= h >> 16;
    return h;
}

/// Dot product of (dx, dy) with one of 8 unit directions picked by the low three bits of the hash.
inline float gradientDot(std::uint32_t h, float dx, float dy) {
    constexpr float k = 0.70710678F;
    switch (h & 7u) {
        case 0: return dx;
        case 1: return -dx;
        case 2: return dy;
        case 3: return -dy;
        case 4: return (dx + dy) * k;
        case 5: return (dx - dy) * k;
        case 6: return (-dx + dy) * k;
        default: return (-dx - dy) * k;
    }
}

/// Classic 2D gradient noise. Zero on every lattice point, bounded by +-sqrt(2)/2 (unit gradients).
inline float perlin(float x, float y, std::uint32_t seed) {
    if (!std::isfinite(x) || !std::isfinite(y)) return 0.0F;
    x = std::clamp(x, -1.0e6F, 1.0e6F);  // the int cast below is only defined inside the int range
    y = std::clamp(y, -1.0e6F, 1.0e6F);
    const float fx = std::floor(x);
    const float fy = std::floor(y);
    const auto ix = static_cast<std::int32_t>(fx);
    const auto iy = static_cast<std::int32_t>(fy);
    const float tx = x - fx;
    const float ty = y - fy;
    const float n00 = gradientDot(hash(ix, iy, seed), tx, ty);
    const float n10 = gradientDot(hash(ix + 1, iy, seed), tx - 1.0F, ty);
    const float n01 = gradientDot(hash(ix, iy + 1, seed), tx, ty - 1.0F);
    const float n11 = gradientDot(hash(ix + 1, iy + 1, seed), tx - 1.0F, ty - 1.0F);
    const float u = fade(tx);
    const float v = fade(ty);
    return lerp(lerp(n00, n10, u), lerp(n01, n11, u), v);
}

/// Perlin noise moved into [0, 1] (0.5 is the middle, and the value on every lattice point).
inline float noise01(float x, float y, std::uint32_t seed) { return clamp01(0.5F + perlin(x, y, seed) * 0.70710678F); }

/// Stretches the values around 0.5, then clamps. Perlin noise rarely gets near 0 or 1, so layers would stay grey without it.
inline float contrast(float v, float k) { return clamp01(0.5F + (v - 0.5F) * k); }

/// Sum of octaves. `noise(x, y, octave)` returns a value in [0, 1]; each octave doubles the frequency (lacunarity) and
/// scales the amplitude (gain). The sum is divided by the sum of the amplitudes, so the result stays in [0, 1].
template <class Noise>
float fbm(Noise&& noise, float x, float y, int octaves, float lacunarity, float gain) {
    float sum = 0.0F;
    float norm = 0.0F;
    float amp = 1.0F;
    float freq = 1.0F;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * noise(x * freq, y * freq, o);
        norm += amp;
        amp *= gain;
        freq *= lacunarity;
    }
    return norm > 0.0F ? sum / norm : 0.5F;
}

/// 1 on the crest (noise = 0.5), falling to 0 where the noise has moved 1 / stretch away from the middle.
inline float ridge(float n, float stretch) { return 1.0F - clamp01(std::fabs(2.0F * n - 1.0F) * stretch); }

/// fBm of squared ridges: sharp crests along the zero crossings of the noise, which look like mountain ranges.
template <class Noise>
float ridged(Noise&& noise, float x, float y, int octaves, float lacunarity, float gain, float stretch) {
    float sum = 0.0F;
    float norm = 0.0F;
    float amp = 1.0F;
    float freq = 1.0F;
    for (int o = 0; o < octaves; ++o) {
        const float r = ridge(noise(x * freq, y * freq, o), stretch);
        sum += amp * r * r;
        norm += amp;
        amp *= gain;
        freq *= lacunarity;
    }
    return norm > 0.0F ? sum / norm : 0.0F;
}

/// One noise per octave, moved so that the lattice points (where every octave is 0.5) do not line up at the origin.
inline float octaveNoise(float x, float y, int octave, std::uint32_t seed) {
    const auto o = static_cast<float>(octave);
    return noise01(x + 31.7F * o, y - 17.3F * o, seed + 0x9E37u * static_cast<std::uint32_t>(octave));
}

inline float clampPan(float v) { return std::isfinite(v) ? std::clamp(v, -kMaxPan, kMaxPan) : 0.0F; }

// The three layers, sampled in world units. Frequencies are features per world unit.
inline constexpr float kContinentFreq = 1.3F;
inline constexpr float kMountainFreq = 3.2F;
inline constexpr float kDetailFreq = 8.0F;

/// Slow and wide: where is land, where is sea.
inline float continentAt(float wx, float wy, std::uint32_t seed) {
    const auto n = [seed](float x, float y, int o) { return octaveNoise(x, y, o, seed + 101u); };
    return contrast(fbm(n, wx * kContinentFreq, wy * kContinentFreq, 2, 2.0F, 0.4F), 2.4F);
}

/// Ridged: thin crest lines with steep flanks.
inline float mountainsAt(float wx, float wy, std::uint32_t seed) {
    const auto n = [seed](float x, float y, int o) { return octaveNoise(x, y, o, seed + 202u); };
    return ridged(n, wx * kMountainFreq, wy * kMountainFreq, 5, 2.0F, 0.5F, 2.2F);
}

/// Fast and small: the roughness of the ground.
inline float detailAt(float wx, float wy, std::uint32_t seed) {
    const auto n = [seed](float x, float y, int o) { return octaveNoise(x, y, o, seed + 303u); };
    return contrast(fbm(n, wx * kDetailFreq, wy * kDetailFreq, 3, 2.0F, 0.5F), 1.8F);
}

struct Layers {
    int w = 0;
    int h = 0;
    std::vector<float> c, m, d;
};

/// Samples the three layers on a w x h grid. The grid covers `span` world units horizontally, cells are square,
/// (ox, oy) is the world position of the top left corner.
inline void fillLayers(Layers& l, int w, int h, std::uint32_t seed, float ox, float oy, float span) {
    if (w <= 0 || h <= 0) {
        l = Layers{};
        return;
    }
    l.w = w;
    l.h = h;
    const auto count = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    l.c.resize(count);
    l.m.resize(count);
    l.d.resize(count);
    const float cell = span / static_cast<float>(w);
    for (int y = 0; y < h; ++y) {
        const float wy = oy + (static_cast<float>(y) + 0.5F) * cell;
        for (int x = 0; x < w; ++x) {
            const float wx = ox + (static_cast<float>(x) + 0.5F) * cell;
            const auto i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
            l.c[i] = continentAt(wx, wy, seed);
            l.m[i] = mountainsAt(wx, wy, seed);
            l.d[i] = detailAt(wx, wy, seed);
        }
    }
}

enum class Mode { Add, Multiply, Mask, Lerp };

/// 0 where the continent value is clearly below the threshold, 1 clearly above, 0.5 exactly on it.
inline float maskOf(float c, float threshold, float softness) { return smoothstep(threshold - softness, threshold + softness, c); }

/// Height from the three layer values.
///   Add:      the same relief everywhere (the naive blend)
///   Multiply: base times a mountain factor, so relief grows with the continent value (a soft mask without a threshold)
///   Mask:     the mountain layer is added only where the mask is on
///   Lerp:     fades between two finished terrains, flat plains and the full add
inline float blend(Mode mode, float c, float m, float d, float threshold, float softness) {
    const float detail = kDetailAmp * (d - 0.5F);
    const float base = kBase + kContinentWeight * c;
    const float mask = maskOf(c, threshold, softness);
    float h = 0.0F;
    switch (mode) {
        case Mode::Add: h = base + kMountainWeight * m; break;
        case Mode::Multiply: h = base * (kMultBase + kMultGain * m); break;
        case Mode::Mask: h = base + mask * kMountainWeight * m; break;
        case Mode::Lerp: h = lerp(kPlainsBase + kPlainsGain * c, base + kMountainWeight * m, mask); break;
    }
    return clamp01(h + detail);
}

struct Enabled {
    bool continent = true;
    bool mountains = true;
    bool detail = true;
};

struct Field {
    int w = 0;
    int h = 0;
    std::vector<float> height, mask, cont;  // cont = the continent value that was actually used (0.5 when the layer is off)
};

/// Blends the layers. A layer that is switched off counts as neutral: flat 0.5 for continent and detail, 0 for mountains.
inline void blendField(const Layers& l, Enabled on, Mode mode, float threshold, float softness, Field& f) {
    const auto count = static_cast<std::size_t>(std::max(l.w, 0)) * static_cast<std::size_t>(std::max(l.h, 0));
    if (count == 0 || l.c.size() != count || l.m.size() != count || l.d.size() != count) {
        f = Field{};
        return;
    }
    f.w = l.w;
    f.h = l.h;
    f.height.resize(count);
    f.mask.resize(count);
    f.cont.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const float c = on.continent ? l.c[i] : 0.5F;
        const float m = on.mountains ? l.m[i] : 0.0F;
        const float d = on.detail ? l.d[i] : 0.5F;
        f.cont[i] = c;
        f.mask[i] = maskOf(c, threshold, softness);
        f.height[i] = blend(mode, c, m, d, threshold, softness);
    }
}

/// Height difference per cell, central differences with the edge cell standing in for a missing neighbour.
inline void gradientAt(const Field& f, int x, int y, float& dx, float& dy) {
    const auto at = [&f](int xx, int yy) {
        xx = std::clamp(xx, 0, f.w - 1);
        yy = std::clamp(yy, 0, f.h - 1);
        return f.height[static_cast<std::size_t>(yy) * static_cast<std::size_t>(f.w) + static_cast<std::size_t>(xx)];
    };
    dx = (at(x + 1, y) - at(x - 1, y)) * 0.5F;
    dy = (at(x, y + 1) - at(x, y - 1)) * 0.5F;
}

/// Steepness in height per cell.
inline float slopeAt(const Field& f, int x, int y) {
    float dx = 0.0F;
    float dy = 0.0F;
    gradientAt(f, x, y, dx, dy);
    return std::sqrt(dx * dx + dy * dy);
}

struct Relief {
    double lowland = 0.0;   // mean slope on land where the mask is (almost) off
    double highland = 0.0;  // ... and where it is (almost) on
    int nLow = 0;
    int nHigh = 0;
    /// highland / lowland, 0 when either side has no land or the lowland is perfectly flat
    double ratio() const { return (nLow > 0 && nHigh > 0 && lowland > 1e-9) ? highland / lowland : 0.0; }
};

inline constexpr float kMaskOff = 0.05F;  // mask below this counts as lowland, above 1 - this as highland; the band between is the transition

/// Mean slope of the land, split by the mask: clearly off, clearly on, the soft transition in between is left out.
/// Interior cells only (the edges have no full neighbourhood).
inline Relief reliefStats(const Field& f) {
    Relief r;
    const auto count = static_cast<std::size_t>(std::max(f.w, 0)) * static_cast<std::size_t>(std::max(f.h, 0));
    if (f.w < 3 || f.h < 3 || f.height.size() != count || f.mask.size() != count) return r;
    double low = 0.0;
    double high = 0.0;
    for (int y = 1; y < f.h - 1; ++y) {
        for (int x = 1; x < f.w - 1; ++x) {
            const auto i = static_cast<std::size_t>(y) * static_cast<std::size_t>(f.w) + static_cast<std::size_t>(x);
            if (f.height[i] < kSea) continue;
            if (f.mask[i] < kMaskOff) {
                low += static_cast<double>(slopeAt(f, x, y));
                ++r.nLow;
            } else if (f.mask[i] > 1.0F - kMaskOff) {
                high += static_cast<double>(slopeAt(f, x, y));
                ++r.nHigh;
            }
        }
    }
    r.lowland = r.nLow > 0 ? low / r.nLow : 0.0;
    r.highland = r.nHigh > 0 ? high / r.nHigh : 0.0;
    return r;
}

/// Brightness factor for a surface with the given height gradient (per cell, already exaggerated). Light comes from the
/// upper left; flat ground is 1.0, a slope facing the light is brighter, one facing away is darker.
inline float shadeFromSlope(float dzdx, float dzdy) {
    constexpr float lx = -0.5F;
    constexpr float ly = -0.5F;
    constexpr float lz = 0.70710678F;
    const float len = std::sqrt(dzdx * dzdx + dzdy * dzdy + 1.0F);
    return ((-dzdx * lx - dzdy * ly + lz) / len) / lz;
}

inline float shadeAt(const Field& f, int x, int y, float exaggeration) {
    float dx = 0.0F;
    float dy = 0.0F;
    gradientAt(f, x, y, dx, dy);
    return shadeFromSlope(dx * exaggeration, dy * exaggeration);
}

struct Rgb {
    std::uint8_t r, g, b;
};

inline Rgb mix(Rgb a, Rgb b, float t) {
    const auto ch = [t](std::uint8_t p, std::uint8_t q) { return static_cast<std::uint8_t>(std::lround(lerp(static_cast<float>(p), static_cast<float>(q), t))); };
    return {ch(a.r, b.r), ch(a.g, b.g), ch(a.b, b.b)};
}

/// Hypsometric tint: deep to shallow water below the sea level, then sand, grass, forest, rock, snow.
inline Rgb terrainColor(float h) {
    h = clamp01(h);
    if (h < kSea) return mix(Rgb{22, 78, 124}, Rgb{104, 184, 200}, h / kSea);
    struct Stop {
        float h;
        Rgb c;
    };
    static constexpr Stop stops[] = {{kSea, {226, 212, 158}}, {0.33F, {196, 200, 130}}, {0.42F, {120, 170, 92}}, {0.56F, {74, 128, 74}},
                                     {0.70F, {138, 128, 100}}, {0.84F, {150, 146, 142}}, {0.96F, {250, 250, 250}}};
    constexpr std::size_t n = sizeof stops / sizeof stops[0];
    if (h >= stops[n - 1].h) return stops[n - 1].c;
    for (std::size_t i = 1; i < n; ++i) {
        if (h < stops[i].h) return mix(stops[i - 1].c, stops[i].c, (h - stops[i - 1].h) / (stops[i].h - stops[i - 1].h));
    }
    return stops[n - 1].c;
}

/// Plain grey height map, water in blue-grey.
inline Rgb heightColor(float h) {
    h = clamp01(h);
    const auto g = static_cast<std::uint8_t>(std::lround(h * 255.0F));
    if (h < kSea) return {static_cast<std::uint8_t>(g / 2), static_cast<std::uint8_t>(g / 2 + 40), static_cast<std::uint8_t>(g / 2 + 90)};
    return {g, g, g};
}

inline constexpr float kReliefFull = 0.06F;  // slope per cell that gets the full orange in the relief view

/// Paper to orange by slope (height per cell).
inline Rgb reliefColor(float slope) { return mix(Rgb{231, 229, 228}, Rgb{234, 88, 12}, clamp01(slope / kReliefFull)); }

struct Grid {
    int w = 0;
    int h = 0;
    std::vector<float> v;
};

/// Box average of factor x factor blocks. Rows and columns that do not fill a block are dropped.
inline Grid downsample(const std::vector<float>& src, int w, int h, int factor) {
    Grid g;
    if (factor < 1 || w < 1 || h < 1 || src.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) return g;
    g.w = w / factor;
    g.h = h / factor;
    if (g.w < 1 || g.h < 1) return g;  // no full block fits: sizes stay as computed, no data
    g.v.resize(static_cast<std::size_t>(g.w) * static_cast<std::size_t>(g.h));
    const float inv = 1.0F / static_cast<float>(factor * factor);
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            float sum = 0.0F;
            for (int j = 0; j < factor; ++j) {
                for (int i = 0; i < factor; ++i) sum += src[static_cast<std::size_t>(y * factor + j) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x * factor + i)];
            }
            g.v[static_cast<std::size_t>(y) * static_cast<std::size_t>(g.w) + static_cast<std::size_t>(x)] = sum * inv;
        }
    }
    return g;
}

/// Slider: mouse x on the track [x0, x1] to a value in [lo, hi] (clamped), and back.
inline float sliderValue(float mouseX, float x0, float x1, float lo, float hi) {
    if (!(x1 > x0)) return lo;
    return lerp(lo, hi, clamp01((mouseX - x0) / (x1 - x0)));
}

inline float sliderPos(float value, float x0, float x1, float lo, float hi) {
    if (!(hi > lo)) return x0;
    return lerp(x0, x1, clamp01((value - lo) / (hi - lo)));
}

}  // namespace tl
