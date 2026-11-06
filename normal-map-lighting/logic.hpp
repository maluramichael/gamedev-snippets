// Normal map lighting: the logic of the demo, no SDL, so test.cpp can check it.
//
// A normal map stores, per texel, the direction the surface faces at that spot. Lighting a flat picture with it needs
// nothing but a dot product per pixel: how much does the surface at this pixel face the light (N.L)?
//
// Conventions (the part that goes wrong in real projects):
//   * Tangent space: +x to the right, +y UP, +z out of the surface towards the viewer (OpenGL style, "green = up").
//     Image rows run downwards, so up is the direction of SMALLER image y. DirectX-style maps have the green channel
//     the other way round; feeding one into an OpenGL-style shader turns every bump into a dent.
//   * Bytes store n * 0.5 + 0.5, so the flat normal (0,0,1) is (128,128,255), the light blue of every normal map.
//   * Interpolating two normals gives a vector that is shorter than 1. It has to be normalized again before it is used,
//     or the light dims exactly where the normals change fast (edges of bumps, low resolution maps).
//   * The viewer looks straight down at the sprite: V = (0,0,1). The light is a point light above the sprite.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace nm {

struct Vec3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float length(Vec3 v) { return std::sqrt(dot(v, v)); }

/// Unit vector in the direction of v. A (nearly) zero vector has no direction: it becomes "straight out of the surface".
inline Vec3 normalized(Vec3 v) {
    const float len = length(v);
    if (len < 1e-6F) return {0.0F, 0.0F, 1.0F};
    return {v.x / len, v.y / len, v.z / len};
}

// ---------------------------------------------------------------------------------------------------------------------
// Normal map texels

/// Byte -> [-1, 1].
inline float decodeChannel(std::uint8_t b) { return static_cast<float>(b) / 255.0F * 2.0F - 1.0F; }

/// [-1, 1] -> byte, rounded to the nearest of the 256 steps, values outside the range are clamped.
inline std::uint8_t encodeChannel(float c) {
    const float t = (std::clamp(c, -1.0F, 1.0F) * 0.5F + 0.5F) * 255.0F + 0.5F;  // in [0.5, 255.5], so the cast is safe
    return static_cast<std::uint8_t>(t);
}

inline Vec3 decodeNormal(const std::uint8_t* rgb) { return {decodeChannel(rgb[0]), decodeChannel(rgb[1]), decodeChannel(rgb[2])}; }

inline void encodeNormal(Vec3 n, std::uint8_t* rgb) {
    rgb[0] = encodeChannel(n.x);
    rgb[1] = encodeChannel(n.y);
    rgb[2] = encodeChannel(n.z);
}

/// Square RGB texture, row by row, no padding.
struct NormalMap {
    int size = 0;
    std::vector<std::uint8_t> rgb;
};

/// Bakes a normal map from a height function height(u, v) with u, v in [0,1] and the height in units of the tile width.
/// The slope is a central difference over one texel, so a bigger map sees finer detail and the same height function
/// gives the same slopes at every size. Rows run downwards: height rising towards the bottom of the image means the
/// surface faces up, which is +y.
template <class HeightFn>
NormalMap bakeNormalMap(int size, HeightFn&& heightFn) {
    NormalMap map;
    if (size <= 0) return map;
    map.size = size;
    map.rgb.assign(static_cast<std::size_t>(size) * static_cast<std::size_t>(size) * 3U, 0);
    const float e = 1.0F / static_cast<float>(size);  // one texel in uv units
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const float u = (static_cast<float>(x) + 0.5F) * e;
            const float v = (static_cast<float>(y) + 0.5F) * e;
            const float dhdx = (heightFn(u + e, v) - heightFn(u - e, v)) / (2.0F * e);
            const float dhdy = (heightFn(u, v + e) - heightFn(u, v - e)) / (2.0F * e);
            const Vec3 n = normalized({-dhdx, dhdy, 1.0F});
            encodeNormal(n, &map.rgb[(static_cast<std::size_t>(y) * static_cast<std::size_t>(size) + static_cast<std::size_t>(x)) * 3U]);
        }
    }
    return map;
}

/// Bilinear lookup at (u, v) in [0,1] (texel centres at (i + 0.5) / size, the edge texel repeats outside).
/// `flipGreen` reads the map as if its green channel pointed the other way. `renormalize` stretches the interpolated
/// vector back to length 1; without it the result is shorter than 1 wherever neighbouring texels differ.
inline Vec3 sampleNormal(const NormalMap& map, float u, float v, bool flipGreen, bool renormalize) {
    const std::size_t n = static_cast<std::size_t>(std::max(map.size, 0));
    if (n == 0 || map.rgb.size() < n * n * 3U) return {0.0F, 0.0F, 1.0F};
    const float last = static_cast<float>(n - 1);
    const float fx = std::clamp(u * static_cast<float>(n) - 0.5F, 0.0F, last);  // clamped first: the casts below are safe
    const float fy = std::clamp(v * static_cast<float>(n) - 0.5F, 0.0F, last);
    const auto x0 = static_cast<std::size_t>(fx);
    const auto y0 = static_cast<std::size_t>(fy);
    const std::size_t x1 = std::min(x0 + 1, n - 1);
    const std::size_t y1 = std::min(y0 + 1, n - 1);
    const float tx = fx - static_cast<float>(x0);
    const float ty = fy - static_cast<float>(y0);
    const auto texel = [&](std::size_t x, std::size_t y) { return decodeNormal(&map.rgb[(y * n + x) * 3U]); };
    const Vec3 a = texel(x0, y0);
    const Vec3 b = texel(x1, y0);
    const Vec3 c = texel(x0, y1);
    const Vec3 d = texel(x1, y1);
    const auto mix = [](float p, float q, float t) { return p + (q - p) * t; };
    Vec3 out{mix(mix(a.x, b.x, tx), mix(c.x, d.x, tx), ty), mix(mix(a.y, b.y, tx), mix(c.y, d.y, tx), ty),
             mix(mix(a.z, b.z, tx), mix(c.z, d.z, tx), ty)};
    if (flipGreen) out.y = -out.y;
    return renormalize ? normalized(out) : out;
}

// ---------------------------------------------------------------------------------------------------------------------
// Light

/// Unit vector from the pixel (px, py) to the point light. x and y are in pixels of the lit picture with y running down,
/// z is the height of the light above the picture in the same unit. The screen y is flipped because +y is up in tangent space.
inline Vec3 lightDirection(Vec3 light, float px, float py) { return normalized({light.x - px, -(light.y - py), light.z}); }

/// base^exp for a whole exponent, by repeated squaring. Cheaper than std::pow for 40000 pixels per frame.
inline float powInt(float base, int exp) {
    float result = 1.0F;
    while (exp > 0) {
        if ((exp & 1) != 0) result *= base;
        base *= base;
        exp >>= 1;
    }
    return result;
}

struct Lit {
    Vec3 n;
    Vec3 l;
    float diffuse = 0.0F;   // N.L, clamped at 0
    float specular = 0.0F;  // (N.H)^shininess, 0 when the light is behind the surface
};

/// Blinn-Phong. `n` is used exactly as given: an unnormalized normal dims the result, which is the point of the
/// "renormalize" switch. `l` must be a unit vector pointing at the light.
inline Lit lightPixel(Vec3 n, Vec3 l, int shininess) {
    Lit out;
    out.n = n;
    out.l = l;
    out.diffuse = std::max(0.0F, dot(n, l));
    if (out.diffuse > 0.0F) {
        const Vec3 h = normalized({l.x, l.y, l.z + 1.0F});  // half vector between light and viewer (0,0,1)
        out.specular = powInt(std::max(0.0F, dot(n, h)), shininess);
    }
    return out;
}

/// One colour channel of the lit pixel: albedo times (ambient + diffuse), plus the white highlight, clamped to [0,1].
inline float compose(float albedo, float diffuse, float specular, float ambient, float specularStrength) {
    return std::clamp(albedo * (ambient + (1.0F - ambient) * diffuse) + specularStrength * specular, 0.0F, 1.0F);
}

inline std::uint8_t toByte(float c) { return static_cast<std::uint8_t>(std::clamp(c, 0.0F, 1.0F) * 255.0F + 0.5F); }

// ---------------------------------------------------------------------------------------------------------------------
// The sprite: a stone tile with a rim, a domed boss, a groove and four rivets. Everything is built from a height
// function (in units of the tile width), the normal map is baked from it, the colours are flat paint.

inline constexpr float kPlateHeight = 0.05F;
inline constexpr float kRimWidth = 0.045F;
inline constexpr float kDomeHeight = 0.11F;
inline constexpr float kDomeRadius = 0.21F;
inline constexpr float kGrooveRadius = 0.31F;
inline constexpr float kGrooveHalfWidth = 0.028F;
inline constexpr float kGrooveDepth = 0.035F;
inline constexpr float kRivetHeight = 0.035F;
inline constexpr float kRivetRadius = 0.055F;
inline constexpr float kGrainHeight = 0.0025F;
inline constexpr float kGrainFrequency = 64.0F;  // noise cells per tile

inline constexpr float kPi = 3.14159265F;

inline float smoothstep01(float t) {
    const float c = std::clamp(t, 0.0F, 1.0F);
    return c * c * (3.0F - 2.0F * c);
}

/// Height of a spherical cap of the given radius and height, 0 outside.
inline float cap(float distance, float radius, float h) {
    if (distance >= radius) return 0.0F;
    const float t = distance / radius;
    return h * std::sqrt(1.0F - t * t);
}

inline float distanceTo(float u, float v, float cx, float cy) { return std::hypot(u - cx, v - cy); }

inline constexpr float kRivetCentres[4][2] = {{0.15F, 0.15F}, {0.85F, 0.15F}, {0.15F, 0.85F}, {0.85F, 0.85F}};

inline float rivetHeight(float u, float v) {
    float h = 0.0F;
    for (const auto& c : kRivetCentres) h += cap(distanceTo(u, v, c[0], c[1]), kRivetRadius, kRivetHeight);
    return h;
}

inline bool onRivet(float u, float v) {
    for (const auto& c : kRivetCentres) {
        if (distanceTo(u, v, c[0], c[1]) < kRivetRadius) return true;
    }
    return false;
}

/// The large shapes without the fine grain.
inline float shapeHeight(float u, float v) {
    const float edge = std::min(std::min(u, v), std::min(1.0F - u, 1.0F - v));
    const float plate = kPlateHeight * smoothstep01(edge / kRimWidth);
    const float r = distanceTo(u, v, 0.5F, 0.5F);
    const float dome = cap(r, kDomeRadius, kDomeHeight);
    const float d = std::fabs(r - kGrooveRadius);
    const float groove = d < kGrooveHalfWidth ? -kGrooveDepth * 0.5F * (1.0F + std::cos(kPi * d / kGrooveHalfWidth)) : 0.0F;
    return plate + dome + rivetHeight(u, v) + groove;
}

inline float hashCell(int ix, int iy) {
    std::uint32_t h = static_cast<std::uint32_t>(ix) * 374761393U + static_cast<std::uint32_t>(iy) * 668265263U;
    h = (h ^ (h >> 13)) * 1274126177U;
    h ^= h >> 16;
    return static_cast<float>(h & 0xFFFFFFU) / 16777216.0F;
}

/// Smooth value noise in [0,1]: fine stone grain. Integer hashing, so it is the same on every platform.
inline float grain(float u, float v) {
    const float x = std::clamp(u * kGrainFrequency, -1.0e6F, 1.0e6F);  // clamped, so the casts below stay in int range
    const float y = std::clamp(v * kGrainFrequency, -1.0e6F, 1.0e6F);
    const float fx = std::floor(x);
    const float fy = std::floor(y);
    const int ix = static_cast<int>(fx);
    const int iy = static_cast<int>(fy);
    const float tx = smoothstep01(x - fx);
    const float ty = smoothstep01(y - fy);
    const float top = hashCell(ix, iy) + (hashCell(ix + 1, iy) - hashCell(ix, iy)) * tx;
    const float bottom = hashCell(ix, iy + 1) + (hashCell(ix + 1, iy + 1) - hashCell(ix, iy + 1)) * tx;
    return top + (bottom - top) * ty;
}

inline float height(float u, float v) { return shapeHeight(u, v) + kGrainHeight * grain(u, v); }

struct Rgb {
    float r = 0.0F;
    float g = 0.0F;
    float b = 0.0F;
};

/// Flat paint, no shading baked in. All depth in the lit picture comes from the normal map.
inline Rgb albedoAt(float u, float v) {
    const float edge = std::min(std::min(u, v), std::min(1.0F - u, 1.0F - v));
    const float r = distanceTo(u, v, 0.5F, 0.5F);
    Rgb c{0.66F, 0.63F, 0.57F};  // stone
    if (edge < kRimWidth) c = {0.47F, 0.44F, 0.40F};
    if (std::fabs(r - kGrooveRadius) < kGrooveHalfWidth) c = {0.27F, 0.25F, 0.23F};
    if (r < kDomeRadius) c = {0.05F, 0.58F, 0.53F};  // the teal of the site
    if (onRivet(u, v)) c = {0.80F, 0.64F, 0.28F};    // brass
    const float speckle = 0.9F + 0.1F * grain(u, v);
    return {c.r * speckle, c.g * speckle, c.b * speckle};
}

}  // namespace nm
