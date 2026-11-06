// Tests for the normal map lighting logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Conventions used throughout: tangent space has +x to the right and +y UP (OpenGL style, green channel = up),
// +z points out of the surface at the viewer. Image rows run downwards, so "up" is the direction of smaller image y.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace nm;

namespace {

bool near(double a, double b, double eps = 1e-5) { return std::fabs(a - b) < eps; }
bool nearVec(Vec3 a, Vec3 b, double eps = 1e-5) { return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps); }

void normalizing() {
    assert(nearVec(normalized({3, 0, 4}), {0.6F, 0.0F, 0.8F}));
    assert(near(length(normalized({1, 2, 3})), 1.0));
    // A zero vector has no direction. It falls back to "straight up out of the surface" instead of dividing by zero.
    assert(nearVec(normalized({0, 0, 0}), {0, 0, 1}));
    assert(nearVec(normalized({1e-9F, 0, 0}), {0, 0, 1}));
    assert(near(dot({1, 2, 3}, {4, -5, 6}), 12.0));
}

void encodeDecode() {
    // The flat normal (0,0,1) is the typical light blue of normal maps: (128, 128, 255).
    std::uint8_t rgb[3];
    encodeNormal({0, 0, 1}, rgb);
    assert(rgb[0] == 128 && rgb[1] == 128 && rgb[2] == 255);
    // x = +1 maps to 255, x = -1 maps to 0.
    encodeNormal({1, 0, 0}, rgb);
    assert(rgb[0] == 255 && rgb[2] == 128);
    encodeNormal({-1, 0, 0}, rgb);
    assert(rgb[0] == 0);
    // Decoding is the inverse up to the 8 bit grid: one step is 2/255.
    const std::uint8_t flat[3] = {128, 128, 255};
    assert(nearVec(decodeNormal(flat), {0, 0, 1}, 0.005));
    const std::uint8_t left[3] = {0, 128, 128};
    assert(nearVec(decodeNormal(left), {-1, 0, 0}, 0.005));
    // Out of range input is clamped, not wrapped.
    encodeNormal({5, -5, 2}, rgb);
    assert(rgb[0] == 255 && rgb[1] == 0 && rgb[2] == 255);
    // A round trip stays within half a step plus the length error of the grid.
    const Vec3 n = normalized({0.3F, -0.5F, 0.8F});
    encodeNormal(n, rgb);
    assert(nearVec(decodeNormal(rgb), n, 0.01));
}

void bakingFromHeights() {
    // A flat surface has the flat normal everywhere.
    const NormalMap flat = bakeNormalMap(8, [](float, float) { return 0.25F; });
    assert(flat.size == 8 && flat.rgb.size() == 8U * 8U * 3U);
    for (int i = 0; i < 64; ++i) {
        assert(flat.rgb[static_cast<std::size_t>(i) * 3 + 0] == 128);
        assert(flat.rgb[static_cast<std::size_t>(i) * 3 + 1] == 128);
        assert(flat.rgb[static_cast<std::size_t>(i) * 3 + 2] == 255);
    }
    // h = 0.5 u rises to the right with slope 0.5, the normal leans left: (-0.5, 0, 1) / sqrt(1.25).
    const NormalMap right = bakeNormalMap(16, [](float u, float) { return 0.5F * u; });
    const Vec3 a = decodeNormal(&right.rgb[(5U * 16U + 7U) * 3U]);
    assert(nearVec(a, {-0.4472F, 0.0F, 0.8944F}, 0.01));
    // h = 0.5 v rises towards the BOTTOM of the image. The surface faces up (towards smaller image y), and up is +y: green > 128.
    const NormalMap down = bakeNormalMap(16, [](float, float v) { return 0.5F * v; });
    const Vec3 b = decodeNormal(&down.rgb[(5U * 16U + 7U) * 3U]);
    assert(nearVec(b, {0.0F, 0.4472F, 0.8944F}, 0.01));
    // The slope does not depend on the texture size, it is a property of the height function.
    const NormalMap fine = bakeNormalMap(64, [](float u, float) { return 0.5F * u; });
    assert(nearVec(decodeNormal(&fine.rgb[(10U * 64U + 30U) * 3U]), a, 0.01));
}

NormalMap twoTilted() {
    // 2x2 map, top row: left texel leans left 45 degrees, right texel leans right 45 degrees. Bottom row is flat.
    NormalMap m;
    m.size = 2;
    m.rgb.assign(12, 0);
    const float s = 0.70710678F;
    encodeNormal({-s, 0, s}, &m.rgb[0]);
    encodeNormal({s, 0, s}, &m.rgb[3]);
    encodeNormal({0, 0, 1}, &m.rgb[6]);
    encodeNormal({0, 0, 1}, &m.rgb[9]);
    return m;
}

void sampling() {
    const NormalMap m = twoTilted();
    // At a texel centre the sample is that texel: centres are at 0.25 and 0.75.
    assert(nearVec(sampleNormal(m, 0.25F, 0.25F, false, true), {-0.7071F, 0.0F, 0.7071F}, 0.02));
    assert(nearVec(sampleNormal(m, 0.75F, 0.25F, false, true), {0.7071F, 0.0F, 0.7071F}, 0.02));
    // Halfway between the two tilted texels the average is (0, 0, 0.7071): too short to be a normal.
    const Vec3 raw = sampleNormal(m, 0.5F, 0.25F, false, false);
    assert(near(raw.x, 0.0, 0.01) && near(raw.z, 0.7071, 0.02));
    assert(near(length(raw), 0.7071, 0.02));
    // Renormalizing stretches it back to length 1 and keeps the direction: straight up.
    const Vec3 fixed = sampleNormal(m, 0.5F, 0.25F, false, true);
    assert(near(length(fixed), 1.0, 1e-4));
    assert(nearVec(fixed, {0, 0, 1}, 0.01));
    // Outside [0,1] the edge texel is repeated (clamped), no read past the array.
    assert(nearVec(sampleNormal(m, -3.0F, -3.0F, false, true), sampleNormal(m, 0.0F, 0.0F, false, true)));
    assert(nearVec(sampleNormal(m, 9.0F, 0.25F, false, true), sampleNormal(m, 1.0F, 0.25F, false, true)));
    // Flipping the green channel negates y and nothing else.
    NormalMap lean;
    lean.size = 1;
    lean.rgb.assign(3, 0);
    encodeNormal(normalized({0.0F, 0.6F, 0.8F}), &lean.rgb[0]);
    const Vec3 up = sampleNormal(lean, 0.5F, 0.5F, false, true);
    const Vec3 flipped = sampleNormal(lean, 0.5F, 0.5F, true, true);
    assert(up.y > 0.5F);
    assert(near(flipped.y, -up.y) && near(flipped.x, up.x) && near(flipped.z, up.z));
    // An empty map is flat instead of a crash.
    assert(nearVec(sampleNormal(NormalMap{}, 0.5F, 0.5F, false, true), {0, 0, 1}));
}

void lightDirections() {
    // Light 4 above the pixel, 3 to the right on screen: (3, 0, 4) / 5.
    assert(nearVec(lightDirection({13, 10, 4}, 10, 10), {0.6F, 0.0F, 0.8F}));
    // Light 3 higher on screen (smaller image y) points to +y in tangent space, because +y is up.
    assert(nearVec(lightDirection({10, 7, 4}, 10, 10), {0.0F, 0.6F, 0.8F}));
    // Light lower on screen: -y.
    assert(nearVec(lightDirection({10, 13, 4}, 10, 10), {0.0F, -0.6F, 0.8F}));
    // Straight above.
    assert(nearVec(lightDirection({10, 10, 50}, 10, 10), {0, 0, 1}));
    // It is a point light: the same light gives a different direction at another pixel.
    assert(nearVec(lightDirection({13, 10, 4}, 13, 10), {0, 0, 1}));
    // A light sitting exactly in the surface point has no direction, the fallback keeps the result finite.
    assert(nearVec(lightDirection({10, 10, 0}, 10, 10), {0, 0, 1}));
    // The result is always a unit vector.
    assert(near(length(lightDirection({-80, 300, 17}, 10, 10)), 1.0));
}

void powers() {
    assert(near(powInt(2.0F, 10), 1024.0, 1e-3));
    assert(near(powInt(0.5F, 3), 0.125));
    assert(near(powInt(0.7F, 0), 1.0));
    assert(near(powInt(0.0F, 5), 0.0));
    assert(near(powInt(0.0F, 0), 1.0));
    assert(near(powInt(0.9F, 32), std::pow(0.9, 32.0), 1e-5));
    assert(near(powInt(0.9F, 33), std::pow(0.9, 33.0), 1e-5));
}

void shading() {
    // Light straight above a flat pixel: full diffuse, full highlight.
    Lit a = lightPixel({0, 0, 1}, {0, 0, 1}, 32);
    assert(near(a.diffuse, 1.0) && near(a.specular, 1.0));
    // Normal tilted 60 degrees, light straight above: N.L = cos 60 = 0.5. H is also straight up, so N.H = 0.5 as well,
    // and with shininess 4 the highlight is 0.5^4 = 0.0625.
    const Vec3 n60{0.8660254F, 0.0F, 0.5F};
    Lit b = lightPixel(n60, {0, 0, 1}, 4);
    assert(near(b.diffuse, 0.5) && near(b.specular, 0.0625));
    // Mirror geometry: normal 30 degrees to the right, light 60 degrees to the right, viewer straight above.
    // The half vector (0.5, 0, 0.866) equals the normal, so the highlight is exactly 1. N.L = sin30 sin60 + cos30 cos60 = 0.866.
    Lit c = lightPixel({0.5F, 0.0F, 0.8660254F}, {0.8660254F, 0.0F, 0.5F}, 64);
    assert(near(c.diffuse, 0.8660254, 1e-4) && near(c.specular, 1.0, 1e-4));
    // A light below the surface plane lights nothing. The half vector still has a positive N.H here (0.9/|H|),
    // so the highlight must be switched off explicitly.
    Lit d = lightPixel({0, 0, 1}, normalized({1.0F, 0.0F, -0.1F}), 2);
    assert(near(d.diffuse, 0.0) && near(d.specular, 0.0));
    // A normal facing away from the light: negative N.L is clamped to 0, never negative light.
    Lit e = lightPixel({-1, 0, 0}, {0.6F, 0.0F, 0.8F}, 8);
    assert(near(e.diffuse, 0.0) && near(e.specular, 0.0));
    // Grazing light on a flat surface: dim.
    Lit f = lightPixel({0, 0, 1}, normalized({1.0F, 0.0F, 0.1F}), 8);
    assert(f.diffuse > 0.0F && f.diffuse < 0.11F);
    // An unnormalized normal is NOT repaired here, it just dims the light. That is what the "renormalize" switch is about.
    Lit g = lightPixel({0, 0, 0.5F}, {0, 0, 1}, 2);
    assert(near(g.diffuse, 0.5) && near(g.specular, 0.25));
    // The inputs come back so the inspector can show them.
    assert(nearVec(g.n, {0, 0, 0.5F}) && nearVec(g.l, {0, 0, 1}));
}

void composing() {
    // albedo 0.5, ambient 0.2: diffuse 1 gives 0.5 * (0.2 + 0.8) = 0.5, diffuse 0 gives 0.5 * 0.2 = 0.1.
    assert(near(compose(0.5F, 1.0F, 0.0F, 0.2F, 0.9F), 0.5));
    assert(near(compose(0.5F, 0.0F, 0.0F, 0.2F, 0.9F), 0.1));
    // A highlight adds on top: 0.1 + 0.9 * 0.5 = 0.55.
    assert(near(compose(0.5F, 0.0F, 0.5F, 0.2F, 0.9F), 0.55));
    // The result is clamped into [0,1].
    assert(near(compose(0.9F, 1.0F, 1.0F, 0.2F, 0.9F), 1.0));
    assert(near(compose(-1.0F, 0.0F, 0.0F, 0.2F, 0.9F), 0.0));
    assert(toByte(0.0F) == 0 && toByte(1.0F) == 255 && toByte(0.5F) == 128);
    assert(toByte(-3.0F) == 0 && toByte(7.0F) == 255);
}

void surfaceShape() {
    // The corner of the tile is the lowest point.
    assert(near(shapeHeight(0.0F, 0.0F), 0.0));
    // The top of the central dome: plate plus dome.
    assert(near(shapeHeight(0.5F, 0.5F), kPlateHeight + kDomeHeight));
    // A plain stretch of plate, no feature nearby.
    assert(near(shapeHeight(0.5F, 0.05F), kPlateHeight));
    assert(near(shapeHeight(0.72F, 0.5F), kPlateHeight));
    // The bevel ramps up: halfway through its width the plate is at half height (smoothstep(0.5) = 0.5).
    assert(near(shapeHeight(0.5F * kRimWidth, 0.5F), 0.5 * kPlateHeight));
    // The top of a rivet.
    assert(near(shapeHeight(0.15F, 0.15F), kPlateHeight + kRivetHeight));
    // The groove at the bottom of its profile cuts its full depth into the plate.
    assert(near(shapeHeight(0.5F + kGrooveRadius, 0.5F), kPlateHeight - kGrooveDepth));
    // The surface is symmetric left-right and top-bottom.
    assert(near(shapeHeight(0.3F, 0.4F), shapeHeight(0.7F, 0.4F)));
    assert(near(shapeHeight(0.3F, 0.4F), shapeHeight(0.3F, 0.6F)));
    // The dome falls off towards its rim: the height at 90 percent of its radius is below the top.
    assert(shapeHeight(0.5F + 0.9F * kDomeRadius, 0.5F) < shapeHeight(0.5F, 0.5F));
    assert(shapeHeight(0.5F + 0.9F * kDomeRadius, 0.5F) > kPlateHeight);
}

void grainNoise() {
    float lo = 1.0F;
    float hi = 0.0F;
    for (int y = 0; y < 50; ++y) {
        for (int x = 0; x < 50; ++x) {
            const float g = grain(static_cast<float>(x) / 50.0F, static_cast<float>(y) / 50.0F);
            assert(g >= 0.0F && g <= 1.0F);
            lo = std::min(lo, g);
            hi = std::max(hi, g);
        }
    }
    assert(hi - lo > 0.3F);  // it actually varies
    assert(near(grain(0.37F, 0.81F), grain(0.37F, 0.81F)));  // and it is deterministic
    // Total height is the shape plus the small grain on top.
    const float h = height(0.5F, 0.05F);
    assert(h >= kPlateHeight && h <= kPlateHeight + kGrainHeight + 1e-6F);
}

void albedoColors() {
    // Painted regions are flat colours without any shading baked in: the dome is teal-ish, rivets are brass.
    const Rgb dome = albedoAt(0.5F, 0.5F);
    assert(dome.g > dome.r && dome.b > dome.r);
    const Rgb rivet = albedoAt(0.15F, 0.15F);
    assert(rivet.r > rivet.b);
    for (int i = 0; i < 100; ++i) {
        const Rgb c = albedoAt(static_cast<float>(i) / 100.0F, static_cast<float>((i * 37) % 100) / 100.0F);
        assert(c.r >= 0.0F && c.r <= 1.0F && c.g >= 0.0F && c.g <= 1.0F && c.b >= 0.0F && c.b <= 1.0F);
    }
}

}  // namespace

int main() {
    normalizing();
    encodeDecode();
    bakingFromHeights();
    sampling();
    lightDirections();
    powers();
    shading();
    composing();
    surfaceShape();
    grainNoise();
    albedoColors();
    std::puts("normal-map-lighting logic: all asserts passed");
    return 0;
}
