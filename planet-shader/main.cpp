// Planets from noise: a whole planet in one fragment shader, no mesh and no texture.
//
// Every pixel shoots a ray at a ball of radius 1. Where it hits, the surface point (a unit vector) is rotated into the
// planet's own frame and handed to 3D gradient noise: fbm gives a height, the height gives colour bands (sea, beach,
// lowland, rock, snow), a second layer gives clouds, the sun gives the day/night terminator, and a glow shell around the
// edge fakes the atmosphere. The naive way to texture a ball is to map longitude and latitude to a flat noise image; the
// "mapping" chip switches to exactly that, and the seam and the pinched poles show up.
//
// logic.hpp holds the CPU mirror of the shader's noise, mapping, ray and glow math (tested natively in test.cpp). Its
// constants are pasted into the shader source at startup, so they cannot drift apart; the shader code itself repeats the
// functions and has to be kept in sync by hand (hash32, gradient, noise3, octaveStep, fbm, lodOctaves, noiseCoord,
// elevation, toWorld/toLocal, dayFactor, the glow functions).
//
// The planet is rendered into a texture (960x540, or half of it with the "res" chip) and scaled onto the canvas, like the
// ray marching demo. The spin angles are reduced to [0, 2 pi) in double on the CPU, the shader never sees a growing time.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "gl.hpp"
#include "logic.hpp"

namespace {

using namespace gd::planet;

constexpr double kMaxFrameTime = 0.25;
constexpr double kSpinRate = kTwoPi / 60.0;        // one turn per minute
constexpr double kCloudRate = kSpinRate * 1.25;    // the clouds drift a little faster than the ground
constexpr float kPanelTop = 452.0F;
constexpr std::array<int, 4> kOctaveChoices{1, 2, 4, 8};
constexpr std::array<float, 2> kResScale{1.0F, 0.5F};
constexpr Vec3 kSun{0.90F, 0.22F, 0.12F};  // direction towards the sun, fixed in the world (normalised at upload)

// Fragment shader. `vUv` is 0..1 with the origin at the top left, like the canvas.
// The shared constants from logic.hpp are put in front of this text at startup (kBaseFreq, kMaxOctaves, ...).
constexpr const char* kPlanetFs = R"GLSL(
uniform vec3 uCamPos;
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec3 uFwd;
uniform float uPixelAngle;   // radians per pixel of the render target
uniform vec2 uSpin;          // cos and sin of the planet's spin angle, reduced on the CPU
uniform vec2 uCloudSpin;
uniform vec2 uTilt;          // cos and sin of the axis tilt
uniform vec3 uSun;
uniform int uMapping;        // 0: 3D noise on the sphere, 1: longitude and latitude as flat coordinates
uniform int uOctaves;
uniform float uSea;
uniform int uAtmo;
uniform int uClouds;
uniform vec3 uPal[8];        // linear colours: deep, shallow, beach, lowland wet, lowland dry, rock, snow, ice
uniform vec4 uP0;            // ice latitude, cloud cover, atmosphere strength, gloss
uniform vec2 uP1;            // lava, cities
uniform vec3 uAtmoCol;
uniform vec3 uCloudCol;
in vec2 vUv;
out vec4 outColor;

const float kRelief = 0.045;           // how steep the bump shading looks
const vec3 kSunColor = vec3(1.0, 0.97, 0.92) * 1.15;
const vec3 kSunset = vec3(1.0, 0.40, 0.12);

// ---- hash and gradient noise (mirrors hash32, gradient, noise3 in logic.hpp) ----
uint hash32(uint x) {
    x ^= x >> 16u; x *= 0x7feb352du; x ^= x >> 15u; x *= 0x846ca68bu; x ^= x >> 16u;
    return x;
}

float gradient(uint h, vec3 d) {
    uint k = h & 15u;
    float u = k < 8u ? d.x : d.y;
    float v = k < 4u ? d.y : ((k == 12u || k == 14u) ? d.x : d.z);
    return (((k & 1u) == 0u) ? u : -u) + (((k & 2u) == 0u) ? v : -v);
}

float noise3(vec3 p) {
    vec3 fl = floor(p);
    vec3 f = p - fl;
    uint xi = uint(int(fl.x) + kLatticeOffset);
    uint yi = uint(int(fl.y) + kLatticeOffset);
    uint zi = uint(int(fl.z) + kLatticeOffset);
    uint z0 = hash32(zi), z1 = hash32(zi + 1u);
    uint y00 = hash32(yi + z0), y10 = hash32(yi + 1u + z0), y01 = hash32(yi + z1), y11 = hash32(yi + 1u + z1);
    float a = gradient(hash32(xi + y00), f);
    float b = gradient(hash32(xi + 1u + y00), f - vec3(1.0, 0.0, 0.0));
    float c = gradient(hash32(xi + y10), f - vec3(0.0, 1.0, 0.0));
    float d = gradient(hash32(xi + 1u + y10), f - vec3(1.0, 1.0, 0.0));
    float e = gradient(hash32(xi + y01), f - vec3(0.0, 0.0, 1.0));
    float g = gradient(hash32(xi + 1u + y01), f - vec3(1.0, 0.0, 1.0));
    float h = gradient(hash32(xi + y11), f - vec3(0.0, 1.0, 1.0));
    float i = gradient(hash32(xi + 1u + y11), f - vec3(1.0, 1.0, 1.0));
    vec3 w = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    return mix(mix(mix(a, b, w.x), mix(c, d, w.x), w.y), mix(mix(e, g, w.x), mix(h, i, w.x), w.y), w.z);
}

// ---- fbm (mirrors octaveStep, fbm, lodOctaves) ----
vec3 octaveStep(vec3 p) {
    return 2.0 * vec3(dot(vec3(0.0, 0.80, 0.60), p), dot(vec3(-0.80, 0.36, -0.48), p), dot(vec3(-0.60, -0.48, 0.64), p))
           + vec3(1.37, 2.11, 0.73);
}

float fbm(vec3 p, int octaves, float lod) {
    float sum = 0.0, amp = 1.0;
    for (int i = 0; i < kMaxOctaves; i++) {
        if (i >= octaves) break;
        float w = clamp(lod - float(i), 0.0, 1.0);
        if (w <= 0.0) break;   // the rest is faded out as well
        sum += w * amp * noise3(p);
        amp *= 0.5;
        p = octaveStep(p);
    }
    // The divisor counts all requested octaves, not only the visible ones: 1 + 1/2 + ... + 1/2^(n-1).
    float norm = 2.0 * (1.0 - exp2(-float(min(octaves, kMaxOctaves))));
    return norm > 0.0 ? sum / norm : 0.0;
}

float lodOctaves(float pixelWorld) {
    return clamp(log2(1.0 / (2.0 * kBaseFreq * pixelWorld)) + 1.0, 0.0, float(kMaxOctaves));
}

// ---- planet frame (mirrors toWorld, toLocal) ----
vec3 rotY(vec3 v, vec2 cs) { return vec3(v.x * cs.x + v.z * cs.y, v.y, -v.x * cs.y + v.z * cs.x); }
vec3 rotYInv(vec3 v, vec2 cs) { return vec3(v.x * cs.x - v.z * cs.y, v.y, v.x * cs.y + v.z * cs.x); }
vec3 tilt(vec3 v) { return vec3(v.x * uTilt.x - v.y * uTilt.y, v.x * uTilt.y + v.y * uTilt.x, v.z); }
vec3 untilt(vec3 v) { return vec3(v.x * uTilt.x + v.y * uTilt.y, -v.x * uTilt.y + v.y * uTilt.x, v.z); }
vec3 toLocal(vec3 w) { return rotYInv(untilt(w), uSpin); }
vec3 toWorld(vec3 l) { return tilt(rotY(l, uSpin)); }

// ---- mapping and height (mirrors noiseCoord, elevation) ----
vec3 noiseCoord(vec3 u) {
    if (uMapping == 0) return u * kBaseFreq;
    float lon = atan(u.z, u.x);                    // -pi .. pi: the jump at -x is the seam
    float lat = asin(clamp(u.y, -1.0, 1.0));
    return vec3(lon, lat, kSliceZ) * kBaseFreq;
}

float elevation(vec3 u, float lod) { return 0.5 + kHeightGain * fbm(noiseCoord(u), uOctaves, lod); }

// ---- light (mirrors dayFactor, glowOutside, glowInside) ----
float dayFactor(float ndl) { return smoothstep(-0.1, 0.25, ndl); }

vec3 glowColor(float ndl) {
    float tw = exp(-pow(ndl / 0.18, 2.0));    // 1 right at the terminator: sunrise colours
    return mix(uAtmoCol, kSunset, 0.55 * tw);
}

// Glow of the atmosphere shell. `glow` is 1 on the limb and falls to 0; `ndl` is the sun angle at the limb point.
vec3 atmosphere(float glow, float ndl) {
    float day = smoothstep(-0.35, 0.40, ndl);
    return glowColor(ndl) * glow * day * uP0.z * 1.1;
}

// ---- clouds ----
float cloudDensity(vec3 worldDir, float lod) {
    vec3 pc = rotYInv(untilt(worldDir), uCloudSpin);
    float f = fbm(pc * 3.6 + vec3(3.7, 9.2, 1.3), 6, lod);
    float thr = mix(0.24, -0.04, uP0.y);
    return smoothstep(thr, thr + 0.12, f);
}

// ---- stars: random points in the cells of a 3D grid, seen along the ray ----
vec3 stars(vec3 rd) {
    vec3 col = vec3(0.0);
    for (int layer = 0; layer < 2; layer++) {
        float scale = layer == 0 ? 60.0 : 130.0;
        float radius = layer == 0 ? 0.13 : 0.15;
        vec3 p = rd * scale;
        vec3 cell = floor(p);
        uint h = hash32(uint(int(cell.x) + kLatticeOffset) + hash32(uint(int(cell.y) + kLatticeOffset) + hash32(uint(int(cell.z) + kLatticeOffset) + uint(layer) * 977u)));
        if ((h & 255u) > 21u) continue;                        // about 8 percent of the cells hold a star
        vec3 at = cell + vec3(float((h >> 8u) & 255u), float((h >> 16u) & 255u), float((h >> 24u) & 255u)) / 255.0;
        float d = length(p - at);
        float bright = 0.35 + 0.65 * float((h >> 4u) & 15u) / 15.0;
        float tint = float((h >> 12u) & 15u) / 15.0;
        col += mix(vec3(0.75, 0.85, 1.0), vec3(1.0, 0.88, 0.7), tint) * bright * (1.0 - smoothstep(0.3 * radius, radius, d));
    }
    return col;
}

// ---- the surface of the planet ----
vec3 surface(vec3 nw, vec3 rd, float pixelWorld) {
    vec3 pl = toLocal(nw);                       // the point in the planet's own frame
    float lod = lodOctaves(pixelWorld);
    float e = elevation(pl, lod);
    bool ocean = e < uSea;
    float ndlG = dot(nw, uSun);
    float wrap = dayFactor(ndlG);

    // Bump normal from two extra height lookups, only where it can be seen.
    vec3 nb = nw;
    if (!ocean && ndlG > -0.3) {
        vec3 ref = abs(pl.y) < 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
        vec3 t1 = normalize(cross(pl, ref));
        vec3 t2 = cross(pl, t1);
        float eps = max(pixelWorld * 0.7, 0.0004);
        float hx = elevation(normalize(pl + t1 * eps), lod);
        float hy = elevation(normalize(pl + t2 * eps), lod);
        vec3 slope = ((hx - e) * t1 + (hy - e) * t2) / eps;
        nb = normalize(nw - toWorld(slope) * kRelief);
    }

    // Colour bands.
    float t = clamp((e - uSea) / (1.0 - uSea), 0.0, 1.0);
    vec3 albedo;
    vec3 emissive = vec3(0.0);
    float depth = clamp((uSea - e) / kDeepDepth, 0.0, 1.0);
    if (ocean) {
        albedo = mix(uPal[1], uPal[0], depth);
        if (uP1.x > 0.5) {                        // lava: the low ground glows by itself
            float flicker = 0.75 + 0.5 * noise3(pl * 18.0 + vec3(4.0));
            emissive = mix(uPal[1], uPal[0], depth) * (1.2 + 1.2 * depth) * flicker;
            albedo = vec3(0.01);
        }
    } else {
        float moist = fbm(pl * 3.2 + vec3(7.1, 3.3, 1.9), 3, lod);
        vec3 low = mix(uPal[4], uPal[3], smoothstep(-0.25, 0.05, moist));
        albedo = mix(uPal[2], low, smoothstep(0.0, kBeachTop * 1.3, t));
        albedo = mix(albedo, mix(low, uPal[5], 0.55), smoothstep(kLowTop * 0.8, kLowTop * 1.2, t));
        albedo = mix(albedo, uPal[5], smoothstep(kHighTop * 0.9, kHighTop * 1.1, t));
        albedo = mix(albedo, uPal[6], smoothstep(kRockTop * 0.95, kRockTop * 1.05, t));
        albedo *= 0.88 + 0.24 * (noise3(pl * 24.0) + 0.5);
        if (uP1.x > 0.5) {                        // cracks near the lava shore
            float crack = smoothstep(0.80, 0.97, 1.0 - abs(noise3(pl * 15.0 + vec3(2.0)) * 2.0));
            emissive = uPal[1] * crack * (1.0 - smoothstep(0.0, 0.16, t)) * 1.4;
        }
    }
    float latN = abs(pl.y) + 0.07 * fbm(pl * 5.0 + vec3(11.0), 3, lod);
    float iceM = smoothstep(uP0.x, uP0.x + 0.05, latN);
    albedo = mix(albedo, uPal[7], iceM);
    if (iceM > 0.5) emissive *= 0.0;
    bool wet = ocean && uP1.x < 0.5 && iceM < 0.5;

    // Clouds and their shadow.
    float cloud = 0.0;
    float shadow = 0.0;
    if (uClouds == 1 && uP0.y > 0.0) {
        cloud = cloudDensity(nw, lod);
        vec3 ts = uSun - nw * ndlG;                              // sun direction along the ground
        shadow = cloudDensity(normalize(nw - ts * 0.035), lod) * 0.6 * wrap;
    }

    // Light.
    vec3 atmoLin = uAtmo == 1 ? uAtmoCol * uP0.z : vec3(0.0);
    vec3 ambient = vec3(0.010, 0.014, 0.028) + atmoLin * 0.07 * wrap;
    float diff = max(dot(nb, uSun), 0.0) * wrap;
    vec3 col = albedo * (1.0 - shadow) * (kSunColor * diff + ambient) + emissive;

    if (wet && uP0.w > 0.0) {                                    // sun glint and sky reflection on the water
        vec3 v = -rd;
        float spec = pow(max(dot(nw, normalize(uSun + v)), 0.0), 900.0) * wrap * uP0.w;
        float fres = pow(1.0 - max(dot(nw, v), 0.0), 5.0);
        col = mix(col, uAtmoCol * 0.5 * wrap, fres * 0.5 * uP0.w);
        col += kSunColor * spec * 1.1;
    }

    if (uP1.y > 0.5 && !ocean && iceM < 0.5) {                   // city lights on the night side
        float night = 1.0 - smoothstep(-0.12, 0.05, ndlG);
        float lowland = 1.0 - smoothstep(0.0, 0.5, t);
        float region = smoothstep(0.02, 0.16, fbm(pl * 6.0 + vec3(2.0, 5.0, 7.0), 3, lod));
        float dots = smoothstep(0.20, 0.36, noise3(pl * 42.0 + vec3(1.7)));
        col += vec3(1.0, 0.72, 0.38) * dots * region * lowland * night * (1.0 - cloud * 0.85) * 1.6;
    }

    float cloudLight = smoothstep(-0.05, 0.5, ndlG);
    vec3 cloudCol = uCloudCol * (kSunColor * cloudLight * 0.95 + ambient * 1.5);
    col = mix(col, cloudCol, cloud * 0.88);

    if (uAtmo == 1) {                                            // rim glow, the inside half of the shell
        float cosv = max(dot(nw, -rd), 0.0);
        col += atmosphere(pow(1.0 - cosv, 3.0), ndlG);
    }
    return col;
}

void main() {
    vec2 ndc = vec2(vUv.x * 2.0 - 1.0, (kCenterY - vUv.y * 540.0) / 270.0);
    vec3 rd = normalize(uFwd + uRight * (ndc.x * kTanHalfFov * kAspect) + uUp * (ndc.y * kTanHalfFov));
    vec3 ro = uCamPos;

    float b = dot(ro, rd);
    float rmin = sqrt(max(dot(ro, ro) - b * b, 0.0));   // how close the ray passes the centre
    float tc = -b;                                      // at this distance along the ray
    float cov = clamp((1.0 - rmin) / (uPixelAngle * tc) + 0.5, 0.0, 1.0);   // anti-aliased silhouette

    vec3 col = stars(rd) + vec3(0.003, 0.005, 0.010);
    if (uAtmo == 1 && rmin < kAtmoRadius) {             // the outside half of the glow shell
        float k = 1.0 - clamp((rmin - 1.0) / (kAtmoRadius - 1.0), 0.0, 1.0);
        vec3 q = normalize(ro + rd * tc);
        col += atmosphere(k * k * k, dot(q, uSun));
    }
    if (cov > 0.0) {
        float disc = b * b - (dot(ro, ro) - 1.0);
        float t = tc - sqrt(max(disc, 0.0));
        vec3 nw = normalize(ro + rd * t);
        col = mix(col, surface(nw, rd, uPixelAngle * t), cov);
    }

    col = pow(clamp(col, 0.0, 1.0), vec3(1.0 / 2.2));
    float dither = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453) - 0.5;
    outColor = vec4(col + dither / 255.0, 1.0);
}
)GLSL";

// Copies the render target to the canvas. The target is rendered with the same top-down vUv, so flip back for sampling.
constexpr const char* kBlitFs = R"GLSL(
uniform sampler2D uTex;
in vec2 vUv;
out vec4 outColor;
void main() { outColor = vec4(texture(uTex, vec2(vUv.x, 1.0 - vUv.y)).rgb, 1.0); }
)GLSL";

/// The constants of logic.hpp as GLSL, put in front of the shader so both sides use the same numbers.
std::string sharedConstants() {
    char buf[1024];
    SDL_snprintf(buf, sizeof buf,
                 "const float kBaseFreq = %.8f;\nconst int kMaxOctaves = %d;\nconst float kSliceZ = %.8f;\nconst float kHeightGain = %.8f;\n"
                 "const float kAtmoRadius = %.8f;\nconst int kLatticeOffset = %d;\nconst float kDeepDepth = %.8f;\nconst float kBeachTop = %.8f;\n"
                 "const float kLowTop = %.8f;\nconst float kHighTop = %.8f;\nconst float kRockTop = %.8f;\nconst float kTanHalfFov = %.8f;\n"
                 "const float kAspect = %.8f;\nconst float kCenterY = %.8f;\n",
                 static_cast<double>(kBaseFreq), kMaxOctaves, static_cast<double>(kSliceZ), static_cast<double>(kHeightGain), static_cast<double>(kAtmoRadius),
                 kLatticeOffset, static_cast<double>(kDeepDepth), static_cast<double>(kBeachTop), static_cast<double>(kLowTop), static_cast<double>(kHighTop),
                 static_cast<double>(kRockTop), static_cast<double>(kTanHalfFov), static_cast<double>(kAspect), static_cast<double>(kCenterY));
    return std::string(buf);
}

enum class Group { Planet, Mapping, Octaves, Atmo, Clouds, Spin, Res };

struct Chip {
    SDL_FRect rect;
    std::string label;
    Group group;
    int value;
};

struct Label {
    float x, y;
    const char* text;
};

struct Uniforms {
    GLint camPos, right, up, fwd, pixelAngle, spin, cloudSpin, tilt, sun, mapping, octaves, sea, atmo, clouds, pal, p0, p1, atmoCol, cloudCol;
};

struct App {
    gd::gl::Window win;
    gd::gl::Hud hud;
    GLuint planet = 0;
    GLuint blit = 0;
    Uniforms u{};
    GLuint fbo = 0;
    gd::gl::Texture target;
    int targetW = 0;
    int targetH = 0;

    std::vector<Chip> chips;
    std::vector<Label> labels;

    int preset = 0;
    Mapping mapping = Mapping::Sphere;
    int octaves = 3;  // index into kOctaveChoices
    bool atmo = true;
    bool clouds = true;
    bool spinning = true;
    int res = 0;      // index into kResScale
    float sea = 0.0F;

    float yaw = 0.25F;
    float pitch = 0.40F;
    float dist = 3.2F;
    bool dragging = false;
    SDL_FPoint lastMouse{0.0F, 0.0F};

    double spinAngle = 2.1;   // radians, always in [0, 2 pi)
    double cloudAngle = 0.4;
    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

float chipWidth(const App& app, const std::string& label) { return app.hud.width(label, gd::gl::Hud::Style::Bold) + 20.0F; }

// Places a row label and its chips from x to the right; returns the x after the group.
float addGroup(App& app, float x, float y, const char* label, Group group, const std::vector<std::string>& labels) {
    app.labels.push_back(Label{x, y + 3.0F, label});
    x += app.hud.width(label) + 10.0F;
    int value = 0;
    for (const std::string& l : labels) {
        const float w = chipWidth(app, l);
        app.chips.push_back(Chip{SDL_FRect{x, y, w, 28.0F}, l, group, value++});
        x += w + 8.0F;
    }
    return x + 16.0F;
}

void buildChips(App& app) {
    constexpr float row1 = 462.0F;
    constexpr float row2 = 498.0F;
    float x = 24.0F;
    x = addGroup(app, x, row1, "planet:", Group::Planet, {"Earth", "Desert", "Ice", "Lava"});
    x = addGroup(app, x, row1, "mapping:", Group::Mapping, {"3D sphere", "lat/long"});
    addGroup(app, x, row1, "res:", Group::Res, {"1x", "0.5x"});
    x = addGroup(app, 24.0F, row2, "octaves:", Group::Octaves, {"1", "2", "4", "8"});
    x = addGroup(app, x, row2, "atmo:", Group::Atmo, {"off", "on"});
    x = addGroup(app, x, row2, "clouds:", Group::Clouds, {"off", "on"});
    addGroup(app, x, row2, "spin:", Group::Spin, {"off", "on"});
}

bool isActive(const App& app, const Chip& c) {
    switch (c.group) {
        case Group::Planet: return c.value == app.preset;
        case Group::Mapping: return c.value == (app.mapping == Mapping::LatLong ? 1 : 0);
        case Group::Octaves: return c.value == app.octaves;
        case Group::Atmo: return c.value == (app.atmo ? 1 : 0);
        case Group::Clouds: return c.value == (app.clouds ? 1 : 0);
        case Group::Spin: return c.value == (app.spinning ? 1 : 0);
        case Group::Res: return c.value == app.res;
    }
    return false;
}

bool createTarget(App& app) {
    const float s = kResScale[static_cast<std::size_t>(app.res)];
    const int w = static_cast<int>(std::lround(gd::gl::kWidth * s));
    const int h = static_cast<int>(std::lround(gd::gl::kHeight * s));
    if (app.fbo == 0) glGenFramebuffers(1, &app.fbo);
    if (app.target.id != 0) app.target.destroy();
    app.target = gd::gl::Texture::create(w, h, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, GL_LINEAR);
    app.targetW = w;
    app.targetH = h;
    glBindFramebuffer(GL_FRAMEBUFFER, app.fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, app.target.id, 0);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        SDL_Log("render target incomplete: 0x%x", status);
        return false;
    }
    return true;
}

int octaveCount(const App& app) { return kOctaveChoices[static_cast<std::size_t>(app.octaves)]; }

// The sea level is re-measured whenever the height field changes shape, so the ocean keeps its share of the surface.
void updateSea(App& app) { app.sea = seaLevelFor(kPresets[static_cast<std::size_t>(app.preset)].oceanShare, app.mapping, octaveCount(app)); }

void setMapping(App& app, Mapping m) {
    if (m == app.mapping) return;
    app.mapping = m;
    updateSea(app);
    // Show the failure straight away: turn the planet so the seam faces the camera.
    if (m == Mapping::LatLong) app.spinAngle = static_cast<double>(std::fmod(seamFacingSpin(app.yaw, kTilt) + 2.0F * kPi, 2.0F * kPi));
}

void setPreset(App& app, int i) {
    app.preset = std::clamp(i, 0, static_cast<int>(kPresets.size()) - 1);
    updateSea(app);
}

void setOctaves(App& app, int i) {
    app.octaves = std::clamp(i, 0, static_cast<int>(kOctaveChoices.size()) - 1);
    updateSea(app);
}

void click(App& app, SDL_FPoint p) {
    for (const Chip& c : app.chips) {
        if (!SDL_PointInRectFloat(&p, &c.rect)) continue;
        switch (c.group) {
            case Group::Planet: setPreset(app, c.value); break;
            case Group::Mapping: setMapping(app, c.value == 1 ? Mapping::LatLong : Mapping::Sphere); break;
            case Group::Octaves: setOctaves(app, c.value); break;
            case Group::Atmo: app.atmo = c.value == 1; break;
            case Group::Clouds: app.clouds = c.value == 1; break;
            case Group::Spin: app.spinning = c.value == 1; break;
            case Group::Res:
                if (app.res != c.value) {
                    app.res = c.value;
                    createTarget(app);
                }
                break;
        }
        return;
    }
    if (p.y < kPanelTop) {
        app.dragging = true;
        app.lastMouse = p;
    }
}

void drawChips(App& app) {
    gd::gl::Hud& hud = app.hud;
    hud.fillRect(0.0F, kPanelTop, 960.0F, 540.0F - kPanelTop, gd::Color{250, 250, 249, 240});
    hud.fillRect(0.0F, kPanelTop, 960.0F, 2.0F, gd::kTealDark);
    for (const Label& l : app.labels) hud.text(l.x, l.y, l.text, gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool active = isActive(app, c);
        hud.card(c.rect.x, c.rect.y, c.rect.w, c.rect.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
        hud.text(c.rect.x + 10.0F, c.rect.y + 3.0F, c.label, active ? gd::kWhite : gd::kInk, gd::gl::Hud::Style::Bold);
    }
}

// In the naive mapping: a dotted orange line along the seam and tags where the seam and the poles are.
void drawSeamMarks(App& app, const Camera& cam) {
    gd::gl::Hud& hud = app.hud;
    const float spin = static_cast<float>(app.spinAngle);
    for (int i = 0; i <= 60; ++i) {
        const float lat = -kPi / 2.0F + kPi * static_cast<float>(i) / 60.0F;
        const Vec3 w = toWorld(seamPoint(lat), spin, kTilt);
        if (!facesCamera(cam, w)) continue;
        const Screen s = project(cam, w);
        if (s.inFront) hud.fillRect(s.x - 1.5F, s.y - 1.5F, 3.0F, 3.0F, gd::kOrange);
    }
    auto tag = [&](Vec3 local, const char* text, float dx) {
        const Vec3 w = toWorld(local, spin, kTilt);
        if (!facesCamera(cam, w)) return;
        const Screen s = project(cam, w);
        if (!s.inFront) return;
        const float tw = hud.width(text, gd::gl::Hud::Style::Bold) + 12.0F;
        hud.card(s.x + dx, s.y - 12.0F, tw, 24.0F, gd::kOrange, gd::kInk);
        hud.text(s.x + dx + 6.0F, s.y - 10.0F, text, gd::kWhite, gd::gl::Hud::Style::Bold);
    };
    tag(seamPoint(0.0F), "seam", 10.0F);
    tag({0.0F, 1.0F, 0.0F}, "pole", 10.0F);
    tag({0.0F, -1.0F, 0.0F}, "pole", 10.0F);
}

void upload(const App& app, const Camera& cam) {
    const Preset& p = kPresets[static_cast<std::size_t>(app.preset)];
    const Uniforms& u = app.u;
    glUniform3f(u.camPos, cam.pos.x, cam.pos.y, cam.pos.z);
    glUniform3f(u.right, cam.right.x, cam.right.y, cam.right.z);
    glUniform3f(u.up, cam.up.x, cam.up.y, cam.up.z);
    glUniform3f(u.fwd, cam.fwd.x, cam.fwd.y, cam.fwd.z);
    glUniform1f(u.pixelAngle, 2.0F * kTanHalfFov / static_cast<float>(app.targetH));
    glUniform2f(u.spin, static_cast<float>(std::cos(app.spinAngle)), static_cast<float>(std::sin(app.spinAngle)));
    glUniform2f(u.cloudSpin, static_cast<float>(std::cos(app.cloudAngle)), static_cast<float>(std::sin(app.cloudAngle)));
    glUniform2f(u.tilt, std::cos(kTilt), std::sin(kTilt));
    const Vec3 sun = normalize(kSun);
    glUniform3f(u.sun, sun.x, sun.y, sun.z);
    glUniform1i(u.mapping, app.mapping == Mapping::LatLong ? 1 : 0);
    glUniform1i(u.octaves, octaveCount(app));
    glUniform1f(u.sea, app.sea);
    glUniform1i(u.atmo, app.atmo ? 1 : 0);
    glUniform1i(u.clouds, app.clouds ? 1 : 0);
    // Colours are written as sRGB in logic.hpp and lit in linear light (see the gamma post), so convert once here.
    auto linear = [](Vec3 c) { return Vec3{std::pow(c.x, 2.2F), std::pow(c.y, 2.2F), std::pow(c.z, 2.2F)}; };
    std::array<float, 24> pal{};
    for (std::size_t i = 0; i < 8; ++i) {
        const Vec3 c = linear(p.palette[i]);
        pal[i * 3 + 0] = c.x;
        pal[i * 3 + 1] = c.y;
        pal[i * 3 + 2] = c.z;
    }
    glUniform3fv(u.pal, 8, pal.data());
    glUniform4f(u.p0, p.iceLatitude, p.clouds, p.atmosphereStrength, p.gloss);
    glUniform2f(u.p1, p.lava, p.cities);
    const Vec3 a = linear(p.atmosphere), cc = linear(p.cloudColor);
    glUniform3f(u.atmoCol, a.x, a.y, a.z);
    glUniform3f(u.cloudCol, cc.x, cc.y, cc.z);
}

}  // namespace

SDL_AppResult SDL_AppInit(void** appstate, int argc, char** argv) {
    auto* app = new App();
    *appstate = app;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) app->maxFrames = std::atol(argv[i + 1]);
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->win.create("planet-shader")) return SDL_APP_FAILURE;
    if (!app->hud.load()) {
        SDL_Log("hud failed to load");
        return SDL_APP_FAILURE;
    }
    const std::string source = sharedConstants() + kPlanetFs;
    app->planet = gd::gl::compile(source.c_str());
    app->blit = gd::gl::compile(kBlitFs);
    if (app->planet == 0 || app->blit == 0) return SDL_APP_FAILURE;
    const GLuint p = app->planet;
    app->u = Uniforms{gd::gl::uniform(p, "uCamPos"), gd::gl::uniform(p, "uRight"),    gd::gl::uniform(p, "uUp"),       gd::gl::uniform(p, "uFwd"),
                      gd::gl::uniform(p, "uPixelAngle"), gd::gl::uniform(p, "uSpin"), gd::gl::uniform(p, "uCloudSpin"), gd::gl::uniform(p, "uTilt"),
                      gd::gl::uniform(p, "uSun"), gd::gl::uniform(p, "uMapping"),     gd::gl::uniform(p, "uOctaves"), gd::gl::uniform(p, "uSea"),
                      gd::gl::uniform(p, "uAtmo"), gd::gl::uniform(p, "uClouds"),     gd::gl::uniform(p, "uPal"),     gd::gl::uniform(p, "uP0"),
                      gd::gl::uniform(p, "uP1"),   gd::gl::uniform(p, "uAtmoCol"),    gd::gl::uniform(p, "uCloudCol")};
    glUseProgram(app->blit);
    glUniform1i(gd::gl::uniform(app->blit, "uTex"), 0);
    if (!createTarget(*app)) return SDL_APP_FAILURE;
    buildChips(*app);
    updateSea(*app);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) return SDL_APP_SUCCESS;

    if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN && event->button.button == SDL_BUTTON_LEFT) {
        click(app, app.win.toLogical(*event));
    } else if (event->type == SDL_EVENT_MOUSE_MOTION && app.dragging) {
        const SDL_FPoint p = app.win.toLogical(*event);
        app.yaw -= (p.x - app.lastMouse.x) * 0.008F;
        app.pitch = clampPitch(app.pitch + (p.y - app.lastMouse.y) * 0.008F);
        app.lastMouse = p;
    } else if (event->type == SDL_EVENT_MOUSE_BUTTON_UP || event->type == SDL_EVENT_WINDOW_FOCUS_LOST || event->type == SDL_EVENT_WINDOW_MOUSE_LEAVE) {
        app.dragging = false;
    } else if (event->type == SDL_EVENT_MOUSE_WHEEL) {
        app.dist = clampDistance(app.dist * std::exp(-event->wheel.y * 0.1F));
    } else if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat) {
        const SDL_Keycode k = event->key.key;
        if (k >= SDLK_1 && k <= SDLK_4) setPreset(app, static_cast<int>(k - SDLK_1));
        if (k == SDLK_M) setMapping(app, app.mapping == Mapping::Sphere ? Mapping::LatLong : Mapping::Sphere);
        if (k == SDLK_A) app.atmo = !app.atmo;
        if (k == SDLK_C) app.clouds = !app.clouds;
        if (k == SDLK_SPACE) app.spinning = !app.spinning;
        if (k == SDLK_O) setOctaves(app, (app.octaves + 1) % static_cast<int>(kOctaveChoices.size()));
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const double frameTime = std::min(static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency()), kMaxFrameTime);
    app.last = now;
    if (app.spinning) {
        app.spinAngle = advanceAngle(app.spinAngle, kSpinRate, frameTime);
        app.cloudAngle = advanceAngle(app.cloudAngle, kCloudRate, frameTime);
    }

    const Camera cam = makeCamera(app.yaw, app.pitch, app.dist);

    // Pass 1: the planet into the render target.
    glBindFramebuffer(GL_FRAMEBUFFER, app.fbo);
    glViewport(0, 0, app.targetW, app.targetH);
    glUseProgram(app.planet);
    upload(app, cam);
    gd::gl::drawFullscreen();

    // Pass 2: scale it onto the canvas, then the HUD on top.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    app.win.begin();
    glUseProgram(app.blit);
    app.target.bind(0);
    gd::gl::drawFullscreen();

    gd::gl::Hud& hud = app.hud;
    const float pixelWorld = 2.0F * kTanHalfFov / static_cast<float>(app.targetH) * (app.dist - 1.0F);
    const float shown = std::min(static_cast<float>(octaveCount(app)), lodOctaves(pixelWorld));
    char right[96];
    SDL_snprintf(right, sizeof right, "%s · %s · %.1f of %d octaves shown", kPresets[static_cast<std::size_t>(app.preset)].name,
                 app.mapping == Mapping::Sphere ? "3D noise" : "lat/long noise", static_cast<double>(shown), octaveCount(app));
    hud.titleBar(gd::gl::kWidth, "[ PLANET SHADER ]", right);
    if (app.mapping == Mapping::LatLong) drawSeamMarks(app, cam);

    const gd::Color light{196, 208, 222};
    hud.text(24.0F, 402.0F, "drag: orbit · wheel: zoom", light);
    hud.text(24.0F, 424.0F, "keys: 1-4 M A C O space", light);
    drawChips(app);

    app.win.swap();
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        // SDL_AppQuit also runs after a failed SDL_AppInit, when there may be no GL context to delete things in.
        if (app->win.context != nullptr) {
            app->hud.destroy();
            if (app->target.id != 0) app->target.destroy();
            if (app->fbo != 0) glDeleteFramebuffers(1, &app->fbo);
            if (app->planet != 0) glDeleteProgram(app->planet);
            if (app->blit != 0) glDeleteProgram(app->blit);
        }
        app->win.destroy();
        delete app;
    }
}
