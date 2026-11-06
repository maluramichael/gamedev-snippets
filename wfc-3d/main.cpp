// Wave function collapse in 3D: a walled town grows out of 34 tiles and six-sided neighbour rules.
//
// The solver (logic.hpp) and the tile rules (castle.hpp) know nothing about drawing. This file turns the state of the
// solver into a picture: a decided cell shows its tile (pops in from the ground with a flash), an undecided cell is a
// translucent ghost cube that is the bigger, the more likely something solid is still going to appear there, and cells
// the propagation just touched glow orange. Every tile kind is one mesh that is drawn once per cell with instancing; the
// sun casts a shadow map (3x3 filtered) over the island, and the whole scene is flat shaded.
// Drag orbits the camera, the wheel zooms, a click on a ghost collapses that cell now. Time is accumulated frame time
// (clamped); nothing that grows without bound is ever sent to the GPU.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "castle.hpp"
#include "gl.hpp"
#include "logic.hpp"
#include "mesh.hpp"

namespace {

using namespace w3;

constexpr int kWidth = 12;  // grid cells in x
constexpr int kLayers = 6;  // y
constexpr int kDepth = 12;  // z

constexpr int kShadowSize = 2048;
constexpr float kShadowHalf = 12.0F;
constexpr float kFloorY = -2.0F;
constexpr int kWarmSteps = 1500;  // the first frame already shows a town under construction
constexpr double kHoldSeconds = 4.5;  // how long a finished world stays before the next seed (auto mode)

constexpr std::array<double, 4> kSpeeds{40.0, 400.0, 2500.0, 1.0e9};  // steps per second
constexpr int kMaxStepsPerFrame = 6000;
constexpr int kInstanceFloats = 7;  // position (3) and four values per instance

constexpr const char* kTileVs = R"(
uniform mat4 uViewProj;
uniform mat4 uLightVP;
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec3 iPos;    // middle of the bottom face of the cell
layout(location = 4) in vec4 iData;   // scale, glow, color jitter 0..1, ambient occlusion on (1) or off (0)
out vec3 vNormal;
out vec3 vColor;
out float vGlow;
out float vLocalY;
out float vAo;
out vec4 vLight;
void main() {
    vec3 p = iPos + aPos * iData.x;
    vNormal = aNormal;
    vColor = aColor * (0.94 + 0.12 * iData.z);
    vGlow = iData.y;
    vLocalY = aPos.y;
    vAo = iData.w;
    vLight = uLightVP * vec4(p + aNormal * 0.035, 1.0);   // pushed along the normal against shadow acne
    gl_Position = uViewProj * vec4(p, 1.0);
}
)";

// The shadow lookup is shared by the tiles and the floor.
constexpr const char* kShadowGlsl = R"(
precision highp sampler2DShadow;
uniform sampler2DShadow uShadow;
float shadowAt(vec4 l) {
    vec3 c = l.xyz / l.w * 0.5 + 0.5;
    if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;
    float sum = 0.0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            sum += texture(uShadow, vec3(c.xy + vec2(float(x), float(y)) * (1.3 / 2048.0), c.z - 0.0010));
        }
    }
    return sum / 9.0;
}
)";

constexpr const char* kTileFs = R"(
uniform vec3 uSun;
in vec3 vNormal;
in vec3 vColor;
in float vGlow;
in float vLocalY;
in float vAo;
in vec4 vLight;
out vec4 outColor;
void main() {
    vec3 n = normalize(vNormal);
    float diffuse = max(dot(n, uSun), 0.0);
    float sh = shadowAt(vLight);
    vec3 sky = vec3(0.72, 0.83, 0.95);
    vec3 ground = vec3(0.66, 0.60, 0.52);
    vec3 ambient = mix(ground, sky, n.y * 0.5 + 0.5) * 0.52;
    vec3 lit = ambient + vec3(1.0, 0.94, 0.84) * (diffuse * sh) * 0.66;
    float ao = mix(1.0 - 0.22 * vAo, 1.0, smoothstep(0.0, 0.5, vLocalY));
    vec3 color = vColor * lit * ao;
    color = mix(color, vec3(0.80, 1.0, 0.96), vGlow * 0.5) + vec3(0.08, 0.42, 0.38) * vGlow * 0.55;
    outColor = vec4(color, 1.0);
}
)";

constexpr const char* kDepthVs = R"(
uniform mat4 uLightVP;
layout(location = 0) in vec3 aPos;
layout(location = 3) in vec3 iPos;
layout(location = 4) in vec4 iData;
void main() { gl_Position = uLightVP * vec4(iPos + aPos * iData.x, 1.0); }
)";
constexpr const char* kDepthFs = "void main() {}\n";

constexpr const char* kGhostVs = R"(
uniform mat4 uViewProj;
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 3) in vec3 iPos;    // middle of the cell
layout(location = 4) in vec4 iData;   // size, alpha, heat 0..1
out vec3 vNormal;
out float vAlpha;
out float vHeat;
void main() {
    vNormal = aNormal;
    vAlpha = iData.y;
    vHeat = iData.z;
    gl_Position = uViewProj * vec4(iPos + aPos * iData.x, 1.0);
}
)";
constexpr const char* kGhostFs = R"(
in vec3 vNormal;
in float vAlpha;
in float vHeat;
out vec4 outColor;
void main() {
    vec3 base = mix(vec3(0.05, 0.58, 0.53), vec3(0.93, 0.36, 0.05), vHeat);
    float shade = 0.78 + 0.22 * normalize(vNormal).y;
    outColor = vec4(base * shade, vAlpha);
}
)";

constexpr const char* kLineVs = R"(
uniform mat4 uViewProj;
uniform vec3 uCenter;
uniform float uSize;
layout(location = 0) in vec3 aPos;
void main() { gl_Position = uViewProj * vec4(uCenter + aPos * uSize, 1.0); }
)";
constexpr const char* kLineFs = R"(
uniform vec4 uColor;
out vec4 outColor;
void main() { outColor = uColor; }
)";

constexpr const char* kFloorVs = R"(
uniform mat4 uViewProj;
uniform mat4 uLightVP;
uniform float uY;
uniform float uRadius;
out vec3 vWorld;
out vec4 vLight;
void main() {
    vec2 c = vec2(float(gl_VertexID & 1) * 2.0 - 1.0, float((gl_VertexID >> 1) & 1) * 2.0 - 1.0) * uRadius;
    vWorld = vec3(c.x, uY, c.y);
    vLight = uLightVP * vec4(vWorld, 1.0);
    gl_Position = uViewProj * vec4(vWorld, 1.0);
}
)";
constexpr const char* kFloorFs = R"(
in vec3 vWorld;
in vec4 vLight;
out vec4 outColor;
void main() {
    float sh = shadowAt(vLight);
    float fade = 1.0 - smoothstep(7.0, 15.0, length(vWorld.xz));
    outColor = vec4(0.06, 0.20, 0.22, (1.0 - sh) * 0.30 * fade);
}
)";

constexpr const char* kBackgroundFs = R"(
in vec2 vUv;
out vec4 outColor;
void main() {
    vec3 top = vec3(0.985, 0.985, 0.975);
    vec3 bottom = vec3(0.86, 0.93, 0.92);
    float d = length(vUv - vec2(0.5, 0.42));
    outColor = vec4(mix(mix(top, bottom, vUv.y), vec3(0.93, 0.97, 0.96), (1.0 - smoothstep(0.0, 0.7, d)) * 0.5), 1.0);
}
)";

struct Range {
    GLint first = 0;
    GLsizei count = 0;
};

struct Programs {
    GLuint tile = 0, depth = 0, ghost = 0, line = 0, floor = 0, background = 0;
};

struct App {
    gd::gl::Window win;
    gd::gl::Hud hud;
    Programs prog;

    GLuint meshVbo = 0, vao = 0, instVbo = 0, ghostVbo = 0, lineVbo = 0, lineVao = 0, floorVao = 0;
    GLuint shadowFbo = 0, shadowTex = 0;
    std::vector<Range> body, cap;
    Range slab, cube;

    CastleSet castle;
    std::unique_ptr<Solver> solver;
    ChipLayout layout;

    // Settings the chips change.
    Order order = Order::Layers;
    Recovery recovery = Recovery::Backtrack;
    std::size_t speed = 1;
    bool autoRun = true;
    bool spin = true;
    std::uint64_t seed = 8;

    // What the screen shows for each cell.
    std::vector<int> shown;       // tile index while the cell is decided, -1 otherwise
    std::vector<float> appear;    // 0 .. 1, the pop-in of a tile
    std::vector<float> glow;      // flash of a cell that just got decided
    std::vector<float> pulse;     // orange: the propagation just changed this cell
    StepClock clock;
    double holdTimer = 0.0;

    OrbitCamera cam;
    bool dragging = false;
    bool moved = false;
    SDL_FPoint downAt{0.0F, 0.0F};
    SDL_FPoint lastMouse{0.0F, 0.0F};

    std::vector<float> instances;  // solid tiles, grouped by tile index, caps after them
    std::vector<float> ghosts;
    std::vector<std::size_t> bodyStart, bodyCount, capStart, capCount;

    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

// ---------------------------------------------------------------------------------------------- GL setup

// gl.hpp's Window::create asks for no depth buffer; the town needs one, and MSAA smooths the edges.
bool createWindow(gd::gl::Window& win) {
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 0);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    win.window = SDL_CreateWindow("wfc-3d", gd::gl::kWidth, gd::gl::kHeight, SDL_WINDOW_OPENGL | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (win.window == nullptr) {
        SDL_Log("window failed: %s", SDL_GetError());
        return false;
    }
    win.context = SDL_GL_CreateContext(win.window);
    if (win.context == nullptr) {
        SDL_Log("OpenGL ES 3.0 context failed: %s", SDL_GetError());
        return false;
    }
    SDL_GL_SetSwapInterval(1);
    return true;
}

GLuint makeBuffer(GLenum target, const void* data, std::size_t bytes, GLenum usage) {
    GLuint id = 0;
    glGenBuffers(1, &id);
    glBindBuffer(target, id);
    glBufferData(target, static_cast<GLsizeiptr>(bytes), data, usage);
    return id;
}

GLuint makeProgram(const char* vs, const char* fs, bool withShadow) {
    const std::string source = withShadow ? std::string(kShadowGlsl) + fs : std::string(fs);
    return gd::gl::compile(vs, source.c_str());
}

/// Points the two per-instance attributes at `buffer`, `floatOffset` floats from its start.
void bindInstances(GLuint buffer, std::size_t floatOffset) {
    glBindBuffer(GL_ARRAY_BUFFER, buffer);
    const auto stride = static_cast<GLsizei>(kInstanceFloats * sizeof(float));
    const auto base = reinterpret_cast<const char*>(floatOffset * sizeof(float));
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, base);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, stride, base + 3 * sizeof(float));
}

bool buildGpuObjects(App& app) {
    app.prog.tile = makeProgram(kTileVs, kTileFs, true);
    app.prog.depth = gd::gl::compile(kDepthVs, kDepthFs);
    app.prog.ghost = gd::gl::compile(kGhostVs, kGhostFs);
    app.prog.line = gd::gl::compile(kLineVs, kLineFs);
    app.prog.floor = makeProgram(kFloorVs, kFloorFs, true);
    app.prog.background = gd::gl::compile(kBackgroundFs);
    if (app.prog.tile == 0 || app.prog.depth == 0 || app.prog.ghost == 0 || app.prog.line == 0 || app.prog.floor == 0 || app.prog.background == 0) return false;

    // All meshes in one vertex buffer: one range per tile (body and battlements), the slab, the ghost cube.
    Verts all;
    const auto append = [&all](const Verts& v) {
        Range r{static_cast<GLint>(all.size()), static_cast<GLsizei>(v.size())};
        all.insert(all.end(), v.begin(), v.end());
        return r;
    };
    for (int t = 0; t < app.castle.set.size(); ++t) {
        app.body.push_back(append(buildTile(app.castle.tile(t))));
        app.cap.push_back(append(buildCap(app.castle.tile(t))));
    }
    app.slab = append(buildSlab(kWidth, kDepth));
    app.cube = append(buildCube());
    app.meshVbo = makeBuffer(GL_ARRAY_BUFFER, all.data(), all.size() * sizeof(Vertex), GL_STATIC_DRAW);

    app.instVbo = makeBuffer(GL_ARRAY_BUFFER, nullptr, 8, GL_STREAM_DRAW);
    app.ghostVbo = makeBuffer(GL_ARRAY_BUFFER, nullptr, 8, GL_STREAM_DRAW);

    glGenVertexArrays(1, &app.vao);
    glBindVertexArray(app.vao);
    glBindBuffer(GL_ARRAY_BUFFER, app.meshVbo);
    const auto stride = static_cast<GLsizei>(sizeof(Vertex));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(float)));
    glEnableVertexAttribArray(3);
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(3, 1);
    glVertexAttribDivisor(4, 1);
    bindInstances(app.instVbo, 0);

    // The wire cube (12 edges) that marks the cell the solver collapses next.
    std::vector<float> lines;
    const float c[8][3] = {{-0.5F, -0.5F, -0.5F}, {0.5F, -0.5F, -0.5F}, {0.5F, -0.5F, 0.5F}, {-0.5F, -0.5F, 0.5F},
                           {-0.5F, 0.5F, -0.5F},  {0.5F, 0.5F, -0.5F},  {0.5F, 0.5F, 0.5F},  {-0.5F, 0.5F, 0.5F}};
    const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& e : edges) {
        for (const int k : e) lines.insert(lines.end(), {c[k][0], c[k][1], c[k][2]});
    }
    app.lineVbo = makeBuffer(GL_ARRAY_BUFFER, lines.data(), lines.size() * sizeof(float), GL_STATIC_DRAW);
    glGenVertexArrays(1, &app.lineVao);
    glBindVertexArray(app.lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, app.lineVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glGenVertexArrays(1, &app.floorVao);  // the floor needs no vertex data, but WebGL2 wants a VAO bound

    // Shadow map: a depth texture the hardware compares against (that comparison is the free 2x2 filter).
    glGenTextures(1, &app.shadowTex);
    glBindTexture(GL_TEXTURE_2D, app.shadowTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kShadowSize, kShadowSize, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glGenFramebuffers(1, &app.shadowFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, app.shadowFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, app.shadowTex, 0);
    const GLenum none = GL_NONE;
    glDrawBuffers(1, &none);
    glReadBuffer(GL_NONE);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindVertexArray(0);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        SDL_Log("shadow framebuffer incomplete: 0x%x", static_cast<unsigned>(status));
        return false;
    }
    return true;
}

void setMat4(GLuint program, const char* name, const Mat4& m) { glUniformMatrix4fv(gd::gl::uniform(program, name), 1, GL_TRUE, &m.m[0][0]); }
void setVec3(GLuint program, const char* name, Vec3 v) { glUniform3f(gd::gl::uniform(program, name), v.x, v.y, v.z); }

// ---------------------------------------------------------------------------------------------- world state

void clearDisplay(App& app) {
    const auto n = static_cast<std::size_t>(app.solver->cells());
    app.shown.assign(n, -1);
    app.appear.assign(n, 0.0F);
    app.glow.assign(n, 0.0F);
    app.pulse.assign(n, 0.0F);
}

/// What the screen shows follows the solver: a cell that became decided starts its pop-in, a cell that was undone loses
/// its tile.
void syncDisplay(App& app) {
    const Solver& s = *app.solver;
    for (int i = 0; i < s.cells(); ++i) {
        const Mask m = s.at(i);
        const auto idx = static_cast<std::size_t>(i);
        if (count(m) == 1) {
            const int t = lowest(m);
            if (app.shown[idx] != t) {
                app.shown[idx] = t;
                app.appear[idx] = 0.0F;
                app.glow[idx] = 1.0F;
            }
        } else {
            app.shown[idx] = -1;
        }
    }
}

/// Everything that is decided stands completely, nothing glows: the picture of a world that has been there all along.
void settleDisplay(App& app) {
    syncDisplay(app);
    std::fill(app.appear.begin(), app.appear.end(), 1.0F);
    std::fill(app.glow.begin(), app.glow.end(), 0.0F);
    std::fill(app.pulse.begin(), app.pulse.end(), 0.0F);
    app.solver->touched.clear();
}

void startWorld(App& app, std::uint64_t seed) {
    app.seed = seed;
    app.solver->order = app.order;
    app.solver->recovery = app.recovery;
    app.solver->reset(seed);
    app.solver->touched.clear();
    clearDisplay(app);
    app.clock.reset();
    app.holdTimer = 0.0;
}

/// Runs `steps` solver steps and marks what they changed: touched cells pulse orange.
void advance(App& app, long steps) {
    Solver& s = *app.solver;
    s.run(steps);
    for (const int cell : s.touched) app.pulse[static_cast<std::size_t>(cell)] = 1.0F;
    s.touched.clear();
}

/// One decision: steps until the solver has collapsed one cell and settled its consequences (or finished).
void stepDecision(App& app) {
    Solver& s = *app.solver;
    bool collapsed = false;
    long guard = 0;
    while (!s.finished() && guard++ < s.maxSteps()) {
        const bool isCollapse = s.status == Status::Running && s.queue.empty();
        s.step();
        if (isCollapse) collapsed = true;
        if (collapsed && s.status == Status::Running && s.queue.empty()) break;
    }
    for (const int cell : s.touched) app.pulse[static_cast<std::size_t>(cell)] = 1.0F;
    s.touched.clear();
}

void setOrder(App& app, Order o) {
    app.order = o;
    startWorld(app, app.seed);  // same seed, other order: the worlds can be compared
}

void setRecovery(App& app, Recovery r) {
    app.recovery = r;
    startWorld(app, app.seed);
}

void handleChip(App& app, Chip id) {
    switch (id) {
        case Chip::New: startWorld(app, app.seed + 1); break;
        case Chip::Step:
            app.autoRun = false;
            stepDecision(app);
            break;
        case Chip::Auto: app.autoRun = !app.autoRun; break;
        case Chip::Slow: app.speed = 0; break;
        case Chip::Medium: app.speed = 1; break;
        case Chip::Fast: app.speed = 2; break;
        case Chip::Max: app.speed = 3; break;
        case Chip::Entropy: setOrder(app, Order::Entropy); break;
        case Chip::Layers: setOrder(app, Order::Layers); break;
        case Chip::Random: setOrder(app, Order::Random); break;
        case Chip::Undo: setRecovery(app, Recovery::Backtrack); break;
        case Chip::Restart: setRecovery(app, Recovery::Restart); break;
        case Chip::SeedDown: startWorld(app, app.seed > 1 ? app.seed - 1 : 1); break;
        case Chip::SeedUp: startWorld(app, app.seed + 1); break;
        case Chip::Spin: app.spin = !app.spin; break;
        case Chip::Count: break;
    }
}

bool chipActive(const App& app, Chip id) {
    switch (id) {
        case Chip::Auto: return app.autoRun;
        case Chip::Slow: return app.speed == 0;
        case Chip::Medium: return app.speed == 1;
        case Chip::Fast: return app.speed == 2;
        case Chip::Max: return app.speed == 3;
        case Chip::Entropy: return app.order == Order::Entropy;
        case Chip::Layers: return app.order == Order::Layers;
        case Chip::Random: return app.order == Order::Random;
        case Chip::Undo: return app.recovery == Recovery::Backtrack;
        case Chip::Restart: return app.recovery == Recovery::Restart;
        case Chip::Spin: return app.spin;
        default: return false;
    }
}

float hash01(int x, int y, int z, std::uint64_t seed) {
    std::uint64_t h = static_cast<std::uint64_t>(x) * 73856093ULL ^ static_cast<std::uint64_t>(y) * 19349663ULL ^ static_cast<std::uint64_t>(z) * 83492791ULL ^ seed * 2654435761ULL;
    h ^= h >> 13;
    h *= 0x9E3779B97F4A7C15ULL;
    h ^= h >> 29;
    return static_cast<float>(h & 0xFFFFU) / 65535.0F;
}

// ---------------------------------------------------------------------------------------------- drawing

void pushInstance(std::vector<float>& out, Vec3 pos, float a, float b, float c, float d) { out.insert(out.end(), {pos.x, pos.y, pos.z, a, b, c, d}); }

/// Fills `instances`: the solid tiles grouped by tile index (so each kind is one draw call), then the battlements.
void buildInstances(App& app) {
    const Solver& s = *app.solver;
    const std::size_t n = app.castle.tiles.size();
    std::vector<std::vector<float>> bodies(n), caps(n);
    for (int i = 0; i < s.cells(); ++i) {
        const auto idx = static_cast<std::size_t>(i);
        const int t = app.shown[idx];
        if (t < 0 || !app.castle.set.isSolid(t)) continue;
        const Int3 c = s.coords(i);
        const Vec3 base = cellBase(s.width, s.depth, c.x, c.y, c.z);
        const float scale = easeOutBack(app.appear[idx]);
        const float jitter = hash01(c.x, c.y, c.z, app.seed);
        pushInstance(bodies[static_cast<std::size_t>(t)], base, scale, app.glow[idx], jitter, 1.0F);
        if (!hasArms(app.castle.tile(t).kind)) continue;
        bool open = c.y + 1 >= s.height;  // nothing above: battlements
        if (!open) {
            const int above = app.shown[static_cast<std::size_t>(s.index(c.x, c.y + 1, c.z))];
            open = above >= 0 && !app.castle.set.isSolid(above);
        }
        if (open) pushInstance(caps[static_cast<std::size_t>(t)], base, scale, app.glow[idx], jitter, 1.0F);
    }
    app.instances.clear();
    app.bodyStart.assign(n, 0);
    app.bodyCount.assign(n, 0);
    app.capStart.assign(n, 0);
    app.capCount.assign(n, 0);
    // The slab is instance 0.
    pushInstance(app.instances, {0.0F, 0.0F, 0.0F}, 1.0F, 0.0F, 0.5F, 0.0F);
    for (std::size_t t = 0; t < n; ++t) {
        app.bodyStart[t] = app.instances.size() / kInstanceFloats;
        app.bodyCount[t] = bodies[t].size() / kInstanceFloats;
        app.instances.insert(app.instances.end(), bodies[t].begin(), bodies[t].end());
    }
    for (std::size_t t = 0; t < n; ++t) {
        app.capStart[t] = app.instances.size() / kInstanceFloats;
        app.capCount[t] = caps[t].size() / kInstanceFloats;
        app.instances.insert(app.instances.end(), caps[t].begin(), caps[t].end());
    }
}

/// Fills `ghosts`: one translucent cube per undecided cell that could still turn into something solid.
void buildGhosts(App& app, int nextCell) {
    const Solver& s = *app.solver;
    app.ghosts.clear();
    for (int i = 0; i < s.cells(); ++i) {
        const Mask m = s.at(i);
        if (count(m) < 2) continue;
        const auto idx = static_cast<std::size_t>(i);
        const double share = solidShare(app.castle.set, m);
        const float pulse = app.pulse[idx];
        const bool queued = s.queued[idx] != 0;
        if (share < 0.06 && pulse < 0.05F) continue;
        const Int3 c = s.coords(i);
        const Vec3 mid = cellBase(s.width, s.depth, c.x, c.y, c.z) + Vec3{0.0F, 0.5F, 0.0F};
        const float u = static_cast<float>(share);
        const float heat = std::max(pulse, queued ? 0.55F : 0.0F);
        float size = 0.08F + 0.30F * std::sqrt(u) + 0.12F * pulse;
        float alpha = 0.03F + 0.09F * u + 0.12F * heat;
        if (i == nextCell) {
            size = 0.62F;
            alpha = 0.28F;
        }
        pushInstance(app.ghosts, mid, size, alpha, i == nextCell ? 0.85F : heat, 0.0F);
    }
}

void drawTiles(App& app, GLuint program, bool withCaps) {
    glBindVertexArray(app.vao);
    bindInstances(app.instVbo, 0);
    (void)program;
    // The slab: instance 0.
    glDrawArraysInstanced(GL_TRIANGLES, app.slab.first, app.slab.count, 1);
    for (std::size_t t = 0; t < app.body.size(); ++t) {
        if (app.bodyCount[t] == 0 || app.body[t].count == 0) continue;
        bindInstances(app.instVbo, app.bodyStart[t] * kInstanceFloats);
        glDrawArraysInstanced(GL_TRIANGLES, app.body[t].first, app.body[t].count, static_cast<GLsizei>(app.bodyCount[t]));
    }
    if (!withCaps) return;
    for (std::size_t t = 0; t < app.cap.size(); ++t) {
        if (app.capCount[t] == 0 || app.cap[t].count == 0) continue;
        bindInstances(app.instVbo, app.capStart[t] * kInstanceFloats);
        glDrawArraysInstanced(GL_TRIANGLES, app.cap[t].first, app.cap[t].count, static_cast<GLsizei>(app.capCount[t]));
    }
}

const Vec3 kSun = normalize({-0.38F, 0.82F, 0.52F});

Mat4 lightMatrix() {
    const Vec3 centre{0.0F, 1.6F, 0.0F};
    return ortho(kShadowHalf, kShadowHalf, 1.0F, 90.0F) * lookAt(centre + kSun * 40.0F, centre, {0.0F, 1.0F, 0.0F});
}

void hudChip(gd::gl::Hud& hud, const ChipRect& c, const char* label, bool active) {
    hud.card(c.x, c.y, c.w, c.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
    hud.text(c.x + (c.w - hud.width(label, gd::gl::Hud::Style::Bold)) / 2.0F, c.y + 2.0F, label, active ? gd::kWhite : gd::kInk, gd::gl::Hud::Style::Bold);
}

const char* chipLabel(Chip id) {
    switch (id) {
        case Chip::New: return "new";
        case Chip::Step: return "step";
        case Chip::Auto: return "auto";
        case Chip::Slow: return "slow";
        case Chip::Medium: return "mid";
        case Chip::Fast: return "fast";
        case Chip::Max: return "max";
        case Chip::Entropy: return "entropy";
        case Chip::Layers: return "layers";
        case Chip::Random: return "random";
        case Chip::Undo: return "undo";
        case Chip::Restart: return "restart";
        case Chip::SeedDown: return "<";
        case Chip::SeedUp: return ">";
        case Chip::Spin: return "spin";
        case Chip::Count: break;
    }
    return "";
}

void drawHud(App& app) {
    gd::gl::Hud& hud = app.hud;
    const Solver& s = *app.solver;
    char dims[64];
    SDL_snprintf(dims, sizeof dims, "%d×%d×%d cells · %d tiles · 6 neighbours", s.width, s.height, s.depth, app.castle.set.size());
    hud.titleBar(gd::gl::kWidth, "[ WAVE FUNCTION COLLAPSE 3D ]", dims);

    char line[96];
    hud.fillRect(12.0F, 40.0F, 372.0F, 70.0F, gd::Color{250, 250, 249, 220});
    hud.strokeRect(12.0F, 40.0F, 372.0F, 70.0F, gd::kTealDark);
    SDL_snprintf(line, sizeof line, "decided %s / %s", groupDigits(s.collapsedCount()).c_str(), groupDigits(s.cells()).c_str());
    hud.text(22.0F, 46.0F, line, gd::kInk, gd::gl::Hud::Style::Bold);
    const char* state = s.status == Status::Done ? "DONE" : s.status == Status::GaveUp ? "GAVE UP" : s.status == Status::Contradiction ? "CONFLICT" : app.autoRun ? "RUNNING" : "PAUSED";
    const gd::Color stateColor = s.status == Status::Done ? gd::kTeal : (s.status == Status::GaveUp || s.status == Status::Contradiction) ? gd::kOrange : gd::kMuted;
    hud.text(384.0F - 10.0F - hud.width(state, gd::gl::Hud::Style::Bold), 46.0F, state, stateColor, gd::gl::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "steps %s · seed %llu", groupDigits(s.counters.steps).c_str(), static_cast<unsigned long long>(app.seed));
    hud.text(22.0F, 66.0F, line, gd::kMuted);
    SDL_snprintf(line, sizeof line, "conflicts %d · undone %d · restarts %d", s.counters.contradictions, s.counters.backtracks, s.counters.restarts);
    hud.text(22.0F, 86.0F, line, s.counters.contradictions > 0 ? gd::kOrange : gd::kMuted);

    const char* hint = "drag: orbit · wheel: zoom · click: collapse";
    hud.text(960.0F - 12.0F - hud.width(hint), 40.0F, hint, gd::kInk);

    hud.fillRect(0.0F, app.layout.panelTop, 960.0F, app.layout.panelHeight, gd::Color{250, 250, 249, 240});
    hud.fillRect(0.0F, app.layout.panelTop, 960.0F, 2.0F, gd::kTealDark);
    for (const GroupLabel& g : app.layout.labels) hud.text(g.x, g.y, g.text, gd::kMuted);
    for (const ChipRect& c : app.layout.chips) hudChip(hud, c, chipLabel(c.id), chipActive(app, c.id));
    const char* keys = "keys: N S space 1-4";
    hud.text(960.0F - 12.0F - hud.width(keys), app.layout.panelTop + 11.0F, keys, gd::kMuted);
}

void renderFrame(App& app) {
    const Solver& s = *app.solver;
    const Mat4 lightVP = lightMatrix();
    buildInstances(app);
    const int nextCell = (s.status == Status::Running && s.queue.empty()) ? s.pickCell() : -1;
    buildGhosts(app, nextCell);

    glBindBuffer(GL_ARRAY_BUFFER, app.instVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(app.instances.size() * sizeof(float)), app.instances.data(), GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, app.ghostVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(std::max<std::size_t>(app.ghosts.size(), 1) * sizeof(float)), app.ghosts.data(), GL_STREAM_DRAW);

    // 1. The sun's view: depth only.
    glBindFramebuffer(GL_FRAMEBUFFER, app.shadowFbo);
    glViewport(0, 0, kShadowSize, kShadowSize);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0F, 4.0F);
    glUseProgram(app.prog.depth);
    setMat4(app.prog.depth, "uLightVP", lightVP);
    drawTiles(app, app.prog.depth, true);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // 2. The picture.
    app.win.begin();
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);
    const Mat4 viewProj = shiftedPerspective(kFovY, static_cast<float>(gd::gl::kWidth) / gd::gl::kHeight, kNear, kFar, kViewShift) * app.cam.view();

    glDisable(GL_DEPTH_TEST);
    glUseProgram(app.prog.background);
    gd::gl::drawFullscreen();

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, app.shadowTex);
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_DEPTH_TEST);

    // The floor only shows the shadow of the island.
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glUseProgram(app.prog.floor);
    setMat4(app.prog.floor, "uViewProj", viewProj);
    setMat4(app.prog.floor, "uLightVP", lightVP);
    glUniform1f(gd::gl::uniform(app.prog.floor, "uY"), kFloorY);
    glUniform1f(gd::gl::uniform(app.prog.floor, "uRadius"), 18.0F);
    glUniform1i(gd::gl::uniform(app.prog.floor, "uShadow"), 1);
    glBindVertexArray(app.floorVao);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);

    glUseProgram(app.prog.tile);
    setMat4(app.prog.tile, "uViewProj", viewProj);
    setMat4(app.prog.tile, "uLightVP", lightVP);
    setVec3(app.prog.tile, "uSun", kSun);
    glUniform1i(gd::gl::uniform(app.prog.tile, "uShadow"), 1);
    drawTiles(app, app.prog.tile, true);

    // Ghosts of the undecided cells and the frame of the next one.
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glUseProgram(app.prog.ghost);
    setMat4(app.prog.ghost, "uViewProj", viewProj);
    glBindVertexArray(app.vao);
    bindInstances(app.ghostVbo, 0);
    if (!app.ghosts.empty()) glDrawArraysInstanced(GL_TRIANGLES, app.cube.first, app.cube.count, static_cast<GLsizei>(app.ghosts.size() / kInstanceFloats));
    if (nextCell >= 0) {
        const Int3 c = s.coords(nextCell);
        const Vec3 mid = cellBase(s.width, s.depth, c.x, c.y, c.z) + Vec3{0.0F, 0.5F, 0.0F};
        glUseProgram(app.prog.line);
        setMat4(app.prog.line, "uViewProj", viewProj);
        setVec3(app.prog.line, "uCenter", mid);
        glUniform4f(gd::gl::uniform(app.prog.line, "uColor"), 0.93F, 0.36F, 0.05F, 1.0F);
        glBindVertexArray(app.lineVao);
        glUniform1f(gd::gl::uniform(app.prog.line, "uSize"), 1.0F);
        glDrawArrays(GL_LINES, 0, 24);
        glUniform1f(gd::gl::uniform(app.prog.line, "uSize"), 1.04F);
        glDrawArrays(GL_LINES, 0, 24);
    }
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glBindVertexArray(0);

    glDisable(GL_DEPTH_TEST);
    drawHud(app);
}

}  // namespace

// ---------------------------------------------------------------------------------------------- SDL callbacks

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
    if (!createWindow(app->win)) return SDL_APP_FAILURE;
    if (!app->hud.load()) {
        SDL_Log("hud failed to load");
        return SDL_APP_FAILURE;
    }
    app->castle = makeCastle();
    if (!app->castle.set.symmetric()) {
        SDL_Log("tile rules are not symmetric");
        return SDL_APP_FAILURE;
    }
    if (!buildGpuObjects(*app)) return SDL_APP_FAILURE;
    app->layout = chipLayout();
    app->solver = std::make_unique<Solver>(app->castle.set, kWidth, kLayers, kDepth, app->seed, app->order, app->recovery);
    app->solver->trackTouched = true;
    clearDisplay(*app);
    app->solver->run(kWarmSteps);
    settleDisplay(*app);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) return SDL_APP_SUCCESS;

    switch (event->type) {
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (event->button.button == SDL_BUTTON_LEFT) {
                const SDL_FPoint p = app.win.toLogical(*event);
                if (const ChipRect* chip = hitChip(app.layout.chips, p.x, p.y)) {
                    handleChip(app, chip->id);
                } else if (p.y > 30.0F && p.y < app.layout.panelTop) {
                    app.dragging = true;
                    app.moved = false;
                    app.downAt = p;
                    app.lastMouse = p;
                }
            }
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.dragging) {
                if ((event->motion.state & SDL_BUTTON_LMASK) == 0) {  // the release happened outside and got lost
                    app.dragging = false;
                    break;
                }
                const SDL_FPoint p = app.win.toLogical(*event);
                if (std::hypot(p.x - app.downAt.x, p.y - app.downAt.y) > 4.0F) {
                    app.moved = true;
                    app.spin = false;
                }
                if (app.moved) app.cam.orbit(-(p.x - app.lastMouse.x) * 0.008F, (p.y - app.lastMouse.y) * 0.006F);
                app.lastMouse = p;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT && app.dragging) {
                app.dragging = false;
                if (!app.moved) {
                    const SDL_FPoint p = app.win.toLogical(*event);
                    const Ray ray = rayThrough(app.cam, p.x / kCanvasWidth, p.y / kCanvasHeight, kCanvasWidth / kCanvasHeight, kViewShift);
                    const int cell = pickUndecided(*app.solver, ray);
                    if (cell >= 0 && app.solver->collapseCell(cell)) {
                        app.solver->touched.push_back(cell);
                        for (const int c : app.solver->touched) app.pulse[static_cast<std::size_t>(c)] = 1.0F;
                        app.solver->touched.clear();
                    }
                }
            }
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST:
        case SDL_EVENT_WINDOW_MOUSE_LEAVE: app.dragging = false; break;
        case SDL_EVENT_MOUSE_WHEEL: app.cam.zoom(std::clamp(event->wheel.y, -1.0F, 1.0F)); break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here is a toggle or a one-shot
            switch (event->key.key) {
                case SDLK_N: handleChip(app, Chip::New); break;
                case SDLK_S: handleChip(app, Chip::Step); break;
                case SDLK_SPACE: handleChip(app, Chip::Auto); break;
                case SDLK_1: handleChip(app, Chip::Slow); break;
                case SDLK_2: handleChip(app, Chip::Medium); break;
                case SDLK_3: handleChip(app, Chip::Fast); break;
                case SDLK_4: handleChip(app, Chip::Max); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    Solver& s = *app.solver;

    const Uint64 now = SDL_GetPerformanceCounter();
    const double frameTime = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    const auto dt = static_cast<float>(std::clamp(frameTime, 0.0, 0.25));

    if (app.autoRun && !s.finished()) advance(app, app.clock.take(frameTime, kSpeeds[app.speed], kMaxStepsPerFrame));
    if (s.finished() && app.autoRun) {
        app.holdTimer += dt;
        if (app.holdTimer > kHoldSeconds) startWorld(app, app.seed + 1);
    }
    syncDisplay(app);
    for (std::size_t i = 0; i < app.appear.size(); ++i) {
        app.appear[i] = std::min(1.0F, app.appear[i] + dt * 3.2F);
        app.glow[i] = decay(app.glow[i], 2.4F, dt);
        app.pulse[i] = decay(app.pulse[i], 3.6F, dt);
    }
    if (app.spin) app.cam.yaw += 0.10F * dt;

    renderFrame(app);

    app.win.swap();
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app == nullptr) return;
    if (app->win.context != nullptr) {  // after a failed init there may be no context, and GL calls without one throw in Emscripten
        app->hud.destroy();
        for (const GLuint p : {app->prog.tile, app->prog.depth, app->prog.ghost, app->prog.line, app->prog.floor, app->prog.background}) glDeleteProgram(p);
        const GLuint buffers[] = {app->meshVbo, app->instVbo, app->ghostVbo, app->lineVbo};
        glDeleteBuffers(4, buffers);
        const GLuint arrays[] = {app->vao, app->lineVao, app->floorVao};
        glDeleteVertexArrays(3, arrays);
        glDeleteFramebuffers(1, &app->shadowFbo);
        glDeleteTextures(1, &app->shadowTex);
    }
    app->win.destroy();
    delete app;
}
