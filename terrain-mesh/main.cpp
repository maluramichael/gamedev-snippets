// Terrain in 3D: a height field cut into chunks, each chunk a grid mesh, far chunks with fewer vertices.
//
// All the meshes are built once on the CPU (logic.hpp, tested natively) for every chunk and every LOD and sit in one vertex
// buffer. Per frame the CPU only measures how far each chunk is from the camera, picks its LOD and issues one draw call per
// chunk; the shaders do nothing clever. The chips switch the three things that decide whether the chunks fit together:
// world or chunk-local noise coordinates (seams), skirts (cracks between LODs), and how far the fine levels reach. The view
// chips tint the chunks by LOD or show the wireframe, where skirts are orange. The camera flies a circle by itself until you
// grab it (drag to look, WASD or arrows to move, wheel to move forward).

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "gl.hpp"
#include "logic.hpp"

namespace {

constexpr float kPanelTop = 466.0F;
constexpr float kFogDistance = 760.0F;
constexpr float kPathRadius = 205.0F;
constexpr double kPathSpeed = 0.075;   // radians per second, a lap takes about 84 s
constexpr float kFlightHeight = 70.0F;  // above the ground, in the automatic flight
constexpr float kFlightPitch = -0.4F;  // looking down a little, radians
constexpr float kClearance = 5.0F;      // never lower than this above the ground
constexpr float kFlySpeed = 70.0F;      // metres per second with the keys
constexpr float kWheelStep = 18.0F;     // metres per wheel notch
constexpr int kChunkChoices[3] = {16, 32, 64};
constexpr float kLodChoices[3] = {80.0F, 160.0F, 320.0F};

double nowMs() {
#ifdef __EMSCRIPTEN__
    return emscripten_get_now();  // SDL_GetPerformanceCounter has only millisecond resolution in the browser
#else
    return static_cast<double>(SDL_GetPerformanceCounter()) * 1000.0 / static_cast<double>(SDL_GetPerformanceFrequency());
#endif
}

// ---------------------------------------------------------------------------------------------- shaders

constexpr const char* kSkyGlsl = R"(
uniform vec3 uSun;
vec3 sky(vec3 d) {
    float h = clamp(d.y, 0.0, 1.0);
    vec3 horizon = vec3(0.86, 0.93, 0.92);
    vec3 zenith = vec3(0.30, 0.58, 0.78);
    vec3 c = mix(horizon, zenith, pow(h, 0.5));
    float sd = max(dot(d, uSun), 0.0);
    c += vec3(1.0, 0.85, 0.6) * (pow(sd, 8.0) * 0.14 + pow(sd, 260.0) * 0.5);
    return c;
}
)";

constexpr const char* kSkyFs = R"(
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec3 uForward;
uniform vec2 uTan;
in vec2 vUv;
out vec4 outColor;
void main() {
    vec2 ndc = vec2(vUv.x * 2.0 - 1.0, 1.0 - vUv.y * 2.0);
    vec3 dir = normalize(uForward + ndc.x * uTan.x * uRight + ndc.y * uTan.y * uUp);
    outColor = vec4(sky(dir), 1.0);
}
)";

constexpr const char* kTerrainVs = R"(
uniform mat4 uViewProj;
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in float aSkirt;
out vec3 vWorld;
out vec3 vNormal;
out float vSkirt;
void main() {
    vWorld = aPos;
    vNormal = aNormal;
    vSkirt = aSkirt;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

constexpr const char* kTerrainFs = R"(
uniform vec3 uCam;
uniform int uMode;        // 0 shaded, 1 lod tint, 2 wireframe fill, 3 wireframe lines
uniform int uLod;
uniform float uMaxHeight;
uniform float uFogDist;
in vec3 vWorld;
in vec3 vNormal;
in float vSkirt;
out vec4 outColor;

void main() {
    vec3 toCam = uCam - vWorld;
    float dist = length(toCam);
    vec3 V = toCam / dist;
    vec3 fogColor = sky(normalize(vec3(-V.x, 0.02, -V.z)));
    float fog = 1.0 - exp(-pow(dist / uFogDist, 2.0));

    if (uMode == 3) {
        vec3 line = mix(vec3(0.64, 0.98, 0.93), vec3(1.0, 0.55, 0.12), vSkirt);
        outColor = vec4(mix(line, fogColor, fog * 0.8), 1.0);
        return;
    }
    if (uMode == 2) {
        outColor = vec4(mix(vec3(0.05, 0.13, 0.15), fogColor, fog), 1.0);
        return;
    }

    vec3 N = normalize(vNormal);
    float h01 = clamp(vWorld.y / uMaxHeight, 0.0, 1.0);
    float slope = 1.0 - clamp(N.y, 0.0, 1.0);
    vec3 col = mix(vec3(0.27, 0.56, 0.44), vec3(0.62, 0.67, 0.45), smoothstep(0.10, 0.45, h01));
    col = mix(col, vec3(0.94, 0.94, 0.91), smoothstep(0.66, 0.88, h01));
    col = mix(col, vec3(0.48, 0.44, 0.40), smoothstep(0.12, 0.38, slope) * (1.0 - smoothstep(0.75, 0.95, h01)));
    float lit = 0.30 + 0.70 * max(dot(N, uSun), 0.0);
    col *= lit;
    if (uMode == 1) {
        vec3 tint = uLod == 0 ? vec3(0.05, 0.64, 0.58) : (uLod == 1 ? vec3(0.98, 0.62, 0.18) : vec3(0.52, 0.45, 0.86));
        col = mix(col, tint * (0.45 + 0.55 * lit), 0.5);
    }
    col = mix(col, vec3(0.18, 0.44, 0.56) * (0.7 + 0.3 * lit), 1.0 - smoothstep(0.0, 0.8, vWorld.y));   // the flat lowland at height 0 reads as a lake
    if (vSkirt > 0.5) col *= 0.82;
    outColor = vec4(mix(col, fogColor, fog), 1.0);
}
)";

// ---------------------------------------------------------------------------------------------- UI model

enum class Action { Coords, Skirts, View, Chunk, LodDist, Auto };

struct Chip {
    SDL_FRect rect;
    Action action;
    int value;
    std::string label;
};

struct Range {
    std::size_t first = 0;  // first triangle index in the shared index buffer
    GLsizei count = 0;
    std::size_t lineFirst = 0;
    GLsizei lineCount = 0;
};

struct App {
    gd::gl::Window win;
    gd::gl::Hud hud;

    GLuint skyProgram = 0, terrainProgram = 0;
    GLuint vbo = 0, triIbo = 0, lineIbo = 0, triVao = 0, lineVao = 0;
    GLint locLod = -1;
    std::vector<std::array<Range, tmesh::kLodLevels>> ranges;   // per chunk, per LOD
    std::size_t storedVertices = 0;
    double buildMs = 0.0;

    // Settings
    tmesh::Coords coords = tmesh::Coords::World;
    bool skirts = true;
    int view = 1;
    int chunkChoice = 1;
    int lodChoice = 1;
    bool autoFly = true;

    tmesh::Layout layout{32};
    tmesh::LodPlan plan;

    tmesh::FlyCamera cam;
    tmesh::PathClock clock;
    double lastAngle = 0.0;
    bool dragging = false;
    SDL_FPoint lastMouse{0.0F, 0.0F};
    std::uint32_t keys = 0;  // bit per held movement key

    tmesh::Vec3 sun = tmesh::normalize({-0.55F, 0.42F, -0.45F});
    float fps = 60.0F;
    std::vector<Chip> chips;
    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

enum Key : std::uint32_t { kForward = 1, kBack = 2, kLeft = 4, kRight = 8, kUp = 16, kDown = 32 };

// Chip rows: the labels are drawn by the HUD pass from the same table.
struct Label {
    float x, y;
    const char* text;
};
std::vector<Label> gLabels;

void layoutChips(App& app) {
    app.chips.clear();
    gLabels.clear();
    const float rowY[2] = {472.0F, 504.0F};
    const auto place = [&](int row, float& x, const char* title, Action action, std::vector<std::string> names) {
        gLabels.push_back({x, rowY[row] + 5.0F, title});
        x += static_cast<float>(std::strlen(title)) * 10.0F + 8.0F;
        for (std::size_t i = 0; i < names.size(); ++i) {
            const float w = std::max(40.0F, static_cast<float>(names[i].size()) * 10.0F + 18.0F);
            app.chips.push_back({SDL_FRect{x, rowY[row], w, 26.0F}, action, static_cast<int>(i), names[i]});
            x += w + 6.0F;
        }
        x += 14.0F;
    };
    float x = 14.0F;
    place(0, x, "coords", Action::Coords, {"world", "local"});
    place(0, x, "skirts", Action::Skirts, {"on", "off"});
    place(0, x, "view", Action::View, {"shaded", "lod", "wire"});
    x = 14.0F;
    place(1, x, "chunk", Action::Chunk, {"16", "32", "64"});
    place(1, x, "lod dist", Action::LodDist, {"80 m", "160 m", "320 m"});
    place(1, x, "camera", Action::Auto, {"flying", "free"});
}

bool isActive(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Coords: return (app.coords == tmesh::Coords::World) == (c.value == 0);
        case Action::Skirts: return app.skirts == (c.value == 0);
        case Action::View: return app.view == c.value;
        case Action::Chunk: return app.chunkChoice == c.value;
        case Action::LodDist: return app.lodChoice == c.value;
        case Action::Auto: return app.autoFly == (c.value == 0);
    }
    return false;
}

// ---------------------------------------------------------------------------------------------- GPU data

bool createWindow(gd::gl::Window& win) {
    // gl.hpp's Window::create asks for no depth buffer; terrain needs one, and MSAA smooths the wireframe.
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 0);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    win.window = SDL_CreateWindow("terrain-mesh", gd::gl::kWidth, gd::gl::kHeight, SDL_WINDOW_OPENGL | SDL_WINDOW_HIGH_PIXEL_DENSITY);
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

GLuint makeProgram(const char* vertex, const char* fragment) {
    const std::string fs = std::string(kSkyGlsl) + fragment;
    return gd::gl::compile(vertex, fs.c_str());
}

void attribs() {
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, tmesh::kVertexFloats * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, tmesh::kVertexFloats * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, tmesh::kVertexFloats * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));
}

/// Builds every chunk at every LOD for the current settings and uploads the lot. A settings change calls this again.
void rebuildWorld(App& app) {
    const double start = nowMs();
    const int per = app.layout.chunksPerSide();
    std::vector<float> vertices;
    std::vector<std::uint32_t> triangles, lines;
    app.ranges.assign(static_cast<std::size_t>(per) * static_cast<std::size_t>(per), {});
    for (int cz = 0; cz < per; ++cz) {
        for (int cx = 0; cx < per; ++cx) {
            for (int lod = 0; lod < tmesh::kLodLevels; ++lod) {
                const tmesh::ChunkMesh mesh = tmesh::buildChunk(app.layout, app.coords, app.skirts, cx, cz, lod);
                const auto base = static_cast<std::uint32_t>(vertices.size() / tmesh::kVertexFloats);
                Range& r = app.ranges[static_cast<std::size_t>(cz * per + cx)][static_cast<std::size_t>(lod)];
                r.first = triangles.size();
                for (const std::uint32_t i : mesh.indices) triangles.push_back(base + i);
                r.count = static_cast<GLsizei>(mesh.indices.size());
                r.lineFirst = lines.size();
                for (const std::uint32_t i : tmesh::uniqueEdges(mesh.indices)) lines.push_back(base + i);
                r.lineCount = static_cast<GLsizei>(lines.size() - r.lineFirst);
                vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
            }
        }
    }
    app.storedVertices = vertices.size() / tmesh::kVertexFloats;

    glBindVertexArray(app.triVao);
    glBindBuffer(GL_ARRAY_BUFFER, app.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, app.triIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(triangles.size() * sizeof(std::uint32_t)), triangles.data(), GL_STATIC_DRAW);
    glBindVertexArray(app.lineVao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, app.lineIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(lines.size() * sizeof(std::uint32_t)), lines.data(), GL_STATIC_DRAW);
    glBindVertexArray(0);
    app.buildMs = nowMs() - start;
}

bool buildGpuObjects(App& app) {
    app.skyProgram = gd::gl::compile(gd::gl::kFullscreenVs, (std::string(kSkyGlsl) + kSkyFs).c_str());
    app.terrainProgram = makeProgram(kTerrainVs, kTerrainFs);
    if (app.skyProgram == 0 || app.terrainProgram == 0) return false;
    app.locLod = gd::gl::uniform(app.terrainProgram, "uLod");

    glGenBuffers(1, &app.vbo);
    glGenBuffers(1, &app.triIbo);
    glGenBuffers(1, &app.lineIbo);
    glGenVertexArrays(1, &app.triVao);
    glGenVertexArrays(1, &app.lineVao);
    glBindVertexArray(app.triVao);
    glBindBuffer(GL_ARRAY_BUFFER, app.vbo);
    attribs();
    glBindVertexArray(app.lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, app.vbo);
    attribs();
    glBindVertexArray(0);
    rebuildWorld(app);
    return true;
}

void setMat4(GLuint program, const char* name, const tmesh::Mat4& m) { glUniformMatrix4fv(gd::gl::uniform(program, name), 1, GL_TRUE, &m.m[0][0]); }
void setVec3(GLuint program, const char* name, tmesh::Vec3 v) { glUniform3f(gd::gl::uniform(program, name), v.x, v.y, v.z); }

bool inside(SDL_FPoint p, SDL_FRect r) { return SDL_PointInRectFloat(&p, &r); }

/// 41300 -> "41.3k", small numbers as they are. Built by hand: snprintf follows the locale.
std::string formatCount(long n) {
    if (n < 1000) return std::to_string(n);
    return std::to_string(n / 1000) + "." + std::to_string((n % 1000) / 100) + "k";
}

// ---------------------------------------------------------------------------------------------- actions

void setChunkChoice(App& app, int choice) {
    if (choice < 0 || choice > 2 || choice == app.chunkChoice) return;
    app.chunkChoice = choice;
    app.layout = tmesh::Layout{kChunkChoices[choice]};
    rebuildWorld(app);
}
void setCoords(App& app, tmesh::Coords c) {
    if (c == app.coords) return;
    app.coords = c;
    rebuildWorld(app);
}
void setSkirts(App& app, bool on) {
    if (on == app.skirts) return;
    app.skirts = on;
    rebuildWorld(app);
}
void setAuto(App& app, bool on) {
    if (on == app.autoFly) return;
    app.autoFly = on;
    if (on) app.clock.angle = std::fmod(std::atan2(static_cast<double>(app.cam.pos.z), static_cast<double>(app.cam.pos.x)) + tmesh::kTwoPi, tmesh::kTwoPi);
}

void applyChip(App& app, const Chip& c) {
    switch (c.action) {
        case Action::Coords: setCoords(app, c.value == 0 ? tmesh::Coords::World : tmesh::Coords::Local); break;
        case Action::Skirts: setSkirts(app, c.value == 0); break;
        case Action::View: app.view = c.value; break;
        case Action::Chunk: setChunkChoice(app, c.value); break;
        case Action::LodDist: app.lodChoice = c.value; break;
        case Action::Auto: setAuto(app, c.value == 0); break;
    }
}

void releaseInput(App& app) {
    app.dragging = false;
    app.keys = 0;
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
    if (!createWindow(app->win)) return SDL_APP_FAILURE;
    if (!app->hud.load()) {
        SDL_Log("hud failed to load");
        return SDL_APP_FAILURE;
    }
    if (!buildGpuObjects(*app)) return SDL_APP_FAILURE;
    layoutChips(*app);

    // Start on the flight circle a little way round, already at flight height.
    app->clock.angle = 0.55;
    const tmesh::PathPose pose = tmesh::pathPose(app->clock.angle, kPathRadius);
    app->cam.pos = {pose.x, tmesh::surfaceHeight(app->layout, app->coords, pose.x, pose.z) + kFlightHeight, pose.z};
    app->cam.yaw = pose.yaw;
    app->cam.pitch = kFlightPitch;
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
                bool onChip = false;
                for (const Chip& c : app.chips) {
                    if (inside(p, c.rect)) {
                        applyChip(app, c);
                        onChip = true;
                        break;
                    }
                }
                if (!onChip && p.y > 30.0F && p.y < kPanelTop) {
                    app.dragging = true;
                    app.lastMouse = p;
                    setAuto(app, false);
                }
            }
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.dragging) {
                const SDL_FPoint p = app.win.toLogical(*event);
                app.cam.look((p.x - app.lastMouse.x) * 0.005F, -(p.y - app.lastMouse.y) * 0.005F);
                app.lastMouse = p;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.dragging = false;
            if (event->type == SDL_EVENT_WINDOW_FOCUS_LOST) releaseInput(app);
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            setAuto(app, false);
            app.cam.move(std::clamp(event->wheel.y, -3.0F, 3.0F) * kWheelStep, 0.0F, 0.0F);
            break;
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode k = event->key.key;
            if (k == SDLK_W || k == SDLK_UP) app.keys |= kForward;
            if (k == SDLK_S || k == SDLK_DOWN) app.keys |= kBack;
            if (k == SDLK_A || k == SDLK_LEFT) app.keys |= kLeft;
            if (k == SDLK_D || k == SDLK_RIGHT) app.keys |= kRight;
            if (k == SDLK_E) app.keys |= kUp;
            if (k == SDLK_Q) app.keys |= kDown;
            if (event->key.repeat) break;  // the toggles below must not flip while a key is held
            if (k >= SDLK_1 && k <= SDLK_3) setChunkChoice(app, static_cast<int>(k - SDLK_1));
            if (k == SDLK_C) setCoords(app, app.coords == tmesh::Coords::World ? tmesh::Coords::Local : tmesh::Coords::World);
            if (k == SDLK_K) setSkirts(app, !app.skirts);
            if (k == SDLK_V) app.view = (app.view + 1) % 3;
            if (k == SDLK_SPACE) setAuto(app, !app.autoFly);
            break;
        }
        case SDL_EVENT_KEY_UP: {
            const SDL_Keycode k = event->key.key;
            if (k == SDLK_W || k == SDLK_UP) app.keys &= ~static_cast<std::uint32_t>(kForward);
            if (k == SDLK_S || k == SDLK_DOWN) app.keys &= ~static_cast<std::uint32_t>(kBack);
            if (k == SDLK_A || k == SDLK_LEFT) app.keys &= ~static_cast<std::uint32_t>(kLeft);
            if (k == SDLK_D || k == SDLK_RIGHT) app.keys &= ~static_cast<std::uint32_t>(kRight);
            if (k == SDLK_E) app.keys &= ~static_cast<std::uint32_t>(kUp);
            if (k == SDLK_Q) app.keys &= ~static_cast<std::uint32_t>(kDown);
            break;
        }
        default:
            break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const double frameTime = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    const float dt = static_cast<float>(std::clamp(frameTime, 0.0, 0.25));
    if (frameTime > 0.0) app.fps += (static_cast<float>(1.0 / frameTime) - app.fps) * 0.05F;

    // Camera: flying the circle, or free.
    if (app.autoFly) {
        app.clock.advance(frameTime, kPathSpeed);
        const tmesh::PathPose pose = tmesh::pathPose(app.clock.angle, kPathRadius);
        const float target = tmesh::surfaceHeight(app.layout, app.coords, pose.x, pose.z) + kFlightHeight;
        app.cam.pos.x = pose.x;
        app.cam.pos.z = pose.z;
        app.cam.pos.y += (target - app.cam.pos.y) * tmesh::smoothFactor(1.6F, dt);  // follows the ground without every bump
        app.cam.yaw = pose.yaw;
        app.cam.pitch = kFlightPitch;
    } else if (app.keys != 0U) {
        const float ahead = (static_cast<float>((app.keys & kForward) != 0U) - static_cast<float>((app.keys & kBack) != 0U)) * kFlySpeed * dt;
        const float side = (static_cast<float>((app.keys & kRight) != 0U) - static_cast<float>((app.keys & kLeft) != 0U)) * kFlySpeed * dt;
        const float up = (static_cast<float>((app.keys & kUp) != 0U) - static_cast<float>((app.keys & kDown) != 0U)) * kFlySpeed * dt;
        app.cam.move(ahead, side, up);
    }
    tmesh::keepInside(app.cam, 40.0F);
    tmesh::keepAbove(app.cam, tmesh::surfaceHeight(app.layout, app.coords, app.cam.pos.x, app.cam.pos.z), kClearance);

    // LOD per chunk from the camera, and the numbers for the HUD.
    app.plan = tmesh::planLods(app.layout, app.cam.pos.x, app.cam.pos.z, kLodChoices[app.lodChoice]);
    const int per = app.layout.chunksPerSide();
    const long drawn = tmesh::planTriangles(app.layout, app.plan, app.skirts);
    const long allFine = static_cast<long>(per) * per * tmesh::triangleCount(app.layout, 0, app.skirts);

    const tmesh::Mat4 viewProj = tmesh::perspective(tmesh::kFovY, static_cast<float>(gd::gl::kWidth) / gd::gl::kHeight, tmesh::kNear, tmesh::kFar) * app.cam.view();
    const tmesh::Vec3 forward = app.cam.forward();
    const tmesh::Vec3 right = app.cam.right();
    const tmesh::Vec3 up = tmesh::cross(right, forward);

    app.win.begin();
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);

    // 1. Sky: one fullscreen triangle, no depth.
    glDisable(GL_DEPTH_TEST);
    glUseProgram(app.skyProgram);
    setVec3(app.skyProgram, "uSun", app.sun);
    setVec3(app.skyProgram, "uRight", right);
    setVec3(app.skyProgram, "uUp", up);
    setVec3(app.skyProgram, "uForward", forward);
    const float tanHalf = std::tan(tmesh::kFovY / 2.0F);
    glUniform2f(gd::gl::uniform(app.skyProgram, "uTan"), tanHalf * static_cast<float>(gd::gl::kWidth) / gd::gl::kHeight, tanHalf);
    gd::gl::drawFullscreen();

    // 2. Terrain: one draw per chunk at its LOD.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glUseProgram(app.terrainProgram);
    setMat4(app.terrainProgram, "uViewProj", viewProj);
    setVec3(app.terrainProgram, "uCam", app.cam.pos);
    setVec3(app.terrainProgram, "uSun", app.sun);
    glUniform1f(gd::gl::uniform(app.terrainProgram, "uMaxHeight"), tmesh::kHeightScale);
    glUniform1f(gd::gl::uniform(app.terrainProgram, "uFogDist"), kFogDistance);
    const GLint mode = gd::gl::uniform(app.terrainProgram, "uMode");
    const bool wire = app.view == 2;
    glUniform1i(mode, wire ? 2 : app.view);
    if (wire) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0F, 1.0F);
    }
    glBindVertexArray(app.triVao);
    for (std::size_t i = 0; i < app.plan.lods.size(); ++i) {
        const int lod = app.plan.lods[i];
        const Range& r = app.ranges[i][static_cast<std::size_t>(lod)];
        glUniform1i(app.locLod, lod);
        glDrawElements(GL_TRIANGLES, r.count, GL_UNSIGNED_INT, reinterpret_cast<const void*>(r.first * sizeof(std::uint32_t)));
    }
    if (wire) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        glUniform1i(mode, 3);
        glBindVertexArray(app.lineVao);
        for (std::size_t i = 0; i < app.plan.lods.size(); ++i) {
            const Range& r = app.ranges[i][static_cast<std::size_t>(app.plan.lods[i])];
            glDrawElements(GL_LINES, r.lineCount, GL_UNSIGNED_INT, reinterpret_cast<const void*>(r.lineFirst * sizeof(std::uint32_t)));
        }
    }
    glBindVertexArray(0);

    // 3. HUD on top.
    glDisable(GL_DEPTH_TEST);
    gd::gl::Hud& hud = app.hud;
    hud.titleBar(gd::gl::kWidth, "[ TERRAIN MESH ]", "drag: look · WASD: fly · wheel: forward");

    char line[96];
    hud.fillRect(12.0F, 40.0F, 336.0F, 100.0F, gd::Color{250, 250, 249, 220});
    hud.strokeRect(12.0F, 40.0F, 336.0F, 100.0F, gd::kTealDark);
    SDL_snprintf(line, sizeof line, "%d×%d chunks of %d cells · %.0f fps", per, per, app.layout.chunkCells, static_cast<double>(app.fps));
    hud.text(22.0F, 47.0F, line, gd::kInk);
    const std::string tris = "tris drawn " + formatCount(drawn) + " of " + formatCount(allFine);
    hud.text(22.0F, 65.0F, tris, gd::kOrange, gd::gl::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "chunks lod 0/1/2: %d / %d / %d", app.plan.count[0], app.plan.count[1], app.plan.count[2]);
    hud.text(22.0F, 83.0F, line, gd::kInk);
    SDL_snprintf(line, sizeof line, "built %s vertices in %.0f ms", formatCount(static_cast<long>(app.storedVertices)).c_str(), app.buildMs);
    hud.text(22.0F, 101.0F, line, gd::kMuted);
    const char* verdict = "world + skirts: no seams or cracks";
    gd::Color verdictColor = gd::kTealDark;
    if (app.coords == tmesh::Coords::Local) {
        verdict = "local: every chunk the same hill";
        verdictColor = gd::kOrange;
    } else if (!app.skirts) {
        verdict = "no skirts: gaps between levels";
        verdictColor = gd::kOrange;
    }
    hud.text(22.0F, 119.0F, verdict, verdictColor, gd::gl::Hud::Style::Bold);

    hud.fillRect(0.0F, kPanelTop, 960.0F, 74.0F, gd::Color{250, 250, 249, 240});
    hud.fillRect(0.0F, kPanelTop, 960.0F, 2.0F, gd::kTealDark);
    for (const Label& l : gLabels) hud.text(l.x, l.y, l.text, gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool active = isActive(app, c);
        hud.card(c.rect.x, c.rect.y, c.rect.w, c.rect.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
        hud.text(c.rect.x + (c.rect.w - hud.width(c.label, gd::gl::Hud::Style::Bold)) / 2.0F, c.rect.y + 2.0F, c.label, active ? gd::kWhite : gd::kInk, gd::gl::Hud::Style::Bold);
    }
    const char* keys = "keys: C K V space 1 2 3";
    hud.text(960.0F - 18.0F - hud.width(keys), 478.0F, keys, gd::kMuted);

    app.win.swap();
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app == nullptr) return;
    // Also runs after a failed SDL_AppInit: without a context there is nothing to delete, and GL calls would throw.
    if (app->win.context != nullptr) {
        app->hud.destroy();
        glDeleteProgram(app->skyProgram);
        glDeleteProgram(app->terrainProgram);
        const GLuint buffers[] = {app->vbo, app->triIbo, app->lineIbo};
        glDeleteBuffers(3, buffers);
        glDeleteVertexArrays(1, &app->triVao);
        glDeleteVertexArrays(1, &app->lineVao);
    }
    app->win.destroy();
    delete app;
}
