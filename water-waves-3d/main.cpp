// 3D water waves: a flat grid of vertices that the GPU bends with six Gerstner waves.
//
// The mesh is built once (201 x 201 rest positions, one index buffer) and never touched again. Every frame the CPU only
// sends the wave table and the camera; the vertex shader moves each vertex, computes the exact normal and the Jacobian
// of the horizontal shift (small = squeezed together = foam), the fragment shader mixes sky reflection, deep water,
// sun glint, foam and fog. The same maths lives in logic.hpp and is tested there, which is what lets the CPU float a
// buoy on the very surface the GPU draws. Time is accumulated frame time (clamped, see WaveClock), reduced to one
// turn per wave in double before it is uploaded. Drag orbits the camera, the wheel zooms, the chips set the wind.

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

#include "gl.hpp"
#include "logic.hpp"

namespace {

constexpr int kGrid = 201;          // vertices per side, 200 x 200 cells
constexpr float kGridSize = 100.0F;  // metres, so a cell is 0.5 m
constexpr float kBuoyX = -2.0F;      // where the buoy is moored (world x, z)
constexpr float kBuoyZ = 9.0F;
constexpr float kBuoyScale = 1.6F;
constexpr float kFogDistance = 33.0F;
constexpr float kPanelTop = 468.0F;
constexpr const char* kWindNames[3] = {"calm", "breeze", "storm"};
constexpr std::array<gw::Preset, 3> kWinds{gw::kCalm, gw::kBreeze, gw::kStorm};

// Shared by the sky pass and the water: a procedural sky, so reflections and horizon fog use the same colors.
constexpr const char* kSkyGlsl = R"(
uniform vec3 uSun;
vec3 sky(vec3 d) {
    float h = clamp(d.y, 0.0, 1.0);
    vec3 horizon = vec3(0.80, 0.92, 0.92);
    vec3 zenith = vec3(0.27, 0.55, 0.77);
    vec3 c = mix(horizon, zenith, pow(h, 0.45));
    float sd = max(dot(d, uSun), 0.0);
    c += vec3(1.0, 0.82, 0.55) * (pow(sd, 6.0) * 0.12 + pow(sd, 220.0) * 0.50);   // warm haze around the sun
    return mix(c, vec3(1.0, 0.97, 0.88), smoothstep(0.9993, 0.9998, sd));         // the disc itself
}
)";

constexpr const char* kSkyFs = R"(
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec3 uForward;
uniform vec2 uTan;      // tan(fov/2) * aspect, tan(fov/2)
in vec2 vUv;
out vec4 outColor;
void main() {
    vec2 ndc = vec2(vUv.x * 2.0 - 1.0, 1.0 - vUv.y * 2.0);
    vec3 dir = normalize(uForward + ndc.x * uTan.x * uRight + ndc.y * uTan.y * uUp);
    outColor = vec4(sky(dir), 1.0);
}
)";

// One Gerstner sum per vertex: position, tangents, normal, Jacobian. Mirrors gw::frame() line by line.
constexpr const char* kWaterVs = R"(
const int MAX_WAVES = 8;
uniform mat4 uViewProj;
uniform vec4 uWaveA[MAX_WAVES];   // direction x, direction z, wave number k, amplitude A
uniform vec4 uWaveB[MAX_WAVES];   // omega * t reduced to one turn, minus the wave's phase offset; steepness Q
uniform int uCount;
layout(location = 0) in vec2 aRest;   // rest position on the flat grid
out vec3 vWorld;
out vec3 vNormal;
out float vJacobian;
out float vHeight;
out vec2 vRest;
void main() {
    vec3 p = vec3(aRest.x, 0.0, aRest.y);
    vec3 tx = vec3(1.0, 0.0, 0.0);    // d position / d x
    vec3 tz = vec3(0.0, 0.0, 1.0);    // d position / d z
    for (int i = 0; i < uCount; ++i) {
        vec2 d = uWaveA[i].xy;
        float k = uWaveA[i].z;
        float a = uWaveA[i].w;
        float q = uWaveB[i].y;
        float theta = k * dot(d, aRest) - uWaveB[i].x;
        float s = sin(theta);
        float c = cos(theta);
        p += vec3(q * a * d.x * c, a * s, q * a * d.y * c);
        float ka = k * a;
        float sharp = q * ka;
        tx += vec3(-sharp * d.x * d.x * s, ka * d.x * c, -sharp * d.x * d.y * s);
        tz += vec3(-sharp * d.x * d.y * s, ka * d.y * c, -sharp * d.y * d.y * s);
    }
    vec3 n = cross(tz, tx);          // n.y is the Jacobian of the horizontal map
    vJacobian = n.y;
    vNormal = normalize(n);
    vHeight = p.y;
    vWorld = p;
    vRest = aRest;
    gl_Position = uViewProj * vec4(p, 1.0);
}
)";

constexpr const char* kWaterFs = R"(
uniform vec3 uCam;
uniform int uMode;           // 0 shaded, 1 wireframe fill, 2 wireframe lines
uniform float uFoam;         // 1 = on
uniform float uAmpSum;       // sum of the amplitudes, maps the height to 0..1
uniform float uFogDist;
in vec3 vWorld;
in vec3 vNormal;
in float vJacobian;
in float vHeight;
in vec2 vRest;
out vec4 outColor;

float hash(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}
float vnoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x), mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
}
float fbm(vec2 p) { return 0.5 * vnoise(p) + 0.3 * vnoise(p * 2.1 + 7.0) + 0.2 * vnoise(p * 4.3 + 3.0); }

void main() {
    vec3 toCam = uCam - vWorld;
    float dist = length(toCam);
    vec3 V = toCam / dist;
    vec3 fogColor = sky(normalize(vec3(-V.x, 0.0, -V.z)));                       // the horizon in this direction
    float fog = 1.0 - exp(-pow(max(dist - 12.0, 0.0) / uFogDist, 2.0));
    if (uMode == 2) { outColor = vec4(mix(vec3(0.64, 0.98, 0.93), fogColor, fog), 1.0); return; }
    if (uMode == 1) { outColor = vec4(mix(vec3(0.03, 0.20, 0.22), fogColor, fog), 1.0); return; }

    vec3 N = normalize(vNormal);
    float ndv = clamp(dot(N, V), 0.0, 1.0);
    float fresnel = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);        // Schlick: grazing views mirror the sky
    vec3 R = reflect(-V, N);
    R.y = abs(R.y);                                           // a reflection below the horizon would see nothing
    vec3 reflection = sky(R);

    float h01 = clamp(vHeight / uAmpSum * 0.5 + 0.5, 0.0, 1.0);
    vec3 body = mix(vec3(0.02, 0.19, 0.23), vec3(0.07, 0.56, 0.53), h01 * h01);   // deep teal, lighter on the crests
    body *= 0.55 + 0.45 * clamp(dot(N, uSun) * 0.5 + 0.5, 0.0, 1.0);
    vec3 color = mix(body, reflection, fresnel);

    vec3 H = normalize(uSun + V);
    color += vec3(1.0, 0.92, 0.75) * pow(max(dot(N, H), 0.0), 380.0) * 1.2;      // sun glint

    if (uFoam > 0.5) {
        float squeeze = 1.0 - smoothstep(0.30, 0.80, vJacobian);                  // J falls where the surface is squeezed
        float n = fbm(vRest * 2.4);                                               // rest coordinates: the foam rides with the water
        float foam = smoothstep(0.42, 0.62, squeeze * 1.15 + (n - 0.5) * 0.9);
        vec3 foamColor = mix(vec3(0.74, 0.95, 0.92), vec3(1.0), n);
        foamColor *= 0.75 + 0.25 * clamp(dot(N, uSun) * 0.5 + 0.5, 0.0, 1.0);
        color = mix(color, foamColor, foam * 0.92);
    }
    outColor = vec4(mix(color, fogColor, fog), 1.0);
}
)";

constexpr const char* kBuoyVs = R"(
uniform mat4 uViewProj;
uniform mat4 uModel;
layout(location = 0) in vec3 aPos;
out vec3 vLocal;
out vec3 vWorld;
void main() {
    vLocal = aPos;
    vec4 w = uModel * vec4(aPos, 1.0);
    vWorld = w.xyz;
    gl_Position = uViewProj * w;
}
)";

constexpr const char* kBuoyFs = R"(
uniform vec3 uCam;
uniform float uFogDist;
in vec3 vLocal;
in vec3 vWorld;
out vec4 outColor;
void main() {
    vec3 N = normalize(cross(dFdx(vWorld), dFdy(vWorld)));                     // flat shading from screen derivatives
    vec3 toCam = uCam - vWorld;
    float dist = length(toCam);
    vec3 V = toCam / dist;
    if (dot(N, V) < 0.0) N = -N;
    float y = vLocal.y;
    vec3 color = vec3(0.93, 0.35, 0.05);                                       // orange
    if ((y > 0.02 && y < 0.2) || (y > 0.58 && y < 1.0)) color = vec3(0.97, 0.96, 0.93);   // white stripes
    float lit = 0.66 + 0.34 * max(dot(N, uSun), 0.0) + 0.12 * N.y;
    color *= lit;
    color += vec3(1.0, 0.95, 0.8) * pow(max(dot(N, normalize(uSun + V)), 0.0), 40.0) * 0.35;
    vec3 fogColor = sky(normalize(vec3(-V.x, 0.0, -V.z)));
    float fog = 1.0 - exp(-pow(max(dist - 12.0, 0.0) / uFogDist, 2.0));
    outColor = vec4(mix(color, fogColor, fog), 1.0);
}
)";

struct App {
    gd::gl::Window win;
    gd::gl::Hud hud;

    GLuint skyProgram = 0, waterProgram = 0, buoyProgram = 0;
    GLuint gridVbo = 0, gridVao = 0, gridIbo = 0, lineVao = 0, lineIbo = 0;
    GLuint buoyVbo = 0, buoyIbo = 0, buoyVao = 0;
    GLsizei triangleCount = 0, lineCount = 0, buoyIndexCount = 0;

    gw::OrbitCamera cam;
    gw::WaveClock clock;
    gw::Preset current = gw::kBreeze;
    std::size_t wind = 1;
    bool wireframe = false;
    bool foam = true;
    bool paused = false;

    bool dragging = false;
    SDL_FPoint lastMouse{0.0F, 0.0F};

    gw::Vec3 sun = gw::normalize({-0.35F, 0.2F, -0.9F});
    float fps = 60.0F;
    gw::BuoyPose buoy;
    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

// gl.hpp's Window::create asks for no depth buffer; a 3D scene needs one (waves hide waves), and MSAA smooths the crests.
bool createWindow(gd::gl::Window& win) {
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 0);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    win.window = SDL_CreateWindow("water-waves-3d", gd::gl::kWidth, gd::gl::kHeight, SDL_WINDOW_OPENGL | SDL_WINDOW_HIGH_PIXEL_DENSITY);
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

GLuint makeProgram(const char* vertex, const char* fragment, bool withSky) {
    const std::string fs = withSky ? std::string(kSkyGlsl) + fragment : std::string(fragment);
    return gd::gl::compile(vertex, fs.c_str());
}

GLuint makeBuffer(GLenum target, const void* data, std::size_t bytes) {
    GLuint id = 0;
    glGenBuffers(1, &id);
    glBindBuffer(target, id);
    glBufferData(target, static_cast<GLsizeiptr>(bytes), data, GL_STATIC_DRAW);
    return id;
}

bool buildGpuObjects(App& app) {
    app.skyProgram = makeProgram(gd::gl::kFullscreenVs, kSkyFs, true);
    app.waterProgram = makeProgram(kWaterVs, kWaterFs, true);
    app.buoyProgram = makeProgram(kBuoyVs, kBuoyFs, true);
    if (app.skyProgram == 0 || app.waterProgram == 0 || app.buoyProgram == 0) return false;

    // The water: rest positions once, two index buffers (triangles and lines) on two VAOs that share the vertex buffer.
    const std::vector<float> positions = gw::gridPositions(kGrid, kGridSize);
    const std::vector<std::uint16_t> triangles = gw::gridTriangles(kGrid);
    const std::vector<std::uint16_t> lines = gw::gridLines(kGrid);
    app.triangleCount = static_cast<GLsizei>(triangles.size());
    app.lineCount = static_cast<GLsizei>(lines.size());
    app.gridVbo = makeBuffer(GL_ARRAY_BUFFER, positions.data(), positions.size() * sizeof(float));
    glGenVertexArrays(1, &app.gridVao);
    glBindVertexArray(app.gridVao);
    glBindBuffer(GL_ARRAY_BUFFER, app.gridVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
    app.gridIbo = makeBuffer(GL_ELEMENT_ARRAY_BUFFER, triangles.data(), triangles.size() * sizeof(std::uint16_t));
    glGenVertexArrays(1, &app.lineVao);
    glBindVertexArray(app.lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, app.gridVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
    app.lineIbo = makeBuffer(GL_ELEMENT_ARRAY_BUFFER, lines.data(), lines.size() * sizeof(std::uint16_t));

    const gw::Mesh buoy = gw::buoyMesh(20);
    app.buoyIndexCount = static_cast<GLsizei>(buoy.idx.size());
    glGenVertexArrays(1, &app.buoyVao);
    glBindVertexArray(app.buoyVao);
    app.buoyVbo = makeBuffer(GL_ARRAY_BUFFER, buoy.v.data(), buoy.v.size() * sizeof(gw::Vec3));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(gw::Vec3), nullptr);
    app.buoyIbo = makeBuffer(GL_ELEMENT_ARRAY_BUFFER, buoy.idx.data(), buoy.idx.size() * sizeof(std::uint16_t));
    glBindVertexArray(0);
    return true;
}

void setMat4(GLuint program, const char* name, const gw::Mat4& m) { glUniformMatrix4fv(gd::gl::uniform(program, name), 1, GL_TRUE, &m.m[0][0]); }
void setVec3(GLuint program, const char* name, gw::Vec3 v) { glUniform3f(gd::gl::uniform(program, name), v.x, v.y, v.z); }

// Chip rectangles (canvas pixels).
SDL_FRect windChip(std::size_t i) { return SDL_FRect{86.0F + static_cast<float>(i) * 88.0F, 480.0F, 80.0F, 34.0F}; }
SDL_FRect wireChip() { return SDL_FRect{364.0F, 480.0F, 112.0F, 34.0F}; }
SDL_FRect foamChip() { return SDL_FRect{488.0F, 480.0F, 74.0F, 34.0F}; }
SDL_FRect pauseChip() { return SDL_FRect{574.0F, 480.0F, 84.0F, 34.0F}; }

bool inside(SDL_FPoint p, SDL_FRect r) { return SDL_PointInRectFloat(&p, &r); }

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
                for (std::size_t i = 0; i < kWinds.size(); ++i) {
                    if (inside(p, windChip(i))) {
                        app.wind = i;
                        onChip = true;
                    }
                }
                if (inside(p, wireChip())) { app.wireframe = !app.wireframe; onChip = true; }
                if (inside(p, foamChip())) { app.foam = !app.foam; onChip = true; }
                if (inside(p, pauseChip())) { app.paused = !app.paused; onChip = true; }
                if (!onChip && p.y > 30.0F && p.y < kPanelTop) {
                    app.dragging = true;
                    app.lastMouse = p;
                }
            }
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.dragging) {
                const SDL_FPoint p = app.win.toLogical(*event);
                app.cam.orbit(-(p.x - app.lastMouse.x) * 0.008F, (p.y - app.lastMouse.y) * 0.006F);
                app.lastMouse = p;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.dragging = false;
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            app.cam.zoom(std::clamp(event->wheel.y, -3.0F, 3.0F));
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            if (event->key.key >= SDLK_1 && event->key.key <= SDLK_3) app.wind = static_cast<std::size_t>(event->key.key - SDLK_1);
            if (event->key.key == SDLK_W) app.wireframe = !app.wireframe;
            if (event->key.key == SDLK_F) app.foam = !app.foam;
            if (event->key.key == SDLK_SPACE) app.paused = !app.paused;
            break;
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
    if (frameTime > 0.0) app.fps += (static_cast<float>(1.0 / frameTime) - app.fps) * 0.05F;

    // Time: one value per frame, used by the GPU upload and by the buoy below.
    app.clock.advance(frameTime, app.paused);
    // The wind chips ease between presets instead of snapping, the phase is untouched so nothing jumps sideways.
    const float ease = 1.0F - std::exp(-3.0F * static_cast<float>(std::min(frameTime, 0.25)));
    const gw::Preset target = kWinds[app.wind];
    app.current.amplitude += (target.amplitude - app.current.amplitude) * ease;
    app.current.steepness += (target.steepness - app.current.steepness) * ease;
    const gw::Waves waves = gw::makeWaves(app.current);
    const double t = app.clock.time;

    // CPU side of the very same water: where the buoy floats and how it leans.
    app.buoy = gw::buoy(waves, kBuoyX, kBuoyZ, t);

    // Uniform arrays for the vertex shader.
    std::array<float, gw::kMaxWaves * 4> waveA{}, waveB{};
    const int count = static_cast<int>(std::min<std::size_t>(waves.size(), gw::kMaxWaves));
    for (int i = 0; i < count; ++i) {
        const gw::Wave& w = waves[static_cast<std::size_t>(i)];
        const auto o = static_cast<std::size_t>(i) * 4U;
        waveA[o] = w.dirX;
        waveA[o + 1] = w.dirZ;
        waveA[o + 2] = gw::wavenumber(w);
        waveA[o + 3] = w.amplitude;
        waveB[o] = static_cast<float>(gw::timePhase(w, t)) - w.phase;
        waveB[o + 1] = w.steepness;
    }

    const gw::Mat4 viewProj = gw::perspective(gw::kFovY, static_cast<float>(gd::gl::kWidth) / gd::gl::kHeight, gw::kNear, gw::kFar) * app.cam.view();
    const gw::Vec3 eye = app.cam.eye();
    const gw::Basis basis = app.cam.basis();

    app.win.begin();
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);

    // 1. Sky: one fullscreen triangle, no depth.
    glDisable(GL_DEPTH_TEST);
    glUseProgram(app.skyProgram);
    setVec3(app.skyProgram, "uSun", app.sun);
    setVec3(app.skyProgram, "uRight", basis.right);
    setVec3(app.skyProgram, "uUp", basis.up);
    setVec3(app.skyProgram, "uForward", basis.forward);
    const float tanHalf = std::tan(gw::kFovY / 2.0F);
    glUniform2f(gd::gl::uniform(app.skyProgram, "uTan"), tanHalf * static_cast<float>(gd::gl::kWidth) / gd::gl::kHeight, tanHalf);
    gd::gl::drawFullscreen();

    // 2. Water.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glUseProgram(app.waterProgram);
    setMat4(app.waterProgram, "uViewProj", viewProj);
    setVec3(app.waterProgram, "uCam", eye);
    setVec3(app.waterProgram, "uSun", app.sun);
    glUniform1i(gd::gl::uniform(app.waterProgram, "uCount"), count);
    glUniform4fv(gd::gl::uniform(app.waterProgram, "uWaveA"), gw::kMaxWaves, waveA.data());
    glUniform4fv(gd::gl::uniform(app.waterProgram, "uWaveB"), gw::kMaxWaves, waveB.data());
    glUniform1f(gd::gl::uniform(app.waterProgram, "uFoam"), app.foam ? 1.0F : 0.0F);
    glUniform1f(gd::gl::uniform(app.waterProgram, "uAmpSum"), std::max(gw::maxHeight(waves), 0.05F));
    glUniform1f(gd::gl::uniform(app.waterProgram, "uFogDist"), kFogDistance);
    const GLint mode = gd::gl::uniform(app.waterProgram, "uMode");
    glUniform1i(mode, app.wireframe ? 1 : 0);
    glBindVertexArray(app.gridVao);
    glDrawElements(GL_TRIANGLES, app.triangleCount, GL_UNSIGNED_SHORT, nullptr);
    if (app.wireframe) {
        glUniform1i(mode, 2);
        glBindVertexArray(app.lineVao);
        glDrawElements(GL_LINES, app.lineCount, GL_UNSIGNED_SHORT, nullptr);
    }

    // 3. Buoy: translate to the surface point, turn +y onto the surface normal.
    const gw::Mat4 model = gw::translate(app.buoy.position) * gw::alignUp(app.buoy.up) * gw::scale(kBuoyScale);
    glUseProgram(app.buoyProgram);
    setMat4(app.buoyProgram, "uViewProj", viewProj);
    setMat4(app.buoyProgram, "uModel", model);
    setVec3(app.buoyProgram, "uCam", eye);
    setVec3(app.buoyProgram, "uSun", app.sun);
    glUniform1f(gd::gl::uniform(app.buoyProgram, "uFogDist"), kFogDistance);
    glBindVertexArray(app.buoyVao);
    glDrawElements(GL_TRIANGLES, app.buoyIndexCount, GL_UNSIGNED_SHORT, nullptr);
    glBindVertexArray(0);

    // 4. HUD on top.
    glDisable(GL_DEPTH_TEST);
    gd::gl::Hud& hud = app.hud;
    hud.titleBar(gd::gl::kWidth, "[ WATER WAVES 3D ]", "201×201 vertices · 6 Gerstner waves");

    const float tilt = std::acos(std::clamp(app.buoy.up.y, -1.0F, 1.0F)) * 180.0F / 3.14159265F;
    char line[64];
    hud.fillRect(12.0F, 40.0F, 264.0F, 66.0F, gd::Color{250, 250, 249, 215});
    hud.strokeRect(12.0F, 40.0F, 264.0F, 66.0F, gd::kTealDark);
    SDL_snprintf(line, sizeof line, "t = %.1f s   %.0f fps", t, static_cast<double>(app.fps));
    hud.text(22.0F, 48.0F, line, gd::kInk);
    SDL_snprintf(line, sizeof line, "buoy %+.2f m · tilt %.0f°", static_cast<double>(app.buoy.position.y), static_cast<double>(tilt));
    hud.text(22.0F, 68.0F, line, gd::kOrange, gd::gl::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "steepness sum %.2f / 1", static_cast<double>(gw::steepnessSum(waves)));
    hud.text(22.0F, 86.0F, line, gd::kMuted);

    hud.fillRect(0.0F, kPanelTop, 960.0F, 72.0F, gd::Color{250, 250, 249, 240});
    hud.fillRect(0.0F, kPanelTop, 960.0F, 2.0F, gd::kTealDark);
    hud.text(24.0F, 486.0F, "wind:", gd::kMuted);
    const auto chip = [&hud](SDL_FRect c, const char* label, bool active) {
        hud.card(c.x, c.y, c.w, c.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
        hud.text(c.x + (c.w - hud.width(label, gd::gl::Hud::Style::Bold)) / 2.0F, c.y + 6.0F, label, active ? gd::kWhite : gd::kInk, gd::gl::Hud::Style::Bold);
    };
    for (std::size_t i = 0; i < kWinds.size(); ++i) chip(windChip(i), kWindNames[i], i == app.wind);
    chip(wireChip(), "wireframe", app.wireframe);
    chip(foamChip(), "foam", app.foam);
    chip(pauseChip(), "pause", app.paused);
    const char* hint = "drag: orbit · wheel: zoom";
    hud.text(960.0F - 24.0F - hud.width(hint), 478.0F, hint, gd::kInk);
    const char* keys = "keys: 1 2 3 W F space";
    hud.text(960.0F - 24.0F - hud.width(keys), 500.0F, keys, gd::kMuted);

    app.win.swap();
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        // SDL_AppQuit also runs after a failed SDL_AppInit: GL objects exist only if the context does.
        if (app->win.context != nullptr) {
            app->hud.destroy();
            glDeleteProgram(app->skyProgram);
            glDeleteProgram(app->waterProgram);
            glDeleteProgram(app->buoyProgram);
            const GLuint buffers[] = {app->gridVbo, app->gridIbo, app->lineIbo, app->buoyVbo, app->buoyIbo};
            glDeleteBuffers(5, buffers);
            glDeleteVertexArrays(1, &app->gridVao);
            glDeleteVertexArrays(1, &app->lineVao);
            glDeleteVertexArrays(1, &app->buoyVao);
        }
        app->win.destroy();
        delete app;
    }
}
