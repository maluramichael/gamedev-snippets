// 2D water ripples: simulation on the CPU, shading on the GPU.
//
// The wave equation runs on a 240x135 height field in fixed steps (accumulator, see the update-loop demo). Every frame the
// heights go to the GPU as a float texture, and the fragment shader does the rest: normal from the height gradient,
// refraction of a procedural tile floor along that normal, a specular highlight of one light. The simulation stays on the
// CPU because it is tiny (32k cells) and has to stay deterministic and testable; the per-pixel work (518k pixels, bilinear
// lookups, lighting) is what the GPU is for. Click and drag make waves, the chips set the damping and switch the rain.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "gl.hpp"
#include "logic.hpp"

namespace {

constexpr int kCols = 240;
constexpr int kRows = 135;
constexpr float kCellPx = static_cast<float>(gd::gl::kWidth) / kCols;  // 4 canvas pixels per cell
constexpr double kStepsPerSecond = 120.0;
constexpr double kMaxFrameTime = 0.25;  // after a stall do not catch up with hundreds of steps
constexpr std::array<float, 3> kDamping{0.97F, 0.99F, 0.999F};

// Fragment shader. `vUv` is 0..1 with the origin at the top left, like the canvas.
constexpr const char* kWaterFs = R"(
uniform highp sampler2D uHeights;   // R32F, one texel per cell, NEAREST (filtering floats needs an extension in WebGL2)
uniform vec2 uGrid;                 // cells in x and y
in vec2 vUv;
out vec4 outColor;

float cell(ivec2 i) { return texelFetch(uHeights, clamp(i, ivec2(0), ivec2(uGrid) - 1), 0).r; }

// Bilinear height at a point of the canvas, by hand because of the NEAREST texture.
float height(vec2 uv) {
    vec2 g = uv * uGrid - 0.5;
    ivec2 i = ivec2(floor(g));
    vec2 f = fract(g);
    return mix(mix(cell(i), cell(i + ivec2(1, 0)), f.x), mix(cell(i + ivec2(0, 1)), cell(i + ivec2(1, 1)), f.x), f.y);
}

// Tile floor in canvas pixels: teal checker, a few orange tiles, light grout.
vec3 floorColor(vec2 p) {
    const float tile = 60.0;
    vec2 t = p / tile;
    vec2 id = floor(t);
    vec2 f = fract(t);
    vec3 teal = vec3(0.051, 0.580, 0.533);
    vec3 light = mix(teal, vec3(0.80, 0.98, 0.95), 0.35);
    vec3 c = mix(teal, light, mod(id.x + id.y, 2.0));
    if (mod(id.x * 3.0 + id.y * 5.0, 11.0) < 0.5) c = vec3(0.918, 0.345, 0.047);
    vec2 edge = min(f, 1.0 - f) * tile;
    float grout = 1.0 - smoothstep(0.8, 2.0, min(edge.x, edge.y));
    return mix(c, vec3(0.98, 0.98, 0.97), grout);
}

void main() {
    vec2 e = 1.0 / uGrid;
    vec2 slope = 0.5 * vec2(height(vUv + vec2(e.x, 0.0)) - height(vUv - vec2(e.x, 0.0)),
                            height(vUv + vec2(0.0, e.y)) - height(vUv - vec2(0.0, e.y)));   // height change per cell
    vec3 n = normalize(vec3(-slope * 5.0, 1.0));

    vec2 p = vUv * vec2(960.0, 540.0);
    vec3 color = floorColor(p + slope * 150.0);   // refraction: look up the floor somewhere else, along the slope
    color *= 0.92 + 0.08 * (1.0 - vUv.y);        // a little depth

    vec3 light = normalize(vec3(-0.45, -0.55, 0.7));
    float lit = dot(n, light) - light.z;         // 0 on flat water
    color *= 1.0 + lit * 2.2;
    float spec = pow(max(dot(n, normalize(light + vec3(0.0, 0.0, 1.0))), 0.0), 120.0);
    color += vec3(spec) * 0.9;
    outColor = vec4(color, 1.0);
}
)";

struct App {
    gd::gl::Window win;
    gd::gl::Hud hud;
    GLuint program = 0;
    gd::gl::Texture heights;
    gd::water::Field field{kCols, kRows};

    Uint64 last = 0;
    double accumulator = 0.0;
    std::size_t dampingIndex = 1;
    bool rain = true;
    double rainTimer = 0.6;

    bool dragging = false;
    bool hasLast = false;
    SDL_FPoint lastCell{0.0F, 0.0F};

    std::uint32_t rng = 12345U;
    long maxFrames = -1;
    long frames = 0;

    float random() {  // xorshift32 -> [0, 1)
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return static_cast<float>(rng >> 8) / 16777216.0F;
    }
};

SDL_FRect dampingChip(std::size_t i) { return SDL_FRect{130.0F + static_cast<float>(i) * 92.0F, 480.0F, 80.0F, 34.0F}; }
SDL_FRect rainChip() { return SDL_FRect{500.0F, 480.0F, 80.0F, 34.0F}; }

SDL_FPoint toCell(SDL_FPoint logical) { return SDL_FPoint{logical.x / kCellPx - 0.5F, logical.y / kCellPx - 0.5F}; }

// The first frame already shows rings on their way.
void primeField(App& app) {
    const float d = kDamping[app.dampingIndex];
    app.field.drop(62.0F, 40.0F, 4.0F, 1.0F);
    for (int i = 0; i < 80; ++i) app.field.step(d);
    app.field.drop(172.0F, 92.0F, 3.0F, 0.8F);
    for (int i = 0; i < 45; ++i) app.field.step(d);
    app.field.drop(118.0F, 66.0F, 3.5F, 0.9F);
    for (int i = 0; i < 14; ++i) app.field.step(d);
}

// Stroke between two cell positions: a drop every 1.5 cells so a fast drag leaves a line, not dots.
void stroke(App& app, SDL_FPoint from, SDL_FPoint to) {
    const float dx = to.x - from.x;
    const float dy = to.y - from.y;
    const float distance = std::sqrt(dx * dx + dy * dy);
    const int n = std::max(1, static_cast<int>(distance / 1.5F));
    for (int i = 1; i <= n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(n);
        app.field.drop(from.x + dx * t, from.y + dy * t, 2.5F, 0.15F);
    }
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
    if (!app->win.create("water-ripples-2d")) return SDL_APP_FAILURE;
    if (!app->hud.load()) {
        SDL_Log("hud failed to load");
        return SDL_APP_FAILURE;
    }
    app->program = gd::gl::compile(kWaterFs);
    if (app->program == 0) return SDL_APP_FAILURE;
    glUseProgram(app->program);
    glUniform1i(gd::gl::uniform(app->program, "uHeights"), 0);
    glUniform2f(gd::gl::uniform(app->program, "uGrid"), static_cast<float>(kCols), static_cast<float>(kRows));
    app->heights = gd::gl::Texture::create(kCols, kRows, GL_R32F, GL_RED, GL_FLOAT, GL_NEAREST, app->field.heights().data());
    primeField(*app);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) return SDL_APP_SUCCESS;

    if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN && event->button.button == SDL_BUTTON_LEFT) {
        const SDL_FPoint p = app.win.toLogical(*event);
        bool onChip = false;
        for (std::size_t i = 0; i < kDamping.size(); ++i) {
            const SDL_FRect c = dampingChip(i);
            if (SDL_PointInRectFloat(&p, &c)) {
                app.dampingIndex = i;
                onChip = true;
            }
        }
        const SDL_FRect r = rainChip();
        if (SDL_PointInRectFloat(&p, &r)) {
            app.rain = !app.rain;
            onChip = true;
        }
        if (!onChip) {
            app.dragging = true;
            app.lastCell = toCell(p);
            app.hasLast = true;
            app.field.drop(app.lastCell.x, app.lastCell.y, 4.0F, 1.0F);
        }
    } else if (event->type == SDL_EVENT_MOUSE_MOTION && app.dragging) {
        const SDL_FPoint cell = toCell(app.win.toLogical(*event));
        if (app.hasLast) stroke(app, app.lastCell, cell);
        app.lastCell = cell;
        app.hasLast = true;
    } else if (event->type == SDL_EVENT_MOUSE_BUTTON_UP) {
        app.dragging = false;
        app.hasLast = false;
    } else if (event->type == SDL_EVENT_KEY_DOWN) {
        if (event->key.key >= SDLK_1 && event->key.key <= SDLK_3) app.dampingIndex = static_cast<std::size_t>(event->key.key - SDLK_1);
        if (event->key.key == SDLK_R) app.rain = !app.rain;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const double frameTime = std::min(static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency()), kMaxFrameTime);
    app.last = now;

    if (app.rain) {
        app.rainTimer -= frameTime;
        if (app.rainTimer <= 0.0) {
            app.field.drop(8.0F + app.random() * (kCols - 16), 8.0F + app.random() * (kRows - 16), 2.0F + app.random() * 1.5F, 0.5F + app.random() * 0.5F);
            app.rainTimer = 0.35 + static_cast<double>(app.random()) * 0.9;
        }
    }

    const double stepTime = 1.0 / kStepsPerSecond;
    app.accumulator += frameTime;
    while (app.accumulator >= stepTime) {
        app.field.step(kDamping[app.dampingIndex]);
        app.accumulator -= stepTime;
    }

    app.win.begin();
    glUseProgram(app.program);
    app.heights.bind(0);
    app.heights.update(app.field.heights().data());
    gd::gl::drawFullscreen();

    gd::gl::Hud& hud = app.hud;
    hud.titleBar(gd::gl::kWidth, "[ WATER RIPPLES ]", "240×135 cells · 120 steps/s");
    hud.fillRect(0.0F, 468.0F, 960.0F, 72.0F, gd::Color{250, 250, 249, 240});
    hud.fillRect(0.0F, 468.0F, 960.0F, 2.0F, gd::kTealDark);
    hud.text(40.0F, 486.0F, "damping:", gd::kMuted);
    char label[16];
    for (std::size_t i = 0; i < kDamping.size(); ++i) {
        const SDL_FRect c = dampingChip(i);
        const bool active = i == app.dampingIndex;
        hud.card(c.x, c.y, c.w, c.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
        SDL_snprintf(label, sizeof label, "%.3g", static_cast<double>(kDamping[i]));
        hud.text(c.x + (c.w - hud.width(label, gd::gl::Hud::Style::Bold)) / 2.0F, c.y + 6.0F, label, active ? gd::kWhite : gd::kInk, gd::gl::Hud::Style::Bold);
    }
    hud.text(440.0F, 486.0F, "rain:", gd::kMuted);
    const SDL_FRect rc = rainChip();
    hud.card(rc.x, rc.y, rc.w, rc.h, app.rain ? gd::kTeal : gd::kWhite, gd::kInk);
    const char* rainLabel = app.rain ? "on" : "off";
    hud.text(rc.x + (rc.w - hud.width(rainLabel, gd::gl::Hud::Style::Bold)) / 2.0F, rc.y + 6.0F, rainLabel, app.rain ? gd::kWhite : gd::kInk, gd::gl::Hud::Style::Bold);
    const char* hint = "click or drag: waves";
    hud.text(960.0F - 40.0F - hud.width(hint), 478.0F, hint, gd::kInk);
    const char* keys = "keys: 1 2 3 R";
    hud.text(960.0F - 40.0F - hud.width(keys), 500.0F, keys, gd::kMuted);

    app.win.swap();
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        app->heights.destroy();
        glDeleteProgram(app->program);
        app->win.destroy();
        delete app;
    }
}
