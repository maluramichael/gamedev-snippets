// Pixel water: a falling sand cellular automaton, drawn through one small texture.
//
// The world is 240 x 104 cells, every cell is one pixel of a texture that is scaled up 4x with nearest filtering.
// The grid starts right under the 30 px title bar; the strip below it holds the numbers and the buttons. The rules
// live in logic.hpp (no SDL, covered by test.cpp). Here: a fixed 60 Hz step with an accumulator (see the delta time
// and fixed timestep demos), a brush that paints while the mouse is held, and the pixel colours. Every particle keeps
// a fixed random tint from the moment it is created, so the colour variation travels with the water instead of
// flickering, and water gets darker with depth and lighter on its surface.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCell = 4;
constexpr int kTop = 30;                                // the world starts right under the title bar
constexpr int kWorldBottom = kTop + pw::kRows * kCell;  // 446
constexpr double kStepTime = 1.0 / 60.0;
constexpr double kMaxFrameTime = 0.25;
constexpr int kWaterCap = 4800;  // the tap stops here, so the scene does not fill up to the ceiling
constexpr int kWarmup = 420;     // steps run before the first frame, so the jet is already flowing
constexpr std::array<int, 3> kBrushRadius{1, 3, 6};
constexpr std::array<int, 3> kFlowRange{1, 4, 8};
constexpr float kChipY = 502.0F;
constexpr float kChipH = 28.0F;

enum class Action { Water, Sand, Wall, Erase, Brush0, Brush1, Brush2, Tap, Pause, Clear, Basin, Flow };

struct Chip {
    Action action;
    SDL_FRect rect;
};

struct Rgb {
    float r, g, b;
};

Rgb lerp(Rgb a, Rgb b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }

Uint8 byte(float v) { return static_cast<Uint8>(std::clamp(v, 0.0F, 255.0F)); }

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture = nullptr;
    gd::Hud hud;
    pw::World world{pw::kCols, pw::kRows};
    pw::Counts counts;
    std::vector<Uint8> pixels = std::vector<Uint8>(static_cast<std::size_t>(pw::kCols) * pw::kRows * 4);
    std::vector<Chip> chips;
    pw::Kind material = pw::Kind::Water;
    int brush = 1;
    int flow = 1;
    bool tap = true;
    bool paused = false;
    bool mouseDown = false;
    std::pair<int, int> mouseCell{-1, -1};
    std::optional<std::pair<int, int>> strokeFrom;
    float mouseX = -100.0F;
    float mouseY = -100.0F;
    double accumulator = 0.0;
    Uint64 last = 0;
    double stepMicros = 0.0;
    bool dirty = true;
    long maxFrames = -1;
    long frames = 0;
};

std::pair<int, int> cellAt(float x, float y) {
    return {static_cast<int>(std::floor(x / kCell)), static_cast<int>(std::floor((y - kTop) / kCell))};
}

bool inWorld(std::pair<int, int> c) { return c.first >= 0 && c.second >= 0 && c.first < pw::kCols && c.second < pw::kRows; }

void resetScene(App& app) {
    app.world.clear();
    pw::buildBasin(app.world);
    app.strokeFrom.reset();
    for (int i = 0; i < kWarmup; ++i) {
        if (app.tap) pw::pourTap(app.world, pw::kTapX, pw::kTapY, 2, kWaterCap);
        app.world.step();
    }
    app.counts = app.world.counts();
    app.dirty = true;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Water: app.material = pw::Kind::Water; break;
        case Action::Sand: app.material = pw::Kind::Sand; break;
        case Action::Wall: app.material = pw::Kind::Wall; break;
        case Action::Erase: app.material = pw::Kind::Empty; break;
        case Action::Brush0: app.brush = 0; break;
        case Action::Brush1: app.brush = 1; break;
        case Action::Brush2: app.brush = 2; break;
        case Action::Tap: app.tap = !app.tap; break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Clear:
            app.world.clear();
            app.counts = app.world.counts();
            app.dirty = true;
            break;
        case Action::Basin: resetScene(app); break;
        case Action::Flow:
            app.flow = (app.flow + 1) % static_cast<int>(kFlowRange.size());
            app.world.spread = kFlowRange[static_cast<std::size_t>(app.flow)];
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Water: return app.material == pw::Kind::Water;
        case Action::Sand: return app.material == pw::Kind::Sand;
        case Action::Wall: return app.material == pw::Kind::Wall;
        case Action::Erase: return app.material == pw::Kind::Empty;
        case Action::Brush0: return app.brush == 0;
        case Action::Brush1: return app.brush == 1;
        case Action::Brush2: return app.brush == 2;
        case Action::Tap: return app.tap;
        case Action::Pause: return app.paused;
        default: return false;
    }
}

std::string label(const App& app, Action action) {
    switch (action) {
        case Action::Water: return "1 Water";
        case Action::Sand: return "2 Sand";
        case Action::Wall: return "3 Wall";
        case Action::Erase: return "4 Erase";
        case Action::Brush0: return "1";
        case Action::Brush1: return "3";
        case Action::Brush2: return "6";
        case Action::Tap: return "Tap";
        case Action::Pause: return "Pause";
        case Action::Clear: return "Clear";
        case Action::Basin: return "Basin";
        case Action::Flow: return "Flow " + std::to_string(kFlowRange[static_cast<std::size_t>(app.flow)]);
    }
    return {};
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        float gapBefore;
        float width;  // 0 = fit the label
    };
    const std::array<Def, 12> defs{{{Action::Water, 0, 0},
                                    {Action::Sand, 6, 0},
                                    {Action::Wall, 6, 0},
                                    {Action::Erase, 6, 0},
                                    {Action::Brush0, 74, 30},
                                    {Action::Brush1, 6, 30},
                                    {Action::Brush2, 6, 30},
                                    {Action::Tap, 20, 0},
                                    {Action::Pause, 6, 0},
                                    {Action::Clear, 6, 0},
                                    {Action::Basin, 6, 0},
                                    {Action::Flow, 6, 96}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = d.width > 0 ? d.width : app.hud.width(label(app, d.action), gd::Hud::Style::Bold) + 16.0F;
        app.chips.push_back({d.action, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

/// Colour of one cell. depth = water cells directly above it, surface = air directly above it.
void shade(const pw::Cell& c, int depth, bool surface, Uint8* out) {
    const float v = (static_cast<float>(c.tint) - 128.0F) / 128.0F;  // -1 .. 1, fixed per particle
    Rgb col{};
    Uint8 alpha = 255;
    switch (c.kind) {
        case pw::Kind::Empty: alpha = 0; break;
        case pw::Kind::Wall: col = {31.0F + v * 8.0F, 41.0F + v * 8.0F, 58.0F + v * 8.0F}; break;
        case pw::Kind::Sand: {
            const float k = 1.0F + 0.15F * v;
            col = {222.0F * k, 168.0F * k, 76.0F * k};
            break;
        }
        case pw::Kind::Water: {
            const float t = std::min(static_cast<float>(depth), 40.0F) / 40.0F;
            col = lerp(Rgb{26.0F, 176.0F, 182.0F}, Rgb{6.0F, 84.0F, 112.0F}, t * 0.9F);
            const float k = 1.0F + 0.09F * v;
            col = {col.r * k, col.g * k, col.b * k};
            if (surface) col = lerp(col, Rgb{176.0F, 242.0F, 234.0F}, 0.55F);
            break;
        }
    }
    out[0] = byte(col.r);
    out[1] = byte(col.g);
    out[2] = byte(col.b);
    out[3] = alpha;
}

void uploadWorld(App& app) {
    std::array<int, pw::kCols> depth{};  // contiguous water above, per column
    for (int y = 0; y < pw::kRows; ++y) {
        for (int x = 0; x < pw::kCols; ++x) {
            const pw::Cell& c = app.world.cell(x, y);
            const bool water = c.kind == pw::Kind::Water;
            const bool surface = water && app.world.kind(x, y - 1) == pw::Kind::Empty;
            auto& d = depth[static_cast<std::size_t>(x)];
            shade(c, water ? d : 0, surface, &app.pixels[(static_cast<std::size_t>(y) * pw::kCols + static_cast<std::size_t>(x)) * 4]);
            d = water ? d + 1 : 0;
        }
    }
    SDL_UpdateTexture(app.texture, nullptr, app.pixels.data(), pw::kCols * 4);
    app.dirty = false;
}

// SDL's performance counter only ticks in whole milliseconds in the browser build, far too coarse for a step that takes
// a few hundred microseconds. The browser clock itself has finer steps.
#ifdef __EMSCRIPTEN__
double nowMicros() { return emscripten_get_now() * 1000.0; }
#else
double nowMicros() { return static_cast<double>(SDL_GetPerformanceCounter()) * 1e6 / static_cast<double>(SDL_GetPerformanceFrequency()); }
#endif

/// One 60 Hz tick: paint with the held brush, then (unless paused) tap and step.
void tick(App& app) {
    if (app.mouseDown && inWorld(app.mouseCell)) {
        const auto from = app.strokeFrom.value_or(app.mouseCell);
        const bool thin = app.material == pw::Kind::Water || app.material == pw::Kind::Sand;
        pw::stroke(app.world, from.first, from.second, app.mouseCell.first, app.mouseCell.second, kBrushRadius[static_cast<std::size_t>(app.brush)], app.material, thin);
        app.strokeFrom = app.mouseCell;
        app.dirty = true;
    } else {
        app.strokeFrom.reset();
    }
    if (app.paused) {
        app.counts = app.world.counts();
        return;
    }
    if (app.tap) pw::pourTap(app.world, pw::kTapX, pw::kTapY, 2, kWaterCap);
    const double t0 = nowMicros();
    app.world.step();
    const double us = nowMicros() - t0;
    app.stepMicros = app.stepMicros == 0.0 ? us : app.stepMicros * 0.9 + us * 0.1;
    app.counts = app.world.counts();
    app.dirty = true;
}

void drawSwatch(App& app, float x, float y, gd::Color c, const char* name, int n) {
    gd::fillRect(app.renderer, x, y + 5.0F, 12.0F, 12.0F, c);
    gd::strokeRect(app.renderer, x, y + 5.0F, 12.0F, 12.0F, gd::kInk);
    char text[48];
    SDL_snprintf(text, sizeof text, "%s %d", name, n);
    app.hud.text(app.renderer, x + 18.0F, y, text, gd::kInk, gd::Hud::Style::Bold);
}

void draw(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char text[96];
    SDL_snprintf(text, sizeof text, "%dx%d cells · 60 steps/s", pw::kCols, pw::kRows);
    app.hud.titleBar(r, kWidth, "[ PIXEL WATER ]", text);

    if (app.dirty) uploadWorld(app);
    const SDL_FRect dst{0.0F, static_cast<float>(kTop), static_cast<float>(pw::kCols * kCell), static_cast<float>(pw::kRows * kCell)};
    SDL_RenderTexture(r, app.texture, nullptr, &dst);
    gd::fillRect(r, 0.0F, static_cast<float>(kWorldBottom), static_cast<float>(kWidth), 2.0F, gd::kInk);

    // Brush outline under the cursor.
    if (inWorld(app.mouseCell)) {
        const float rad = (static_cast<float>(kBrushRadius[static_cast<std::size_t>(app.brush)]) + 0.5F) * kCell;
        gd::ring(r, static_cast<float>(app.mouseCell.first * kCell + kCell / 2), static_cast<float>(kTop + app.mouseCell.second * kCell + kCell / 2), rad, gd::Color{17, 24, 39, 200});
    }

    // Numbers.
    const float y = static_cast<float>(kWorldBottom) + 8.0F;
    drawSwatch(app, 24.0F, y, gd::Color{26, 176, 182}, "water", app.counts.water);
    drawSwatch(app, 214.0F, y, gd::Color{222, 168, 76}, "sand", app.counts.sand);
    drawSwatch(app, 394.0F, y, gd::Color{31, 41, 58}, "stone", app.counts.wall);
    SDL_snprintf(text, sizeof text, "step %.0f us", app.stepMicros);
    app.hud.text(r, static_cast<float>(kWidth) - 24.0F - app.hud.width(text, gd::Hud::Style::Bold), y, text, gd::kTeal, gd::Hud::Style::Bold);
    app.hud.text(r, 24.0F, y + 24.0F, "hold the mouse to paint · wheel: brush size · keys: 1-4 T Space R B F", gd::kMuted);

    // Chips.
    for (const Chip& c : app.chips) {
        if (c.action == Action::Brush0) app.hud.text(r, c.rect.x - 58.0F, kChipY + 3.0F, "brush", gd::kMuted);
    }
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const std::string name = label(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(name, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 3.0F, name, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

}  // namespace

SDL_AppResult SDL_AppInit(void** appstate, int argc, char** argv) {
    auto* app = new App();
    *appstate = app;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) {
            app->maxFrames = std::atol(argv[i + 1]);
        }
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("pixel-water", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    app->texture = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, pw::kCols, pw::kRows);
    if (app->texture == nullptr) {
        SDL_Log("world texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->texture, SDL_SCALEMODE_NEAREST);
    SDL_SetTextureBlendMode(app->texture, SDL_BLENDMODE_BLEND);
    layoutChips(*app);
    app->world.spread = kFlowRange[static_cast<std::size_t>(app->flow)];
    resetScene(*app);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouseX = event->motion.x;
            app.mouseY = event->motion.y;
            app.mouseCell = cellAt(app.mouseX, app.mouseY);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouseCell = cellAt(p.x, p.y);
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inWorld(app.mouseCell)) {
                app.mouseDown = true;
                app.strokeFrom = app.mouseCell;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.mouseDown = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.mouseDown = false;
            app.strokeFrom.reset();
            app.mouseCell = {-1, -1};
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            if (event->wheel.y > 0) app.brush = std::min(app.brush + 1, 2);
            if (event->wheel.y < 0) app.brush = std::max(app.brush - 1, 0);
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here toggles or triggers, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Water); break;
                case SDLK_2: apply(app, Action::Sand); break;
                case SDLK_3: apply(app, Action::Wall); break;
                case SDLK_4: apply(app, Action::Erase); break;
                case SDLK_T: apply(app, Action::Tap); break;
                case SDLK_P:
                case SDLK_SPACE: apply(app, Action::Pause); break;
                case SDLK_R: apply(app, Action::Clear); break;
                case SDLK_B: apply(app, Action::Basin); break;
                case SDLK_F: apply(app, Action::Flow); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const double frameTime = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    app.accumulator += std::min(frameTime, kMaxFrameTime);
    while (app.accumulator >= kStepTime) {
        tick(app);
        app.accumulator -= kStepTime;
    }

    draw(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        SDL_DestroyTexture(app->texture);
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
