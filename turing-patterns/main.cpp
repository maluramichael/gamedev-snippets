// Turing patterns (Gray-Scott reaction-diffusion) with SDL3 main callbacks.
//
// Two chemicals on a 160x105 grid, drawn four pixels per cell. A spreads fast, B slowly, B eats A and grows. Two numbers
// (feed and kill) decide whether that ends in spots, stripes, coral or cells that divide, so the demo has a map of those
// two numbers: drag in it and the pattern flips while it runs. The two classic mistakes are switchable and orange:
// updating the grid in place (the scan order leaks into the pattern) and a time step above the diffusion limit
// (a checkerboard grows until the clamp stops it). The simulation, its limits and the picker maths live in logic.hpp
// and are covered by test.cpp. Each frame runs a fixed number of steps (capped by a time budget), it is a visual
// simulation and needs no real-time clock; the first frame is already a grown pattern because init runs ahead.

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
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kGridW = 160;
constexpr int kGridH = 105;
constexpr int kCell = 4;  // pixels per cell
constexpr float kFieldX = 24.0F;
constexpr float kFieldY = 42.0F;
constexpr float kFieldW = static_cast<float>(kGridW * kCell);
constexpr float kFieldH = static_cast<float>(kGridH * kCell);
constexpr int kBrush = 5;  // radius in cells, a smaller blob dies out at the mitosis values
constexpr float kPanelX = 688.0F;
constexpr float kPanelW = 248.0F;
constexpr tp::MapRect kMap{kPanelX, 64.0F, kPanelW, 150.0F};
constexpr float kChipH = 34.0F;
constexpr int kStartPreset = 2;       // coral: the first frame shows a grown maze
constexpr int kStartSteps = 1100;     // how far init runs ahead
constexpr int kPresetSteps = 240;     // head start after picking a preset
constexpr double kBudgetMs = 12.0;    // per frame, so a slow device lowers the speed instead of the frame rate
constexpr std::array<int, 3> kStepsPerFrame{4, 12, 36};
constexpr std::array<float, 2> kDts{1.0F, 1.5F};

double nowMs() {
#ifdef __EMSCRIPTEN__
    return emscripten_get_now();  // performance.now(), the performance counter is only millisecond-exact in the browser
#else
    return 1000.0 * static_cast<double>(SDL_GetPerformanceCounter()) / static_cast<double>(SDL_GetPerformanceFrequency());
#endif
}

enum class Action { Preset0, Preset1, Preset2, Preset3, Reseed, Pause, Buffered, InPlace, DtGood, DtBad, Speed0, Speed1, Speed2 };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
    bool naive;  // orange when on
};

enum class Drag { None, PaintB, Erase, Map };

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* fieldTexture = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    tp::Field field{kGridW, kGridH};
    tp::Params params;
    tp::Update update = tp::Update::Buffered;
    int preset = kStartPreset;      // -1 once feed/kill were moved by hand
    int seedPreset = kStartPreset;  // which preset's seeds Reseed uses
    std::uint32_t seedCounter = 1;
    bool paused = false;
    int speed = 1;
    long steps = 0;
    Drag drag = Drag::None;
    SDL_FPoint mouse{-100.0F, -100.0F};
    SDL_FPoint lastPaint{-1.0F, -1.0F};
    std::vector<Uint8> pixels = std::vector<Uint8>(static_cast<std::size_t>(kGridW) * kGridH * 4U);
    std::array<std::array<Uint8, 3>, 256> palette{};
    long maxFrames = -1;
    long frames = 0;
};

Uint8 mixChannel(float a, float b, float t) { return static_cast<Uint8>(std::lround(a + (b - a) * t)); }

/// Paper-white for empty, teal through the front of the pattern to a deep teal in the middle of it.
void buildPalette(App& app) {
    constexpr std::array<std::array<float, 3>, 3> stops{{{242.0F, 250.0F, 249.0F}, {40.0F, 168.0F, 156.0F}, {8.0F, 84.0F, 80.0F}}};
    for (std::size_t i = 0; i < app.palette.size(); ++i) {
        const float t = static_cast<float>(i) / 255.0F;
        const std::size_t s = t < 0.5F ? 0U : 1U;
        const float u = t < 0.5F ? t * 2.0F : (t - 0.5F) * 2.0F;
        for (std::size_t c = 0; c < 3; ++c) app.palette[i][c] = mixChannel(stops[s][c], stops[s + 1][c], u);
    }
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float x;
        float y;
        bool naive;
    };
    const float bottom = 492.0F;
    float x = 24.0F;
    auto add = [&app](Action a, const char* label, float cx, float cy, bool naive) {
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({a, label, SDL_FRect{cx, cy, w, kChipH}, naive});
        return cx + w;
    };
    x = add(Action::Preset0, "1 Spots", x, bottom, false) + 8.0F;
    x = add(Action::Preset1, "2 Stripes", x, bottom, false) + 8.0F;
    x = add(Action::Preset2, "3 Coral", x, bottom, false) + 8.0F;
    x = add(Action::Preset3, "4 Mitosis", x, bottom, false) + 20.0F;
    x = add(Action::Reseed, "R Reseed", x, bottom, false) + 8.0F;
    x = add(Action::Pause, "Space Pause", x, bottom, false) + 20.0F;
    x = add(Action::Speed0, "1x", x, bottom, false) + 8.0F;
    x = add(Action::Speed1, "3x", x, bottom, false) + 8.0F;
    add(Action::Speed2, "9x", x, bottom, false);
    x = kPanelX;
    x = add(Action::Buffered, "Buffered", x, 366.0F, false) + 8.0F;
    add(Action::InPlace, "In place", x, 366.0F, true);
    x = kPanelX;
    x = add(Action::DtGood, "dt 1.0", x, 430.0F, false) + 8.0F;
    add(Action::DtBad, "dt 1.5", x, 430.0F, true);
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Preset0: return app.preset == 0;
        case Action::Preset1: return app.preset == 1;
        case Action::Preset2: return app.preset == 2;
        case Action::Preset3: return app.preset == 3;
        case Action::Reseed: return false;
        case Action::Pause: return app.paused;
        case Action::Buffered: return app.update == tp::Update::Buffered;
        case Action::InPlace: return app.update == tp::Update::InPlace;
        case Action::DtGood: return app.params.dt == kDts[0];
        case Action::DtBad: return app.params.dt == kDts[1];
        case Action::Speed0: return app.speed == 0;
        case Action::Speed1: return app.speed == 1;
        case Action::Speed2: return app.speed == 2;
    }
    return false;
}

void runSteps(App& app, int count, double budgetMs) {
    const double start = nowMs();
    for (int i = 0; i < count; ++i) {
        tp::step(app.field, app.params, app.update);
        ++app.steps;
        if (budgetMs > 0.0 && nowMs() - start > budgetMs) break;
    }
}

void startPreset(App& app, int index) {
    const tp::Preset& p = tp::kPresets[static_cast<std::size_t>(index)];
    app.preset = index;
    app.seedPreset = index;
    app.params.feed = p.feed;
    app.params.kill = p.kill;
    tp::plant(app.field, p, ++app.seedCounter);
    app.steps = 0;  // the counter belongs to this field
    runSteps(app, kPresetSteps, 0.0);
}

void reseed(App& app) {
    tp::plant(app.field, tp::kPresets[static_cast<std::size_t>(app.seedPreset)], ++app.seedCounter);
    app.steps = 0;
    runSteps(app, kPresetSteps, 0.0);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Preset0: startPreset(app, 0); break;
        case Action::Preset1: startPreset(app, 1); break;
        case Action::Preset2: startPreset(app, 2); break;
        case Action::Preset3: startPreset(app, 3); break;
        case Action::Reseed: reseed(app); break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Buffered: app.update = tp::Update::Buffered; break;
        case Action::InPlace: app.update = tp::Update::InPlace; break;
        case Action::DtGood: app.params.dt = kDts[0]; break;
        case Action::DtBad: app.params.dt = kDts[1]; break;
        case Action::Speed0: app.speed = 0; break;
        case Action::Speed1: app.speed = 1; break;
        case Action::Speed2: app.speed = 2; break;
    }
}

bool inRect(SDL_FPoint p, float x, float y, float w, float h) { return p.x >= x && p.x < x + w && p.y >= y && p.y < y + h; }
bool inField(SDL_FPoint p) { return inRect(p, kFieldX, kFieldY, kFieldW, kFieldH); }
bool inMap(SDL_FPoint p) { return inRect(p, kMap.x, kMap.y, kMap.w, kMap.h); }

int cellX(float px) { return static_cast<int>(std::floor((px - kFieldX) / static_cast<float>(kCell))); }
int cellY(float py) { return static_cast<int>(std::floor((py - kFieldY) / static_cast<float>(kCell))); }

/// Stamp the brush along the way from the last position, so a fast stroke leaves no gaps.
void stamp(App& app, SDL_FPoint to) {
    const SDL_FPoint from = app.lastPaint.x < 0.0F ? to : app.lastPaint;
    const float dx = to.x - from.x;
    const float dy = to.y - from.y;
    const int n = std::max(1, static_cast<int>(std::ceil(std::max(std::fabs(dx), std::fabs(dy)) / static_cast<float>(kCell))));
    for (int i = 1; i <= n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(n);
        const int cx = cellX(from.x + dx * t);
        const int cy = cellY(from.y + dy * t);
        if (app.drag == Drag::PaintB) tp::paint(app.field, cx, cy, kBrush);
        else tp::erase(app.field, cx, cy, kBrush);
    }
    app.lastPaint = to;
}

void setParamsFromMap(App& app, SDL_FPoint p) {
    tp::mapToParams(kMap, p.x, p.y, app.params.feed, app.params.kill);
    app.preset = -1;
}

void nudge(App& app, float df, float dk) {
    app.params.feed = std::clamp(app.params.feed + df, tp::kFeedMin, tp::kFeedMax);
    app.params.kill = std::clamp(app.params.kill + dk, tp::kKillMin, tp::kKillMax);
    app.preset = -1;
}

void endDrag(App& app) {
    app.drag = Drag::None;
    app.lastPaint = {-1.0F, -1.0F};
}

void uploadField(App& app) {
    for (std::size_t i = 0; i < app.field.a.size(); ++i) {
        const float l = tp::level(app.field.a[i], app.field.b[i]);
        const auto& c = app.palette[static_cast<std::size_t>(l * 255.0F + 0.5F)];
        Uint8* px = &app.pixels[i * 4U];
        px[0] = c[0];
        px[1] = c[1];
        px[2] = c[2];
        px[3] = 255;
    }
    SDL_UpdateTexture(app.fieldTexture, nullptr, app.pixels.data(), kGridW * 4);
}

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, kMap.x, kMap.y, kMap.w, kMap.h, gd::kWhite);
    for (int i = 1; i < 4; ++i) {
        gd::fillRect(r, kMap.x + kMap.w * static_cast<float>(i) / 4.0F, kMap.y, 1.0F, kMap.h, gd::kGrid);
        gd::fillRect(r, kMap.x, kMap.y + kMap.h * static_cast<float>(i) / 4.0F, kMap.w, 1.0F, gd::kGrid);
    }
    // The orange line: beyond it (to the right) no uniform mixture of A and B exists any more.
    constexpr int kSegments = 64;
    float px = 0.0F;
    float py = 0.0F;
    bool havePrev = false;
    for (int i = 0; i <= kSegments; ++i) {
        const float f = tp::kFeedMin + (tp::kFeedMax - tp::kFeedMin) * static_cast<float>(i) / static_cast<float>(kSegments);
        const float k = tp::killLimit(f);
        float x = 0.0F;
        float y = 0.0F;
        const bool visible = k >= tp::kKillMin && k <= tp::kKillMax;
        tp::paramsToMap(kMap, f, k, x, y);
        if (visible && havePrev) {
            gd::setColor(r, gd::kOrange);
            SDL_RenderLine(r, px, py, x, y);
            SDL_RenderLine(r, px + 1.0F, py, x + 1.0F, y);
        }
        havePrev = visible;
        px = x;
        py = y;
    }
    // Presets as numbered dots.
    for (std::size_t i = 0; i < tp::kPresets.size(); ++i) {
        float x = 0.0F;
        float y = 0.0F;
        tp::paramsToMap(kMap, tp::kPresets[i].feed, tp::kPresets[i].kill, x, y);
        gd::disc(r, x, y, 4.0F, gd::kTeal);
        gd::ring(r, x, y, 4.0F, gd::kInk);
        char n[2] = {static_cast<char>('1' + static_cast<int>(i)), '\0'};
        app.hud.text(r, x + 7.0F, y - 20.0F, n, gd::kTealDark, gd::Hud::Style::Bold);
    }
    // The current feed/kill: crosshair and marker.
    float cx = 0.0F;
    float cy = 0.0F;
    tp::paramsToMap(kMap, app.params.feed, app.params.kill, cx, cy);
    gd::fillRect(r, kMap.x, cy, kMap.w, 1.0F, gd::Color{17, 24, 39, 90});
    gd::fillRect(r, cx, kMap.y, 1.0F, kMap.h, gd::Color{17, 24, 39, 90});
    gd::disc(r, cx, cy, 6.0F, gd::kOrange);
    gd::ring(r, cx, cy, 6.0F, gd::kInk);
    gd::strokeRect(r, kMap.x, kMap.y, kMap.w, kMap.h, gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ TURING PATTERNS ]", "Gray-Scott reaction-diffusion");

    // The field.
    gd::card(r, kFieldX, kFieldY, kFieldW, kFieldH, gd::kWhite, gd::kInk);
    uploadField(app);
    const SDL_FRect dst{kFieldX, kFieldY, kFieldW, kFieldH};
    SDL_RenderTexture(r, app.fieldTexture, nullptr, &dst);
    gd::strokeRect(r, kFieldX, kFieldY, kFieldW, kFieldH, gd::kInk);
    if (inField(app.mouse)) {
        gd::ring(r, app.mouse.x, app.mouse.y, (static_cast<float>(kBrush) + 0.5F) * static_cast<float>(kCell), gd::Color{17, 24, 39, 200});
    }

    // Warnings for the two mistakes, on the field where they show.
    // The limit is the one of the buffered update, in place a cell already sees new neighbours and behaves differently.
    const bool tooBig = app.update == tp::Update::Buffered && app.params.dt > tp::maxStableDt(app.params);
    float wy = kFieldY + 8.0F;
    auto warn = [&](const char* text) {
        const float w = app.hud.width(text, gd::Hud::Style::Bold) + 16.0F;
        gd::card(r, kFieldX + 8.0F, wy, w, 28.0F, gd::kWhite, gd::kOrange);
        app.hud.text(r, kFieldX + 16.0F, wy + 3.0F, text, gd::kOrange, gd::Hud::Style::Bold);
        wy += 36.0F;
    };
    if (app.update == tp::Update::InPlace) warn("in place: scan order leaks in");
    if (tooBig) warn("dt above limit: checkerboard grows");

    // Right column: the map, then numbers and the two switches.
    app.hud.text(r, kPanelX, 42.0F, "Feed / kill map", gd::kInk, gd::Hud::Style::Bold);
    drawMap(app);
    app.hud.text(r, kPanelX, 220.0F, "x kill  y feed", gd::kMuted);
    app.hud.text(r, kPanelX, 242.0F, "orange: uniform limit", gd::kMuted);

    char line[64];
    SDL_snprintf(line, sizeof line, "f %.4f  k %.4f", static_cast<double>(app.params.feed), static_cast<double>(app.params.kill));
    app.hud.text(r, kPanelX, 272.0F, line, gd::kInk, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "steps %ld  B %d%%", app.steps, static_cast<int>(std::lround(100.0F * tp::coverage(app.field, 0.1F))));
    app.hud.text(r, kPanelX, 294.0F, line, gd::kInk);
    if (app.update == tp::Update::Buffered) {
        SDL_snprintf(line, sizeof line, "dt %.1f  limit %.2f", static_cast<double>(app.params.dt), static_cast<double>(tp::maxStableDt(app.params)));
    } else {
        SDL_snprintf(line, sizeof line, "dt %.1f  (limit differs)", static_cast<double>(app.params.dt));
    }
    app.hud.text(r, kPanelX, 316.0F, line, tooBig ? gd::kOrange : gd::kMuted);

    app.hud.text(r, kPanelX, 344.0F, "Update (U)", gd::kMuted);
    app.hud.text(r, kPanelX, 408.0F, "Time step (D)", gd::kMuted);

    app.hud.text(r, kFieldX, kFieldY + kFieldH + 4.0F, "drag: paint B   right drag: erase   arrows: nudge", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const gd::Color fill = on ? (c.naive ? gd::kOrange : gd::kTeal) : gd::kWhite;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("turing-patterns", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    app->fieldTexture = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kGridW, kGridH);
    if (app->fieldTexture == nullptr) {
        SDL_Log("field texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->fieldTexture, SDL_SCALEMODE_LINEAR);
    buildPalette(*app);
    layoutChips(*app);
    const tp::Preset& start = tp::kPresets[static_cast<std::size_t>(kStartPreset)];
    app->params.feed = start.feed;
    app->params.kill = start.kill;
    tp::plant(app->field, start, app->seedCounter);
    runSteps(*app, kStartSteps, 0.0);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = {event->motion.x, event->motion.y};
            if (app.drag == Drag::Map) setParamsFromMap(app, app.mouse);
            else if (app.drag != Drag::None) stamp(app, app.mouse);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            if (event->button.button != SDL_BUTTON_LEFT && event->button.button != SDL_BUTTON_RIGHT) break;
            bool onChip = false;
            if (event->button.button == SDL_BUTTON_LEFT) {
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) {
                        apply(app, c.action);
                        onChip = true;
                        break;
                    }
                }
            }
            if (onChip) break;
            if (inField(p)) {
                app.drag = event->button.button == SDL_BUTTON_LEFT ? Drag::PaintB : Drag::Erase;
                app.lastPaint = {-1.0F, -1.0F};
                stamp(app, p);
            } else if (inMap(p) && event->button.button == SDL_BUTTON_LEFT) {
                app.drag = Drag::Map;
                setParamsFromMap(app, p);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP: endDrag(app); break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: endDrag(app); break;
        case SDL_EVENT_KEY_DOWN:
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                // Arrows are meant to repeat, everything else is a toggle or a restart and must not.
                case SDLK_UP: nudge(app, 0.001F, 0.0F); break;
                case SDLK_DOWN: nudge(app, -0.001F, 0.0F); break;
                case SDLK_LEFT: nudge(app, 0.0F, -0.0005F); break;
                case SDLK_RIGHT: nudge(app, 0.0F, 0.0005F); break;
                default: break;
            }
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_1: apply(app, Action::Preset0); break;
                case SDLK_2: apply(app, Action::Preset1); break;
                case SDLK_3: apply(app, Action::Preset2); break;
                case SDLK_4: apply(app, Action::Preset3); break;
                case SDLK_R: apply(app, Action::Reseed); break;
                case SDLK_SPACE: apply(app, Action::Pause); break;
                case SDLK_U: apply(app, app.update == tp::Update::Buffered ? Action::InPlace : Action::Buffered); break;
                case SDLK_D: apply(app, app.params.dt == kDts[0] ? Action::DtBad : Action::DtGood); break;
                case SDLK_S: app.speed = (app.speed + 1) % 3; break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    if (!app.paused) runSteps(app, kStepsPerFrame[static_cast<std::size_t>(app.speed)], kBudgetMs);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->fieldTexture != nullptr) SDL_DestroyTexture(app->fieldTexture);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
