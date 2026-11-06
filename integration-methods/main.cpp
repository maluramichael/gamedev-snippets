// Integration methods with SDL3 main callbacks.
//
// One planet on a circular orbit and one mass on a spring, each simulated by four methods at the same time from the
// same start: explicit Euler, semi-implicit Euler (the same two lines in the other order), Verlet and RK4. Both
// systems keep their energy forever in real life, so the energy a method reports is the error it makes. The dt chips
// change the step size (and restart, Verlet cannot change its step mid-run). The simulation, the energy bookkeeping and the
// numbers on the screen live in logic.hpp and are covered by test.cpp; this file only owns the loop and the picture.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;

constexpr double kWarmUp = 6.0;  // simulated seconds already run when the page opens, so the first frame has a story
constexpr std::array<double, 5> kDts{0.02, 0.05, 0.1, 0.2, 0.4};
constexpr std::size_t kDefaultDt = 2;

constexpr SDL_FRect kOrbitBox{24.0F, 38.0F, 452.0F, 180.0F};
constexpr SDL_FRect kSpringBox{484.0F, 38.0F, 452.0F, 180.0F};
constexpr SDL_FRect kOrbitGraph{24.0F, 226.0F, 452.0F, 130.0F};
constexpr SDL_FRect kSpringGraph{484.0F, 226.0F, 452.0F, 130.0F};
constexpr float kGutter = 52.0F;  // room for the scale labels on the left of a graph
constexpr float kReadoutY = 362.0F;
constexpr float kShowChipY = 394.0F;
constexpr float kHintY = 434.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;

constexpr float kOrbitScale = 42.0F;   // px per world unit
constexpr float kSpringScale = 60.0F;
constexpr float kMassHalf = 13.0F;

constexpr gd::Color kBlue{37, 99, 235};
constexpr gd::Color kViolet{124, 58, 237};
constexpr std::array<gd::Color, im::kMethods> kColors{gd::kOrange, gd::kTeal, kBlue, kViolet};
constexpr std::array<const char*, im::kMethods> kNames{"explicit Euler", "semi-implicit Euler", "Verlet", "RK4"};
constexpr std::array<float, im::kMethods> kPlanetRadius{9.0F, 7.5F, 6.0F, 4.5F};
constexpr gd::Color kRing{107, 114, 128, 150};

enum class Kind { Play, Restart, Dt, Show };

struct Chip {
    Kind kind;
    std::size_t index;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    im::World world;
    std::vector<Chip> chips;
    std::size_t dtIndex = kDefaultDt;
    std::array<bool, im::kMethods> shown{true, true, true, true};
    bool paused = false;
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

bool inside(const SDL_FRect& r, float x, float y) { return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h; }

float px(double v) { return static_cast<float>(std::clamp(v, -1e5, 1e5)); }

void restart(App& app) { app.world.restart(kDts[app.dtIndex]); }

void togglePause(App& app) { app.paused = !app.paused; }

void setDt(App& app, std::size_t index) {
    if (index >= kDts.size()) return;
    app.dtIndex = index;
    restart(app);
}

void toggleShown(App& app, std::size_t method) {
    if (method < im::kMethods) app.shown[method] = !app.shown[method];
}

void layoutChips(App& app) {
    float x = 24.0F;
    auto add = [&](Kind kind, std::size_t index, const char* label, float y, float gapBefore) {
        x += gapBefore;
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({kind, index, SDL_FRect{x, y, w, y == kChipY ? kChipH : 30.0F}});
        x += w;
    };
    add(Kind::Play, 0, "Space Pause", kChipY, 0.0F);
    add(Kind::Restart, 0, "R Restart", kChipY, 8.0F);
    static const std::array<const char*, 5> dtLabels{"dt 0.02", "dt 0.05", "dt 0.10", "dt 0.20", "dt 0.40"};
    for (std::size_t i = 0; i < kDts.size(); ++i) add(Kind::Dt, i, dtLabels[i], kChipY, i == 0 ? 32.0F : 8.0F);
    x = 24.0F;
    static const std::array<const char*, im::kMethods> showLabels{"1 explicit Euler", "2 semi-implicit Euler", "3 Verlet", "4 RK4"};
    for (std::size_t m = 0; m < im::kMethods; ++m) add(Kind::Show, m, showLabels[m], kShowChipY, m == 0 ? 0.0F : 8.0F);
}

void apply(App& app, const Chip& chip) {
    switch (chip.kind) {
        case Kind::Play: togglePause(app); break;
        case Kind::Restart: restart(app); break;
        case Kind::Dt: setDt(app, chip.index); break;
        case Kind::Show: toggleShown(app, chip.index); break;
    }
}

/// 1 px lines are thin on a 960x540 canvas that gets scaled up, so draw a second one beside it.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, bool thick = true) {
    gd::setColor(r, c);
    SDL_RenderLine(r, x0, y0, x1, y1);
    if (!thick) return;
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    SDL_RenderLine(r, x0 + (flat ? 0.0F : 1.0F), y0 + (flat ? 1.0F : 0.0F), x1 + (flat ? 0.0F : 1.0F), y1 + (flat ? 1.0F : 0.0F));
}

void dashedLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, float dash = 6.0F, float gap = 5.0F) {
    const float len = std::hypot(x1 - x0, y1 - y0);
    if (len < 1e-3F) return;
    const float ux = (x1 - x0) / len;
    const float uy = (y1 - y0) / len;
    for (float s = 0.0F; s < len; s += dash + gap) {
        const float e = std::min(s + dash, len);
        thickLine(r, x0 + ux * s, y0 + uy * s, x0 + ux * e, y0 + uy * e, c, false);
    }
}

void clipTo(SDL_Renderer* r, const SDL_FRect& box, float inset = 0.0F) {
    const SDL_Rect clip{static_cast<int>(box.x + inset), static_cast<int>(box.y + inset), static_cast<int>(box.w - 2.0F * inset),
                        static_cast<int>(box.h - 2.0F * inset)};
    SDL_SetRenderClipRect(r, &clip);
}

void unclip(SDL_Renderer* r) { SDL_SetRenderClipRect(r, nullptr); }

/// A body that left the panel gets a ring on the panel's edge, in the direction it went.
void edgeMarker(App& app, const SDL_FRect& box, float cx, float cy, float x, float y, gd::Color c, const char* label) {
    const float dx = x - cx;
    const float dy = y - cy;
    const float hw = box.w / 2.0F - 12.0F;
    const float hh = box.h / 2.0F - 12.0F;
    const float t = std::min(std::fabs(dx) > 1e-3F ? hw / std::fabs(dx) : 1e9F, std::fabs(dy) > 1e-3F ? hh / std::fabs(dy) : 1e9F);
    const float ex = cx + dx * t;
    const float ey = cy + dy * t;
    gd::ring(app.renderer, ex, ey, 6.0F, c);
    gd::ring(app.renderer, ex, ey, 5.0F, c);
    const float lx = dx > 0.0F ? ex - 12.0F - app.hud.width(label) : ex + 12.0F;
    app.hud.text(app.renderer, lx, ey - 11.0F, label, c);
}

void title(App& app, const SDL_FRect& box, const char* text) {
    app.hud.text(app.renderer, box.x + 12.0F, box.y + 6.0F, text, gd::kInk, gd::Hud::Style::Bold);
}

void drawOrbit(App& app) {
    SDL_Renderer* r = app.renderer;
    const im::Run& run = app.world.orbit;
    const SDL_FRect& box = kOrbitBox;
    gd::card(r, box.x, box.y, box.w, box.h, gd::kWhite, gd::kInk);
    title(app, box, "ORBIT");
    app.hud.text(r, box.x + 12.0F, box.y + box.h - 28.0F, "grey ring: the exact orbit", gd::kMuted);

    const float cx = box.x + box.w / 2.0F;
    const float cy = box.y + box.h / 2.0F + 4.0F;
    clipTo(r, box, 2.0F);
    gd::ring(r, cx, cy, kOrbitScale, kRing);
    gd::ring(r, cx, cy, kOrbitScale + 1.0F, kRing);
    gd::disc(r, cx, cy, 9.0F, gd::kMuted, gd::kInk);

    for (std::size_t m = 0; m < im::kMethods; ++m) {
        if (!app.shown[m]) continue;
        gd::Color line = kColors[m];
        line.a = 150;
        const auto& items = run.trail[m].items;
        for (std::size_t i = 1; i < items.size(); ++i) {
            thickLine(r, px(cx + items[i - 1].value.x * kOrbitScale), px(cy + items[i - 1].value.y * kOrbitScale), px(cx + items[i].value.x * kOrbitScale),
                      px(cy + items[i].value.y * kOrbitScale), line);
        }
        if (run.dt >= 0.1) {  // big steps: show where each one landed
            for (const auto& item : items) {
                gd::fillRect(r, px(cx + item.value.x * kOrbitScale) - 1.5F, px(cy + item.value.y * kOrbitScale) - 1.5F, 3.0F, 3.0F, kColors[m]);
            }
        }
    }
    unclip(r);

    for (std::size_t m = 0; m < im::kMethods; ++m) {
        if (!app.shown[m]) continue;
        const float x = px(cx + run.body[m].pos.x * kOrbitScale);
        const float y = px(cy + run.body[m].pos.y * kOrbitScale);
        clipTo(r, box, 2.0F);
        gd::disc(r, x, y, kPlanetRadius[m], kColors[m], gd::kInk);
        unclip(r);
        if (!inside(box, x, y)) {
            char label[24];
            SDL_snprintf(label, sizeof label, "r %.1f", im::length(run.body[m].pos));
            edgeMarker(app, box, cx, cy, x, y, kColors[m], label);
        }
    }
}

void drawSpring(App& app) {
    SDL_Renderer* r = app.renderer;
    const im::Run& run = app.world.spring;
    const SDL_FRect& box = kSpringBox;
    gd::card(r, box.x, box.y, box.w, box.h, gd::kWhite, gd::kInk);
    title(app, box, "SPRING");
    app.hud.text(r, box.x + box.w - 12.0F - app.hud.width("dashed: start amplitude"), box.y + 6.0F, "dashed: start amplitude", gd::kMuted);

    const float wall = box.x + 22.0F;
    const float rest = box.x + box.w / 2.0F + 10.0F;
    const float top = box.y + 36.0F;
    const float laneH = (box.h - 44.0F) / 4.0F;
    clipTo(r, box, 2.0F);
    gd::fillRect(r, wall - 8.0F, top - 6.0F, 8.0F, laneH * 4.0F + 6.0F, gd::kInk);
    dashedLine(r, rest, top - 6.0F, rest, top + laneH * 4.0F, kRing);
    for (const float side : {-1.0F, 1.0F}) dashedLine(r, rest + side * kSpringScale, top - 6.0F, rest + side * kSpringScale, top + laneH * 4.0F, gd::kTeal);

    std::array<float, im::kMethods> massX{};
    for (std::size_t m = 0; m < im::kMethods; ++m) {
        massX[m] = px(rest + run.body[m].pos.x * kSpringScale);
        if (!app.shown[m] || run.body[m].gone) continue;
        const float y = top + laneH * (static_cast<float>(m) + 0.5F);
        const float x0 = wall;
        const float x1 = massX[m] - kMassHalf;
        constexpr int kZigs = 12;
        float lastX = x0;
        float lastY = y;
        for (int i = 1; i <= kZigs; ++i) {
            const float nx = x0 + (x1 - x0) * static_cast<float>(i) / static_cast<float>(kZigs);
            const float ny = y + (i == kZigs ? 0.0F : (i % 2 == 0 ? -7.0F : 7.0F));
            thickLine(r, lastX, lastY, nx, ny, kColors[m]);
            lastX = nx;
            lastY = ny;
        }
        gd::fillRect(r, massX[m] - kMassHalf, y - kMassHalf, kMassHalf * 2.0F, kMassHalf * 2.0F, kColors[m]);
        gd::strokeRect(r, massX[m] - kMassHalf, y - kMassHalf, kMassHalf * 2.0F, kMassHalf * 2.0F, gd::kInk);
    }
    unclip(r);
    for (std::size_t m = 0; m < im::kMethods; ++m) {
        if (!app.shown[m]) continue;
        const float y = top + laneH * (static_cast<float>(m) + 0.5F);
        if (massX[m] < wall + 18.0F || massX[m] > box.x + box.w - 14.0F) {
            // Too far out to draw: a ring on the panel edge on the side it went.
            const bool leftSide = massX[m] < rest;
            const float ex = leftSide ? wall + 18.0F : box.x + box.w - 14.0F;
            gd::ring(r, ex, y, 6.0F, kColors[m]);
            gd::ring(r, ex, y, 5.0F, kColors[m]);
            char label[24];
            SDL_snprintf(label, sizeof label, "x %.1f", run.body[m].pos.x);
            app.hud.text(r, leftSide ? ex + 12.0F : ex - 12.0F - app.hud.width(label), y - 11.0F, label, kColors[m]);
        }
    }
}

void drawGraph(App& app, const SDL_FRect& box, const im::Run& run) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, box.x, box.y, box.w, box.h, gd::kWhite, gd::kInk);
    const float midY = box.y + box.h / 2.0F;
    const float halfH = box.h / 2.0F - 6.0F;
    const float plotX = box.x + kGutter;
    const float plotW = box.w - kGutter - 4.0F;
    const double now = run.time();
    const double left = std::max(0.0, now - im::kPlotWindow);
    auto xOf = [&](double t) { return plotX + static_cast<float>((t - left) / im::kPlotWindow) * plotW; };
    auto yOf = [&](double drift) { return midY - static_cast<float>(im::symlog(drift)) * halfH; };

    clipTo(r, box, 2.0F);
    // One grid line per decade, 0.1 % to 100 %, mirrored below zero.
    for (const double d : {0.001, 0.01, 0.1, 1.0}) {
        gd::fillRect(r, plotX, yOf(d), plotW, 1.0F, gd::kGrid);
        gd::fillRect(r, plotX, yOf(-d), plotW, 1.0F, gd::kGrid);
    }
    gd::fillRect(r, plotX, midY, plotW, 1.0F, gd::kMuted);
    gd::fillRect(r, plotX - 1.0F, box.y + 2.0F, 1.0F, box.h - 4.0F, gd::kGrid);
    clipTo(r, SDL_FRect{plotX, box.y, plotW + 2.0F, box.h}, 2.0F);
    for (std::size_t m = 0; m < im::kMethods; ++m) {
        if (!app.shown[m]) continue;
        const auto& items = run.drift[m].items;
        for (std::size_t i = 1; i < items.size(); ++i) {
            thickLine(r, xOf(items[i - 1].t), yOf(items[i - 1].value), xOf(items[i].t), yOf(items[i].value), kColors[m]);
        }
    }
    unclip(r);

    app.hud.text(r, box.x + 8.0F, box.y + 3.0F, "100%", gd::kMuted);
    app.hud.text(r, box.x + 8.0F, yOf(0.01) - 11.0F, "1%", gd::kMuted);
    app.hud.text(r, box.x + 8.0F, midY - 11.0F, "0", gd::kMuted);
    app.hud.text(r, box.x + 8.0F, yOf(-0.01) - 11.0F, "-1%", gd::kMuted);
    app.hud.text(r, box.x + 8.0F, box.y + box.h - 25.0F, "-100%", gd::kMuted);
    const char* label = "energy drift, last 20 s";
    app.hud.text(r, box.x + box.w - 8.0F - app.hud.width(label), box.y + box.h - 25.0F, label, gd::kMuted);
}

void drawReadout(App& app, const SDL_FRect& box, const im::Run& run) {
    const float cell = box.w / static_cast<float>(im::kMethods);
    for (std::size_t m = 0; m < im::kMethods; ++m) {
        const float x = box.x + cell * static_cast<float>(m);
        gd::fillRect(app.renderer, x + 4.0F, kReadoutY + 6.0F, 10.0F, 10.0F, app.shown[m] ? kColors[m] : gd::kGrid);
        const std::string text = !app.shown[m] ? "off" : im::formatDrift(run.driftOf(m));
        app.hud.text(app.renderer, x + 20.0F, kReadoutY, text, app.shown[m] ? kColors[m] : gd::kMuted, gd::Hud::Style::Bold);
    }
}

void drawChips(App& app) {
    for (const Chip& c : app.chips) {
        bool on = false;
        gd::Color fill = gd::kWhite;
        std::string label;
        switch (c.kind) {
            case Kind::Play:
                on = app.paused;
                label = app.paused ? "Space Run" : "Space Pause";
                break;
            case Kind::Restart: label = "R Restart"; break;
            case Kind::Dt:
                on = app.dtIndex == c.index;
                label = std::string("dt ") + (c.index == 0 ? "0.02" : c.index == 1 ? "0.05" : c.index == 2 ? "0.10" : c.index == 3 ? "0.20" : "0.40");
                break;
            case Kind::Show:
                on = app.shown[c.index];
                fill = kColors[c.index];
                label = std::string(1, static_cast<char>('1' + c.index)) + " " + kNames[c.index];
                break;
        }
        const gd::Color bg = c.kind == Kind::Show ? (on ? fill : gd::kWhite) : (on ? gd::kTeal : gd::kWhite);
        gd::card(app.renderer, c.rect.x, c.rect.y, c.rect.w, c.rect.h, bg, gd::kInk);
        const float tx = c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F;
        app.hud.text(app.renderer, tx, c.rect.y + (c.rect.h - 22.0F) / 2.0F, label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "t = %.1f s    dt = %.2f s%s", app.world.orbit.time(), app.world.dt, app.paused ? "    paused" : "");
    app.hud.titleBar(r, kWidth, "[ INTEGRATION METHODS ]", right);

    drawOrbit(app);
    drawSpring(app);
    drawGraph(app, kOrbitGraph, app.world.orbit);
    drawGraph(app, kSpringGraph, app.world.spring);
    drawReadout(app, kOrbitGraph, app.world.orbit);
    drawReadout(app, kSpringGraph, app.world.spring);

    app.hud.text(r, 24.0F, kHintY, "Same start, same dt. A dt chip restarts the run. Keys: Space, R, left/right = dt, 1 to 4.", gd::kMuted);
    drawChips(app);
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
    if (!SDL_CreateWindowAndRenderer("integration-methods", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    restart(*app);
    app->world.warmUp(kWarmUp);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    SDL_ConvertEventToRenderCoordinates(static_cast<App*>(appstate)->renderer, event);
    auto& app = *static_cast<App*>(appstate);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (event->button.button == SDL_BUTTON_LEFT) {
                for (const Chip& c : app.chips) {
                    if (inside(c.rect, event->button.x, event->button.y)) apply(app, c);
                }
            }
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            if (event->key.repeat) break;  // every key here is a toggle or a restart
            switch (event->key.key) {
                case SDLK_SPACE: togglePause(app); break;
                case SDLK_R: restart(app); break;
                case SDLK_LEFT: setDt(app, app.dtIndex > 0 ? app.dtIndex - 1 : 0); break;
                case SDLK_RIGHT: setDt(app, std::min(app.dtIndex + 1, kDts.size() - 1)); break;
                case SDLK_1: toggleShown(app, 0); break;
                case SDLK_2: toggleShown(app, 1); break;
                case SDLK_3: toggleShown(app, 2); break;
                case SDLK_4: toggleShown(app, 3); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double frameSeconds = static_cast<double>(now - app.lastTicks) * 1e-9;
    app.lastTicks = now;
    if (!app.paused) app.world.advance(frameSeconds);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
