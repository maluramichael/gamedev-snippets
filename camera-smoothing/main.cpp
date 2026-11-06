// Camera smoothing with SDL3 main callbacks.
//
// A scripted player runs, stops, runs faster and teleports. Three cameras chase it, each in its own lane:
//   A  cam += gap * 0.1                       a fixed fraction per FRAME
//   B  cam += gap * (1 - exp(-lambda * dt))   the same fraction derived from the frame TIME
//   C  a critically damped spring             position and velocity, closed form
// lambda is chosen so that A and B are the same camera at 60 fps. Then the frame rate is switched (15, 60 or 144
// frames per second, simulated on any display): A changes completely, B and C hardly. Everything, the player included,
// advances in whole simulated frames, so 15 fps really looks like 15 fps. The scope on the right plots the gap or the
// camera speed over the last four seconds. The maths lives in logic.hpp and is covered by test.cpp.

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

constexpr float kLaneX = 24.0F;
constexpr float kLaneW = 912.0F;
constexpr float kLaneH = 118.0F;
constexpr float kLaneTop0 = 42.0F;
constexpr float kLaneStep = 126.0F;
constexpr float kViewX = 32.0F;  // the world strip inside a lane
constexpr float kViewW = 560.0F;
constexpr float kScopeX = 612.0F;
constexpr float kScopeW = 312.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;

constexpr double kLerpFactor = 0.1;  // the old constant, per frame, at the reference rate
constexpr double kReferenceFps = 60.0;
constexpr double kOmega = 10.0;  // spring frequency in rad/s
constexpr double kWindow = 4.0;  // seconds shown in the scope
constexpr double kLagRange = 300.0;
constexpr double kSpeedRange = 1600.0;
constexpr int kMaxSteps = 8;  // simulated frames per displayed frame, the rest of a stall is dropped
constexpr double kWarmUp = 3.2;  // seconds simulated before the first frame, so the player is already mid-run
constexpr std::array<int, 3> kRates{15, 60, 144};

const double kLambda = cs::lambdaForLerp(kLerpFactor, kReferenceFps);

constexpr gd::Color kInkSoft{17, 24, 39, 120};

enum class Action { Rate15, Rate60, Rate144, Scope, Restart };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Cam {
    double pos = 0.0;
    double speed = 0.0;  // px/s over the last frame, measured the same way for all three
    cs::Spring spring;
    cs::Trace lag;
    cs::Trace speedTrace;
};

struct Lane {
    const char* label;
    const char* note;
    gd::Color color;
};

constexpr std::array<Lane, 3> kLanes{{
    {"A  cam += gap * 0.1", "per frame", gd::kOrange},
    {"B  cam += gap * (1 - exp(-6.32 dt))", "per second", gd::kTeal},
    {"C  critically damped spring, omega 10", "with velocity", gd::kTeal},
}};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::array<Cam, 3> cams;
    cs::FrameClock clock;
    double simTime = 0.0;
    int rateIndex = 0;
    bool showSpeed = false;
    Uint64 lastCounter = 0;
    long maxFrames = -1;
    long frames = 0;
};

double frameStep(const App& app) { return 1.0 / static_cast<double>(kRates[static_cast<std::size_t>(app.rateIndex)]); }

float laneTop(std::size_t i) { return kLaneTop0 + kLaneStep * static_cast<float>(i); }

/// Rounded, signed whole number for the readouts. Avoids "-0" and keeps the width bounded.
long shown(double v) { return std::lround(std::clamp(v, -99999.0, 99999.0)); }

void stepFrame(App& app) {
    const double dt = frameStep(app);
    app.simTime += dt;
    const double target = cs::playerPos(app.simTime);
    for (std::size_t i = 0; i < app.cams.size(); ++i) {
        Cam& cam = app.cams[i];
        const double before = cam.pos;
        if (i == 0) {
            cam.pos = cs::lerpStep(cam.pos, target, kLerpFactor);
        } else if (i == 1) {
            cam.pos = cs::expStep(cam.pos, target, kLambda, dt);
        } else {
            cs::springStep(cam.spring, target, kOmega, dt);
            cam.pos = cam.spring.pos;
        }
        cam.speed = (cam.pos - before) / dt;
        cam.lag.push(app.simTime, target - cam.pos, kWindow);
        cam.speedTrace.push(app.simTime, cam.speed, kWindow);
    }
}

void restart(App& app) {
    app.simTime = 0.0;
    app.clock.reset();
    for (Cam& cam : app.cams) cam = Cam{};
    const int frames = static_cast<int>(std::ceil(kWarmUp / frameStep(app)));
    for (int i = 0; i < frames; ++i) stepFrame(app);
}

void setRate(App& app, int index) {
    if (index == app.rateIndex) return;
    app.rateIndex = index;
    app.clock.reset();  // the rest of a 15 fps frame must not become a burst of 144 fps frames
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Rate15: setRate(app, 0); break;
        case Action::Rate60: setRate(app, 1); break;
        case Action::Rate144: setRate(app, 2); break;
        case Action::Scope: app.showSpeed = !app.showSpeed; break;
        case Action::Restart: restart(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Rate15: return app.rateIndex == 0;
        case Action::Rate60: return app.rateIndex == 1;
        case Action::Rate144: return app.rateIndex == 2;
        case Action::Scope: return app.showSpeed;
        case Action::Restart: return false;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        const char* sizeFrom;  // the widest label this chip can show, so the row does not move
    };
    const std::array<Def, 5> defs{{{Action::Rate15, "1 15 fps", 0.0F, "1 15 fps"},
                                   {Action::Rate60, "2 60 fps", 8.0F, "2 60 fps"},
                                   {Action::Rate144, "3 144 fps", 8.0F, "3 144 fps"},
                                   {Action::Scope, "S Scope: lag", 24.0F, "S Scope: speed"},
                                   {Action::Restart, "R Restart", 24.0F, "R Restart"}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.sizeFrom, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void lineThick(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c) {
    gd::setColor(r, c);
    SDL_RenderLine(r, x0, y0, x1, y1);
    SDL_RenderLine(r, x0, y0 + 1.0F, x1, y1 + 1.0F);
}

void setClip(SDL_Renderer* r, float x, float y, float w, float h) {
    const SDL_Rect clip{static_cast<int>(x), static_cast<int>(y), static_cast<int>(w), static_cast<int>(h)};
    SDL_SetRenderClipRect(r, &clip);
}

/// The world strip of one lane: ground marks scrolling past, the camera line in the middle, the player, and a bracket
/// from the camera line to the player that is the gap.
void drawView(App& app, std::size_t i, float top) {
    SDL_Renderer* r = app.renderer;
    const Cam& cam = app.cams[i];
    const gd::Color color = kLanes[i].color;
    const float viewTop = top + 28.0F;
    const float baseline = top + 98.0F;
    const float viewH = 82.0F;
    const float centerX = kViewX + kViewW * 0.5F;
    const double target = cs::playerPos(app.simTime);
    const double lag = target - cam.pos;

    gd::fillRect(r, kViewX, viewTop, kViewW, viewH, gd::kPaper);
    setClip(r, kViewX, viewTop, kViewW, viewH);

    // Ground marks every 60 px of world, a taller one every 240.
    gd::fillRect(r, kViewX, baseline, kViewW, viewH - (baseline - viewTop), gd::Color{17, 24, 39, 22});
    const double firstMark = std::floor((cam.pos - static_cast<double>(kViewW) * 0.5) / 60.0);
    for (int k = 0; k < static_cast<int>(kViewW / 60.0F) + 2; ++k) {
        const double world = (firstMark + k) * 60.0;
        const auto x = static_cast<float>(static_cast<double>(centerX) + world - cam.pos);
        const bool tall = std::fmod(std::fabs(world), 240.0) < 1.0;
        gd::fillRect(r, x - 1.0F, baseline - (tall ? 16.0F : 7.0F), 2.0F, tall ? 16.0F : 7.0F, kInkSoft);
    }
    gd::fillRect(r, kViewX, baseline, kViewW, 2.0F, gd::kInk);

    // The camera line.
    for (float y = viewTop + 2.0F; y < baseline; y += 8.0F) gd::fillRect(r, centerX - 1.0F, y, 2.0F, 4.0F, color);

    // Gap bracket and player. A player farther away than the strip is pinned to its edge.
    const float edgeL = kViewX + 16.0F;
    const float edgeR = kViewX + kViewW - 16.0F;
    const float playerX = std::clamp(static_cast<float>(static_cast<double>(centerX) + lag), edgeL, edgeR);
    const float bracketY = top + 66.0F;
    if (std::fabs(lag) >= 1.0) {
        lineThick(r, std::min(centerX, playerX), bracketY, std::max(centerX, playerX), bracketY, color);
        gd::fillRect(r, centerX - 1.0F, bracketY - 4.0F, 2.0F, 10.0F, color);
        gd::fillRect(r, playerX - 1.0F, bracketY - 4.0F, 2.0F, 10.0F, color);
        char label[24];
        SDL_snprintf(label, sizeof label, "%ld px", shown(std::fabs(lag)));
        const float lw = app.hud.width(label, gd::Hud::Style::Bold);
        const float lx = std::clamp((centerX + playerX) * 0.5F - lw * 0.5F, kViewX + 4.0F, kViewX + kViewW - lw - 4.0F);
        gd::fillRect(r, lx - 4.0F, top + 40.0F, lw + 8.0F, 22.0F, gd::kPaper);  // keeps the camera line from running through the text
        app.hud.text(r, lx, top + 40.0F, label, color, gd::Hud::Style::Bold);
    }
    gd::fillRect(r, playerX - 12.0F, baseline - 24.0F, 24.0F, 24.0F, gd::kInk);
    gd::fillRect(r, playerX + (lag < 0.0 ? -7.0F : 1.0F), baseline - 18.0F, 6.0F, 6.0F, gd::kWhite);
    SDL_SetRenderClipRect(r, nullptr);
    gd::strokeRect(r, kViewX, viewTop, kViewW, viewH, gd::kGrid);
}

/// The scope: the last kWindow seconds of the gap or the camera speed, "now" at the right edge.
void drawScope(App& app, std::size_t i, float top) {
    SDL_Renderer* r = app.renderer;
    const Cam& cam = app.cams[i];
    const float scopeTop = top + 28.0F;
    const float scopeH = 82.0F;
    const double range = app.showSpeed ? kSpeedRange : kLagRange;
    const auto& trace = app.showSpeed ? cam.speedTrace : cam.lag;

    gd::fillRect(app.renderer, kScopeX, scopeTop, kScopeW, scopeH, gd::kPaper);
    const auto zeroY = static_cast<float>(cs::scopeY(0.0, range, static_cast<double>(scopeTop), static_cast<double>(scopeH)));
    gd::fillRect(r, kScopeX, zeroY, kScopeW, 1.0F, gd::Color{107, 114, 128, 150});
    setClip(r, kScopeX, scopeTop, kScopeW, scopeH);
    const auto& s = trace.samples;
    for (std::size_t k = 1; k < s.size(); ++k) {
        const auto x0 = static_cast<float>(cs::scopeX(s[k - 1].t, app.simTime, kWindow, static_cast<double>(kScopeX), static_cast<double>(kScopeW)));
        const auto x1 = static_cast<float>(cs::scopeX(s[k].t, app.simTime, kWindow, static_cast<double>(kScopeX), static_cast<double>(kScopeW)));
        const auto y0 = static_cast<float>(cs::scopeY(s[k - 1].v, range, static_cast<double>(scopeTop), static_cast<double>(scopeH - 1.0F)));
        const auto y1 = static_cast<float>(cs::scopeY(s[k].v, range, static_cast<double>(scopeTop), static_cast<double>(scopeH - 1.0F)));
        lineThick(r, x0, y0, x1, y1, kLanes[i].color);
    }
    SDL_SetRenderClipRect(r, nullptr);
    gd::strokeRect(r, kScopeX, scopeTop, kScopeW, scopeH, gd::kGrid);

    char value[32];
    if (app.showSpeed) {
        SDL_snprintf(value, sizeof value, "%ld px/s", shown(cam.speed));
    } else {
        SDL_snprintf(value, sizeof value, "%ld px", shown(cs::playerPos(app.simTime) - cam.pos));
    }
    app.hud.text(r, kScopeX, top + 5.0F, app.showSpeed ? "speed, 4 s" : "gap, 4 s", gd::kMuted);
    app.hud.text(r, kScopeX + kScopeW - app.hud.width(value, gd::Hud::Style::Bold), top + 5.0F, value, kLanes[i].color, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ CAMERA SMOOTHING ]", "lerp vs exp decay vs spring");

    for (std::size_t i = 0; i < kLanes.size(); ++i) {
        const float top = laneTop(i);
        gd::card(r, kLaneX, top, kLaneW, kLaneH, gd::kWhite, gd::kInk);
        app.hud.text(r, kViewX, top + 5.0F, kLanes[i].label, kLanes[i].color, gd::Hud::Style::Bold);
        app.hud.text(r, kViewX + kViewW - app.hud.width(kLanes[i].note), top + 5.0F, kLanes[i].note, gd::kMuted);
        drawView(app, i, top);
        drawScope(app, i, top);
    }

    const int fps = kRates[static_cast<std::size_t>(app.rateIndex)];
    char line[96];
    SDL_snprintf(line, sizeof line, "%d fps = %.1f ms per frame", fps, 1000.0 / static_cast<double>(fps));
    app.hud.text(r, 40.0F, 418.0F, line, gd::kInk, gd::Hud::Style::Bold);
    const char* scale = "gap = target - cam   scope range ±300 px, ±1600 px/s";
    app.hud.text(r, kLaneX + kLaneW - 16.0F - app.hud.width(scale), 418.0F, scale, gd::kMuted);
    app.hud.text(r, 40.0F, 442.0F, "A and B match at 60 fps. At other rates A moves a lot, B and C barely.", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const std::string label = c.action == Action::Scope ? (app.showSpeed ? "S Scope: speed" : "S Scope: lag") : c.label;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("camera-smoothing", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    app->lastCounter = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c.action);
            }
            break;
        }
        case SDL_EVENT_WINDOW_FOCUS_LOST:
        case SDL_EVENT_WINDOW_FOCUS_GAINED:
            // Time spent in the background must not become a burst of frames.
            app.clock.reset();
            app.lastCounter = SDL_GetPerformanceCounter();
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Rate15); break;
                case SDLK_2: apply(app, Action::Rate60); break;
                case SDLK_3: apply(app, Action::Rate144); break;
                case SDLK_S: apply(app, Action::Scope); break;
                case SDLK_R: apply(app, Action::Restart); break;
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
    const double realDt = static_cast<double>(now - app.lastCounter) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.lastCounter = now;
    const int steps = app.clock.advance(realDt, frameStep(app), kMaxSteps);
    for (int i = 0; i < steps; ++i) stepFrame(app);

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
