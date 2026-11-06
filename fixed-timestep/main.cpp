// Fixed timestep: why the drawn position is interpolated.
//
// Logic runs in fixed steps, here deliberately slow so the effect is visible. The orange square is drawn
// at the latest logic position and jumps from step to step. The teal square is drawn between the last two
// logic states (alpha = leftover time / step) and moves smoothly. Click a rate (or keys 1 to 4) to change the
// logic rate. The stall button (key S) simulates half a second of freeze: the frame time clamp keeps the loop
// from running hundreds of catch-up steps (the spiral of death).

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>

#include "hud.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kSize = 56.0F;
constexpr float kLeft = 40.0F;
constexpr float kRight = kWidth - 40.0F - kSize;
constexpr float kSpeed = 360.0F;        // pixels per second
constexpr double kMaxFrameTime = 0.25;  // clamp: avoids the spiral of death after a stall
constexpr std::array<int, 4> kRates{5, 10, 20, 60};

struct Body {
    float prev = kLeft;
    float cur = kLeft;
    float dir = 1.0F;

    void step(double dt) {
        prev = cur;
        cur += dir * kSpeed * static_cast<float>(dt);
        if (cur < kLeft || cur > kRight) {
            cur = std::clamp(cur, kLeft, kRight);
            dir = -dir;
        }
    }
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    Uint64 last = 0;
    double accumulator = 0.0;
    double stall = 0.0;
    int rateIndex = 1;
    Body body;
    long maxFrames = -1;
    long frames = 0;
    int stepsThisFrame = 0;
    int stalledSteps = 0;
    double stalledFor = 0.0;
};

SDL_FRect chip(int i) { return SDL_FRect{40.0F + 150.0F + static_cast<float>(i) * 92.0F, 476.0F, 80.0F, 34.0F}; }
SDL_FRect stallButton() { return SDL_FRect{kWidth - 40.0F - 200.0F, 476.0F, 200.0F, 34.0F}; }

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
    if (!SDL_CreateWindowAndRenderer("fixed-timestep", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) {
        return SDL_APP_SUCCESS;
    }
    if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        const SDL_FPoint p{event->button.x, event->button.y};
        const SDL_FRect stallRect = stallButton();
        if (SDL_PointInRectFloat(&p, &stallRect)) {
            app.stall = 0.5;
        } else {
            bool onChip = false;
            for (int i = 0; i < 4; ++i) {
                const SDL_FRect c = chip(i);
                if (SDL_PointInRectFloat(&p, &c)) {
                    app.rateIndex = i;
                    onChip = true;
                }
            }
            if (!onChip) app.rateIndex = (app.rateIndex + 1) % 4;
        }
    }
    if (event->type == SDL_EVENT_KEY_DOWN) {
        if (event->key.key >= SDLK_1 && event->key.key <= SDLK_4) {
            app.rateIndex = static_cast<int>(event->key.key - SDLK_1);
        } else if (event->key.key == SDLK_S) {
            app.stall = 0.5;
        }
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    double frameTime = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    frameTime += app.stall;
    app.stall = 0.0;
    frameTime = std::min(frameTime, kMaxFrameTime);

    const double stepTime = 1.0 / kRates[static_cast<std::size_t>(app.rateIndex)];
    app.accumulator += frameTime;
    app.stepsThisFrame = 0;
    while (app.accumulator >= stepTime) {
        app.body.step(stepTime);
        app.accumulator -= stepTime;
        ++app.stepsThisFrame;
    }
    if (app.stepsThisFrame > 1) {
        app.stalledSteps = app.stepsThisFrame;
        app.stalledFor = 2.0;
    }
    app.stalledFor = std::max(0.0, app.stalledFor - frameTime);

    const auto alpha = static_cast<float>(app.accumulator / stepTime);
    const float smooth = app.body.prev + (app.body.cur - app.body.prev) * alpha;

    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char text[96];
    SDL_snprintf(text, sizeof text, "logic %d steps/s", kRates[static_cast<std::size_t>(app.rateIndex)]);
    app.hud.titleBar(r, kWidth, "[ FIXED TIMESTEP ]", text);

    const float y1 = 130.0F;
    const float y2 = 270.0F;
    gd::fillRect(r, kLeft, y1 + kSize + 10.0F, kRight - kLeft + kSize, 2.0F, gd::Color{17, 24, 39, 60});
    gd::fillRect(r, kLeft, y2 + kSize + 10.0F, kRight - kLeft + kSize, 2.0F, gd::Color{17, 24, 39, 60});
    gd::fillRect(r, app.body.cur, y1, kSize, kSize, gd::Color{234, 88, 12, 50});
    gd::card(r, app.body.cur, y1, kSize, kSize, gd::kOrange, gd::kInk);
    gd::card(r, smooth, y2, kSize, kSize, gd::kTeal, gd::kInk);
    app.hud.text(r, kLeft, y1 - 30.0F, "drawn at the last logic state", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kLeft, y2 - 30.0F, "interpolated with alpha", gd::kInk, gd::Hud::Style::Bold);

    app.hud.text(r, kLeft, 380.0F, "alpha", gd::kMuted);
    gd::fillRect(r, kLeft + 70.0F, 378.0F, 300.0F, 22.0F, gd::kWhite);
    gd::fillRect(r, kLeft + 70.0F, 378.0F, 300.0F * alpha, 22.0F, gd::kTeal);
    gd::strokeRect(r, kLeft + 70.0F, 378.0F, 300.0F, 22.0F, gd::kInk);
    SDL_snprintf(text, sizeof text, "%.2f", static_cast<double>(alpha));
    app.hud.text(r, kLeft + 382.0F, 380.0F, text, gd::kInk, gd::Hud::Style::Bold);
    if (app.stalledFor > 0.0) {
        SDL_snprintf(text, sizeof text, "stall: %d steps caught up in one frame (clamped to 0.25 s)", app.stalledSteps);
        app.hud.text(r, kLeft, 414.0F, text, gd::kOrange, gd::Hud::Style::Bold);
    }

    app.hud.text(r, kLeft, 484.0F, "logic rate:", gd::kMuted);
    for (int i = 0; i < 4; ++i) {
        const SDL_FRect c = chip(i);
        const bool active = i == app.rateIndex;
        gd::card(r, c.x, c.y, c.w, c.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
        SDL_snprintf(text, sizeof text, "%d", kRates[static_cast<std::size_t>(i)]);
        app.hud.text(r, c.x + (c.w - app.hud.width(text, gd::Hud::Style::Bold)) / 2.0F, c.y + 6.0F, text, active ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    const SDL_FRect s = stallButton();
    gd::card(r, s.x, s.y, s.w, s.h, gd::kWhite, gd::kOrange);
    app.hud.text(r, s.x + (s.w - app.hud.width("simulate stall (S)", gd::Hud::Style::Bold)) / 2.0F, s.y + 6.0F, "simulate stall (S)", gd::kOrange, gd::Hud::Style::Bold);
    SDL_RenderPresent(r);

    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
