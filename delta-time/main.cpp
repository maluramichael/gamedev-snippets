// Delta time with SDL3 main callbacks.
//
// Two squares run the same lap. The orange one moves a fixed number of pixels per frame, the teal one moves
// speed * delta time. Click a rate (or press 1 to 4) to change how often frames are produced: the orange
// square follows the frame rate, the teal one does not.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#include "hud.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kSize = 56.0F;
constexpr float kMargin = 40.0F;
constexpr float kSpeed = 360.0F;                   // pixels per second
constexpr float kPixelsPerFrame = kSpeed / 60.0F;  // same speed, but only at exactly 60 frames per second
constexpr double kMaxDelta = 0.25;                 // clamp after a stall (tab switch, breakpoint)
constexpr double kSlack = 0.002;                   // display refresh jitter: a frame 2 ms early still counts as due
constexpr std::array<int, 4> kRates{15, 30, 60, 144};
constexpr int kTrail = 6;
constexpr float kLeft = kMargin;
constexpr float kRight = kWidth - kMargin - kSize;

struct Runner {
    float x = kLeft;
    float dir = 1.0F;
    float travelled = 0.0F;  // pixels in the current measuring second
    std::array<float, kTrail> trail{};

    void move(float distance) {
        for (int i = kTrail - 1; i > 0; --i) trail[static_cast<std::size_t>(i)] = trail[static_cast<std::size_t>(i) - 1];
        trail[0] = x;
        const float before = x;
        x += dir * distance;
        if (x < kLeft || x > kRight) {
            x = std::clamp(x, kLeft, kRight);
            dir = -dir;
        }
        travelled += std::fabs(x - before);
    }
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    Uint64 lastFrame = 0;
    Uint64 lastSecond = 0;
    int rateIndex = 2;
    Runner perFrame;
    Runner perSecond;
    long maxFrames = -1;
    long frames = 0;
    int framesThisSecond = 0;
    int fps = 0;
    float perFrameSpeed = 0.0F;
    float perSecondSpeed = 0.0F;
};

SDL_FRect chip(int i) {
    return SDL_FRect{kMargin + 270.0F + static_cast<float>(i) * 92.0F, 476.0F, 80.0F, 34.0F};
}

void drawLane(App& app, float y, const Runner& runner, gd::Color color, const char* formula, const char* caption, float speed) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, kLeft, y + kSize + 10.0F, kRight - kLeft + kSize, 2.0F, gd::Color{17, 24, 39, 60});
    for (int i = kTrail - 1; i >= 0; --i) {
        const float alpha = 70.0F * (1.0F - static_cast<float>(i) / static_cast<float>(kTrail));
        gd::fillRect(r, runner.trail[static_cast<std::size_t>(i)], y, kSize, kSize, gd::Color{color.r, color.g, color.b, static_cast<Uint8>(alpha)});
    }
    gd::card(r, runner.x, y, kSize, kSize, color, gd::kInk);
    app.hud.text(r, kLeft, y - 54.0F, formula, gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kLeft, y - 32.0F, caption, gd::kMuted);
    char text[48];
    SDL_snprintf(text, sizeof text, "%4.0f px/s", static_cast<double>(speed));
    app.hud.text(r, kWidth - kMargin - app.hud.width(text, gd::Hud::Style::Bold), y - 54.0F, text, color, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("delta-time", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    app->lastFrame = SDL_GetPerformanceCounter();
    app->lastSecond = app->lastFrame;
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
    if (event->type == SDL_EVENT_KEY_DOWN && event->key.key >= SDLK_1 && event->key.key <= SDLK_4) {
        app.rateIndex = static_cast<int>(event->key.key - SDLK_1);
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const auto frequency = static_cast<double>(SDL_GetPerformanceFrequency());
    const double sinceFrame = static_cast<double>(now - app.lastFrame) / frequency;

    const double interval = 1.0 / kRates[static_cast<std::size_t>(app.rateIndex)];
    if (sinceFrame + kSlack >= interval) {
        const double delta = std::min(sinceFrame, kMaxDelta);
        app.lastFrame = now;

        app.perFrame.move(kPixelsPerFrame);
        app.perSecond.move(kSpeed * static_cast<float>(delta));

        SDL_Renderer* r = app.renderer;
        gd::backdrop(r, kWidth, kHeight);
        char text[96];
        SDL_snprintf(text, sizeof text, "%d fps measured", app.fps);
        app.hud.titleBar(r, kWidth, "[ DELTA TIME ]", text);
        drawLane(app, 150.0F, app.perFrame, gd::kOrange, "x += 6", "moves per frame", app.perFrameSpeed);
        drawLane(app, 330.0F, app.perSecond, gd::kTeal, "x += 360 * dt", "moves per second", app.perSecondSpeed);

        app.hud.text(r, kMargin, 484.0F, "frame rate:", gd::kMuted);
        for (int i = 0; i < 4; ++i) {
            const SDL_FRect c = chip(i);
            const bool active = i == app.rateIndex;
            gd::card(r, c.x, c.y, c.w, c.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
            SDL_snprintf(text, sizeof text, "%d", kRates[static_cast<std::size_t>(i)]);
            app.hud.text(r, c.x + (c.w - app.hud.width(text, gd::Hud::Style::Bold)) / 2.0F, c.y + 6.0F, text, active ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
        }
        SDL_RenderPresent(r);

        ++app.framesThisSecond;
        ++app.frames;
    }

    if (static_cast<double>(now - app.lastSecond) / frequency >= 1.0) {
        app.fps = app.framesThisSecond;
        app.framesThisSecond = 0;
        app.perFrameSpeed = app.perFrame.travelled;
        app.perSecondSpeed = app.perSecond.travelled;
        app.perFrame.travelled = 0.0F;
        app.perSecond.travelled = 0.0F;
        app.lastSecond = now;
    }
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
