// Fixed timestep update loop with SDL3 main callbacks.
//
// Rendering runs as fast as the display allows. Game logic runs in fixed steps of
// 1/60 s, fed from an accumulator. The drawn position is interpolated between the
// last two logic states, so motion stays smooth at any frame rate.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "hud.hpp"

namespace {

constexpr double kStep = 1.0 / 60.0;    // logic step in seconds
constexpr double kMaxFrameTime = 0.25;  // clamp: avoids the spiral of death after a stall
constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kSize = 64.0F;
constexpr float kLeft = 40.0F;
constexpr float kRight = kWidth - 40.0F - kSize;

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    Uint64 last = 0;
    double accumulator = 0.0;
    float prevX = kLeft;
    float x = kLeft;
    float speed = 360.0F;  // pixels per second
    long maxFrames = -1;   // --frames N: exit after N rendered frames (smoke test)
    long frames = 0;
    double secondTimer = 0.0;
    int updatesThisSecond = 0;
    int framesThisSecond = 0;
    int ups = 0;
    int fps = 0;
};

void update(App& app) {
    app.prevX = app.x;
    app.x += app.speed * static_cast<float>(kStep);
    if (app.x < kLeft || app.x > kRight) {
        app.x = std::clamp(app.x, kLeft, kRight);
        app.speed = -app.speed;
    }
}

void stat(App& app, float x, float y, const char* label, int value, gd::Color color) {
    gd::card(app.renderer, x, y, 240.0F, 78.0F, gd::kWhite, gd::kInk);
    gd::fillRect(app.renderer, x, y, 6.0F, 78.0F, color);
    char text[16];
    SDL_snprintf(text, sizeof text, "%d", value);
    app.hud.text(app.renderer, x + 22.0F, y + 8.0F, text, color, gd::Hud::Style::Large);
    app.hud.text(app.renderer, x + 22.0F, y + 52.0F, label, gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("update-loop", kWidth, kHeight, 0, &app->window, &app->renderer)) {
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

SDL_AppResult SDL_AppEvent(void*, SDL_Event* event) {
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) {
        return SDL_APP_SUCCESS;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    double frameTime = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    frameTime = std::min(frameTime, kMaxFrameTime);

    app.accumulator += frameTime;
    while (app.accumulator >= kStep) {
        update(app);
        app.accumulator -= kStep;
        ++app.updatesThisSecond;
    }

    app.secondTimer += frameTime;
    ++app.framesThisSecond;
    if (app.secondTimer >= 1.0) {
        app.ups = app.updatesThisSecond;
        app.fps = app.framesThisSecond;
        app.updatesThisSecond = 0;
        app.framesThisSecond = 0;
        app.secondTimer -= 1.0;
    }

    const float alpha = static_cast<float>(app.accumulator / kStep);
    const float drawX = app.prevX + (app.x - app.prevX) * alpha;

    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ UPDATE LOOP ]", "fixed timestep 1/60 s");
    stat(app, 40.0F, 56.0F, "updates per second", app.ups, gd::kTeal);
    stat(app, 300.0F, 56.0F, "frames per second", app.fps, gd::kOrange);

    gd::fillRect(r, kLeft, 300.0F, kRight - kLeft + kSize, 2.0F, gd::Color{17, 24, 39, 60});
    gd::card(r, drawX, 226.0F, kSize, kSize, gd::kTeal, gd::kInk);

    app.hud.text(r, 40.0F, 372.0F, "accumulator", gd::kInk, gd::Hud::Style::Bold);
    const float barW = kWidth - 80.0F;
    gd::fillRect(r, 40.0F, 400.0F, barW, 26.0F, gd::kWhite);
    gd::fillRect(r, 40.0F, 400.0F, barW * alpha, 26.0F, gd::kTeal);
    gd::strokeRect(r, 40.0F, 400.0F, barW, 26.0F, gd::kInk);
    app.hud.text(r, 40.0F, 436.0F, "leftover time inside one 1/60 s step, used as alpha for the interpolation", gd::kMuted);
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
