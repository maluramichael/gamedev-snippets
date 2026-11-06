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

namespace {

constexpr double kStep = 1.0 / 60.0;      // logic step in seconds
constexpr double kMaxFrameTime = 0.25;    // clamp: avoids the spiral of death after a stall
constexpr int kWidth = 640;
constexpr int kHeight = 360;
constexpr float kSize = 40.0F;

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    Uint64 last = 0;
    double accumulator = 0.0;
    float prevX = 0.0F;
    float x = 0.0F;
    float speed = 240.0F;  // pixels per second
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
    if (app.x < 0.0F || app.x > kWidth - kSize) {
        app.x = std::clamp(app.x, 0.0F, kWidth - kSize);
        app.speed = -app.speed;
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
    if (!SDL_CreateWindowAndRenderer("update-loop", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
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

    SDL_SetRenderDrawColor(app.renderer, 255, 255, 255, 255);
    SDL_RenderClear(app.renderer);
    SDL_SetRenderDrawColor(app.renderer, 30, 100, 200, 255);
    const SDL_FRect box{drawX, (kHeight - kSize) / 2.0F, kSize, kSize};
    SDL_RenderFillRect(app.renderer, &box);
    SDL_SetRenderDrawColor(app.renderer, 40, 40, 40, 255);
    char text[64];
    SDL_snprintf(text, sizeof text, "updates/s %d   frames/s %d", app.ups, app.fps);
    SDL_RenderDebugText(app.renderer, 8.0F, 8.0F, text);
    SDL_RenderPresent(app.renderer);

    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
