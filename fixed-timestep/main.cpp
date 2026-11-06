// Fixed timestep: why the drawn position is interpolated.
//
// Logic runs in fixed steps, here deliberately slow so the effect is visible. The upper square is drawn
// at the latest logic position and jumps from step to step. The lower square is drawn between the last two
// logic states (alpha = leftover time / step) and moves smoothly. Click (or keys 1 to 4) to change the
// logic rate. Key S simulates a stall of half a second: the frame time clamp keeps the loop from
// running hundreds of catch-up steps (the spiral of death).

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>

namespace {

constexpr int kWidth = 640;
constexpr int kHeight = 360;
constexpr float kSize = 40.0F;
constexpr float kSpeed = 240.0F;       // pixels per second
constexpr double kMaxFrameTime = 0.25; // clamp: avoids the spiral of death after a stall
constexpr std::array<int, 4> kRates{5, 10, 20, 60};

struct Body {
    float prev = 0.0F;
    float cur = 0.0F;
    float dir = 1.0F;

    void step(double dt) {
        prev = cur;
        cur += dir * kSpeed * static_cast<float>(dt);
        if (cur < 0.0F || cur > kWidth - kSize) {
            cur = std::clamp(cur, 0.0F, kWidth - kSize);
            dir = -dir;
        }
    }
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    Uint64 last = 0;
    double accumulator = 0.0;
    double stall = 0.0;
    int rateIndex = 1;
    Body body;
    long maxFrames = -1;
    long frames = 0;
    int stepsThisFrame = 0;
};

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
    SDL_SetRenderVSync(app->renderer, 1);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) {
        return SDL_APP_SUCCESS;
    }
    if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        app.rateIndex = (app.rateIndex + 1) % 4;
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

    const auto alpha = static_cast<float>(app.accumulator / stepTime);
    const float smooth = app.body.prev + (app.body.cur - app.body.prev) * alpha;

    SDL_SetRenderDrawColor(app.renderer, 255, 255, 255, 255);
    SDL_RenderClear(app.renderer);
    SDL_SetRenderDrawColor(app.renderer, 200, 70, 50, 255);
    const SDL_FRect upper{app.body.cur, 110.0F, kSize, kSize};
    SDL_RenderFillRect(app.renderer, &upper);
    SDL_SetRenderDrawColor(app.renderer, 30, 100, 200, 255);
    const SDL_FRect lower{smooth, 230.0F, kSize, kSize};
    SDL_RenderFillRect(app.renderer, &lower);

    SDL_SetRenderDrawColor(app.renderer, 40, 40, 40, 255);
    char text[96];
    SDL_snprintf(text, sizeof text, "logic %d steps/s   (click or 1-4, S = stall)", kRates[static_cast<std::size_t>(app.rateIndex)]);
    SDL_RenderDebugText(app.renderer, 8.0F, 8.0F, text);
    SDL_RenderDebugText(app.renderer, 8.0F, 92.0F, "drawn at the last logic state");
    SDL_RenderDebugText(app.renderer, 8.0F, 212.0F, "interpolated with alpha");
    SDL_snprintf(text, sizeof text, "alpha %.2f   steps this frame %d", static_cast<double>(alpha), app.stepsThisFrame);
    SDL_RenderDebugText(app.renderer, 8.0F, 330.0F, text);
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
