// Delta time with SDL3 main callbacks.
//
// Two squares run the same lap. The upper one moves a fixed number of pixels per frame,
// the lower one moves speed * delta time. Click (or keys 1 to 4) to change how often
// frames are produced: the upper square follows the frame rate, the lower one does not.

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
constexpr float kSpeed = 240.0F;                  // pixels per second
constexpr float kPixelsPerFrame = kSpeed / 60.0F; // same speed, but only at exactly 60 frames per second
constexpr double kMaxDelta = 0.25;                // clamp after a stall (tab switch, breakpoint)
constexpr std::array<int, 4> kRates{15, 30, 60, 144};

struct Runner {
    float x = 0.0F;
    float dir = 1.0F;

    void move(float distance) {
        x += dir * distance;
        if (x < 0.0F || x > kWidth - kSize) {
            x = std::clamp(x, 0.0F, kWidth - kSize);
            dir = -dir;
        }
    }
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    Uint64 last = 0;
    double sinceFrame = 0.0;
    int rateIndex = 2;
    Runner perFrame;
    Runner perSecond;
    long maxFrames = -1;
    long frames = 0;
    double secondTimer = 0.0;
    int framesThisSecond = 0;
    int fps = 0;
};

void selectRate(App& app, int index) {
    app.rateIndex = ((index % 4) + 4) % 4;
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
    if (!SDL_CreateWindowAndRenderer("delta-time", kWidth, kHeight, 0, &app->window, &app->renderer)) {
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
        selectRate(app, app.rateIndex + 1);
    }
    if (event->type == SDL_EVENT_KEY_DOWN && event->key.key >= SDLK_1 && event->key.key <= SDLK_4) {
        selectRate(app, static_cast<int>(event->key.key - SDLK_1));
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const double elapsed = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    app.sinceFrame += elapsed;
    app.secondTimer += elapsed;

    const double interval = 1.0 / kRates[static_cast<std::size_t>(app.rateIndex)];
    if (app.sinceFrame >= interval) {
        const double delta = std::min(app.sinceFrame, kMaxDelta);
        app.sinceFrame = 0.0;

        app.perFrame.move(kPixelsPerFrame);
        app.perSecond.move(kSpeed * static_cast<float>(delta));

        SDL_SetRenderDrawColor(app.renderer, 255, 255, 255, 255);
        SDL_RenderClear(app.renderer);
        SDL_SetRenderDrawColor(app.renderer, 200, 70, 50, 255);
        const SDL_FRect upper{app.perFrame.x, 110.0F, kSize, kSize};
        SDL_RenderFillRect(app.renderer, &upper);
        SDL_SetRenderDrawColor(app.renderer, 30, 100, 200, 255);
        const SDL_FRect lower{app.perSecond.x, 230.0F, kSize, kSize};
        SDL_RenderFillRect(app.renderer, &lower);

        SDL_SetRenderDrawColor(app.renderer, 40, 40, 40, 255);
        char text[96];
        SDL_snprintf(text, sizeof text, "x += 4          (per frame)");
        SDL_RenderDebugText(app.renderer, 8.0F, 92.0F, text);
        SDL_snprintf(text, sizeof text, "x += 240 * dt   (per second)");
        SDL_RenderDebugText(app.renderer, 8.0F, 212.0F, text);
        SDL_snprintf(text, sizeof text, "target %d fps   measured %d fps   (click or 1-4)",
                     kRates[static_cast<std::size_t>(app.rateIndex)], app.fps);
        SDL_RenderDebugText(app.renderer, 8.0F, 8.0F, text);
        SDL_RenderPresent(app.renderer);

        ++app.framesThisSecond;
        ++app.frames;
    }

    if (app.secondTimer >= 1.0) {
        app.fps = app.framesThisSecond;
        app.framesThisSecond = 0;
        app.secondTimer -= 1.0;
    }
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
