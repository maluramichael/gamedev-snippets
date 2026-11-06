// A tiny CPU ray tracer. One ray per pixel goes into a scene of three balls on a checkerboard, the result is written
// into a pixel buffer, uploaded to a streaming texture and drawn scaled up.
//
// Shadows (one extra ray per lit point) and reflections (a recursive ray, at most 3 bounces) can be switched off, and the
// resolution changed, so the stats on the right show what each of them costs. The mouse moves the light.
//
// The trace is spread over frames: a frame renders rows from the top until a time budget of 8 ms is used up and continues
// where it stopped in the next frame. Fast settings finish the image within one frame, slow ones (320x180 in a browser)
// sweep down the image and the stats show how far the pass got. A new pass only starts when the light or a setting changed.
//
// The math (rays, spheres, shading, shadow bias) lives in logic.hpp and is checked by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr SDL_FRect kView{30.0F, 50.0F, 640.0F, 360.0F};  // where the image is drawn; 640x360 is an integer multiple of every resolution
constexpr double kBudgetMs = 8.0;                         // trace time per frame, leaves room for drawing at 60 fps
constexpr double kFrameMs = 1000.0 / 60.0;

struct Resolution {
    int w;
    int h;
    const char* label;
};
constexpr std::array<Resolution, 3> kResolutions{{{80, 45, "80x45"}, {160, 90, "160x90"}, {320, 180, "320x180"}}};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture = nullptr;
    gd::Hud hud;

    rt::Scene scene = rt::demoScene();
    rt::Camera camera;
    rt::Options options;
    int resolution = 2;
    std::vector<std::uint32_t> pixels;

    bool dirty = true;       // something changed, a new pass is due
    bool rendering = false;  // a pass is in progress
    int row = 0;             // next row of the pass
    std::uint64_t passRays = 0;
    double passMs = 0.0;

    // numbers for the stats
    std::uint64_t imageRays = 0;
    double imageMs = 0.0;
    double frameMs = 0.0;
    double raysPerSecond = 0.0;
    int done = 0;  // percent of the current pass

    long maxFrames = -1;
    long frames = 0;
};

SDL_FRect resChip(int i) { return SDL_FRect{150.0F + static_cast<float>(i) * 108.0F, 424.0F, 100.0F, 34.0F}; }
SDL_FRect shadowChip() { return SDL_FRect{150.0F, 466.0F, 130.0F, 34.0F}; }
SDL_FRect reflectChip() { return SDL_FRect{288.0F, 466.0F, 170.0F, 34.0F}; }

// SDL's performance counter only ticks in whole milliseconds in the browser build, far too coarse for a frame that
// takes a couple of milliseconds. The browser clock itself has 0.1 ms steps.
#ifdef __EMSCRIPTEN__
double nowMs() { return emscripten_get_now(); }
#else
double nowMs() { return static_cast<double>(SDL_GetPerformanceCounter()) * 1000.0 / static_cast<double>(SDL_GetPerformanceFrequency()); }
#endif

std::string withCommas(std::uint64_t n) {
    std::string s = std::to_string(n);
    for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3) s.insert(static_cast<std::size_t>(i), ",");
    return s;
}

bool setResolution(App& app, int index) {
    const Resolution& res = kResolutions[static_cast<std::size_t>(index)];
    SDL_Texture* texture = SDL_CreateTexture(app.renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, res.w, res.h);
    if (texture == nullptr) {
        SDL_Log("texture failed: %s", SDL_GetError());
        return false;
    }
    SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_NEAREST);  // big square pixels are the point of the low resolutions
    SDL_DestroyTexture(app.texture);
    app.texture = texture;
    app.resolution = index;
    app.pixels.assign(static_cast<std::size_t>(res.w) * static_cast<std::size_t>(res.h), rt::pack(rt::kSkyHorizon));
    SDL_UpdateTexture(app.texture, nullptr, app.pixels.data(), res.w * 4);
    // The numbers of the old resolution do not describe this one: without the reset, rays/pixel would divide the old ray count
    // by the new pixel count until the first pass has finished.
    app.imageRays = 0;
    app.imageMs = 0.0;
    app.frameMs = 0.0;
    app.raysPerSecond = 0.0;
    app.done = 0;
    app.rendering = false;
    app.dirty = true;
    return true;
}

/// A shadow or reflection switch changes what a ray costs. A running pass would finish with mixed rows and a mixed ray count,
/// so it starts over.
void toggleOption(App& app, bool& option) {
    option = !option;
    app.rendering = false;
    app.dirty = true;
}

void setLightFromMouse(App& app, float x, float y) {
    if (x < kView.x || y < kView.y || x >= kView.x + kView.w || y >= kView.y + kView.h) return;
    app.scene.light = rt::lightFromMouse(app.camera, (x - kView.x) / kView.w, (y - kView.y) / kView.h, kView.w / kView.h);
    app.dirty = true;
}

/// One frame of tracing: rows from where the last frame stopped until the budget is used up.
void traceSome(App& app) {
    if (!app.rendering && app.dirty) {
        app.rendering = true;
        app.dirty = false;
        app.row = 0;
        app.passRays = 0;
        app.passMs = 0.0;
    }
    if (!app.rendering) return;

    const Resolution& res = kResolutions[static_cast<std::size_t>(app.resolution)];
    const double start = nowMs();
    std::uint64_t rays = 0;
    do {
        rt::renderRows(app.scene, app.camera, app.options, res.w, res.h, app.row, app.row + 1, app.pixels.data(), rays);
        ++app.row;
    } while (app.row < res.h && nowMs() - start < kBudgetMs);  // at least one row per frame, so a pass always ends
    app.frameMs = nowMs() - start;

    app.passRays += rays;
    app.passMs += app.frameMs;
    app.done = app.row * 100 / res.h;
    SDL_UpdateTexture(app.texture, nullptr, app.pixels.data(), res.w * 4);

    if (app.row >= res.h) {
        app.rendering = false;
        app.imageRays = app.passRays;
        app.imageMs = app.passMs;
        app.raysPerSecond = app.passMs > 0.0 ? static_cast<double>(app.passRays) / (app.passMs / 1000.0) : 0.0;
    }
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const Resolution& res = kResolutions[static_cast<std::size_t>(app.resolution)];
    const float x = 700.0F;
    gd::card(r, x, kView.y, 230.0F, kView.h, gd::kWhite, gd::kInk);
    app.hud.text(r, x + 12.0F, kView.y + 10.0F, "STATS", gd::kTealDark, gd::Hud::Style::Bold);

    const double pixelCount = static_cast<double>(res.w) * static_cast<double>(res.h);
    char value[40];
    float y = kView.y + 42.0F;
    const auto line = [&](const char* label, const std::string& text, gd::Color color) {
        app.hud.text(r, x + 12.0F, y, label, gd::kMuted);
        app.hud.text(r, x + 218.0F - app.hud.width(text, gd::Hud::Style::Bold), y, text, color, gd::Hud::Style::Bold);
        y += 28.0F;
    };
    line("resolution", res.label, gd::kInk);
    line("pixels", withCommas(static_cast<std::uint64_t>(pixelCount)), gd::kInk);
    line("rays/image", withCommas(app.imageRays), gd::kInk);
    SDL_snprintf(value, sizeof value, "%.2f", app.imageRays > 0 ? static_cast<double>(app.imageRays) / pixelCount : 0.0);
    line("rays/pixel", value, gd::kInk);
    if (app.raysPerSecond >= 1.0e6) {
        SDL_snprintf(value, sizeof value, "%.2f M", app.raysPerSecond / 1.0e6);
    } else {
        SDL_snprintf(value, sizeof value, "%.0f k", app.raysPerSecond / 1.0e3);
    }
    line("rays/s", value, gd::kTealDark);
    SDL_snprintf(value, sizeof value, "%.1f ms", app.frameMs);
    line("this frame", value, gd::kInk);
    const bool slow = app.imageMs > kFrameMs;
    SDL_snprintf(value, sizeof value, "%.1f ms", app.imageMs);
    line("whole image", value, slow ? gd::kOrange : gd::kTealDark);
    SDL_snprintf(value, sizeof value, "%d%%", app.rendering ? app.done : 100);
    line("done", value, app.rendering ? gd::kOrange : gd::kInk);

    // whole image vs the 16.7 ms a frame has at 60 fps
    app.hud.text(r, x + 12.0F, y + 2.0F, "vs 16.7 ms (60 fps)", gd::kMuted);
    const float barW = 206.0F;
    const float barY = y + 28.0F;
    gd::fillRect(r, x + 12.0F, barY, barW, 12.0F, gd::kGrid);
    const auto fill = static_cast<float>(std::min(app.imageMs / kFrameMs, 1.0));
    gd::fillRect(r, x + 12.0F, barY, barW * fill, 12.0F, slow ? gd::kOrange : gd::kTeal);
    gd::strokeRect(r, x + 12.0F, barY, barW, 12.0F, gd::kInk);
}

void drawChip(App& app, const SDL_FRect& c, const char* label, bool active) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, c.x, c.y, c.w, c.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
    app.hud.text(r, c.x + (c.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.y + 6.0F, label, active ? gd::kWhite : gd::kInk,
                 gd::Hud::Style::Bold);
}

void drawView(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kView.x, kView.y, kView.w, kView.h, gd::kWhite, gd::kInk);
    SDL_RenderTexture(r, app.texture, nullptr, &kView);

    const SDL_Rect clip{static_cast<int>(kView.x), static_cast<int>(kView.y), static_cast<int>(kView.w), static_cast<int>(kView.h)};
    SDL_SetRenderClipRect(r, &clip);
    if (app.rendering) {  // the sweep: everything below the line is still the previous pass
        const float y = kView.y + static_cast<float>(app.row) * kView.h / static_cast<float>(kResolutions[static_cast<std::size_t>(app.resolution)].h);
        gd::fillRect(r, kView.x, y, kView.w, 2.0F, gd::kOrange);
    }
    // the light: a glowing disc where it hangs, a thin line down to its spot on the floor
    float u = 0.0F;
    float v = 0.0F;
    const rt::Vec3 l = app.scene.light;
    float fu = 0.0F;
    float fv = 0.0F;
    if (rt::project(app.camera, l, u, v) && rt::project(app.camera, rt::Vec3{l.x, 0.0F, l.z}, fu, fv)) {
        const float lx = kView.x + u * kView.w;
        const float ly = kView.y + v * kView.h;
        const float fx = kView.x + fu * kView.w;
        const float fy = kView.y + fv * kView.h;
        gd::setColor(r, gd::Color{234, 88, 12, 140});
        SDL_RenderLine(r, lx, ly, fx, fy);
        gd::ring(r, fx, fy, 5.0F, gd::Color{234, 88, 12, 200});
        gd::disc(r, lx, ly, 16.0F, gd::Color{255, 237, 160, 200}, gd::Color{255, 237, 160, 0});
        gd::disc(r, lx, ly, 6.0F, gd::kOrange);
        gd::ring(r, lx, ly, 6.0F, gd::kInk);
    }
    SDL_SetRenderClipRect(r, nullptr);
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
    if (!SDL_CreateWindowAndRenderer("raytracing", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    app->scene.light = rt::lightFromMouse(app->camera, 0.33F, 0.74F, kView.w / kView.h);  // a start position with shadows to look at
    if (!setResolution(*app, app->resolution)) return SDL_APP_FAILURE;
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) {
        return SDL_APP_SUCCESS;
    }
    if (event->type == SDL_EVENT_MOUSE_MOTION) {
        setLightFromMouse(app, event->motion.x, event->motion.y);
    }
    if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        const SDL_FPoint p{event->button.x, event->button.y};
        for (int i = 0; i < 3; ++i) {
            const SDL_FRect c = resChip(i);
            if (SDL_PointInRectFloat(&p, &c) && i != app.resolution && !setResolution(app, i)) return SDL_APP_FAILURE;
        }
        const SDL_FRect shadows = shadowChip();
        const SDL_FRect reflections = reflectChip();
        if (SDL_PointInRectFloat(&p, &shadows)) {
            toggleOption(app, app.options.shadows);
        }
        if (SDL_PointInRectFloat(&p, &reflections)) {
            toggleOption(app, app.options.reflections);
        }
        setLightFromMouse(app, p.x, p.y);  // a tap on the image also moves the light (touch screens have no hover)
    }
    if (event->type == SDL_EVENT_KEY_DOWN) {
        if (event->key.repeat) return SDL_APP_CONTINUE;  // all keys here switch something, none is meant to repeat
        if (event->key.key >= SDLK_1 && event->key.key <= SDLK_3) {
            const int index = static_cast<int>(event->key.key - SDLK_1);
            if (index != app.resolution && !setResolution(app, index)) return SDL_APP_FAILURE;
        } else if (event->key.key == SDLK_S) {
            toggleOption(app, app.options.shadows);
        } else if (event->key.key == SDLK_R) {
            toggleOption(app, app.options.reflections);
        }
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    traceSome(app);

    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ RAYTRACING ]", "CPU, one thread");
    drawView(app);
    drawStats(app);

    app.hud.text(r, 30.0F, 432.0F, "resolution:", gd::kMuted);
    for (int i = 0; i < 3; ++i) drawChip(app, resChip(i), kResolutions[static_cast<std::size_t>(i)].label, i == app.resolution);
    app.hud.text(r, 30.0F, 474.0F, "features:", gd::kMuted);
    drawChip(app, shadowChip(), app.options.shadows ? "[x] shadows" : "[ ] shadows", app.options.shadows);
    drawChip(app, reflectChip(), app.options.reflections ? "[x] reflections" : "[ ] reflections", app.options.reflections);
    app.hud.text(r, 30.0F, 512.0F, "mouse: light    keys: 1 2 3 resolution    S shadows    R reflections", gd::kMuted);
    SDL_RenderPresent(r);

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
