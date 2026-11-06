// FizzleFade with SDL3 main callbacks.
//
// The screen transition of Wolfenstein 3D, next to the version everybody writes first. Left: pick a random pixel,
// look at a flag, pick again if it was taken. The counter of wasted tries explodes towards the end, because almost
// every pick hits a finished pixel. Right: a 17-bit shift register cut into x and y, every pixel exactly once, no
// flags. Both panels get the same number of tries per second, so the chart below compares like with like. The
// register is not free either: it also visits states that fall outside the screen, which is why the random picker
// starts ahead and loses around the halfway mark. The logic lives in logic.hpp and is covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

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
constexpr float kPanelW = 400.0F;
constexpr float kPanelH = 240.0F;
constexpr float kPanelY = 64.0F;
constexpr std::array<float, 2> kPanelX{60.0F, 500.0F};
constexpr float kChartX = 60.0F;
constexpr float kChartY = 386.0F;
constexpr float kChartW = 840.0F;
constexpr float kChartH = 80.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr double kMaxDelta = 0.25;       // clamp after a stall (tab switch)
constexpr double kHoldSeconds = 2.5;     // both finished: show the result, then start over
constexpr uint64_t kSampleEvery = 256;   // chart resolution in tries
constexpr double kWarmStart = 0.8;       // first frame already shows the fade at 80 % of the register cycle

struct Resolution {
    int w;
    int h;
};
// Each one scales to the 400x240 panel by a whole number (4, 2, 1), so the pixels stay square.
constexpr std::array<Resolution, 3> kResolutions{{{100, 60}, {200, 120}, {400, 240}}};
// Seconds the register needs for one full cycle: slow, normal, fast.
constexpr std::array<double, 3> kCycleSeconds{6.0, 3.0, 1.0};

constexpr gd::Color kScene[4] = {{31, 41, 55}, {55, 65, 81}, {250, 204, 21}, {255, 255, 255}};

enum class Action { Size, Speed, Restart, Pause };

struct Chip {
    Action action;
    int value;
    std::string label;
    SDL_FRect rect;
};

struct Panel {
    std::vector<uint8_t> pixels;  // RGBA, rows of exactly w * 4 bytes
    SDL_Texture* texture = nullptr;
    bool dirty = true;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    int resIndex = 1;
    int speedIndex = 1;
    bool paused = false;
    int w = 200;
    int h = 120;

    Panel naivePanel;
    Panel fizzlePanel;
    ff::NaiveFade naive{200, 120, 1};
    ff::Fizzle fizzle{200, 120};
    std::vector<uint32_t> naiveSamples;   // covered pixels after every kSampleEvery-th try
    std::vector<uint32_t> fizzleSamples;
    double budget = 0.0;                  // fractional tries carried over between frames
    double hold = 0.0;
    uint64_t run = 0;

    Uint64 lastCounter = 0;
    long maxFrames = -1;
    long frames = 0;
};

uint64_t seedFor(uint64_t run) { return 0x9E3779B97F4A7C15ULL * (run + 1); }

void setPixel(Panel& p, int w, int x, int y, gd::Color c) {
    uint8_t* at = p.pixels.data() + (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4;
    at[0] = c.r;
    at[1] = c.g;
    at[2] = c.b;
    at[3] = 255;
    p.dirty = true;
}

gd::Color mix(gd::Color a, gd::Color b, double t) {
    const auto ch = [t](Uint8 from, Uint8 to) { return static_cast<Uint8>(std::lround(from + (to - from) * t)); };
    return {ch(a.r, b.r), ch(a.g, b.g), ch(a.b, b.b)};
}

/// The "old picture" that the fade covers: sky, sun and two hills, so every covered pixel is easy to see.
void paintScene(Panel& p, int w, int h) {
    p.pixels.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4, 255);
    const double aspect = static_cast<double>(w) / static_cast<double>(h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double u = (x + 0.5) / w;
            const double v = (y + 0.5) / h;
            gd::Color c = mix(gd::kTealSoft, kScene[3], v);
            const double dx = (u - 0.72) * aspect;
            const double dy = v - 0.28;
            if (dx * dx + dy * dy < 0.02) c = kScene[2];
            if (v > 0.66 + 0.06 * std::sin(u * 9.0 + 1.0) + 0.03 * std::sin(u * 23.0)) c = kScene[1];
            if (v > 0.8 + 0.05 * std::sin(u * 6.0) + 0.02 * std::sin(u * 31.0 + 2.0)) c = kScene[0];
            uint8_t* at = p.pixels.data() + (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4;
            at[0] = c.r;
            at[1] = c.g;
            at[2] = c.b;
        }
    }
    p.dirty = true;
}

void tryNaive(App& app) {
    const ff::Step s = app.naive.next();
    if (s.visible) setPixel(app.naivePanel, app.w, s.x, s.y, gd::kOrange);
    if (app.naive.tries() % kSampleEvery == 0) app.naiveSamples.push_back(app.naive.covered());
}

void tryFizzle(App& app) {
    const ff::Step s = app.fizzle.next();
    if (s.visible) setPixel(app.fizzlePanel, app.w, s.x, s.y, gd::kTeal);
    if (app.fizzle.steps() % kSampleEvery == 0) app.fizzleSamples.push_back(static_cast<uint32_t>(app.fizzle.emitted()));
}

void runTries(App& app, uint64_t n) {
    for (uint64_t i = 0; i < n && !app.naive.done(); ++i) tryNaive(app);
    for (uint64_t i = 0; i < n && !app.fizzle.done(); ++i) tryFizzle(app);
}

bool bothDone(const App& app) { return app.naive.done() && app.fizzle.done(); }

/// New run on the current size: fresh fades, fresh pictures, no leftover time or tries.
void reset(App& app, bool warmStart) {
    ++app.run;
    app.naive = ff::NaiveFade(app.w, app.h, seedFor(app.run));
    app.fizzle = ff::Fizzle(app.w, app.h);
    app.naiveSamples.clear();
    app.fizzleSamples.clear();
    paintScene(app.naivePanel, app.w, app.h);
    paintScene(app.fizzlePanel, app.w, app.h);
    app.budget = 0.0;
    app.hold = 0.0;
    if (warmStart) runTries(app, static_cast<uint64_t>(static_cast<double>(app.fizzle.length()) * kWarmStart));
}

SDL_Texture* makeTexture(SDL_Renderer* r, int w, int h) {
    SDL_Texture* t = SDL_CreateTexture(r, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, w, h);
    if (t == nullptr) {
        SDL_Log("texture %dx%d failed: %s", w, h, SDL_GetError());
        return nullptr;
    }
    SDL_SetTextureScaleMode(t, SDL_SCALEMODE_NEAREST);
    return t;
}

/// Switches to resolution `index`. The new textures exist before the old ones are destroyed, so a failure changes nothing.
bool setResolution(App& app, int index, bool warmStart) {
    const Resolution res = kResolutions[static_cast<std::size_t>(index)];
    if (!ff::Fizzle::supported(res.w, res.h)) {
        SDL_Log("resolution %dx%d is not supported by the register", res.w, res.h);
        return false;
    }
    SDL_Texture* a = makeTexture(app.renderer, res.w, res.h);
    SDL_Texture* b = makeTexture(app.renderer, res.w, res.h);
    if (a == nullptr || b == nullptr) {
        SDL_DestroyTexture(a);
        SDL_DestroyTexture(b);
        return false;
    }
    SDL_DestroyTexture(app.naivePanel.texture);
    SDL_DestroyTexture(app.fizzlePanel.texture);
    app.naivePanel.texture = a;
    app.fizzlePanel.texture = b;
    app.resIndex = index;
    app.w = res.w;
    app.h = res.h;
    reset(app, warmStart);
    return true;
}

void apply(App& app, Action action, int value) {
    switch (action) {
        case Action::Size:
            if (value != app.resIndex) setResolution(app, value, false);
            break;
        case Action::Speed:
            app.speedIndex = value;
            app.budget = 0.0;
            break;
        case Action::Restart: reset(app, false); break;
        case Action::Pause: app.paused = !app.paused; break;
    }
}

bool active(const App& app, const Chip& chip) {
    switch (chip.action) {
        case Action::Size: return app.resIndex == chip.value;
        case Action::Speed: return app.speedIndex == chip.value;
        case Action::Restart: return false;
        case Action::Pause: return app.paused;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        int value;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Action::Size, 0, "1 100x60", 0.0F},
                                   {Action::Size, 1, "2 200x120", 8.0F},
                                   {Action::Size, 2, "3 400x240", 8.0F},
                                   {Action::Speed, 0, "4 Slow", 20.0F},
                                   {Action::Speed, 1, "5 Normal", 8.0F},
                                   {Action::Speed, 2, "6 Fast", 8.0F},
                                   {Action::Restart, 0, "R Restart", 20.0F},
                                   {Action::Pause, 0, "P Pause", 8.0F}}};
    float x = 30.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float width = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.value, d.label, SDL_FRect{x, kChipY, width, kChipH}});
        x += width;
    }
}

void uploadIfDirty(Panel& p, int w) {
    if (!p.dirty || p.texture == nullptr) return;
    SDL_UpdateTexture(p.texture, nullptr, p.pixels.data(), w * 4);
    p.dirty = false;
}

void drawPanel(App& app, Panel& p, float x, const char* title, gd::Color accent, bool done) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, x, 38.0F, title, accent, gd::Hud::Style::Bold);
    gd::card(r, x, kPanelY, kPanelW, kPanelH, gd::kWhite, gd::kInk);
    uploadIfDirty(p, app.w);
    const SDL_FRect dst{x, kPanelY, kPanelW, kPanelH};
    SDL_RenderTexture(r, p.texture, nullptr, &dst);
    gd::strokeRect(r, x, kPanelY, kPanelW, kPanelH, gd::kInk);
    if (done) {
        gd::card(r, x + 10.0F, kPanelY + 10.0F, 64.0F, 28.0F, accent, gd::kInk);
        app.hud.text(r, x + 10.0F + 12.0F, kPanelY + 14.0F, "DONE", gd::kWhite, gd::Hud::Style::Bold);
    }
}

void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c) {
    gd::setColor(r, c);
    SDL_RenderLine(r, x0, y0, x1, y1);
    SDL_RenderLine(r, x0, y0 + 1.0F, x1, y1 + 1.0F);
}

void drawCurve(SDL_Renderer* r, const std::vector<uint32_t>& samples, uint64_t tries, uint32_t covered, uint32_t total, bool done, double xMax,
               gd::Color color) {
    const auto px = [xMax](double tr) { return kChartX + static_cast<float>(std::min(1.0, tr / xMax)) * kChartW; };
    const auto py = [total](double cov) {
        return kChartY + kChartH - 2.0F - static_cast<float>(std::min(1.0, cov / static_cast<double>(total))) * (kChartH - 4.0F);
    };
    float lx = px(0.0);
    float ly = py(0.0);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const float x = px(static_cast<double>((i + 1) * kSampleEvery));
        const float y = py(static_cast<double>(samples[i]));
        thickLine(r, lx, ly, x, y, color);
        lx = x;
        ly = y;
    }
    const float ex = px(static_cast<double>(tries));
    const float ey = py(static_cast<double>(covered));
    thickLine(r, lx, ly, ex, ey, color);
    if (done) thickLine(r, ex, ey, kChartX + kChartW, ey, color);
}

void drawChart(App& app) {
    SDL_Renderer* r = app.renderer;
    const auto total = static_cast<uint32_t>(app.w * app.h);
    gd::fillRect(r, kChartX, kChartY, kChartW, kChartH, gd::kWhite);
    gd::setColor(r, gd::kGrid);
    SDL_RenderLine(r, kChartX, kChartY + kChartH / 2.0F, kChartX + kChartW, kChartY + kChartH / 2.0F);
    const double xMax = static_cast<double>(std::max<uint64_t>(app.fizzle.length(), app.naive.tries()));
    // The orange curve goes last: it ends on the same 100 % line as the teal one and would be hidden under it.
    drawCurve(r, app.fizzleSamples, app.fizzle.steps(), static_cast<uint32_t>(app.fizzle.emitted()), total, app.fizzle.done(), xMax, gd::kTeal);
    drawCurve(r, app.naiveSamples, app.naive.tries(), app.naive.covered(), total, app.naive.done(), xMax, gd::kOrange);
    gd::strokeRect(r, kChartX, kChartY, kChartW, kChartH, gd::kInk);
    const char* label = "covered over tries";
    app.hud.text(r, kChartX + kChartW - app.hud.width(label) - 6.0F, kChartY + kChartH - 24.0F, label, gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "%s  |  full cycle in %d s", app.paused ? "paused" : "running", static_cast<int>(kCycleSeconds[static_cast<std::size_t>(app.speedIndex)]));
    app.hud.titleBar(r, kWidth, "[ FIZZLEFADE ]", right);

    drawPanel(app, app.naivePanel, kPanelX[0], "NAIVE: RANDOM PICK", gd::kOrange, app.naive.done());
    drawPanel(app, app.fizzlePanel, kPanelX[1], "LFSR: EACH PIXEL ONCE", gd::kTeal, app.fizzle.done());

    const auto total = static_cast<uint64_t>(app.w) * static_cast<uint64_t>(app.h);
    const gd::Hud::Style bold = gd::Hud::Style::Bold;
    const float lineY[3] = {312.0F, 334.0F, 356.0F};

    {
        const float x = kPanelX[0];
        const std::string l1 = "tries " + ff::groupDigits(app.naive.tries()) + "  covered " + ff::formatTenths(ff::percentTenths(app.naive.covered(), total)) + "%";
        const uint64_t percentWasted = app.naive.tries() == 0 ? 0 : app.naive.wasted() * 100 / app.naive.tries();
        const std::string l2 = "wasted " + ff::groupDigits(app.naive.wasted()) + " (" + std::to_string(percentWasted) + "%)";
        const std::string l3 = "state: " + ff::groupDigits(app.naive.flagCount()) + " 'filled' flags";
        app.hud.text(r, x, lineY[0], l1, gd::kInk, bold);
        app.hud.text(r, x, lineY[1], l2, gd::kOrange, bold);
        app.hud.text(r, x, lineY[2], l3, gd::kMuted);
    }
    {
        const float x = kPanelX[1];
        const std::string l1 = "steps " + ff::groupDigits(app.fizzle.steps()) + "  covered " + ff::formatTenths(ff::percentTenths(app.fizzle.emitted(), total)) + "%";
        const std::string l2 = "skipped " + ff::groupDigits(app.fizzle.skipped()) + " off screen";
        char mask[48];
        SDL_snprintf(mask, sizeof mask, "state: %d-bit register, mask 0x%X", app.fizzle.bits(), static_cast<unsigned>(app.fizzle.mask()));
        app.hud.text(r, x, lineY[0], l1, gd::kInk, bold);
        app.hud.text(r, x, lineY[1], l2, gd::kTealDark, bold);
        app.hud.text(r, x, lineY[2], mask, gd::kMuted);
    }

    drawChart(app);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const std::string label = c.action == Action::Pause && app.paused ? "P Play" : c.label;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
    }
}

void step(App& app, double delta) {
    if (app.paused) return;
    if (bothDone(app)) {
        app.hold += delta;
        if (app.hold >= kHoldSeconds) reset(app, false);
        return;
    }
    const double rate = static_cast<double>(app.fizzle.length()) / kCycleSeconds[static_cast<std::size_t>(app.speedIndex)];
    app.budget += delta * rate;
    const auto n = static_cast<uint64_t>(app.budget);
    app.budget -= static_cast<double>(n);
    runTries(app, n);
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
    if (!SDL_CreateWindowAndRenderer("fizzle-fade", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!setResolution(*app, app->resIndex, true)) return SDL_APP_FAILURE;
    layoutChips(*app);
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
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action, c.value);
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Size, 0); break;
                case SDLK_2: apply(app, Action::Size, 1); break;
                case SDLK_3: apply(app, Action::Size, 2); break;
                case SDLK_4: apply(app, Action::Speed, 0); break;
                case SDLK_5: apply(app, Action::Speed, 1); break;
                case SDLK_6: apply(app, Action::Speed, 2); break;
                case SDLK_R: apply(app, Action::Restart, 0); break;
                case SDLK_P:
                case SDLK_SPACE: apply(app, Action::Pause, 0); break;
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
    const double elapsed = static_cast<double>(now - app.lastCounter) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.lastCounter = now;
    step(app, std::min(elapsed, kMaxDelta));
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app == nullptr) return;
    app->hud.destroy();
    SDL_DestroyTexture(app->naivePanel.texture);
    SDL_DestroyTexture(app->fizzlePanel.texture);
    if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
    if (app->window != nullptr) SDL_DestroyWindow(app->window);
    delete app;
}
