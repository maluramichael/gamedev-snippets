// The fingerprint of a linear congruential generator, with SDL3 main callbacks.
//
// Left: every pair of consecutive numbers (x_n, x_n+1) as one dot. An LCG cannot hide that the next number is a linear
// function of the previous one, so the dots fall onto parallel lines (tiny LCG: right away; the real rand()s: zoom
// into the first percent or per mille of the x axis). Right: one bit of every number as a row of coins. The low bits of
// a power-of-two LCG repeat after a few draws, the bit row below the coins shows for every bit whether it does.
// The generators and the statistics live in logic.hpp and are covered by test.cpp.

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

constexpr float kPlotX = 40.0F;
constexpr float kPlotY = 46.0F;
constexpr float kPlotSize = 340.0F;
constexpr float kRightX = 420.0F;
constexpr float kCoinY = 70.0F;
constexpr int kCoinCols = 43;
constexpr int kCoinRows = 11;
constexpr float kCoinStride = 12.0F;
constexpr float kBitsY = 238.0F;
constexpr float kBitStride = 16.0F;
constexpr float kCardY = 294.0F;
constexpr float kChipH = 34.0F;
constexpr float kChipRowA = 420.0F;
constexpr float kChipRowB = 464.0F;

constexpr std::size_t kStatDraws = 100000;     // draws behind the statistics
constexpr std::size_t kPeriodWindow = 4096;    // draws used to look for a bit period
constexpr std::size_t kMaxPeriod = 1024;       // longest bit period that is searched for
constexpr std::size_t kMaxPoints = 4096;       // dots in the plot
constexpr std::size_t kPointBudget = 3000000;  // draws spent looking for them
constexpr std::array<int, 5> kZooms{1, 100, 1000, 10000, 100000};
constexpr double kRevealPerSecond = 2500.0;

enum class Action { Generator, Zoom, NewSeed };

struct Chip {
    Action action;
    int arg;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    lcg::Kind kind = lcg::Kind::Tiny;
    std::uint32_t seedNumber = 0;
    int zoom = 0;  // index into kZooms
    int bit = 0;
    std::vector<std::uint32_t> outs;
    std::array<std::size_t, 32> periods{};
    std::vector<lcg::Point> points;
    double revealed = 0.0;
    double same = 0.0;
    std::size_t run = 0;
    Uint64 lastNs = 0;
    long maxFrames = -1;
    long frames = 0;
};

const char* kindName(lcg::Kind kind) {
    switch (kind) {
        case lcg::Kind::Tiny: return "Tiny";
        case lcg::Kind::Diy32: return "DIY 32";
        case lcg::Kind::Ansi: return "ANSI";
        case lcg::Kind::Msvc: return "MSVC";
        case lcg::Kind::ParkMiller: return "Park-Miller";
        case lcg::Kind::Xorshift32: return "xorshift";
        case lcg::Kind::Pcg32: return "PCG32";
        case lcg::Kind::Wolf: return "Wolf3D";
    }
    return "";
}

const char* kindFormula(lcg::Kind kind) {
    switch (kind) {
        case lcg::Kind::Tiny: return "x = 65x + 1 mod 4096";
        case lcg::Kind::Diy32: return "x = 1664525x + 1013904223 mod 2^32";
        case lcg::Kind::Ansi: return "x = 1103515245x + 12345, out = x >> 16";
        case lcg::Kind::Msvc: return "x = 214013x + 2531011 mod 2^31, out = x >> 16";
        case lcg::Kind::ParkMiller: return "x = 16807x mod (2^31 - 1)";
        case lcg::Kind::Xorshift32: return "x ^= x << 13; x ^= x >> 17; x ^= x << 5";
        case lcg::Kind::Pcg32: return "PCG32, 64 bit state, xorshift + rotate";
        case lcg::Kind::Wolf: return "table of 256 bytes, index + 1";
    }
    return "";
}

/// Orange marks the generators with a visible weakness in this demo, teal the ones without.
bool weak(lcg::Kind kind) { return kind != lcg::Kind::Xorshift32 && kind != lcg::Kind::Pcg32; }

double xMax(const App& app) { return 1.0 / static_cast<double>(kZooms[static_cast<std::size_t>(app.zoom)]); }

void rebuildStats(App& app) {
    const auto seq = lcg::bitSequence(app.outs, app.bit);
    app.same = lcg::sameAsPrevious(seq);
    app.run = lcg::longestRun(seq);
}

/// Everything that depends on the generator, the seed or the zoom.
void rebuild(App& app) {
    const lcg::Rng rng = lcg::make(app.kind, lcg::seedFor(app.seedNumber));
    app.outs = lcg::draw(rng, kStatDraws);
    const std::vector<std::uint32_t> head(app.outs.begin(), app.outs.begin() + static_cast<std::ptrdiff_t>(kPeriodWindow));
    app.periods = lcg::bitPeriods(head, lcg::outBits(app.kind), kMaxPeriod);
    // A narrow strip is cheap to reach for generators whose output is the whole state, the others are scanned.
    if (app.zoom > 0 && lcg::outputIsState(app.kind)) {
        app.points = lcg::statePairs(app.kind, xMax(app), kMaxPoints, lcg::seedFor(app.seedNumber));
    } else {
        app.points = lcg::collectPairs(rng, xMax(app), kMaxPoints, kPointBudget);
    }
    app.bit = std::clamp(app.bit, 0, lcg::outBits(app.kind) - 1);
    app.revealed = 600.0;  // something to see on the first frame, the rest fills in
    rebuildStats(app);
}

void setBit(App& app, int bit) {
    app.bit = std::clamp(bit, 0, lcg::outBits(app.kind) - 1);
    rebuildStats(app);
}

void layoutChips(App& app) {
    float x = 40.0F;
    for (int i = 0; i < lcg::kKindCount; ++i) {
        const std::string label = std::to_string(i + 1) + " " + kindName(static_cast<lcg::Kind>(i));
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({Action::Generator, i, label, SDL_FRect{x, kChipRowA, w, kChipH}});
        x += w + 6.0F;
    }
    x = 40.0F + app.hud.width("ZOOM", gd::Hud::Style::Bold) + 16.0F;
    for (std::size_t i = 0; i < kZooms.size(); ++i) {
        const std::string label = (i == 0 ? std::string("Z ") : std::string()) + std::to_string(kZooms[i]) + "×";
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({Action::Zoom, static_cast<int>(i), label, SDL_FRect{x, kChipRowB, w, kChipH}});
        x += w + 6.0F;
    }
    x += 40.0F;
    const std::string label = "R New seed";
    app.chips.push_back({Action::NewSeed, 0, label, SDL_FRect{x, kChipRowB, app.hud.width(label, gd::Hud::Style::Bold) + 24.0F, kChipH}});
}

void apply(App& app, Action action, int arg) {
    switch (action) {
        case Action::Generator:
            app.kind = static_cast<lcg::Kind>(arg);
            rebuild(app);
            break;
        case Action::Zoom:
            app.zoom = arg;
            rebuild(app);
            break;
        case Action::NewSeed:
            ++app.seedNumber;
            rebuild(app);
            break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Generator: return static_cast<int>(app.kind) == c.arg;
        case Action::Zoom: return app.zoom == c.arg;
        case Action::NewSeed: return false;
    }
    return false;
}

SDL_FRect bitCell(int k) { return SDL_FRect{kRightX + static_cast<float>(k) * kBitStride, kBitsY, 14.0F, 24.0F}; }

void drawPlot(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPlotX - 6.0F, kPlotY - 6.0F, kPlotSize + 12.0F, kPlotSize + 12.0F, gd::kWhite, gd::kInk);
    for (int i = 1; i < 10; ++i) {
        const float o = kPlotSize * static_cast<float>(i) / 10.0F;
        gd::fillRect(r, kPlotX + o, kPlotY, 1.0F, kPlotSize, gd::kGrid);
        gd::fillRect(r, kPlotX, kPlotY + o, kPlotSize, 1.0F, gd::kGrid);
    }
    const double xm = xMax(app);
    const gd::Color dot = weak(app.kind) ? gd::Color{234, 88, 12, 210} : gd::Color{13, 148, 136, 210};
    const std::size_t shown = std::min(app.points.size(), static_cast<std::size_t>(app.revealed));
    for (std::size_t i = 0; i < shown; ++i) {
        const lcg::Point& p = app.points[i];
        const float px = kPlotX + static_cast<float>(p.x / xm) * kPlotSize;
        const float py = kPlotY + static_cast<float>(1.0 - p.y) * kPlotSize;
        gd::fillRect(app.renderer, px - 1.0F, py - 1.0F, 2.0F, 2.0F, dot);
    }
    char line[64];
    const int z = kZooms[static_cast<std::size_t>(app.zoom)];
    if (z == 1) {
        SDL_snprintf(line, sizeof line, "%d pairs, x(n) in 0..1", static_cast<int>(app.points.size()));
    } else {
        SDL_snprintf(line, sizeof line, "%d pairs, x(n) in 0..1/%d", static_cast<int>(app.points.size()), z);
    }
    app.hud.text(r, kPlotX - 6.0F, kPlotY + kPlotSize + 12.0F, line, gd::kMuted);
}

void drawCoins(App& app) {
    SDL_Renderer* r = app.renderer;
    char line[64];
    SDL_snprintf(line, sizeof line, "COIN FLIPS: bit %d of every number", app.bit);
    app.hud.text(r, kRightX, kPlotY - 6.0F, line, gd::kInk, gd::Hud::Style::Bold);
    const std::size_t cells = static_cast<std::size_t>(kCoinCols * kCoinRows);
    for (std::size_t i = 0; i < cells && i < app.outs.size(); ++i) {
        const float cx = kRightX + static_cast<float>(i % kCoinCols) * kCoinStride;
        const float cy = kCoinY + static_cast<float>(i / kCoinCols) * kCoinStride;
        const bool heads = lcg::bitOf(app.outs[i], app.bit) != 0;
        gd::fillRect(r, cx, cy, 11.0F, 11.0F, heads ? gd::kTeal : gd::kTealSoft);
    }
}

void drawBits(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kRightX, kBitsY - 26.0F, "PICK A BIT  (orange: repeats within 1024)", gd::kMuted);
    const int bits = lcg::outBits(app.kind);
    for (int k = 0; k < bits; ++k) {
        const SDL_FRect c = bitCell(k);
        const bool repeats = app.periods[static_cast<std::size_t>(k)] != 0;
        gd::fillRect(r, c.x, c.y, c.w, c.h, repeats ? gd::kOrange : gd::kTeal);
        if (k == app.bit) {
            gd::strokeRect(r, c.x - 2.0F, c.y - 2.0F, c.w + 4.0F, c.h + 4.0F, gd::kInk);
            gd::strokeRect(r, c.x - 1.0F, c.y - 1.0F, c.w + 2.0F, c.h + 2.0F, gd::kInk);
        }
    }
    app.hud.text(r, kRightX, kBitsY + 28.0F, "bit 0", gd::kMuted);
    char top[16];
    SDL_snprintf(top, sizeof top, "bit %d", bits - 1);
    app.hud.text(r, kRightX + static_cast<float>(bits) * kBitStride - 2.0F - app.hud.width(top), kBitsY + 28.0F, top, gd::kMuted);
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kRightX, kCardY, 516.0F, 112.0F, gd::kWhite, gd::kInk);
    const float x = kRightX + 12.0F;
    float y = kCardY + 6.0F;
    char line[96];
    if (app.bit == 0) {
        SDL_snprintf(line, sizeof line, "bit 0 = x %% 2");
    } else {
        SDL_snprintf(line, sizeof line, "bit %d = (x >> %d) & 1", app.bit, app.bit);
    }
    app.hud.text(r, x, y, line, gd::kInk, gd::Hud::Style::Bold);
    y += 24.0F;

    const gd::Color sameColor = std::fabs(app.same - 0.5) > 0.02 ? gd::kOrange : gd::kTeal;
    const std::string label = "same as previous  ";
    app.hud.text(r, x, y, label, gd::kInk);
    SDL_snprintf(line, sizeof line, "%.1f%%", app.same * 100.0);
    const float vx = x + app.hud.width(label);
    app.hud.text(r, vx, y, line, sameColor, gd::Hud::Style::Bold);
    app.hud.text(r, vx + app.hud.width(line, gd::Hud::Style::Bold) + 10.0F, y, "(fair: 50%)", gd::kMuted);
    y += 24.0F;

    const std::size_t period = app.periods[static_cast<std::size_t>(app.bit)];
    if (period != 0) {
        SDL_snprintf(line, sizeof line, "repeats every %d flips", static_cast<int>(period));
        app.hud.text(r, x, y, line, gd::kOrange, gd::Hud::Style::Bold);
    } else {
        SDL_snprintf(line, sizeof line, "no repeat within %d flips", static_cast<int>(kMaxPeriod));
        app.hud.text(r, x, y, line, gd::kTeal, gd::Hud::Style::Bold);
    }
    y += 24.0F;

    const gd::Color runColor = (app.run < 10 || app.run > 30) ? gd::kOrange : gd::kTeal;
    const std::string runLabel = "longest run  ";
    app.hud.text(r, x, y, runLabel, gd::kInk);
    SDL_snprintf(line, sizeof line, "%d", static_cast<int>(app.run));
    const float rx = x + app.hud.width(runLabel);
    app.hud.text(r, rx, y, line, runColor, gd::Hud::Style::Bold);
    app.hud.text(r, rx + app.hud.width(line, gd::Hud::Style::Bold) + 10.0F, y, "(fair: ~17 in 100000)", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ LCG LATTICE ]", kindFormula(app.kind));
    drawPlot(app);
    drawCoins(app);
    drawBits(app);
    drawStats(app);

    app.hud.text(r, 40.0F, kChipRowB + 6.0F, "ZOOM", gd::kMuted, gd::Hud::Style::Bold);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label,
                     on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, 40.0F, 508.0F, "1-8 generator   left/right: bit   Z: zoom   R: new seed   click a bit to pick it", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("lcg-lattice", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    rebuild(*app);
    app->lastNs = SDL_GetTicksNS();
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
                    apply(app, c.action, c.arg);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int k = 0; k < lcg::outBits(app.kind); ++k) {
                const SDL_FRect cell = bitCell(k);
                const SDL_FRect hit{cell.x - 1.0F, cell.y, cell.w + 2.0F, cell.h};  // no dead gap between the cells
                if (SDL_PointInRectFloat(&p, &hit)) {
                    setBit(app, k);
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            // Scrubbing through the bits may repeat, everything else is a toggle or a restart and must not.
            if (event->key.key == SDLK_LEFT) setBit(app, app.bit - 1);
            if (event->key.key == SDLK_RIGHT) setBit(app, app.bit + 1);
            if (event->key.repeat) break;
            if (event->key.key >= SDLK_1 && event->key.key < SDLK_1 + lcg::kKindCount) {
                apply(app, Action::Generator, static_cast<int>(event->key.key - SDLK_1));
            } else if (event->key.key == SDLK_Z) {
                apply(app, Action::Zoom, (app.zoom + 1) % static_cast<int>(kZooms.size()));
            } else if (event->key.key == SDLK_R) {
                apply(app, Action::NewSeed, 0);
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(static_cast<double>(now - app.lastNs) / 1e9, 0.25);
    app.lastNs = now;
    app.revealed += kRevealPerSecond * dt;
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
