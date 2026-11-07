// Storyteller timelines with SDL3 main callbacks.
//
// Three RimWorld storytellers run on the same 200 days. Cassandra and Phoebe follow a calendar of "on" windows (shaded) and
// drop their major threats inside them. Randy has no calendar: a weighted coin every 1000 ticks, and a threat is forced when
// 13 days passed without one (hollow marker). The timeline is one colony. The histogram next to the numbers is pooled over
// 100 long runs, so the shape of the gaps is visible and not just the luck of one colony. At the start everything is already
// drawn, R rolls a new colony and replays it. The rules, the numbers and their limits live in logic.hpp and are covered
// by test.cpp.

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
constexpr double kHorizon = 200.0;
constexpr double kPlaySpeed = 50.0;  // days per second while the timeline replays
constexpr std::uint64_t kFirstSeed = 3;

// Pool for the histogram and the yearly rates: long runs, so the start and the cut-off do not bend the numbers.
constexpr int kPoolRuns = 100;
constexpr double kPoolDays = 1200.0;
constexpr std::uint64_t kPoolSeed = 1000;
constexpr int kBins = 30;
constexpr double kBinDays = 1.0;

// Timeline.
constexpr float kAxisX0 = 150.0F;
constexpr float kAxisX1 = 936.0F;
constexpr float kPxPerDay = (kAxisX1 - kAxisX0) / static_cast<float>(kHorizon);
constexpr float kTimeTop = 58.0F;
constexpr float kTimeBottom = 230.0F;

// Histogram card.
constexpr float kCardY = 262.0F;
constexpr float kCardH = 178.0F;
constexpr float kHistX = 372.0F;
constexpr float kBinPx = 18.0F;
constexpr float kHistTop = 292.0F;
constexpr float kHistBase = 400.0F;

constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;

constexpr gd::Color kSlate{71, 85, 105};

constexpr std::array<stl::Teller, 3> kTellers{stl::Teller::Cassandra, stl::Teller::Phoebe, stl::Teller::Randy};
constexpr std::array<const char*, 3> kNames{"Cassandra", "Phoebe", "Randy"};
constexpr std::array<const char*, 3> kSubs{"on 4.6 off 6", "on 8 off 8", "random coin"};
constexpr std::array<gd::Color, 3> kColors{gd::kTeal, kSlate, gd::kOrange};

enum class Show { Cassandra, Phoebe, Randy, All };
enum class Action { ShowCassandra, ShowPhoebe, ShowRandy, ShowAll, NewColony, Replay };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Colony {
    std::array<std::vector<stl::Event>, 3> events;
    std::array<std::vector<double>, 3> threats;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    Show show = Show::All;
    std::uint64_t seed = kFirstSeed;
    Colony colony;
    std::array<stl::Pool, 3> pools;
    double revealed = kHorizon;  // days of the timeline drawn so far
    bool playing = false;
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

gd::Color withAlpha(gd::Color c, Uint8 a) { return gd::Color{c.r, c.g, c.b, a}; }

float dayX(double day) { return kAxisX0 + static_cast<float>(day) * kPxPerDay; }

bool shown(const App& app, int teller) {
    switch (app.show) {
        case Show::Cassandra: return teller == 0;
        case Show::Phoebe: return teller == 1;
        case Show::Randy: return teller == 2;
        case Show::All: return true;
    }
    return true;
}

int shownCount(const App& app) { return app.show == Show::All ? 3 : 1; }

void rollColony(App& app) {
    for (std::size_t i = 0; i < kTellers.size(); ++i) {
        app.colony.events[i] = stl::simulate(kTellers[i], kHorizon, app.seed);
        app.colony.threats[i] = stl::threatDays(app.colony.events[i]);
    }
}

void replay(App& app) {
    app.revealed = 0.0;
    app.playing = true;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ShowCassandra: app.show = Show::Cassandra; break;
        case Action::ShowPhoebe: app.show = Show::Phoebe; break;
        case Action::ShowRandy: app.show = Show::Randy; break;
        case Action::ShowAll: app.show = Show::All; break;
        case Action::NewColony:
            ++app.seed;
            rollColony(app);
            replay(app);
            break;
        case Action::Replay: replay(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ShowCassandra: return app.show == Show::Cassandra;
        case Action::ShowPhoebe: return app.show == Show::Phoebe;
        case Action::ShowRandy: return app.show == Show::Randy;
        case Action::ShowAll: return app.show == Show::All;
        default: return false;
    }
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 6> defs{{{Action::ShowCassandra, "1 Cassandra", 0.0F},
                                   {Action::ShowPhoebe, "2 Phoebe", 8.0F},
                                   {Action::ShowRandy, "3 Randy", 8.0F},
                                   {Action::ShowAll, "4 All three", 8.0F},
                                   {Action::NewColony, "R New colony", 24.0F},
                                   {Action::Replay, "P Replay", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void centerText(App& app, float cx, float y, const char* s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, cx - app.hud.width(s, style) / 2.0F, y, s, c, style);
}

void drawTimeline(App& app) {
    SDL_Renderer* r = app.renderer;
    const int lanes = shownCount(app);
    const float laneH = (kTimeBottom - kTimeTop) / static_cast<float>(lanes);

    // Year labels and dividers (a RimWorld year is 60 days).
    for (int y = 0; y * stl::kYearDays < kHorizon; ++y) {
        const double from = y * stl::kYearDays;
        const double to = std::min(from + stl::kYearDays, kHorizon);
        char label[24];
        SDL_snprintf(label, sizeof label, "year %d", y + 1);
        if (to - from < 30.0) SDL_snprintf(label, sizeof label, "y%d", y + 1);
        centerText(app, (dayX(from) + dayX(to)) / 2.0F, 38.0F, label, gd::kMuted);
        if (y > 0) {
            for (float py = 36.0F; py < kTimeBottom + 18.0F; py += 8.0F) gd::fillRect(r, dayX(from), py, 1.0F, 4.0F, gd::kMuted);
        }
    }

    int lane = 0;
    for (int t = 0; t < 3; ++t) {
        if (!shown(app, t)) continue;
        const float top = kTimeTop + static_cast<float>(lane) * laneH;
        ++lane;
        const gd::Color color = kColors[static_cast<std::size_t>(t)];
        gd::fillRect(r, kAxisX0, top + 2.0F, kAxisX1 - kAxisX0, laneH - 4.0F, withAlpha(gd::kWhite, 200));
        gd::strokeRect(r, kAxisX0, top + 2.0F, kAxisX1 - kAxisX0, laneH - 4.0F, gd::kGrid);

        // The calendar of the two scheduled storytellers.
        if (kTellers[static_cast<std::size_t>(t)] != stl::Teller::Randy) {
            const stl::Cycle& cycle = kTellers[static_cast<std::size_t>(t)] == stl::Teller::Cassandra ? stl::kCassandra : stl::kPhoebe;
            for (const stl::Window& w : stl::windows(cycle, kHorizon)) {
                gd::fillRect(r, dayX(w.from), top + 3.0F, dayX(w.to) - dayX(w.from), laneH - 6.0F, withAlpha(color, 46));
            }
        }

        // Name and rule.
        const float labelY = top + laneH / 2.0F - 20.0F;
        app.hud.text(r, 24.0F, labelY, kNames[static_cast<std::size_t>(t)], color, gd::Hud::Style::Bold);
        app.hud.text(r, 24.0F, labelY + 22.0F, kSubs[static_cast<std::size_t>(t)], gd::kMuted);

        const float mid = top + laneH * (lanes == 1 ? 0.62F : 0.66F);
        const float radius = lanes == 1 ? 9.0F : 6.0F;

        // Randy's other events: small ticks, to show he fires all the time and mostly something harmless.
        if (kTellers[static_cast<std::size_t>(t)] == stl::Teller::Randy) {
            for (const stl::Event& e : app.colony.events[static_cast<std::size_t>(t)]) {
                if (e.category == stl::Category::ThreatBig || e.day > app.revealed) continue;
                gd::fillRect(r, dayX(e.day), top + laneH - 12.0F, 1.0F, 8.0F, withAlpha(gd::kMuted, 140));
            }
        }

        // Counts per year (only what is revealed so far).
        for (int y = 0; y * stl::kYearDays < kHorizon; ++y) {
            const double from = y * stl::kYearDays;
            const double to = std::min(from + stl::kYearDays, kHorizon);
            if (from > app.revealed) continue;  // a year the playhead has not reached yet shows no count
            const int n = stl::countIn(app.colony.threats[static_cast<std::size_t>(t)], from, std::min(to, app.revealed + 1e-9));
            char label[32];
            if (to - from >= 100.0) {
                SDL_snprintf(label, sizeof label, "%d attacks", n);
            } else {
                SDL_snprintf(label, sizeof label, "%d", n);
            }
            app.hud.text(r, dayX(from) + 6.0F, top + 4.0F, label, color, gd::Hud::Style::Bold);
        }

        // The major threats.
        for (const stl::Event& e : app.colony.events[static_cast<std::size_t>(t)]) {
            if (e.category != stl::Category::ThreatBig || e.day > app.revealed) continue;
            const float x = dayX(e.day);
            if (e.forced) {
                gd::disc(r, x, mid, radius, gd::kWhite);
                gd::ring(r, x, mid, radius, color);
                gd::ring(r, x, mid, radius - 1.0F, color);
                gd::ring(r, x, mid, radius - 2.0F, color);
            } else {
                gd::disc(r, x, mid, radius, color);
                gd::ring(r, x, mid, radius, gd::kInk);
            }
        }
    }

    // Day axis.
    for (int d = 0; d <= 200; d += 50) {
        const float x = dayX(d);
        gd::fillRect(r, x, kTimeBottom + 1.0F, 1.0F, 5.0F, gd::kInk);
        char label[8];
        SDL_snprintf(label, sizeof label, "%d", d);
        const float w = app.hud.width(label);
        app.hud.text(r, std::clamp(x - w / 2.0F, 2.0F, static_cast<float>(kWidth) - w - 2.0F), kTimeBottom + 8.0F, label, gd::kMuted);
    }
    app.hud.text(r, 24.0F, kTimeBottom + 8.0F, "day", gd::kMuted);

    // Playhead while it replays.
    if (app.revealed < kHorizon) gd::fillRect(r, dayX(app.revealed), 36.0F, 2.0F, kTimeBottom - 34.0F, gd::kTealDark);
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 24.0F, kCardY, 320.0F, kCardH, gd::kWhite, gd::kInk);
    char head[64];
    SDL_snprintf(head, sizeof head, "%d colonies × %d days", kPoolRuns, static_cast<int>(kPoolDays));
    app.hud.text(r, 38.0F, kCardY + 8.0F, head, gd::kMuted);
    for (int t = 0; t < 3; ++t) {
        const auto i = static_cast<std::size_t>(t);
        const float y = kCardY + 34.0F + static_cast<float>(t) * 46.0F;
        const Uint8 a = shown(app, t) ? 255 : 90;
        const gd::Color color = withAlpha(kColors[i], a);
        const gd::Color ink = withAlpha(gd::kInk, a);
        gd::fillRect(r, 38.0F, y + 4.0F, 12.0F, 12.0F, color);
        app.hud.text(r, 58.0F, y, kNames[i], color, gd::Hud::Style::Bold);
        char perYear[24];
        SDL_snprintf(perYear, sizeof perYear, "%.1f/yr", app.pools[i].perYear);
        app.hud.text(r, 160.0F, y, perYear, ink, gd::Hud::Style::Bold);
        char gap[24];
        SDL_snprintf(gap, sizeof gap, "gap %.1f d", app.pools[i].meanGap);
        app.hud.text(r, 232.0F, y, gap, ink);
        const int n = stl::countIn(app.colony.threats[i], 0.0, app.revealed + 1e-9);
        char run[48];
        if (app.revealed >= kHorizon) {
            SDL_snprintf(run, sizeof run, "this colony: %d (%.1f/yr)", n, stl::perYear(n, kHorizon));
        } else {
            SDL_snprintf(run, sizeof run, "this colony: %d so far", n);
        }
        app.hud.text(r, 58.0F, y + 22.0F, run, withAlpha(gd::kMuted, a));
    }
}

void drawHistogram(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 356.0F, kCardY, 580.0F, kCardH, gd::kWhite, gd::kInk);
    app.hud.text(r, 372.0F, kCardY + 8.0F, "days between major attacks, % of gaps", gd::kMuted);
    app.hud.text(r, 790.0F, kCardY + 8.0F, "tick = mean", gd::kMuted);

    double peak = 0.0;
    for (int t = 0; t < 3; ++t) {
        const auto i = static_cast<std::size_t>(t);
        if (!shown(app, t) || app.pools[i].gapCount <= 0) continue;
        for (const int c : app.pools[i].bins) peak = std::max(peak, static_cast<double>(c) / app.pools[i].gapCount);
    }
    if (peak <= 0.0) peak = 1.0;
    // Scale in steps of 5 percent, the top gridline at the first step above the highest bar.
    constexpr double kStep = 0.05;
    const double top = std::max(kStep, std::ceil(peak / kStep) * kStep);
    const float plotW = kBinPx * static_cast<float>(kBins);
    for (double v = kStep; v <= top + 1e-9; v += kStep) {
        const float y = kHistBase - static_cast<float>(v / top) * (kHistBase - kHistTop);
        gd::fillRect(r, kHistX, y, plotW, 1.0F, gd::kGrid);
        char scale[16];
        SDL_snprintf(scale, sizeof scale, "%.0f%%", v * 100.0);
        app.hud.text(r, kHistX + plotW - app.hud.width(scale), y + 1.0F, scale, gd::kMuted);
    }

    const int count = shownCount(app);
    const float barW = count == 1 ? 15.0F : 5.0F;
    int slot = 0;
    for (int t = 0; t < 3; ++t) {
        if (!shown(app, t)) continue;
        const auto i = static_cast<std::size_t>(t);
        const stl::Pool& p = app.pools[i];
        for (int b = 0; b < kBins; ++b) {
            if (p.gapCount <= 0 || p.bins[static_cast<std::size_t>(b)] == 0) continue;
            const double share = static_cast<double>(p.bins[static_cast<std::size_t>(b)]) / p.gapCount;
            const float h = static_cast<float>(share / top) * (kHistBase - kHistTop);
            const float x = kHistX + static_cast<float>(b) * kBinPx + (count == 1 ? 2.0F : 1.0F + static_cast<float>(slot) * 6.0F);
            gd::fillRect(r, x, kHistBase - h, barW, h, kColors[i]);
        }
        ++slot;
    }
    gd::fillRect(r, kHistX, kHistBase, kBinPx * kBins, 1.0F, gd::kInk);

    // Mean gap of each storyteller as a short tick below the axis.
    for (int t = 0; t < 3; ++t) {
        if (!shown(app, t)) continue;
        const auto i = static_cast<std::size_t>(t);
        const float x = kHistX + static_cast<float>(app.pools[i].meanGap / kBinDays) * kBinPx;
        gd::fillRect(r, x - 1.0F, kHistBase + 1.0F, 3.0F, 8.0F, kColors[i]);
    }
    for (int d = 0; d <= kBins; d += 5) {
        char label[8];
        if (d == kBins) {
            SDL_snprintf(label, sizeof label, "%d+", d);
        } else {
            SDL_snprintf(label, sizeof label, "%d", d);
        }
        const float x = kHistX + static_cast<float>(d) * kBinPx;
        app.hud.text(r, x - app.hud.width(label) / 2.0F, kHistBase + 12.0F, label, gd::kMuted);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "colony %llu · day %d of %d", static_cast<unsigned long long>(app.seed - kFirstSeed + 1),
                 static_cast<int>(std::min(app.revealed, kHorizon)), static_cast<int>(kHorizon));
    app.hud.titleBar(r, kWidth, "[ STORYTELLERS ]", right);
    drawTimeline(app);
    drawStats(app);
    drawHistogram(app);
    app.hud.text(r, 40.0F, 444.0F, "filled = random threat   hollow = forced after 13 days   shaded = on window", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, 826.0F, kChipY + 6.0F, "Space: skip", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("storyteller-timelines", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    rollColony(*app);
    for (std::size_t i = 0; i < kTellers.size(); ++i) {
        app->pools[i] = stl::pool(kTellers[i], kPoolRuns, kPoolDays, kPoolSeed, kBins, kBinDays);
    }
    app->lastTicks = SDL_GetTicks();
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
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here switches or restarts, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::ShowCassandra); break;
                case SDLK_2: apply(app, Action::ShowPhoebe); break;
                case SDLK_3: apply(app, Action::ShowRandy); break;
                case SDLK_4: apply(app, Action::ShowAll); break;
                case SDLK_R: apply(app, Action::NewColony); break;
                case SDLK_P: apply(app, Action::Replay); break;
                case SDLK_SPACE:
                    app.revealed = kHorizon;
                    app.playing = false;
                    break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicks();
    const double dt = std::min(static_cast<double>(now - app.lastTicks) / 1000.0, 0.25);
    app.lastTicks = now;
    if (app.playing) {
        app.revealed += dt * kPlaySpeed;
        if (app.revealed >= kHorizon) {
            app.revealed = kHorizon;
            app.playing = false;
        }
    }
    drawScene(app);
    SDL_RenderPresent(app.renderer);
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
