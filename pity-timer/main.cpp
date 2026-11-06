// Pity timer with SDL3 main callbacks.
//
// Ten thousand players roll for the same rare drop until it drops, and the histogram shows how long each had to wait.
// With plain random the first bar is the tallest and a long tail runs far to the right: a 1 % rate is not "every 100
// tries". The orange zone right of the 1 / rate mark is everyone who still has nothing there. The chips switch the
// chance function (random, PRD, soft + hard pity), the steppers change the rate and the guarantee. The black line is the
// exact distribution from logic.hpp, the bars are the dice, so you see both agree. All the math is in logic.hpp and
// covered by test.cpp.

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

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kPlayers = 10000;
constexpr int kPlayersPerFrame = 400;  // the roll animates over 25 frames
constexpr float kPlotLeft = 64.0F;
constexpr float kPlotRight = 936.0F;
constexpr float kPlotTop = 160.0F;
constexpr float kPlotBottom = 386.0F;
constexpr std::array<double, 7> kRatesPercent{0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0};
constexpr std::array<int, 7> kGuarantees{20, 30, 50, 75, 100, 150, 200};
constexpr gd::Color kOrangeSoft{234, 88, 12, 30};
constexpr gd::Color kMutedSoft{107, 114, 128, 120};

enum class Action { ModeRandom, ModePrd, ModePity, Roll, RateDown, RateUp, GuaranteeDown, GuaranteeUp };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    pity::Mode mode = pity::Mode::Random;
    std::size_t rateIndex = 1;       // 1 %
    std::size_t guaranteeIndex = 4;  // try 100
    pity::Model model;
    pity::Exact exactMean;     // exact distribution of the selected model
    pity::Exact exactRandom;   // plain random at the same rate, the ghost line
    pity::Layout layout{1, 1};
    int mark = 100;

    std::vector<int> waits;
    pity::Rng rng{1};
    std::uint64_t nextSeed = 20270730;
    pity::Summary summary;
    std::vector<int> bars;
    int overflow = 0;

    long maxFrames = -1;
    long frames = 0;
};

double rate(const App& app) { return kRatesPercent[app.rateIndex] / 100.0; }
int guarantee(const App& app) { return kGuarantees[app.guaranteeIndex]; }

void refreshStats(App& app) {
    app.summary = pity::summarize(app.waits, app.mark);
    app.bars = pity::histogram(app.waits, app.layout.binWidth, app.layout.bins, app.overflow);
}

/// Roll up to `count` more players and refresh what is shown. Returns nothing: progress is waits.size().
void rollPlayers(App& app, int count) {
    const int room = kPlayers - static_cast<int>(app.waits.size());
    for (int i = 0; i < std::min(count, room); ++i) app.waits.push_back(pity::rollPlayer(app.model, app.rng));
    refreshStats(app);
}

/// Everything derived from mode, rate and guarantee, then a fresh run. The old results are thrown away: they belong to
/// another setup, so they must not stay on screen next to the new numbers.
void rebuild(App& app, bool instant) {
    app.model = pity::makeModel(app.mode, rate(app), guarantee(app));
    app.exactMean = pity::exact(app.model);
    app.exactRandom = pity::exact(pity::makeModel(pity::Mode::Random, rate(app), guarantee(app)));
    app.layout = pity::layoutFor(rate(app));
    app.mark = pity::markAttempts(rate(app));
    app.waits.clear();
    app.rng = pity::Rng(app.nextSeed++);
    rollPlayers(app, instant ? kPlayers : 0);
}

bool enabled(const App& app, Action action) {
    if (action == Action::GuaranteeDown || action == Action::GuaranteeUp) return app.mode == pity::Mode::Pity;
    return true;
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ModeRandom: return app.mode == pity::Mode::Random;
        case Action::ModePrd: return app.mode == pity::Mode::Prd;
        case Action::ModePity: return app.mode == pity::Mode::Pity;
        default: return false;
    }
}

void apply(App& app, Action action) {
    if (!enabled(app, action)) return;
    switch (action) {
        case Action::ModeRandom: app.mode = pity::Mode::Random; break;
        case Action::ModePrd: app.mode = pity::Mode::Prd; break;
        case Action::ModePity: app.mode = pity::Mode::Pity; break;
        case Action::Roll: break;
        case Action::RateDown: app.rateIndex = app.rateIndex > 0 ? app.rateIndex - 1 : 0; break;
        case Action::RateUp: app.rateIndex = std::min(app.rateIndex + 1, kRatesPercent.size() - 1); break;
        case Action::GuaranteeDown: app.guaranteeIndex = app.guaranteeIndex > 0 ? app.guaranteeIndex - 1 : 0; break;
        case Action::GuaranteeUp: app.guaranteeIndex = std::min(app.guaranteeIndex + 1, kGuarantees.size() - 1); break;
    }
    rebuild(app, false);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float x;
        float y;
        float w;  // 0 = fit the label
    };
    constexpr float kRow1 = 446.0F;
    constexpr float kRow2 = 488.0F;
    const std::array<Def, 8> defs{{{Action::ModeRandom, "1 Random", 40.0F, kRow1, 0.0F},
                                   {Action::ModePrd, "2 PRD", 0.0F, kRow1, 0.0F},
                                   {Action::ModePity, "3 Pity", 0.0F, kRow1, 0.0F},
                                   {Action::Roll, "Space Roll 10,000", 0.0F, kRow1, 0.0F},
                                   {Action::RateDown, "-", 150.0F, kRow2, 34.0F},
                                   {Action::RateUp, "+", 274.0F, kRow2, 34.0F},
                                   {Action::GuaranteeDown, "-", 520.0F, kRow2, 34.0F},
                                   {Action::GuaranteeUp, "+", 644.0F, kRow2, 34.0F}}};
    float flow = 40.0F;
    for (std::size_t i = 0; i < defs.size(); ++i) {
        const Def& d = defs[i];
        float x = d.x;
        float w = d.w;
        if (w == 0.0F) {
            w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
            if (i == 3) flow += 32.0F;  // the roll button sits apart from the mode switch
            x = flow;
            flow += w + 8.0F;
        }
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, 34.0F}});
    }
}

void drawChip(App& app, const Chip& c) {
    SDL_Renderer* r = app.renderer;
    const bool on = active(app, c.action);
    const bool can = enabled(app, c.action);
    gd::Color fill = on ? gd::kTeal : gd::kWhite;
    gd::Color edge = gd::kInk;
    gd::Color ink = on ? gd::kWhite : gd::kInk;
    if (!can) {
        fill = gd::kPaper;
        edge = kMutedSoft;
        ink = kMutedSoft;
    }
    gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, edge);
    app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, ink, gd::Hud::Style::Bold);
}

void drawStepper(App& app, float x, const char* label, const std::string& value, bool can) {
    const gd::Color ink = can ? gd::kInk : kMutedSoft;
    app.hud.text(app.renderer, x, 494.0F, label, can ? gd::kMuted : kMutedSoft);
    app.hud.text(app.renderer, x + 144.0F + (90.0F - app.hud.width(value, gd::Hud::Style::Bold)) / 2.0F, 494.0F, value, ink, gd::Hud::Style::Bold);
}

std::string percent(double share, int decimals = 1) {
    char buf[32];
    SDL_snprintf(buf, sizeof buf, decimals == 1 ? "%.1f%%" : "%.2f%%", share * 100.0);
    return buf;
}

void drawStatCards(App& app) {
    SDL_Renderer* r = app.renderer;
    const bool isRandom = app.mode == pity::Mode::Random;
    const gd::Color tone = isRandom ? gd::kOrange : gd::kTeal;
    char line[96];

    SDL_snprintf(line, sizeof line, "empty after %d tries", app.mark);
    gd::card(r, 24.0F, 40.0F, 290.0F, 90.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, 46.0F, line, gd::kMuted);
    app.hud.text(r, 38.0F, 68.0F, percent(app.summary.shareAbove), tone, gd::Hud::Style::Large);
    const std::string exactText = "exact math: " + percent(app.exactMean.stillEmpty(app.mark));
    app.hud.text(r, 38.0F, 108.0F, exactText, gd::kInk);

    gd::card(r, 330.0F, 40.0F, 270.0F, 90.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 344.0F, 46.0F, "mean wait in tries", gd::kMuted);
    SDL_snprintf(line, sizeof line, "%.1f", app.summary.mean);
    app.hud.text(r, 344.0F, 68.0F, line, tone, gd::Hud::Style::Large);
    const double real = app.exactMean.mean > 0.0 ? 1.0 / app.exactMean.mean : 0.0;
    const std::string realText = "real rate " + percent(real, 2);
    app.hud.text(r, 344.0F, 108.0F, realText, gd::kInk);

    gd::card(r, 616.0F, 40.0F, 320.0F, 90.0F, gd::kWhite, gd::kInk);
    const int done = static_cast<int>(app.waits.size());
    if (done < kPlayers) {
        const std::string rolling = "rolling " + pity::thousands(done) + " / " + pity::thousands(kPlayers);
        app.hud.text(r, 630.0F, 46.0F, rolling, gd::kOrange);
    } else {
        app.hud.text(r, 630.0F, 46.0F, pity::thousands(kPlayers) + " players, one drop each", gd::kMuted);
    }
    SDL_snprintf(line, sizeof line, "median %d   95th %d", app.summary.median, app.summary.p95);
    app.hud.text(r, 630.0F, 70.0F, line, gd::kInk, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "longest wait %d", app.summary.max);
    app.hud.text(r, 630.0F, 94.0F, line, gd::kInk, gd::Hud::Style::Bold);
}

std::string formulaText(const App& app) {
    char buf[96];
    switch (app.mode) {
        case pity::Mode::Random: SDL_snprintf(buf, sizeof buf, "chance on every try: %.1f%%", rate(app) * 100.0); break;
        case pity::Mode::Prd: SDL_snprintf(buf, sizeof buf, "chance on try n: n x %.4f%%", app.model.prdC * 100.0); break;
        case pity::Mode::Pity:
            SDL_snprintf(buf, sizeof buf, "base %.1f%%, ramp from try %d, sure at try %d", rate(app) * 100.0, pity::softStart(app.model.hardPity),
                         app.model.hardPity);
            break;
    }
    return buf;
}

/// 1 px lines are thin on a scaled-up 960x540 canvas, so draw a few side by side.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    for (int o = -half; o <= half; ++o) SDL_RenderLine(r, x0, y0 + static_cast<float>(o), x1, y1 + static_cast<float>(o));
}

void drawPlot(App& app) {
    SDL_Renderer* r = app.renderer;
    const float width = kPlotRight - kPlotLeft;
    const float height = kPlotBottom - kPlotTop;
    const int bins = app.layout.bins;
    const float barW = width / static_cast<float>(bins);

    const std::vector<double> expected = pity::binProbabilities(app.exactMean, app.layout.binWidth, bins);
    const std::vector<double> ghost = pity::binProbabilities(app.exactRandom, app.layout.binWidth, bins);
    double top = 1.0;
    for (int k = 0; k < bins; ++k) {
        const auto i = static_cast<std::size_t>(k);
        top = std::max({top, static_cast<double>(app.bars[i]), expected[i] * kPlayers, ghost[i] * kPlayers});
    }
    const double yMax = pity::niceCeil(top * 1.04);
    const auto yOf = [&](double count) { return kPlotBottom - static_cast<float>(count / yMax) * height; };

    // Heading row: what the bars are, and the chance function in use.
    char head[64];
    if (app.layout.binWidth == 1) {
        SDL_snprintf(head, sizeof head, "players per try");
    } else {
        SDL_snprintf(head, sizeof head, "players per bar of %d tries", app.layout.binWidth);
    }
    app.hud.text(r, kPlotLeft, 134.0F, head, gd::kMuted);
    const std::string formula = formulaText(app);
    app.hud.text(r, kPlotRight - app.hud.width(formula), 134.0F, formula, gd::kInk);

    // Grid and y labels.
    for (int g = 0; g <= 4; ++g) {
        const double v = yMax * g / 4.0;
        const float y = yOf(v);
        gd::fillRect(r, kPlotLeft, y, width, 1.0F, g == 0 ? gd::kInk : gd::kGrid);
        char label[16];
        SDL_snprintf(label, sizeof label, "%d", static_cast<int>(v));
        app.hud.text(r, kPlotLeft - 6.0F - app.hud.width(label), y - 11.0F, label, gd::kMuted);
    }

    // The empty-handed zone: everyone right of the 1 / rate mark.
    const float markX = kPlotLeft + static_cast<float>(app.mark) / static_cast<float>(app.layout.binWidth) * barW;
    gd::fillRect(r, markX, kPlotTop, kPlotRight - markX, height, kOrangeSoft);

    // Bars: the dice.
    const gd::Color barColor = app.mode == pity::Mode::Random ? gd::kOrange : gd::kTeal;
    for (int k = 0; k < bins; ++k) {
        const float h = static_cast<float>(app.bars[static_cast<std::size_t>(k)]) / static_cast<float>(yMax) * height;
        if (h <= 0.0F) continue;
        gd::fillRect(r, kPlotLeft + static_cast<float>(k) * barW, kPlotBottom - h, std::max(1.0F, barW - 1.0F), h, barColor);
    }

    // Lines: the exact math. Ghost of plain random in orange when another model is selected, the model itself in ink.
    const auto polyline = [&](const std::vector<double>& probs, gd::Color color, int half) {
        for (int k = 0; k + 1 < bins; ++k) {
            const auto i = static_cast<std::size_t>(k);
            const float x0 = kPlotLeft + (static_cast<float>(k) + 0.5F) * barW;
            thickLine(r, x0, yOf(probs[i] * kPlayers), x0 + barW, yOf(probs[i + 1] * kPlayers), color, half);
        }
    };
    if (app.mode != pity::Mode::Random) polyline(ghost, gd::kOrange, 0);
    polyline(expected, gd::kInk, 1);

    // The mark: dashed.
    for (float y = kPlotTop; y < kPlotBottom; y += 10.0F) gd::fillRect(r, markX - 1.0F, y, 2.0F, 6.0F, gd::kOrange);

    // X ticks at multiples of the nominal wait.
    for (int i = 0; i <= 5; ++i) {
        const int tries = i * app.mark;
        const float x = kPlotLeft + static_cast<float>(tries) / static_cast<float>(app.layout.binWidth) * barW;
        if (x > kPlotRight + 1.0F) break;
        const std::string label = pity::thousands(tries);
        const bool atMark = i == 1;
        const float w = app.hud.width(label, atMark ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        app.hud.text(r, std::min(x - w / 2.0F, kPlotRight - w + 8.0F), kPlotBottom + 4.0F, label, atMark ? gd::kOrange : gd::kMuted,
                     atMark ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
    }

    // Legend, top right inside the empty zone.
    float ly = kPlotTop + 6.0F;
    const auto legend = [&](const std::string& text, gd::Color color) {
        app.hud.text(r, kPlotRight - 8.0F - app.hud.width(text), ly, text, color);
        ly += 22.0F;
    };
    legend("shaded: still empty", gd::kOrange);
    legend("black line: exact math", gd::kInk);
    if (app.mode != pity::Mode::Random) legend("thin orange: plain random", gd::kOrange);
    if (app.overflow > 0) legend(pity::thousands(app.overflow) + " players beyond the right edge", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    const char* name = app.mode == pity::Mode::Random ? "random" : app.mode == pity::Mode::Prd ? "pseudo-random (PRD)" : "soft + hard pity";
    app.hud.titleBar(r, kWidth, "[ PITY TIMER ]", name);
    drawStatCards(app);
    drawPlot(app);
    app.hud.text(r, 40.0F, 418.0F, "1-3 mode   Space roll again   Up/Down drop rate   Left/Right guarantee", gd::kMuted);
    for (const Chip& c : app.chips) drawChip(app, c);
    char value[16];
    SDL_snprintf(value, sizeof value, "%.1f%%", kRatesPercent[app.rateIndex]);
    drawStepper(app, 40.0F, "Drop rate", value, true);
    SDL_snprintf(value, sizeof value, "try %d", guarantee(app));
    drawStepper(app, 410.0F, "Guarantee", value, app.mode == pity::Mode::Pity);
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
    if (!SDL_CreateWindowAndRenderer("pity-timer", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    rebuild(*app, true);  // the first frame already shows a finished run
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
                    apply(app, c.action);
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: if (!event->key.repeat) apply(app, Action::ModeRandom); break;
                case SDLK_2: if (!event->key.repeat) apply(app, Action::ModePrd); break;
                case SDLK_3: if (!event->key.repeat) apply(app, Action::ModePity); break;
                case SDLK_SPACE: if (!event->key.repeat) apply(app, Action::Roll); break;
                case SDLK_UP: apply(app, Action::RateUp); break;
                case SDLK_DOWN: apply(app, Action::RateDown); break;
                case SDLK_RIGHT: apply(app, Action::GuaranteeUp); break;
                case SDLK_LEFT: apply(app, Action::GuaranteeDown); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    if (static_cast<int>(app.waits.size()) < kPlayers) rollPlayers(app, kPlayersPerFrame);
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
