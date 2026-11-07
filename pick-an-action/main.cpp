// Pick an action with SDL3 main callbacks.
//
// Four need sliders feed six actions with a score each. What the agent does with those scores is the selection rule, and
// the rule is what the agent's character comes from: Max is a robot, Weighted random sometimes lets the starving watch
// TV, Band keeps only plausible options, Dual (rank plus weight) lets the urgent need win outright. Every change of a
// slider or rule rolls 1000 picks at once, so the histogram next to the exact chance (the tick on each bar) and the
// stupidity rate are always on screen. The scoring and selection live in logic.hpp and are covered by test.cpp.

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
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kSliderX = 24.0F;
constexpr float kSliderW = 240.0F;
constexpr float kSliderTop = 66.0F;
constexpr float kSliderStep = 58.0F;
constexpr float kRowsTop = 68.0F;
constexpr float kRowStep = 44.0F;
constexpr float kHistX = 660.0F;
constexpr float kHistW = 210.0F;
constexpr double kLiveInterval = 0.6;  // seconds between the single live picks
constexpr double kFlash = 0.45;        // seconds the picked row stays lit
constexpr int kBatch = 1000;
constexpr std::size_t kLogSize = 64;

constexpr std::array<const char*, 4> kNeedNames{"Hunger", "Tired", "Bored", "Lonely"};

enum class Action { Max, Weighted, Band, Dual, Roll, Starving, Content, Mixed };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

const std::array<pick::Needs, 3> kPresets{{{0.95, 0.55, 0.5, 0.3}, {0.2, 0.2, 0.3, 0.1}, {0.85, 0.15, 0.9, 0.4}}};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    pick::Needs needs = kPresets[0];
    pick::Rule rule = pick::Rule::Weighted;
    std::vector<pick::Option> opts;
    std::vector<double> probs;
    pick::Tally tally{pick::kActions};
    pick::Rng rng{0x1234ABCDULL};
    std::vector<std::size_t> log;  // oldest first
    std::size_t lastPick = 0;
    double flash = 0.0;
    double liveTimer = 0.0;
    int dragging = -1;
    Uint64 lastNs = 0;
    long maxFrames = -1;
    long frames = 0;
};

void roll(App& app, int count) {
    for (int i = 0; i < count; ++i) {
        const auto choice = pick::pickFrom(app.probs, app.rng.next());
        if (!choice) return;
        app.tally.add(*choice, app.opts);
        app.lastPick = *choice;
        if (app.log.size() >= kLogSize) app.log.erase(app.log.begin());
        app.log.push_back(*choice);
    }
}

/// Needs or rule changed: recompute the exact chances and start the statistics over with a full batch.
void refresh(App& app) {
    app.opts = pick::options(app.needs);
    app.probs = pick::probabilities(app.opts, app.rule);
    app.tally.clear();
    app.log.clear();
    app.liveTimer = 0.0;
    app.flash = 0.0;
    roll(app, kBatch);
}

void setNeed(App& app, int index, double value) {
    const double v = std::round(pick::clamp01(value) * 100.0) / 100.0;
    if (v == app.needs[static_cast<std::size_t>(index)]) return;
    app.needs[static_cast<std::size_t>(index)] = v;
    refresh(app);
}

SDL_FRect sliderRect(int i) { return SDL_FRect{kSliderX - 4.0F, kSliderTop + kSliderStep * static_cast<float>(i), kSliderW + 16.0F, 46.0F}; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Action::Max, "1 Max", 0.0F},
                                   {Action::Weighted, "2 Weighted", 8.0F},
                                   {Action::Band, "3 Band", 8.0F},
                                   {Action::Dual, "4 Dual", 8.0F},
                                   {Action::Roll, "Space +1000", 24.0F},
                                   {Action::Starving, "5 Starving", 24.0F},
                                   {Action::Content, "6 Content", 8.0F},
                                   {Action::Mixed, "7 Mixed", 8.0F}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void applyPreset(App& app, std::size_t index) {
    app.needs = kPresets[index];
    app.dragging = -1;
    refresh(app);
}

void setRule(App& app, pick::Rule rule) {
    app.rule = rule;
    refresh(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Max: setRule(app, pick::Rule::Max); break;
        case Action::Weighted: setRule(app, pick::Rule::Weighted); break;
        case Action::Band: setRule(app, pick::Rule::Band); break;
        case Action::Dual: setRule(app, pick::Rule::Dual); break;
        case Action::Roll: roll(app, kBatch); break;
        case Action::Starving: applyPreset(app, 0); break;
        case Action::Content: applyPreset(app, 1); break;
        case Action::Mixed: applyPreset(app, 2); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Max: return app.rule == pick::Rule::Max;
        case Action::Weighted: return app.rule == pick::Rule::Weighted;
        case Action::Band: return app.rule == pick::Rule::Band;
        case Action::Dual: return app.rule == pick::Rule::Dual;
        case Action::Roll: return false;
        case Action::Starving: return app.needs == kPresets[0];
        case Action::Content: return app.needs == kPresets[1];
        case Action::Mixed: return app.needs == kPresets[2];
    }
    return false;
}

const char* ruleText(pick::Rule rule) {
    switch (rule) {
        case pick::Rule::Max: return "max: the highest score always wins";
        case pick::Rule::Weighted: return "weighted: chance = score / sum of scores";
        case pick::Rule::Band: return "band: weighted, only scores within 30% of the best";
        case pick::Rule::Dual: return "dual: highest rank only, weighted inside it";
    }
    return "";
}

void drawSliders(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kSliderX, 44.0F, "NEEDS (drag)", gd::kMuted);
    for (int i = 0; i < 4; ++i) {
        const float y = kSliderTop + kSliderStep * static_cast<float>(i);
        const double value = app.needs[static_cast<std::size_t>(i)];
        const bool urgent = value >= pick::kCritical;
        const gd::Color tone = urgent ? gd::kOrange : gd::kTeal;
        app.hud.text(r, kSliderX, y, kNeedNames[static_cast<std::size_t>(i)], gd::kInk, gd::Hud::Style::Bold);
        char num[16];
        SDL_snprintf(num, sizeof num, "%.2f", value);
        app.hud.text(r, kSliderX + kSliderW - app.hud.width(num, gd::Hud::Style::Bold), y, num, urgent ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);
        if (urgent) app.hud.text(r, kSliderX + 80.0F, y, "URGENT", gd::kOrange);
        const float ty = y + 28.0F;
        gd::fillRect(r, kSliderX, ty, kSliderW, 8.0F, gd::kGrid);
        gd::fillRect(r, kSliderX, ty, kSliderW * static_cast<float>(value), 8.0F, tone);
        gd::strokeRect(r, kSliderX, ty, kSliderW, 8.0F, gd::kInk);
        const float tx = kSliderX + kSliderW * static_cast<float>(pick::kCritical);
        gd::fillRect(r, tx, ty - 5.0F, 2.0F, 18.0F, gd::kMuted);
        const float kx = kSliderX + kSliderW * static_cast<float>(value);
        gd::card(r, kx - 6.0F, ty - 7.0F, 12.0F, 22.0F, app.dragging == i ? gd::kTeal : gd::kWhite, gd::kInk);
    }
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const double rate = app.tally.dumbRate();
    const double exact = pick::expectedDumbRate(app.opts, app.probs);
    const gd::Color tone = app.tally.dumb > 0 ? gd::kOrange : gd::kTeal;
    gd::card(r, 24.0F, 298.0F, 266.0F, 130.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, 306.0F, "STUPIDITY RATE", gd::kMuted, gd::Hud::Style::Bold);
    char num[32];
    SDL_snprintf(num, sizeof num, "%.1f%%", rate * 100.0);
    app.hud.text(r, 38.0F, 328.0F, num, tone, gd::Hud::Style::Large);
    SDL_snprintf(num, sizeof num, "exact %.1f%%", exact * 100.0);
    app.hud.text(r, 160.0F, 344.0F, num, gd::kInk, gd::Hud::Style::Bold);
    SDL_snprintf(num, sizeof num, "%ld picks", app.tally.picks);
    app.hud.text(r, 38.0F, 376.0F, num, gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, 38.0F, 398.0F, "under half the best", gd::kMuted);
}

void drawRows(App& app) {
    SDL_Renderer* r = app.renderer;
    const double best = pick::bestWeight(app.opts);
    app.hud.text(r, 316.0F, 44.0F, "ACTION", gd::kMuted);
    app.hud.text(r, 412.0F, 44.0F, "SCORE", gd::kMuted);
    app.hud.text(r, 594.0F, 44.0F, "RANK", gd::kMuted);
    app.hud.text(r, kHistX, 44.0F, "PICKS  tick = exact chance", gd::kMuted);
    const double total = static_cast<double>(std::max(1L, app.tally.picks));
    for (std::size_t i = 0; i < pick::kActions; ++i) {
        const float y = kRowsTop + kRowStep * static_cast<float>(i);
        if (app.flash > 0.0 && i == app.lastPick) gd::fillRect(r, 308.0F, y - 4.0F, 632.0F, 38.0F, gd::kTealSoft);
        const pick::Option& o = app.opts[i];
        app.hud.text(r, 316.0F, y + 6.0F, pick::actionName(i), gd::kInk, gd::Hud::Style::Bold);

        gd::fillRect(r, 412.0F, y + 6.0F, 130.0F, 14.0F, gd::kGrid);
        gd::fillRect(r, 412.0F, y + 6.0F, 130.0F * static_cast<float>(pick::clamp01(o.weight)), 14.0F, gd::kTealDark);
        gd::strokeRect(r, 412.0F, y + 6.0F, 130.0F, 14.0F, gd::kInk);
        char num[16];
        SDL_snprintf(num, sizeof num, "%.2f", o.weight);
        app.hud.text(r, 548.0F, y + 5.0F, num, gd::kInk);

        SDL_snprintf(num, sizeof num, "R%d", o.rank);
        app.hud.text(r, 600.0F, y + 5.0F, num, o.rank == 2 ? gd::kOrange : gd::kMuted, gd::Hud::Style::Bold);

        const bool dumb = pick::isDumb(o.weight, best);
        const double share = static_cast<double>(app.tally.counts[i]) / total;
        gd::fillRect(r, kHistX, y + 6.0F, kHistW, 14.0F, gd::kGrid);
        gd::fillRect(r, kHistX, y + 6.0F, kHistW * static_cast<float>(share), 14.0F, dumb ? gd::kOrange : gd::kTeal);
        gd::strokeRect(r, kHistX, y + 6.0F, kHistW, 14.0F, gd::kInk);
        if (app.probs[i] > 0.0) gd::fillRect(r, kHistX + kHistW * static_cast<float>(app.probs[i]) - 1.0F, y + 1.0F, 3.0F, 24.0F, gd::kInk);
        SDL_snprintf(num, sizeof num, "%.1f%%", share * 100.0);
        app.hud.text(r, kHistX + kHistW + 8.0F, y + 5.0F, num, gd::kInk);
    }
}

void drawLog(App& app) {
    SDL_Renderer* r = app.renderer;
    const double best = pick::bestWeight(app.opts);
    app.hud.text(r, 316.0F, 346.0F, "RECENT PICKS (newest first, orange = dumb)", gd::kMuted);
    float x = 316.0F;
    float y = 370.0F;
    constexpr float kRight = 940.0F;
    for (std::size_t n = 0; n < app.log.size(); ++n) {
        const std::size_t i = app.log[app.log.size() - 1 - n];
        const char* name = pick::actionName(i);
        const float w = app.hud.width(name, gd::Hud::Style::Bold);
        if (x + w > kRight) {
            x = 316.0F;
            y += 24.0F;
            if (y > 410.0F) break;
        }
        app.hud.text(r, x, y, name, pick::isDumb(app.opts[i].weight, best) ? gd::kOrange : gd::kInk, n == 0 ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        x += w + 20.0F;
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ PICK AN ACTION ]", ruleText(app.rule));
    drawSliders(app);
    drawStats(app);
    drawRows(app);
    drawLog(app);
    app.hud.text(r, 24.0F, 446.0F, "drag a slider: new needs   Space: +1000 picks   tick = exact chance, bar = rolled", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void tick(App& app) {
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(0.25, static_cast<double>(now - app.lastNs) * 1e-9);
    app.lastNs = now;
    app.flash = std::max(0.0, app.flash - dt);
    app.liveTimer += dt;
    if (app.liveTimer >= kLiveInterval) {
        app.liveTimer = 0.0;
        roll(app, 1);
        app.flash = kFlash;
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
    if (!SDL_CreateWindowAndRenderer("pick-an-action", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    refresh(*app);
    app->lastNs = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.dragging >= 0) setNeed(app, app.dragging, pick::valueFromX(event->motion.x, kSliderX, kSliderW));
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int i = 0; i < 4; ++i) {
                const SDL_FRect rect = sliderRect(i);
                if (SDL_PointInRectFloat(&p, &rect)) {
                    app.dragging = i;
                    setNeed(app, i, pick::valueFromX(p.x, kSliderX, kSliderW));
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = -1;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging = -1; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here switches or rolls, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Max); break;
                case SDLK_2: apply(app, Action::Weighted); break;
                case SDLK_3: apply(app, Action::Band); break;
                case SDLK_4: apply(app, Action::Dual); break;
                case SDLK_5: apply(app, Action::Starving); break;
                case SDLK_6: apply(app, Action::Content); break;
                case SDLK_7: apply(app, Action::Mixed); break;
                case SDLK_SPACE: apply(app, Action::Roll); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    tick(app);
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
