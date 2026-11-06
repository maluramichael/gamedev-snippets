// Damage stacking with SDL3 main callbacks.
//
// Six percentage modifiers sit in two pots. Everything in the Increased pot is added up and applied once, every line
// in the More pot multiplies on its own. A click moves a line to the other pot, the mouse wheel changes its percentage.
// The upper right shows the total damage, next to what the two naive readings would give ("add every percentage",
// "multiply every percentage"). The lower right shows what each line is really worth next to what it says on the label:
// a More line always keeps its printed value, an Increased line loses value with every other line in its pool.
// The math, including the floor at zero, is in logic.hpp and covered by test.cpp. All numbers are example values.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr double kBase = 100.0;  // example base damage

constexpr float kPotY = 44.0F;
constexpr float kPotH = 380.0F;
constexpr float kPotW = 210.0F;
constexpr float kIncX = 24.0F;
constexpr float kMoreX = 250.0F;
constexpr float kTokenTop = 100.0F;   // from the top of a pot
constexpr float kTokenStep = 46.0F;
constexpr float kTokenH = 40.0F;

constexpr float kRightX = 480.0F;
constexpr float kRightW = 456.0F;
constexpr float kTotalY = 44.0F;
constexpr float kTotalH = 178.0F;
constexpr float kWorthY = 236.0F;
constexpr float kWorthH = 188.0F;

constexpr float kChipY = 466.0F;
constexpr float kChipH = 34.0F;
constexpr double kMaxDt = 0.25;
constexpr double kSmoothing = 14.0;  // 1/s, how fast the bars settle

constexpr std::array<ds::Rule, 3> kRules{ds::Rule::Buckets, ds::Rule::AddAll, ds::Rule::MultiplyAll};
constexpr std::array<const char*, 3> kRuleNames{"buckets", "add all", "multiply all"};
constexpr std::array<const char*, 3> kRuleTags{"correct", "naive", "naive"};

enum class Action { RuleBuckets, RuleAddAll, RuleMultiplyAll, Reset };

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
    std::vector<ds::Mod> mods = ds::defaultMods();
    ds::Rule rule = ds::Rule::Buckets;
    std::array<double, 3> shownDamage{};
    std::vector<double> shownWorth;
    bool primed = false;
    SDL_FPoint mouse{-100.0F, -100.0F};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

std::size_t ruleIndex(ds::Rule rule) { return static_cast<std::size_t>(rule); }

gd::Color bucketColor(ds::Bucket b) { return b == ds::Bucket::Increased ? gd::kOrange : gd::kTeal; }

float potX(ds::Bucket b) { return b == ds::Bucket::Increased ? kIncX : kMoreX; }

/// Position of line `i` inside its pot: how many lines of the same pot come before it.
SDL_FRect tokenRect(const std::vector<ds::Mod>& mods, std::size_t i) {
    int slot = 0;
    for (std::size_t k = 0; k < i; ++k) {
        if (mods[k].bucket == mods[i].bucket) ++slot;
    }
    return SDL_FRect{potX(mods[i].bucket) + 12.0F, kPotY + kTokenTop + kTokenStep * static_cast<float>(slot), kPotW - 24.0F, kTokenH};
}

std::optional<std::size_t> tokenAt(const App& app, SDL_FPoint p) {
    for (std::size_t i = 0; i < app.mods.size(); ++i) {
        const SDL_FRect r = tokenRect(app.mods, i);
        if (SDL_PointInRectFloat(&p, &r)) return i;
    }
    return std::nullopt;
}

SDL_FRect compareRowRect(std::size_t r) { return SDL_FRect{kRightX + 8.0F, kTotalY + 92.0F + 26.0F * static_cast<float>(r), kRightW - 16.0F, 24.0F}; }

void setRule(App& app, ds::Rule rule) { app.rule = rule; }

void reset(App& app) {
    app.mods = ds::defaultMods();
    app.rule = ds::Rule::Buckets;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::RuleBuckets: setRule(app, ds::Rule::Buckets); break;
        case Action::RuleAddAll: setRule(app, ds::Rule::AddAll); break;
        case Action::RuleMultiplyAll: setRule(app, ds::Rule::MultiplyAll); break;
        case Action::Reset: reset(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::RuleBuckets: return app.rule == ds::Rule::Buckets;
        case Action::RuleAddAll: return app.rule == ds::Rule::AddAll;
        case Action::RuleMultiplyAll: return app.rule == ds::Rule::MultiplyAll;
        case Action::Reset: return false;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 4> defs{{{Action::RuleBuckets, "B Buckets (correct)", 0.0F},
                                   {Action::RuleAddAll, "A Add all", 8.0F},
                                   {Action::RuleMultiplyAll, "M Multiply all", 8.0F},
                                   {Action::Reset, "R Reset", 24.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void moveToken(App& app, std::size_t i) {
    if (i < app.mods.size()) app.mods[i].bucket = ds::other(app.mods[i].bucket);
}

/// Bars chase their targets with a frame rate independent smoothing. The first frame starts at the targets.
void update(App& app, double dt) {
    app.shownWorth.resize(app.mods.size(), 0.0);
    for (std::size_t r = 0; r < kRules.size(); ++r) {
        const double target = ds::damage(kBase, app.mods, kRules[r]);
        app.shownDamage[r] = app.primed ? ds::smooth(app.shownDamage[r], target, dt, kSmoothing) : target;
    }
    for (std::size_t i = 0; i < app.mods.size(); ++i) {
        const std::optional<double> w = ds::worth(app.mods, app.rule, i);
        const double target = w ? *w : 0.0;
        app.shownWorth[i] = app.primed ? ds::smooth(app.shownWorth[i], target, dt, kSmoothing) : target;
    }
    app.primed = true;
}

void textRight(App& app, float right, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, right - app.hud.width(s, style), y, s, c, style);
}

void drawPot(App& app, ds::Bucket bucket) {
    SDL_Renderer* r = app.renderer;
    const bool inc = bucket == ds::Bucket::Increased;
    const float x = potX(bucket);
    const bool buckets = app.rule == ds::Rule::Buckets;
    gd::card(r, x, kPotY, kPotW, kPotH, gd::kWhite, gd::kInk);

    int count = 0;
    for (const ds::Mod& m : app.mods) count += m.bucket == bucket ? 1 : 0;
    const double mult = inc ? ds::increasedMultiplier(app.mods) : ds::moreMultiplier(app.mods);
    const gd::Color tone = buckets ? bucketColor(bucket) : gd::kMuted;

    app.hud.text(r, x + 12.0F, kPotY + 8.0F, inc ? "INCREASED" : "MORE", tone, gd::Hud::Style::Bold);
    std::string sub;
    if (!buckets) {
        sub = "pots ignored";
    } else if (inc) {
        sub = "added: " + ds::formatPercent(ds::sumIncreasedPercent(app.mods), 0);
    } else {
        sub = "each one multiplies";
    }
    app.hud.text(r, x + 12.0F, kPotY + 30.0F, sub, gd::kMuted);
    app.hud.text(r, x + 12.0F, kPotY + 52.0F, "x" + ds::formatFixed(mult, 2), tone, gd::Hud::Style::Large);
    if (count == 0) app.hud.text(r, x + 12.0F, kPotY + kTokenTop + 10.0F, "(empty)", gd::kMuted);
}

void drawTokens(App& app, std::optional<std::size_t> hover) {
    SDL_Renderer* r = app.renderer;
    for (std::size_t i = 0; i < app.mods.size(); ++i) {
        const SDL_FRect t = tokenRect(app.mods, i);
        const bool hot = hover && *hover == i;
        const gd::Color tone = bucketColor(app.mods[i].bucket);
        gd::card(r, t.x, t.y, t.w, t.h, hot ? gd::kTealSoft : gd::kWhite, gd::kInk);
        gd::fillRect(r, t.x, t.y, 6.0F, t.h, tone);
        app.hud.text(r, t.x + 16.0F, t.y + 9.0F, std::to_string(i + 1), gd::kMuted);
        app.hud.text(r, t.x + 42.0F, t.y + 9.0F, ds::formatPercent(app.mods[i].percent, 0), gd::kInk, gd::Hud::Style::Bold);
        const bool inc = app.mods[i].bucket == ds::Bucket::Increased;
        textRight(app, t.x + t.w - 10.0F, t.y + 9.0F, inc ? "->" : "<-", hot ? gd::kInk : gd::kMuted, gd::Hud::Style::Bold);
    }
}

void drawTotal(App& app) {
    SDL_Renderer* r = app.renderer;
    const float x = kRightX;
    const float y = kTotalY;
    gd::card(r, x, y, kRightW, kTotalH, gd::kWhite, gd::kInk);
    const std::size_t sel = ruleIndex(app.rule);
    const bool correct = app.rule == ds::Rule::Buckets;
    const gd::Color tone = correct ? gd::kTeal : gd::kOrange;

    app.hud.text(r, x + 14.0F, y + 8.0F, "TOTAL DAMAGE", gd::kMuted);
    textRight(app, x + kRightW - 14.0F, y + 8.0F, std::string(kRuleNames[sel]) + " (" + kRuleTags[sel] + ")", tone, gd::Hud::Style::Bold);

    const std::string big = ds::formatFixed(app.shownDamage[sel], 0);
    app.hud.text(r, x + 14.0F, y + 30.0F, big, tone, gd::Hud::Style::Large);
    const std::string eq = "base " + ds::formatFixed(kBase, 0) + " x " + ds::formatFixed(ds::multiplier(app.mods, app.rule), 2);
    app.hud.text(r, x + 190.0F, y + 42.0F, eq, gd::kInk, gd::Hud::Style::Bold);

    // The three readings of the same six lines on one shared scale.
    const float barX = x + 150.0F;
    const float barMax = 230.0F;
    double scale = 1.0;
    for (const double d : app.shownDamage) scale = std::max(scale, d);
    const float baseX = barX + static_cast<float>(kBase / scale) * barMax;
    app.hud.text(r, baseX - app.hud.width("base") / 2.0F, y + 70.0F, "base", gd::kMuted);
    for (std::size_t k = 0; k < kRules.size(); ++k) {
        const SDL_FRect row = compareRowRect(k);
        const bool on = k == sel;
        const gd::Color c = k == 0 ? gd::kTeal : gd::kOrange;
        if (on) gd::fillRect(r, row.x, row.y, row.w, row.h, gd::kTealSoft);
        app.hud.text(r, x + 14.0F, row.y + 1.0F, kRuleNames[k], on ? gd::kInk : gd::kMuted, on ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        const float w = static_cast<float>(app.shownDamage[k] / scale) * barMax;
        gd::fillRect(r, barX, row.y + 4.0F, w, 16.0F, gd::Color{c.r, c.g, c.b, static_cast<Uint8>(on ? 255 : 120)});
        gd::strokeRect(r, barX, row.y + 4.0F, w, 16.0F, gd::kInk);
        app.hud.text(r, barX + w + 10.0F, row.y + 1.0F, ds::formatFixed(app.shownDamage[k], 0), on ? gd::kInk : gd::kMuted, gd::Hud::Style::Bold);
    }
    gd::fillRect(r, baseX, y + 92.0F, 1.0F, 80.0F, gd::kInk);
}

void drawWorth(App& app, std::optional<std::size_t> hover) {
    SDL_Renderer* r = app.renderer;
    const float x = kRightX;
    const float y = kWorthY;
    gd::card(r, x, y, kRightW, kWorthH, gd::kWhite, gd::kInk);
    app.hud.text(r, x + 14.0F, y + 8.0F, "WORTH PER LINE", gd::kMuted);
    textRight(app, x + kRightW - 14.0F, y + 8.0F, "box = listed, bar = real", gd::kMuted);

    const float zeroX = x + 186.0F;  // 1 px per percent point
    for (std::size_t i = 0; i < app.mods.size(); ++i) {
        const float ry = y + 34.0F + 24.0F * static_cast<float>(i);
        if (hover && *hover == i) gd::fillRect(r, x + 6.0F, ry - 2.0F, kRightW - 12.0F, 24.0F, gd::kTealSoft);
        const gd::Color tone = bucketColor(app.mods[i].bucket);
        gd::fillRect(r, x + 14.0F, ry + 5.0F, 10.0F, 10.0F, tone);
        app.hud.text(r, x + 32.0F, ry - 1.0F, std::to_string(i + 1) + " " + ds::formatPercent(app.mods[i].percent, 0), gd::kInk, gd::Hud::Style::Bold);

        const auto listed = static_cast<float>(app.mods[i].percent);
        const float realPx = std::clamp(static_cast<float>(app.shownWorth[i] * 100.0), -50.0F, 215.0F);
        gd::fillRect(r, std::min(zeroX, zeroX + realPx), ry + 4.0F, std::fabs(realPx), 12.0F, tone);  // negative values go left
        const float lx = std::min(zeroX, zeroX + listed);
        gd::strokeRect(r, lx, ry + 1.0F, std::fabs(listed), 18.0F, gd::kInk);

        const std::optional<double> w = ds::worth(app.mods, app.rule, i);
        std::string label = "n/a";
        if (w) label = *w * 100.0 > 999.0 ? ">999%" : ds::formatPercent(*w * 100.0, 0);
        textRight(app, x + kRightW - 14.0F, ry - 1.0F, label, gd::kInk, gd::Hud::Style::Bold);
    }
    gd::fillRect(r, zeroX, y + 30.0F, 1.0F, 150.0F, gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ INCREASED VS MORE ]", "base damage 100, example values");

    const std::optional<std::size_t> hover = tokenAt(app, app.mouse);
    drawPot(app, ds::Bucket::Increased);
    drawPot(app, ds::Bucket::More);
    drawTokens(app, hover);
    drawTotal(app);
    drawWorth(app, hover);

    app.hud.text(r, 40.0F, 432.0F, "click a line: move it   wheel: change %   1-6: move line", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const gd::Color onColor = c.action == Action::RuleBuckets ? gd::kTeal : gd::kOrange;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? onColor : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
    }
    app.hud.text(r, 40.0F, 512.0F, "Increased: percentages are summed first.  More: each one multiplies.", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("damage-stacking", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: app.mouse = SDL_FPoint{event->motion.x, event->motion.y}; break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE: app.mouse = SDL_FPoint{-100.0F, -100.0F}; break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (const auto t = tokenAt(app, p)) {
                moveToken(app, *t);
                return SDL_APP_CONTINUE;
            }
            for (std::size_t k = 0; k < kRules.size(); ++k) {
                const SDL_FRect row = compareRowRect(k);
                if (SDL_PointInRectFloat(&p, &row)) setRule(app, kRules[k]);
            }
            break;
        }
        case SDL_EVENT_MOUSE_WHEEL: {
            // The wheel carries no usable position here, the last mouse position says which line is meant.
            if (const auto t = tokenAt(app, app.mouse)) {
                int notches = event->wheel.integer_y;
                if (event->wheel.direction == SDL_MOUSEWHEEL_FLIPPED) notches = -notches;
                app.mods[*t].percent = ds::stepPercent(app.mods[*t].percent, notches);
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_B: apply(app, Action::RuleBuckets); break;
                case SDLK_A: apply(app, Action::RuleAddAll); break;
                case SDLK_M: apply(app, Action::RuleMultiplyAll); break;
                case SDLK_R: apply(app, Action::Reset); break;
                default:
                    if (event->key.key >= SDLK_1 && event->key.key <= SDLK_6) moveToken(app, static_cast<std::size_t>(event->key.key - SDLK_1));
                    break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(kMaxDt, static_cast<double>(now - app.lastTicks) / 1e9);
    app.lastTicks = now;
    update(app, dt);
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
