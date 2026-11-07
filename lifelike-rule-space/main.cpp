// Life-like rule space with SDL3 main callbacks.
//
// A rule is a pair of digit sets, Birth and Survive. B3/S23 is Conway's Life, but 262,144 rules share the same shape of
// description and behave nothing alike. The B and S chips edit the digits, the presets jump to famous rules, and a random soup
// (or the R-pentomino) runs at once, so the effect of one digit shows within seconds. The right panel names what the run does
// so far (dies, still, periodic, growing, active) from the exact grid history and the population over time. The rule parser, the
// torus stepper and the run classifier are in logic.hpp and covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCols = 156;
constexpr int kRows = 84;
constexpr float kCell = 4.0F;
constexpr float kGridX = 24.0F;
constexpr float kGridY = 40.0F;
constexpr float kGridW = kCols * kCell;
constexpr float kGridH = kRows * kCell;
constexpr float kPanelX = 672.0F;
constexpr float kPanelW = 264.0F;
constexpr double kRandomRuleSoup = 0.5;  // start density for a random rule
constexpr int kWarmup = 60;  // generations run before the first frame, so the graph and the label already say something
constexpr std::array<int, 3> kSpeeds{10, 30, 90};
constexpr int kMaxStepsPerFrame = 4;
constexpr float kChipH = 30.0F;
constexpr std::array<float, 3> kRowY{392.0F, 428.0F, 466.0F};

enum class Action { Birth, Survive, Preset, Pause, Step, Speed, Soup, Pentomino, Replicator, RandomRule, Clear };

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
    lrs::Rule rule = lrs::kPresets[0].rule;
    lrs::Grid cur{kCols, kRows};
    lrs::Grid prev{kCols, kRows};  // the generation before, for the ghosts of cells that just died
    lrs::Tracker tracker;
    lrs::Rng rng{0x1F2E3D4CULL};
    double soupDensity = lrs::kPresets[0].soup;
    int generation = 0;
    bool paused = false;
    int speed = 1;
    double accumulator = 0.0;
    Uint64 lastTicks = 0;
    bool painting = false;
    bool paintValue = true;
    bool havePaintCell = false;
    lrs::Cell paintCell{};
    long maxFrames = -1;
    long frames = 0;
};

void restartTracking(App& app) {
    app.tracker.reset();
    app.tracker.record(app.cur);
    app.accumulator = 0.0;
}

void freshStart(App& app) {
    app.generation = 0;
    app.prev = app.cur;  // no ghosts on the first picture
    restartTracking(app);
}

void stepOnce(App& app) {
    lrs::step(app.cur, app.prev, app.rule);
    std::swap(app.cur, app.prev);  // prev now holds the generation that just ended
    ++app.generation;
    app.tracker.record(app.cur);
}

void reseedSoup(App& app) {
    lrs::fillSoup(app.cur, app.soupDensity, app.rng);
    freshStart(app);
}

void placeCentered(App& app, const std::vector<std::pair<int, int>>& cells, int size) {
    app.cur.clear();
    for (const auto& [x, y] : cells) app.cur.set(kCols / 2 - size / 2 + x, kRows / 2 - size / 2 + y, true);
    freshStart(app);
}

void layoutChips(App& app) {
    auto addChip = [&app](Action action, int arg, std::string label, float x, float y, float w) {
        app.chips.push_back({action, arg, std::move(label), SDL_FRect{x, y, w, kChipH}});
    };
    auto widthOf = [&app](const std::string& s) { return app.hud.width(s, gd::Hud::Style::Bold) + 24.0F; };

    // Digit chips: B0..B8 in the first row, S0..S8 in the second.
    constexpr float kDigitX = 102.0F;
    constexpr float kDigitW = 30.0F;
    constexpr float kDigitPitch = 34.0F;
    for (int n = 0; n <= 8; ++n) {
        const std::string label = std::to_string(n);
        addChip(Action::Birth, n, label, kDigitX + kDigitPitch * static_cast<float>(n), kRowY[0], kDigitW);
        addChip(Action::Survive, n, label, kDigitX + kDigitPitch * static_cast<float>(n), kRowY[1], kDigitW);
    }

    // Run controls to the right of the digits.
    float x = 428.0F;
    auto next = [&](Action action, int arg, const std::string& label, float y) {
        const float w = widthOf(label);
        addChip(action, arg, label, x, y, w);
        x += w + 8.0F;
    };
    next(Action::Pause, 0, "Space Pause", kRowY[0]);
    next(Action::Step, 0, "N Step", kRowY[0]);
    next(Action::Speed, 0, "F Speed 90/s", kRowY[0]);
    x = 428.0F;
    next(Action::Soup, 0, "R Soup", kRowY[1]);
    next(Action::Pentomino, 0, "E Pentomino", kRowY[1]);
    next(Action::RandomRule, 0, "X Random rule", kRowY[1]);
    next(Action::Clear, 0, "C Clear", kRowY[1]);

    // Presets in the third row.
    x = 24.0F;
    for (std::size_t i = 0; i < lrs::kPresets.size(); ++i) {
        next(Action::Preset, static_cast<int>(i), std::to_string(i + 1) + " " + lrs::kPresets[i].name, kRowY[2]);
    }
    x += 16.0F;
    next(Action::Replicator, 0, "H Replicator", kRowY[2]);
}

void apply(App& app, Action action, int arg) {
    switch (action) {
        case Action::Birth:
            app.rule.birth = static_cast<std::uint16_t>(app.rule.birth ^ (1U << arg));
            restartTracking(app);
            break;
        case Action::Survive:
            app.rule.survive = static_cast<std::uint16_t>(app.rule.survive ^ (1U << arg));
            restartTracking(app);
            break;
        case Action::Preset:
            app.rule = lrs::kPresets[static_cast<std::size_t>(arg)].rule;
            app.soupDensity = lrs::kPresets[static_cast<std::size_t>(arg)].soup;
            reseedSoup(app);
            break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Step:
            app.paused = true;  // a single step only makes sense while the run is stopped
            stepOnce(app);
            break;
        case Action::Speed: app.speed = (app.speed + 1) % static_cast<int>(kSpeeds.size()); break;
        case Action::Soup: reseedSoup(app); break;
        case Action::Pentomino: placeCentered(app, lrs::rPentomino(), 3); break;
        case Action::Replicator:
            // It only replicates in HighLife, so the chip switches the rule as well.
            app.rule = lrs::kPresets[1].rule;
            app.soupDensity = lrs::kPresets[1].soup;
            placeCentered(app, lrs::replicatorRing(), 5);
            break;
        case Action::RandomRule:
            app.rule = lrs::ruleFromIndex(static_cast<std::uint32_t>(app.rng.next() % lrs::kRuleCount));
            app.soupDensity = kRandomRuleSoup;
            reseedSoup(app);
            break;
        case Action::Clear:
            app.cur.clear();
            freshStart(app);
            break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Birth: return (app.rule.birth >> c.arg & 1U) != 0;
        case Action::Survive: return (app.rule.survive >> c.arg & 1U) != 0;
        case Action::Preset: return app.rule == lrs::kPresets[static_cast<std::size_t>(c.arg)].rule;
        case Action::Pause: return app.paused;
        default: return false;
    }
}

std::string chipLabel(const App& app, const Chip& c) {
    if (c.action == Action::Speed) return "F Speed " + std::to_string(kSpeeds[static_cast<std::size_t>(app.speed)]) + "/s";
    return c.label;
}

bool cellAt(float px, float py, lrs::Cell& out) {
    if (px < kGridX || py < kGridY || px >= kGridX + kGridW || py >= kGridY + kGridH) return false;
    out = {static_cast<int>((px - kGridX) / kCell), static_cast<int>((py - kGridY) / kCell)};
    return true;
}

void paintTo(App& app, lrs::Cell target) {
    const lrs::Cell from = app.havePaintCell ? app.paintCell : target;
    for (const lrs::Cell& c : lrs::lineCells(from, target)) app.cur.set(c.x, c.y, app.paintValue);
    app.paintCell = target;
    app.havePaintCell = true;
    app.prev = app.cur;
    restartTracking(app);
}

void stopPainting(App& app) {
    app.painting = false;
    app.havePaintCell = false;
}

std::string percent(int pop) {
    const int total = kCols * kRows;
    const int tenths = static_cast<int>((static_cast<long long>(pop) * 1000 + total / 2) / total);
    return std::to_string(tenths / 10) + "." + std::to_string(tenths % 10) + "%";
}

void drawGrid(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kGridX - 1.0F, kGridY - 1.0F, kGridW + 2.0F, kGridH + 2.0F, gd::kWhite, gd::kInk);
    for (int y = 0; y < kRows; ++y) {
        const std::uint8_t* now = app.cur.row(y);
        const std::uint8_t* before = app.prev.row(y);
        int x = 0;
        while (x < kCols) {
            // 0 empty, 1 alive, 2 just died. One rectangle per run of the same kind.
            const auto kindAt = [&](int i) { return now[i] != 0 ? 1 : (before[i] != 0 ? 2 : 0); };
            const int kind = kindAt(x);
            int end = x + 1;
            while (end < kCols && kindAt(end) == kind) ++end;
            if (kind != 0) {
                gd::fillRect(r, kGridX + static_cast<float>(x) * kCell, kGridY + static_cast<float>(y) * kCell, static_cast<float>(end - x) * kCell, kCell,
                             kind == 1 ? gd::kTeal : gd::kTealSoft);
            }
            x = end;
        }
    }
}

const char* kindText(lrs::Verdict v, std::string& scratch) {
    switch (v.kind) {
        case lrs::Kind::Dies: return "DIES OUT";
        case lrs::Kind::Still: return "STILL";
        case lrs::Kind::Periodic:
            scratch = "PERIODIC " + std::to_string(v.period);
            return scratch.c_str();
        case lrs::Kind::Growing: return "GROWING";
        case lrs::Kind::Active: return "ACTIVE";
    }
    return "";
}

gd::Color kindColor(lrs::Kind kind) {
    switch (kind) {
        case lrs::Kind::Dies:
        case lrs::Kind::Growing: return gd::kOrange;
        case lrs::Kind::Still:
        case lrs::Kind::Periodic: return gd::kTealDark;
        case lrs::Kind::Active: return gd::kInk;
    }
    return gd::kInk;
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    const float px = kPanelX + 14.0F;
    gd::card(r, kPanelX, kGridY - 1.0F, kPanelW, kGridH + 2.0F, gd::kWhite, gd::kInk);

    app.hud.text(r, px, 48.0F, "RULE", gd::kMuted);
    app.hud.text(r, px, 66.0F, lrs::formatRule(app.rule), gd::kInk, gd::Hud::Style::Large);
    const char* name = lrs::presetName(app.rule);
    const bool known = name[0] != '\0';
    app.hud.text(r, px, 112.0F, known ? name : "custom rule", known ? gd::kTealDark : gd::kMuted, gd::Hud::Style::Bold);
    const std::string index = "#" + lrs::groupDigits(lrs::ruleIndex(app.rule)) + " of " + lrs::groupDigits(lrs::kRuleCount);
    app.hud.text(r, px, 134.0F, index, gd::kMuted);

    std::string scratch;
    const lrs::Verdict verdict = app.tracker.verdict();
    const char* label = kindText(verdict, scratch);
    const gd::Hud::Style style = app.hud.width(label, gd::Hud::Style::Large) <= kPanelW - 28.0F ? gd::Hud::Style::Large : gd::Hud::Style::Bold;
    app.hud.text(r, px, 160.0F, label, kindColor(verdict.kind), style);

    const int pop = app.cur.population();
    app.hud.text(r, px, 204.0F, "gen " + std::to_string(app.generation), gd::kInk);
    app.hud.text(r, px, 224.0F, "alive " + lrs::groupDigits(static_cast<std::uint32_t>(pop)) + " (" + percent(pop) + ")", gd::kInk);

    // Population over time, scaled to its own maximum.
    const float gx = px;
    const float gy = 252.0F;
    const float gw = static_cast<float>(lrs::Tracker::kHistory);
    const float gh = 70.0F;
    gd::fillRect(r, gx, gy, gw, gh, gd::kPaper);
    const auto& pops = app.tracker.populations();
    const int peak = std::max(1, pops.empty() ? 1 : *std::max_element(pops.begin(), pops.end()));
    for (std::size_t i = 0; i < pops.size(); ++i) {
        const float h = std::max(pops[i] > 0 ? 1.0F : 0.0F, std::round(gh * static_cast<float>(pops[i]) / static_cast<float>(peak)));
        gd::fillRect(r, gx + static_cast<float>(i), gy + gh - h, 1.0F, h, gd::kTeal);
    }
    gd::strokeRect(r, gx, gy, gw, gh, gd::kInk);
    app.hud.text(r, gx, gy + gh + 4.0F, "density, max " + percent(peak), gd::kMuted);

    const char* note = "";
    gd::Color noteColor = gd::kMuted;
    switch (lrs::growthNote(app.rule)) {
        case lrs::GrowthNote::Blinks: note = "B0: background flips"; break;
        case lrs::GrowthNote::Spreads: note = "B1: spreads everywhere"; noteColor = gd::kOrange; break;
        case lrs::GrowthNote::Bounded: note = "no B0-B3: stays boxed"; noteColor = gd::kOrange; break;
        case lrs::GrowthNote::None: break;
    }
    app.hud.text(r, px, 346.0F, note, noteColor, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ LIFE-LIKE RULE SPACE ]", "262,144 rules, torus " + std::to_string(kCols) + "x" + std::to_string(kRows));
    drawGrid(app);
    drawPanel(app);

    app.hud.text(r, 24.0F, kRowY[0] + 4.0F, "Birth", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, 24.0F, kRowY[1] + 4.0F, "Survive", gd::kMuted, gd::Hud::Style::Bold);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const std::string label = chipLabel(app, c);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, 24.0F, 506.0F, "Chips edit the rule, presets reseed. Drag on the grid: draw, right drag: erase.", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("lifelike-rule-space", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    reseedSoup(*app);
    for (int i = 0; i < kWarmup; ++i) stepOnce(*app);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            lrs::Cell c;
            if (app.painting) {
                if (cellAt(event->motion.x, event->motion.y, c)) {
                    paintTo(app, c);
                } else {
                    app.havePaintCell = false;  // outside the grid: the next cell inside starts a fresh stroke
                }
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            if (event->button.button != SDL_BUTTON_LEFT && event->button.button != SDL_BUTTON_RIGHT) break;
            if (event->button.button == SDL_BUTTON_LEFT) {
                bool onChip = false;
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) {
                        apply(app, c.action, c.arg);
                        onChip = true;
                        break;
                    }
                }
                if (onChip) break;
            }
            lrs::Cell c;
            if (!cellAt(p.x, p.y, c)) break;
            app.painting = true;
            app.havePaintCell = false;
            // A left press on a live cell erases the stroke, on a dead cell it draws. The right button always erases.
            app.paintValue = event->button.button == SDL_BUTTON_LEFT && !app.cur.alive(c.x, c.y);
            paintTo(app, c);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: stopPainting(app); break;
        case SDL_EVENT_KEY_DOWN:
            // N may repeat (hold it to scrub through generations), every other key toggles or restarts something.
            if (event->key.repeat && event->key.key != SDLK_N) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: case SDLK_2: case SDLK_3: case SDLK_4: case SDLK_5: apply(app, Action::Preset, static_cast<int>(event->key.key - SDLK_1)); break;
                case SDLK_SPACE: apply(app, Action::Pause, 0); break;
                case SDLK_N: apply(app, Action::Step, 0); break;
                case SDLK_F: apply(app, Action::Speed, 0); break;
                case SDLK_R: apply(app, Action::Soup, 0); break;
                case SDLK_E: apply(app, Action::Pentomino, 0); break;
                case SDLK_H: apply(app, Action::Replicator, 0); break;
                case SDLK_X: apply(app, Action::RandomRule, 0); break;
                case SDLK_C: apply(app, Action::Clear, 0); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(0.25, static_cast<double>(now - app.lastTicks) * 1e-9);
    app.lastTicks = now;
    if (!app.paused) {
        const double period = 1.0 / static_cast<double>(kSpeeds[static_cast<std::size_t>(app.speed)]);
        app.accumulator += dt;
        int steps = 0;
        while (app.accumulator >= period && steps < kMaxStepsPerFrame) {
            stepOnce(app);
            app.accumulator -= period;
            ++steps;
        }
        if (app.accumulator > period) app.accumulator = period;  // fell behind: drop the backlog instead of racing to catch up
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
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
