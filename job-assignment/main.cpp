// Assigning workers to jobs with SDL3 main callbacks.
//
// Circles are workers, squares are jobs. Greedy lets every worker, one after the other, take the nearest job that is
// still free: it is quick, but the result depends on who goes first and the paths tend to cross. Optimal is the
// assignment problem solved by the Hungarian method: the shortest total distance, and by the triangle inequality a
// crossing can always be untied into a shorter pair, so the optimum has no crossings. Both totals are always on screen,
// next to the best and worst greedy over every possible order and a brute-force check. The scene is static, so there
// is no time step; everything is recomputed when something moves. The maths lives in logic.hpp, covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
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
constexpr int kMinCount = 2;
constexpr int kMaxCount = ja::kBruteForceMax;
constexpr double kGrab = 20.0;  // px around a worker or job that picks it up
constexpr float kWorkerRadius = 14.0F;
constexpr float kJobHalf = 12.0F;
constexpr float kChipY = 494.0F;
constexpr float kChipH = 34.0F;
constexpr float kCardY = 402.0F;
constexpr float kCardH = 74.0F;
constexpr gd::Color kOrangeSoft{255, 237, 213};

enum class Mode { Greedy, Optimal };
enum class Action { Greedy, Optimal, Reverse, Fewer, More, NewLayout };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

/// Everything derived from the layout, recomputed whenever something moves.
struct Result {
    ja::Costs costs;
    ja::Assignment greedy;
    ja::Assignment optimal;
    ja::Assignment brute;
    double greedyTotal = 0.0;
    double optimalTotal = 0.0;
    double bruteTotal = 0.0;
    std::vector<ja::Vec2> greedyCrossings;
    std::vector<ja::Vec2> optimalCrossings;
    ja::OrderRange range;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    ja::Layout layout = ja::initialLayout();
    Result result;
    Mode mode = Mode::Greedy;
    bool reversed = false;
    std::uint32_t nextSeed = 1000;
    ja::Vec2 mouse{-100.0, -100.0};
    std::optional<ja::Pick> dragging;
    long maxFrames = -1;
    long frames = 0;
};

ja::Vec2 toVec(SDL_FPoint p) { return {static_cast<double>(p.x), static_cast<double>(p.y)}; }

int count(const App& app) { return static_cast<int>(app.layout.workers.size()); }

void recompute(App& app) {
    Result& r = app.result;
    const ja::Layout& l = app.layout;
    r.costs = ja::costMatrix(l.workers, l.jobs);
    r.greedy = ja::greedy(r.costs, ja::processingOrder(count(app), app.reversed));
    r.optimal = ja::hungarian(r.costs);
    r.brute = ja::bruteForce(r.costs);
    r.greedyTotal = ja::totalCost(r.costs, r.greedy);
    r.optimalTotal = ja::totalCost(r.costs, r.optimal);
    r.bruteTotal = ja::totalCost(r.costs, r.brute);
    r.greedyCrossings = ja::crossings(l.workers, l.jobs, r.greedy);
    r.optimalCrossings = ja::crossings(l.workers, l.jobs, r.optimal);
    r.range = ja::greedyOrderRange(r.costs);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 6> defs{{{Action::Greedy, "1 Greedy", 0.0F},
                                   {Action::Optimal, "2 Optimal", 8.0F},
                                   {Action::Reverse, "3 Reverse order", 24.0F},
                                   {Action::Fewer, "4 Fewer", 24.0F},
                                   {Action::More, "5 More", 8.0F},
                                   {Action::NewLayout, "R New layout", 24.0F}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

bool enabled(const App& app, Action action) {
    switch (action) {
        case Action::Fewer: return count(app) > kMinCount;
        case Action::More: return count(app) < kMaxCount;
        default: return true;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Greedy: return app.mode == Mode::Greedy;
        case Action::Optimal: return app.mode == Mode::Optimal;
        case Action::Reverse: return app.reversed;
        default: return false;
    }
}

void apply(App& app, Action action) {
    if (!enabled(app, action)) return;
    app.dragging.reset();
    switch (action) {
        case Action::Greedy: app.mode = Mode::Greedy; break;
        case Action::Optimal: app.mode = Mode::Optimal; break;
        case Action::Reverse: app.reversed = !app.reversed; break;
        case Action::Fewer: app.layout = ja::resized(app.layout, count(app) - 1, app.nextSeed++, ja::kPlayField, ja::kMinGap); break;
        case Action::More: app.layout = ja::resized(app.layout, count(app) + 1, app.nextSeed++, ja::kPlayField, ja::kMinGap); break;
        case Action::NewLayout: app.layout = ja::randomLayout(app.nextSeed++, count(app), ja::kPlayField, ja::kMinGap); break;
    }
    recompute(app);
}

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, ja::Vec2 a, ja::Vec2 b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F),
                       static_cast<float>(b.x) + (flat ? 0.0F : off), static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

std::string px(double v) {
    char buf[32];
    SDL_snprintf(buf, sizeof buf, "%.0f", v);
    return buf;
}

/// Share by which `total` is longer than `best`, as "+27%"; empty when there is no sensible base.
std::string overhead(double total, double best) {
    if (!(best > 1e-9)) return {};
    char buf[32];
    SDL_snprintf(buf, sizeof buf, "%+.0f%%", (total / best - 1.0) * 100.0);
    return buf;
}

/// One half of the result card.
void drawColumn(App& app, float x, float w, const char* title, gd::Color titleColor, bool on, gd::Color tint, const std::string& line2,
                const std::string& line3) {
    SDL_Renderer* r = app.renderer;
    if (on) gd::fillRect(r, x + 1.0F, kCardY + 1.0F, w - 2.0F, kCardH - 2.0F, tint);
    app.hud.text(r, x + 14.0F, kCardY + 6.0F, title, titleColor, gd::Hud::Style::Bold);
    app.hud.text(r, x + 14.0F, kCardY + 28.0F, line2, gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, x + 14.0F, kCardY + 50.0F, line3, gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const Result& res = app.result;
    const bool greedyMode = app.mode == Mode::Greedy;
    const ja::Assignment& shown = greedyMode ? res.greedy : res.optimal;
    const auto& shownCrossings = greedyMode ? res.greedyCrossings : res.optimalCrossings;
    const ja::Layout& l = app.layout;
    const std::size_t n = l.workers.size();

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ JOB ASSIGNMENT ]", greedyMode ? "greedy: nearest free job" : "optimal: Hungarian method");

    // Paths, one per worker.
    for (std::size_t w = 0; w < shown.size(); ++w) {
        thickLine(r, l.workers[w], l.jobs[static_cast<std::size_t>(shown[w])], greedyMode ? gd::kOrange : gd::kTealDark, 1);
    }

    // Jobs.
    const std::optional<ja::Pick> hot = app.dragging ? app.dragging : ja::pickEntity(l, app.mouse, kGrab);
    for (std::size_t j = 0; j < n; ++j) {
        const auto x = static_cast<float>(l.jobs[j].x);
        const auto y = static_cast<float>(l.jobs[j].y);
        gd::fillRect(r, x - kJobHalf, y - kJobHalf, kJobHalf * 2.0F, kJobHalf * 2.0F, gd::kOrange);
        gd::strokeRect(r, x - kJobHalf, y - kJobHalf, kJobHalf * 2.0F, kJobHalf * 2.0F, gd::kInk);
        if (hot && !hot->worker && hot->index == j) gd::strokeRect(r, x - kJobHalf - 4.0F, y - kJobHalf - 4.0F, kJobHalf * 2.0F + 8.0F, kJobHalf * 2.0F + 8.0F, gd::kInk);
    }

    // Crossings of the path set that is on screen.
    for (const ja::Vec2& c : shownCrossings) {
        gd::disc(r, static_cast<float>(c.x), static_cast<float>(c.y), 7.0F, gd::kWhite);
        gd::ring(r, static_cast<float>(c.x), static_cast<float>(c.y), 7.0F, gd::kInk);
        gd::ring(r, static_cast<float>(c.x), static_cast<float>(c.y), 6.0F, gd::kInk);
    }

    // Workers, numbered by turn in greedy mode.
    const std::vector<int> order = ja::processingOrder(static_cast<int>(n), app.reversed);
    for (std::size_t w = 0; w < n; ++w) {
        const auto x = static_cast<float>(l.workers[w].x);
        const auto y = static_cast<float>(l.workers[w].y);
        gd::disc(r, x, y, kWorkerRadius, gd::kTeal);
        gd::ring(r, x, y, kWorkerRadius, gd::kInk);
        if (hot && hot->worker && hot->index == w) gd::ring(r, x, y, kWorkerRadius + 4.0F, gd::kInk);
        if (greedyMode) {
            const auto turn = std::find(order.begin(), order.end(), static_cast<int>(w)) - order.begin() + 1;
            const std::string label = std::to_string(turn);
            app.hud.text(r, x - app.hud.width(label, gd::Hud::Style::Bold) / 2.0F, y - app.hud.lineHeight(gd::Hud::Style::Bold) / 2.0F, label, gd::kWhite, gd::Hud::Style::Bold);
        }
    }

    app.hud.text(r, 24.0F, 374.0F, "drag circles (workers) and squares (jobs)", gd::kMuted);
    const std::string counts = std::to_string(n) + " workers, " + std::to_string(n) + " jobs";
    app.hud.text(r, static_cast<float>(kWidth) - 24.0F - app.hud.width(counts), 374.0F, counts, gd::kMuted);

    // Result card: both methods side by side, the one on screen is tinted.
    gd::card(r, 24.0F, kCardY, 912.0F, kCardH, gd::kWhite, gd::kInk);
    const float half = 456.0F;
    gd::setColor(r, gd::kInk);
    SDL_RenderLine(r, 24.0F + half, kCardY, 24.0F + half, kCardY + kCardH);

    const std::string gOver = overhead(res.greedyTotal, res.optimalTotal);
    drawColumn(app, 24.0F, half, "GREEDY  nearest free job, in turn", gd::kOrange, greedyMode, kOrangeSoft,
               "length " + px(res.greedyTotal) + " px  " + gOver + "  crossings " + std::to_string(res.greedyCrossings.size()),
               "all " + std::to_string(res.range.orders) + " orders: " + px(res.range.best) + " to " + px(res.range.worst));
    drawColumn(app, 24.0F + half, half, "OPTIMAL  Hungarian method", gd::kTealDark, !greedyMode, gd::kTealSoft,
               "length " + px(res.optimalTotal) + " px  crossings " + std::to_string(res.optimalCrossings.size()),
               "brute force over " + std::to_string(res.range.orders) + ": " + px(res.bruteTotal));

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool usable = enabled(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, usable ? gd::kInk : gd::kMuted);
        const gd::Color text = on ? gd::kWhite : usable ? gd::kInk : gd::kMuted;
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, text, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("job-assignment", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    recompute(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = toVec({event->motion.x, event->motion.y});
            if (app.dragging) {
                const ja::Vec2 p = ja::clampToField(app.mouse, ja::kPlayField);
                (app.dragging->worker ? app.layout.workers : app.layout.jobs)[app.dragging->index] = p;
                recompute(app);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = toVec(p);
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                    break;
                }
            }
            if (!onChip) app.dragging = ja::pickEntity(app.layout, app.mouse, kGrab);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging.reset(); break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here toggles or rebuilds the scene, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Greedy); break;
                case SDLK_2: apply(app, Action::Optimal); break;
                case SDLK_3: apply(app, Action::Reverse); break;
                case SDLK_4: apply(app, Action::Fewer); break;
                case SDLK_5: apply(app, Action::More); break;
                case SDLK_R: apply(app, Action::NewLayout); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
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
