// Poisson disc sampling with SDL3 main callbacks.
//
// The same number of points three ways: uniform random, a jittered grid, and Bridson's Poisson disc. Each point is
// drawn with a disc of radius r/2, so two points closer than r show as overlapping discs and an orange line. The
// right panel grows live (grid cells, active points, the ring of candidates around the point being worked on), the
// other two reveal as many points as the right one has, so all three stay comparable. The count comes from a
// finished reference run with the same seed and r/k, which is why the left panels never have more points than the
// right one would end up with. Painting in the right panel lowers the local radius (denser trees), the left panels
// stay uniform on purpose: they show what random looks like at that r. The algorithm and the measuring are in
// logic.hpp and covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr double kDomW = 304.0;
constexpr double kDomH = 304.0;
constexpr float kDomY = 64.0F;
constexpr float kPanelStep = 312.0F;
constexpr float kPanelX0 = 16.0F;
constexpr double kBrush = 36.0;
constexpr int kPrerunSteps = 120;  // steps done before the first frame, so there is already something to see
constexpr int kMaxStepsPerFrame = 400;
constexpr float kChipY = 482.0F;
constexpr float kChipH = 34.0F;
constexpr std::array<double, 3> kSpeeds{60.0, 240.0, 1500.0};  // steps per second
constexpr std::array<const char*, 3> kSpeedNames{"slow", "normal", "fast"};
constexpr std::array<int, 6> kKValues{2, 3, 5, 10, 30, 60};
constexpr double kRadiusMin = 8.0;
constexpr double kRadiusMax = 40.0;

enum class Action { RadiusDown, RadiusUp, KDown, KUp, Grid, Discs, Replay, NewSeed, Speed, Clear };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

/// What was measured on the visible points of one panel, recomputed only when that set changes.
struct PanelStats {
    std::size_t shown = std::numeric_limits<std::size_t>::max();
    std::vector<pd::Pair> pairs;
    std::vector<char> flagged;
    double minSep = std::numeric_limits<double>::infinity();
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    pd::Params params{kDomW, kDomH, 16.0, kKValues[4]};
    std::size_t kIndex = 4;
    std::uint64_t seed = 7;
    pd::DensityMap map{kDomW, kDomH};
    std::optional<pd::Sampler> live;
    std::size_t refCount = 0;
    std::vector<pd::Vec2> randomPts;
    std::vector<pd::Vec2> jitterPts;
    std::vector<double> flatRadii;
    std::array<PanelStats, 3> stats;

    int speed = 1;
    bool showGrid = false;
    bool showDiscs = true;
    double stepAcc = 0.0;
    Uint64 lastTicks = 0;

    pd::Vec2 mouse{-100.0, -100.0};
    bool painting = false;
    bool erasing = false;
    bool dirty = false;  // the map changed, finish the right panel right away

    long maxFrames = -1;
    long frames = 0;
};

float panelX(int i) { return kPanelX0 + kPanelStep * static_cast<float>(i); }

/// Mouse position in the domain of panel i, or nothing when the cursor is elsewhere.
std::optional<pd::Vec2> inPanel(const App& app, int i) {
    const double x = app.mouse.x - static_cast<double>(panelX(i));
    const double y = app.mouse.y - static_cast<double>(kDomY);
    if (x < 0.0 || y < 0.0 || x >= kDomW || y >= kDomH) return std::nullopt;
    return pd::Vec2{x, y};
}

void invalidateStats(App& app) {
    for (PanelStats& s : app.stats) s.shown = std::numeric_limits<std::size_t>::max();
}

/// Reference run on a flat map: its point count is what the random and jittered panels get.
void rebuildNaive(App& app) {
    pd::Sampler ref(app.params, pd::DensityMap(kDomW, kDomH), app.seed);
    ref.run();
    app.refCount = ref.points().size();
    pd::Rng rng(app.seed * 7919U + 13U);
    app.randomPts = pd::randomPoints(app.refCount, kDomW, kDomH, rng);
    app.jitterPts = pd::jitteredGrid(app.refCount, kDomW, kDomH, rng);
    app.flatRadii.assign(std::max(app.randomPts.size(), app.jitterPts.size()), app.params.radius);
    invalidateStats(app);
}

/// Start the live sampler over with the current map and run a few steps so the first frame is not empty.
void restart(App& app) {
    app.live.emplace(app.params, app.map, app.seed);
    for (int i = 0; i < kPrerunSteps && app.live->step(); ++i) {
    }
    app.stepAcc = 0.0;
    app.dirty = false;
    invalidateStats(app);
}

/// While painting the right panel is finished in one go, so the dots follow the brush.
void finishNow(App& app) {
    app.live.emplace(app.params, app.map, app.seed);
    app.live->run();
    app.stepAcc = 0.0;
    app.dirty = false;
    invalidateStats(app);
}

void rebuildAll(App& app) {
    rebuildNaive(app);
    restart(app);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 10> defs{{{Action::RadiusDown, "r -", 0.0F},
                                    {Action::RadiusUp, "r +", 6.0F},
                                    {Action::KDown, "k -", 14.0F},
                                    {Action::KUp, "k +", 6.0F},
                                    {Action::Grid, "G Grid", 24.0F},
                                    {Action::Discs, "D Discs", 6.0F},
                                    {Action::Replay, "R Replay", 24.0F},
                                    {Action::NewSeed, "N New", 6.0F},
                                    {Action::Speed, "S Speed", 24.0F},
                                    {Action::Clear, "C Clear", 6.0F}}};
    float x = 16.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::RadiusDown:
            app.params.radius = std::max(kRadiusMin, app.params.radius - 2.0);
            rebuildAll(app);
            break;
        case Action::RadiusUp:
            app.params.radius = std::min(kRadiusMax, app.params.radius + 2.0);
            rebuildAll(app);
            break;
        case Action::KDown:
            if (app.kIndex > 0) --app.kIndex;
            app.params.k = kKValues[app.kIndex];
            rebuildAll(app);
            break;
        case Action::KUp:
            if (app.kIndex + 1 < kKValues.size()) ++app.kIndex;
            app.params.k = kKValues[app.kIndex];
            rebuildAll(app);
            break;
        case Action::Grid: app.showGrid = !app.showGrid; break;
        case Action::Discs: app.showDiscs = !app.showDiscs; break;
        case Action::Replay: restart(app); break;
        case Action::NewSeed:
            ++app.seed;
            rebuildAll(app);
            break;
        case Action::Speed: app.speed = (app.speed + 1) % static_cast<int>(kSpeeds.size()); break;
        case Action::Clear:
            app.map.clear();
            restart(app);
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Grid: return app.showGrid;
        case Action::Discs: return app.showDiscs;
        case Action::Clear: return !app.map.isUniform();
        default: return false;
    }
}

void refreshStats(PanelStats& st, std::size_t shown, const std::vector<pd::Vec2>& pts, const std::vector<double>& radii) {
    if (st.shown == shown) return;
    st.shown = shown;
    st.pairs = pd::closePairs(pts, radii, shown);
    st.minSep = pd::minSeparation(pts, radii, shown);
    st.flagged.assign(shown, 0);
    for (const pd::Pair& p : st.pairs) {
        st.flagged[p.a] = 1;
        st.flagged[p.b] = 1;
    }
}

void drawText(App& app, float x, float y, const char* text, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, x, y, text, c, style);
}

/// Points, discs and the orange lines between points that are too close. `ox`/`oy` is the panel origin.
void drawPoints(App& app, float ox, float oy, const std::vector<pd::Vec2>& pts, const std::vector<double>& radii, std::size_t shown,
                const PanelStats& st, gd::Color discColor, gd::Color dotColor, const std::vector<char>* activeFlags) {
    SDL_Renderer* r = app.renderer;
    if (app.showDiscs) {
        for (std::size_t i = 0; i < shown; ++i) {
            gd::disc(r, ox + static_cast<float>(pts[i].x), oy + static_cast<float>(pts[i].y), static_cast<float>(radii[i] / 2.0), discColor, 24);
        }
    }
    gd::setColor(r, gd::Color{234, 88, 12, 210});
    for (const pd::Pair& p : st.pairs) {
        const float x0 = ox + static_cast<float>(pts[p.a].x);
        const float y0 = oy + static_cast<float>(pts[p.a].y);
        const float x1 = ox + static_cast<float>(pts[p.b].x);
        const float y1 = oy + static_cast<float>(pts[p.b].y);
        SDL_RenderLine(r, x0, y0, x1, y1);
        SDL_RenderLine(r, x0 + 1.0F, y0, x1 + 1.0F, y1);
    }
    for (std::size_t i = 0; i < shown; ++i) {
        const bool flagged = i < st.flagged.size() && st.flagged[i] != 0;
        const bool isActive = activeFlags != nullptr && i < activeFlags->size() && (*activeFlags)[i] != 0;
        const gd::Color c = (flagged || isActive) ? gd::kOrange : dotColor;
        gd::disc(r, ox + static_cast<float>(pts[i].x), oy + static_cast<float>(pts[i].y), isActive ? 4.0F : 3.0F, c, 12);
    }
}

void drawInfo(App& app, int panel, float x, std::size_t shown, const PanelStats& st, bool good, const char* suffix) {
    char line[96];
    std::snprintf(line, sizeof line, "%zu pts   %zu too close", shown, st.pairs.size());
    drawText(app, x, kDomY + static_cast<float>(kDomH) + 6.0F, line, good ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
    if (std::isinf(st.minSep)) {
        std::snprintf(line, sizeof line, "closest -%s", suffix);
    } else {
        std::snprintf(line, sizeof line, "closest %.2f r%s", st.minSep, suffix);
    }
    drawText(app, x, kDomY + static_cast<float>(kDomH) + 28.0F, line, gd::kMuted);
    (void)panel;
}

void drawPoissonExtras(App& app, float ox, float oy) {
    SDL_Renderer* r = app.renderer;
    const pd::Sampler& s = *app.live;
    if (app.showGrid) {
        const auto cell = static_cast<float>(s.cellSize());
        for (int cy = 0; cy < s.rows(); ++cy) {
            for (int cx = 0; cx < s.cols(); ++cx) {
                if (s.grid()[static_cast<std::size_t>(cy * s.cols() + cx)] >= 0) {
                    gd::fillRect(r, ox + static_cast<float>(cx) * cell, oy + static_cast<float>(cy) * cell, cell, cell, gd::Color{204, 251, 241, 190});
                }
            }
        }
        gd::setColor(r, gd::Color{17, 24, 39, 38});
        for (int cx = 0; cx <= s.cols(); ++cx) SDL_RenderLine(r, ox + static_cast<float>(cx) * cell, oy, ox + static_cast<float>(cx) * cell, oy + static_cast<float>(kDomH));
        for (int cy = 0; cy <= s.rows(); ++cy) SDL_RenderLine(r, ox, oy + static_cast<float>(cy) * cell, ox + static_cast<float>(kDomW), oy + static_cast<float>(cy) * cell);
    }
}

void drawTrials(App& app, float ox, float oy) {
    SDL_Renderer* r = app.renderer;
    const pd::Sampler& s = *app.live;
    if (s.done() || s.lastTrials().empty()) return;
    const pd::Vec2 parent = s.points()[s.lastActive()];
    const double ra = s.radii()[s.lastActive()];
    const float px = ox + static_cast<float>(parent.x);
    const float py = oy + static_cast<float>(parent.y);
    // The ring the candidates are drawn from: between r and 2r around the active point.
    gd::ring(r, px, py, static_cast<float>(ra), gd::Color{234, 88, 12, 150}, 40);
    gd::ring(r, px, py, static_cast<float>(2.0 * ra), gd::Color{234, 88, 12, 150}, 56);
    for (const pd::Trial& t : s.lastTrials()) {
        const float x = ox + static_cast<float>(t.pos.x);
        const float y = oy + static_cast<float>(t.pos.y);
        if (t.accepted) {
            gd::ring(r, x, y, 6.0F, gd::kTealDark, 16);
            gd::ring(r, x, y, 7.0F, gd::kTealDark, 16);
        } else {
            gd::setColor(r, gd::kOrange);
            SDL_RenderLine(r, x - 3.0F, y - 3.0F, x + 3.0F, y + 3.0F);
            SDL_RenderLine(r, x - 3.0F, y + 3.0F, x + 3.0F, y - 3.0F);
        }
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const pd::Sampler& s = *app.live;
    const std::size_t liveN = s.points().size();
    const bool growing = !s.done();

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ POISSON DISC SAMPLING ]", "Bridson: ring of k candidates, grid of r/sqrt(2)");

    const std::size_t shownRandom = std::min(app.randomPts.size(), pd::revealCount(liveN, app.randomPts.size(), app.refCount));
    const std::size_t shownJitter = std::min(app.jitterPts.size(), pd::revealCount(liveN, app.jitterPts.size(), app.refCount));
    refreshStats(app.stats[0], shownRandom, app.randomPts, app.flatRadii);
    refreshStats(app.stats[1], shownJitter, app.jitterPts, app.flatRadii);
    refreshStats(app.stats[2], liveN, s.points(), s.radii());

    const char* titles[3] = {"RANDOM", "JITTERED GRID", "POISSON DISC"};
    for (int i = 0; i < 3; ++i) {
        const float x = panelX(i);
        drawText(app, x, 38.0F, titles[i], i == 2 ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
        gd::fillRect(r, x, kDomY, static_cast<float>(kDomW), static_cast<float>(kDomH), gd::kWhite);
    }

    const SDL_Rect clip0{static_cast<int>(panelX(0)), static_cast<int>(kDomY), static_cast<int>(kDomW), static_cast<int>(kDomH)};
    const SDL_Rect clip1{static_cast<int>(panelX(1)), clip0.y, clip0.w, clip0.h};
    const SDL_Rect clip2{static_cast<int>(panelX(2)), clip0.y, clip0.w, clip0.h};
    const gd::Color naiveDisc{17, 24, 39, 30};
    const gd::Color poissonDisc{13, 148, 136, 70};

    SDL_SetRenderClipRect(r, &clip0);
    drawPoints(app, panelX(0), kDomY, app.randomPts, app.flatRadii, shownRandom, app.stats[0], naiveDisc, gd::kInk, nullptr);
    SDL_SetRenderClipRect(r, &clip1);
    drawPoints(app, panelX(1), kDomY, app.jitterPts, app.flatRadii, shownJitter, app.stats[1], naiveDisc, gd::kInk, nullptr);
    SDL_SetRenderClipRect(r, &clip2);
    drawPoissonExtras(app, panelX(2), kDomY);
    std::vector<char> activeFlags(liveN, 0);
    for (std::size_t a : s.active()) activeFlags[a] = 1;
    drawPoints(app, panelX(2), kDomY, s.points(), s.radii(), liveN, app.stats[2], poissonDisc, gd::kTealDark, &activeFlags);
    drawTrials(app, panelX(2), kDomY);
    if (const auto m = inPanel(app, 2)) {
        gd::ring(r, panelX(2) + static_cast<float>(m->x), kDomY + static_cast<float>(m->y), static_cast<float>(kBrush), gd::Color{17, 24, 39, 120}, 40);
    }
    SDL_SetRenderClipRect(r, nullptr);

    for (int i = 0; i < 3; ++i) {
        gd::strokeRect(r, panelX(i), kDomY, static_cast<float>(kDomW), static_cast<float>(kDomH), i == 2 ? gd::kTeal : gd::kInk);
    }

    drawInfo(app, 0, panelX(0), shownRandom, app.stats[0], app.stats[0].pairs.empty(), "");
    drawInfo(app, 1, panelX(1), shownJitter, app.stats[1], app.stats[1].pairs.empty(), "");
    char suffix[48];
    const double perPoint = liveN > 0 ? static_cast<double>(s.candidatesTested()) / static_cast<double>(liveN) : 0.0;
    std::snprintf(suffix, sizeof suffix, "  %.0f tests/pt", perPoint);
    drawInfo(app, 2, panelX(2), liveN, app.stats[2], app.stats[2].pairs.empty(), suffix);

    char line[160];
    std::snprintf(line, sizeof line, "r = %.0f px   k = %d   cell = %.1f px   speed %s%s%s", app.params.radius, app.params.k, s.cellSize(), kSpeedNames[static_cast<std::size_t>(app.speed)],
                  app.map.isUniform() ? "" : "   painted", growing ? "   growing" : "   done");
    drawText(app, 16.0F, 432.0F, line, gd::kInk, gd::Hud::Style::Bold);
    drawText(app, 16.0F, 456.0F, "drag in the right panel: denser   right drag: erase   arrow keys: r (left/right), k (down/up)", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void paintAt(App& app) {
    const auto m = inPanel(app, 2);
    if (!m) return;
    if (app.painting) app.map.paint(*m, kBrush);
    if (app.erasing) app.map.erase(*m, kBrush);
    app.dirty = true;
}

void stopDragging(App& app) {
    app.painting = false;
    app.erasing = false;
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
    if (!SDL_CreateWindowAndRenderer("poisson-disc", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    rebuildAll(*app);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            if (app.painting || app.erasing) paintAt(app);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = {static_cast<double>(p.x), static_cast<double>(p.y)};
            if (event->button.button == SDL_BUTTON_LEFT) {
                bool onChip = false;
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) {
                        apply(app, c.action);
                        onChip = true;
                    }
                }
                if (!onChip && inPanel(app, 2)) {
                    app.painting = true;
                    app.erasing = false;
                    paintAt(app);
                }
            } else if (event->button.button == SDL_BUTTON_RIGHT && inPanel(app, 2)) {
                app.erasing = true;
                app.painting = false;
                paintAt(app);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.painting = false;
            if (event->button.button == SDL_BUTTON_RIGHT) app.erasing = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            stopDragging(app);
            app.mouse = {-100.0, -100.0};
            break;
        case SDL_EVENT_KEY_DOWN:
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                // Scrubbing r and k is meant to repeat.
                case SDLK_LEFT: apply(app, Action::RadiusDown); break;
                case SDLK_RIGHT: apply(app, Action::RadiusUp); break;
                case SDLK_DOWN: apply(app, Action::KDown); break;
                case SDLK_UP: apply(app, Action::KUp); break;
                default: break;
            }
            if (!event->key.repeat) {
                switch (event->key.key) {
                    case SDLK_G: apply(app, Action::Grid); break;
                    case SDLK_D: apply(app, Action::Discs); break;
                    case SDLK_R: apply(app, Action::Replay); break;
                    case SDLK_N: apply(app, Action::NewSeed); break;
                    case SDLK_S: apply(app, Action::Speed); break;
                    case SDLK_C: apply(app, Action::Clear); break;
                    default: break;
                }
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = static_cast<double>(now - app.lastTicks) * 1e-9;
    app.lastTicks = now;

    if (app.dirty) {
        finishNow(app);
    } else if (!app.live->done()) {
        const int n = pd::stepsDue(app.stepAcc, dt, kSpeeds[static_cast<std::size_t>(app.speed)], kMaxStepsPerFrame);
        for (int i = 0; i < n && app.live->step(); ++i) {
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
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
