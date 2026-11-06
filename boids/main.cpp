// Boids with SDL3 main callbacks.
//
// Every bird follows three rules that only look at the birds within 50 px: keep apart from the ones that are too
// close (separation), fly the way they fly (alignment), drift towards their middle (cohesion). Nobody leads and
// nobody knows the flock, the sliders weigh the three rules. The mouse is a predator the birds flee from.
//
// The second thing to see is the neighbor search. Asking every bird about every other one is n^2 distance tests per
// step; a grid with cells as wide as the perception only asks the nine cells around a bird. The panel shows the
// tests per step and the measured time of a step for both, and "Show grid" draws the cells one bird looks at.
// The rules, the grid and the world (a torus, leaving on one side comes back on the other) live in logic.hpp.

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

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldTop = 32.0F;
constexpr float kFieldHeight = 370.0F;
constexpr float kStep = 1.0F / 60.0F;  // fixed simulation step
constexpr int kMaxStepsPerFrame = 3;   // a slow search slows the simulation down instead of freezing the page
constexpr int kPrerollSteps = 240;     // 4 s of flocking before the first frame
constexpr float kChipH = 30.0F;
constexpr float kChipX = 480.0F;
constexpr std::array<float, 3> kRowCenter{434.0F, 470.0F, 506.0F};
constexpr float kSliderX = 150.0F;
constexpr float kSliderW = 220.0F;
constexpr float kSliderMax = 3.0F;

constexpr gd::Color kInkSoft{17, 24, 39, 60};
constexpr gd::Color kCellTint{234, 88, 12, 38};
constexpr gd::Color kFearRing{234, 88, 12, 150};

enum class Action { SearchGrid, SearchBrute, ShowCells, Birds300, Birds600, Birds1000, Birds4000, Predator, Scatter };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct SliderDef {
    const char* label;
    float boids::Params::*weight;
};

constexpr std::array<SliderDef, 3> kSliders{{{"Separation", &boids::Params::separation},
                                             {"Alignment", &boids::Params::alignment},
                                             {"Cohesion", &boids::Params::cohesion}}};

double nowMs() {
#ifdef __EMSCRIPTEN__
    return emscripten_get_now();  // performance.now(), the performance counter is only millisecond-exact in the browser
#else
    return 1000.0 * static_cast<double>(SDL_GetPerformanceCounter()) / static_cast<double>(SDL_GetPerformanceFrequency());
#endif
}

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<SDL_Vertex> vertices;  // all birds as triangles, one draw call

    boids::Params params;
    boids::Flock flock{boids::World{static_cast<float>(kWidth), kFieldHeight}};
    boids::Search search = boids::Search::Grid;
    int count = 600;
    bool showCells = false;
    bool predatorOn = true;
    std::uint32_t seed = 1;

    SDL_FPoint mouse{-100.0F, -100.0F};
    bool mouseIn = false;
    int dragSlider = -1;

    double accumulator = 0.0;
    Uint64 lastNs = 0;
    double fps = 0.0;
    double stepMs = 0.0;
    bool statsValid = false;  // false right after the search or the bird count changed: the old numbers are not about this setup

    long maxFrames = -1;
    long frames = 0;
};

bool inField(SDL_FPoint p) { return p.y >= kFieldTop && p.y < kFieldTop + kFieldHeight && p.x >= 0.0F && p.x < static_cast<float>(kWidth); }

boids::Predator predatorOf(const App& app) {
    boids::Predator pr;
    pr.active = app.predatorOn && app.mouseIn && inField(app.mouse);
    pr.pos = {app.mouse.x, app.mouse.y - kFieldTop};
    return pr;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        int row;
        float gapBefore;
    };
    const std::array<Def, 9> defs{{{Action::SearchGrid, "G Grid search", 0, 0.0F},
                                   {Action::SearchBrute, "B Brute force", 0, 8.0F},
                                   {Action::ShowCells, "V Show grid", 0, 8.0F},
                                   {Action::Birds300, "1 300", 1, 0.0F},
                                   {Action::Birds600, "2 600", 1, 8.0F},
                                   {Action::Birds1000, "3 1000", 1, 8.0F},
                                   {Action::Birds4000, "4 4000", 1, 8.0F},
                                   {Action::Predator, "P Mouse = predator", 2, 0.0F},
                                   {Action::Scatter, "R Scatter", 2, 8.0F}}};
    float x = kChipX;
    int row = 0;
    for (const Def& d : defs) {
        if (d.row != row) {
            row = d.row;
            x = kChipX;
        }
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kRowCenter[static_cast<std::size_t>(d.row)] - kChipH / 2.0F, w, kChipH}});
        x += w;
    }
}

void resetStats(App& app) {
    app.statsValid = false;
    app.accumulator = 0.0;
}

void setCount(App& app, int count) {
    app.count = count;
    app.flock.setCount(count, app.params);
    resetStats(app);
}

void scatter(App& app) {
    ++app.seed;
    app.flock.reset(app.count, app.seed, app.params);
    resetStats(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::SearchGrid:
            if (app.search != boids::Search::Grid) resetStats(app);
            app.search = boids::Search::Grid;
            break;
        case Action::SearchBrute:
            if (app.search != boids::Search::Brute) resetStats(app);
            app.search = boids::Search::Brute;
            break;
        case Action::ShowCells: app.showCells = !app.showCells; break;
        case Action::Birds300: setCount(app, 300); break;
        case Action::Birds600: setCount(app, 600); break;
        case Action::Birds1000: setCount(app, 1000); break;
        case Action::Birds4000: setCount(app, 4000); break;
        case Action::Predator: app.predatorOn = !app.predatorOn; break;
        case Action::Scatter: scatter(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::SearchGrid: return app.search == boids::Search::Grid;
        case Action::SearchBrute: return app.search == boids::Search::Brute;
        case Action::ShowCells: return app.showCells;
        case Action::Birds300: return app.count == 300;
        case Action::Birds600: return app.count == 600;
        case Action::Birds1000: return app.count == 1000;
        case Action::Birds4000: return app.count == 4000;
        case Action::Predator: return app.predatorOn;
        case Action::Scatter: return false;
    }
    return false;
}

SDL_FRect sliderHit(std::size_t i) { return SDL_FRect{kSliderX - 8.0F, kRowCenter[i] - 14.0F, kSliderW + 16.0F, 28.0F}; }

void setSlider(App& app, int index, float mouseX) {
    const float v = boids::sliderValue(mouseX, kSliderX, kSliderW, 0.0F, kSliderMax);
    app.params.*(kSliders[static_cast<std::size_t>(index)].weight) = std::round(v * 10.0F) / 10.0F;  // steps of 0.1
}

void stepSimulation(App& app) {
    const Uint64 nowNs = SDL_GetTicksNS();
    const double dt = std::min(0.25, static_cast<double>(nowNs - app.lastNs) / 1e9);
    app.lastNs = nowNs;
    if (dt > 1e-6) {
        const double inst = 1.0 / dt;
        app.fps = app.statsValid ? app.fps + (inst - app.fps) * 0.1 : inst;
    }
    app.accumulator += dt;
    int steps = 0;
    double spent = 0.0;
    const boids::Predator predator = predatorOf(app);
    while (app.accumulator >= static_cast<double>(kStep) && steps < kMaxStepsPerFrame) {
        const double t0 = nowMs();
        app.flock.step(kStep, app.params, predator, app.search);
        spent += nowMs() - t0;
        app.accumulator -= static_cast<double>(kStep);
        ++steps;
    }
    if (steps == kMaxStepsPerFrame && app.accumulator >= static_cast<double>(kStep)) app.accumulator = 0.0;  // cannot keep up: drop the backlog
    if (steps > 0) {
        const double perStep = spent / steps;
        app.stepMs = app.statsValid ? app.stepMs + (perStep - app.stepMs) * 0.1 : perStep;
        app.statsValid = true;
    }
}

// ---- drawing --------------------------------------------------------------------------------------------------

SDL_FPoint toScreen(boids::Vec2 p) { return SDL_FPoint{p.x, p.y + kFieldTop}; }

void drawBirds(App& app) {
    const auto& birds = app.flock.birds();
    app.vertices.resize(birds.size() * 3);
    const SDL_FColor calm = gd::fcolor(gd::kTeal);
    const SDL_FColor afraid = gd::fcolor(gd::kOrange);
    for (std::size_t i = 0; i < birds.size(); ++i) {
        const boids::Bird& b = birds[i];
        const boids::Vec2 dir = boids::normalized(b.vel);
        const boids::Vec2 side{-dir.y, dir.x};
        const SDL_FColor c = b.scared ? afraid : calm;
        const SDL_FPoint tip = toScreen(b.pos + dir * 8.0F);
        const SDL_FPoint left = toScreen(b.pos - dir * 5.0F + side * 3.5F);
        const SDL_FPoint right = toScreen(b.pos - dir * 5.0F - side * 3.5F);
        app.vertices[i * 3 + 0] = SDL_Vertex{tip, c, {0, 0}};
        app.vertices[i * 3 + 1] = SDL_Vertex{left, c, {0, 0}};
        app.vertices[i * 3 + 2] = SDL_Vertex{right, c, {0, 0}};
    }
    if (!app.vertices.empty()) SDL_RenderGeometry(app.renderer, nullptr, app.vertices.data(), static_cast<int>(app.vertices.size()), nullptr, 0);
}

/// The cells the first bird looks at in grid mode, its perception circle and the birds that really are neighbors.
void drawProbe(App& app) {
    SDL_Renderer* r = app.renderer;
    const auto& birds = app.flock.birds();
    if (birds.empty()) return;
    const boids::World& world = app.flock.world();
    const boids::Bird& probe = birds[0];
    if (app.search == boids::Search::Grid) {
        const boids::Grid& g = app.flock.grid();
        for (int c = 0; c <= g.cols(); ++c) {
            const float x = std::min(static_cast<float>(c) * g.cellWidth(), static_cast<float>(kWidth));
            gd::fillRect(r, x, kFieldTop, 1.0F, kFieldHeight, kInkSoft);
        }
        for (int c = 0; c <= g.rows(); ++c) {
            const float y = std::min(static_cast<float>(c) * g.cellHeight(), kFieldHeight);
            gd::fillRect(r, 0.0F, kFieldTop + y, static_cast<float>(kWidth), 1.0F, kInkSoft);
        }
        int xs[3];
        int ys[3];
        const int nx = boids::Grid::block(g.cellX(probe.pos.x), g.cols(), xs);
        const int ny = boids::Grid::block(g.cellY(probe.pos.y), g.rows(), ys);
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                gd::fillRect(r, static_cast<float>(xs[ix]) * g.cellWidth(), kFieldTop + static_cast<float>(ys[iy]) * g.cellHeight(), g.cellWidth(), g.cellHeight(), kCellTint);
            }
        }
    }
    const float r2 = app.params.perception * app.params.perception;
    for (std::size_t i = 1; i < birds.size(); ++i) {
        const boids::Vec2 d{boids::wrapDelta(probe.pos.x, birds[i].pos.x, world.width), boids::wrapDelta(probe.pos.y, birds[i].pos.y, world.height)};
        if (boids::lengthSq(d) < r2) {
            const SDL_FPoint p = toScreen(birds[i].pos);
            gd::ring(r, p.x, p.y, 7.0F, gd::kInk, 16);
        }
    }
    const SDL_FPoint p = toScreen(probe.pos);
    gd::ring(r, p.x, p.y, app.params.perception, gd::kInk);
    gd::disc(r, p.x, p.y, 6.0F, gd::kOrange);
    gd::ring(r, p.x, p.y, 6.0F, gd::kInk, 16);
}

void drawPredator(App& app) {
    const boids::Predator pr = predatorOf(app);
    if (!pr.active) return;
    const SDL_FPoint p = toScreen(pr.pos);
    gd::ring(app.renderer, p.x, p.y, app.params.fearRadius, kFearRing);
    gd::disc(app.renderer, p.x, p.y, 9.0F, gd::kOrange);
    gd::ring(app.renderer, p.x, p.y, 9.0F, gd::kInk, 24);
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const bool grid = app.search == boids::Search::Grid;
    const long long n = static_cast<long long>(app.flock.birds().size());
    gd::card(r, 16.0F, kFieldTop + 10.0F, 266.0F, 108.0F, gd::kWhite, gd::kInk);
    char line[96];
    SDL_snprintf(line, sizeof line, "%lld birds, %s", n, grid ? "grid" : "brute force");
    app.hud.text(r, 28.0F, kFieldTop + 18.0F, line, gd::kInk, gd::Hud::Style::Bold);
    const std::string tests = app.statsValid ? boids::formatCount(app.flock.lastTests()) : "...";
    SDL_snprintf(line, sizeof line, "tests/step %s", tests.c_str());
    app.hud.text(r, 28.0F, kFieldTop + 40.0F, line, grid ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
    const std::string all = boids::formatCount(n * (n - 1));
    SDL_snprintf(line, sizeof line, "all pairs   %s", all.c_str());
    app.hud.text(r, 28.0F, kFieldTop + 62.0F, line, gd::kMuted);
    if (app.statsValid) {
        SDL_snprintf(line, sizeof line, "step %.2f ms  %.0f fps", app.stepMs, app.fps);
    } else {
        SDL_snprintf(line, sizeof line, "step ... ms  ... fps");
    }
    app.hud.text(r, 28.0F, kFieldTop + 84.0F, line, gd::kInk);
}

void drawSlider(App& app, std::size_t i) {
    SDL_Renderer* r = app.renderer;
    const SliderDef& def = kSliders[i];
    const float value = app.params.*(def.weight);
    const float cy = kRowCenter[i];
    app.hud.text(r, 24.0F, cy - 11.0F, def.label, gd::kInk, gd::Hud::Style::Bold);
    const float t = boids::sliderFraction(value, 0.0F, kSliderMax);
    gd::fillRect(r, kSliderX, cy - 2.0F, kSliderW, 4.0F, gd::kGrid);
    gd::strokeRect(r, kSliderX, cy - 2.0F, kSliderW, 4.0F, gd::kMuted);
    gd::fillRect(r, kSliderX, cy - 2.0F, kSliderW * t, 4.0F, gd::kTeal);
    const bool held = app.dragSlider == static_cast<int>(i);
    gd::card(r, kSliderX + kSliderW * t - 7.0F, cy - 12.0F, 14.0F, 24.0F, held ? gd::kTeal : gd::kWhite, gd::kInk);
    char text[16];
    SDL_snprintf(text, sizeof text, "%.1f", static_cast<double>(value));
    app.hud.text(r, kSliderX + kSliderW + 16.0F, cy - 11.0F, text, gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ BOIDS ]", "separation + alignment + cohesion");

    const SDL_Rect field{0, static_cast<int>(kFieldTop), kWidth, static_cast<int>(kFieldHeight)};
    SDL_SetRenderClipRect(r, &field);
    drawBirds(app);
    if (app.showCells) drawProbe(app);
    drawPredator(app);
    SDL_SetRenderClipRect(r, nullptr);
    gd::strokeRect(r, 0.5F, kFieldTop, static_cast<float>(kWidth) - 1.0F, kFieldHeight, gd::kInk);

    drawStats(app);
    for (std::size_t i = 0; i < kSliders.size(); ++i) drawSlider(app, i);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("boids", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    app->flock.reset(app->count, app->seed, app->params);
    // Let the flocks form before the first frame, otherwise it shows a random scatter.
    for (int i = 0; i < kPrerollSteps; ++i) app->flock.step(kStep, app->params, boids::Predator{}, boids::Search::Grid);
    app->lastNs = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {event->motion.x, event->motion.y};
            app.mouseIn = true;
            if (app.dragSlider >= 0) setSlider(app, app.dragSlider, app.mouse.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            app.mouseIn = true;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c.action);
            }
            for (std::size_t i = 0; i < kSliders.size(); ++i) {
                const SDL_FRect hit = sliderHit(i);
                if (SDL_PointInRectFloat(&p, &hit)) {
                    app.dragSlider = static_cast<int>(i);
                    setSlider(app, app.dragSlider, p.x);
                }
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragSlider = -1;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.dragSlider = -1;
            app.mouseIn = false;
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_G: apply(app, Action::SearchGrid); break;
                case SDLK_B: apply(app, Action::SearchBrute); break;
                case SDLK_V: apply(app, Action::ShowCells); break;
                case SDLK_1: apply(app, Action::Birds300); break;
                case SDLK_2: apply(app, Action::Birds600); break;
                case SDLK_3: apply(app, Action::Birds1000); break;
                case SDLK_4: apply(app, Action::Birds4000); break;
                case SDLK_P: apply(app, Action::Predator); break;
                case SDLK_R: apply(app, Action::Scatter); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    stepSimulation(app);
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
