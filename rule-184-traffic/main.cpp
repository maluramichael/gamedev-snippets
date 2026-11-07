// Traffic on a ring road with rule 184, SDL3 main callbacks.
//
// Each row of the big picture is one time step, the newest at the bottom, cars drive to the right and the right edge is the
// left edge. Teal cars have a free cell ahead, orange cars have to wait. Below a density of one half the first picture
// still has jams and they dissolve until every car drives. Above it they never do, the jam waves run backwards against the
// driving direction and the cars keep their number. The chart on the right plots speed over density: the line is the
// formula min(1, (1 - d) / d), the small dots were measured by this very code at start-up, the big dot is the road you are
// looking at. Chip 2 updates the road in place (one sweep over one array) to show the classic mistake. The rule, the
// measurement and the mistake live in logic.hpp and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr std::size_t kCells = 180;
constexpr std::size_t kRows = 50;
constexpr float kCell = 5.0F;
constexpr float kDiagX = 30.0F;
constexpr float kDiagY = 40.0F;
constexpr float kDiagW = static_cast<float>(kCells) * kCell;
constexpr float kDiagH = static_cast<float>(kRows) * kCell;
constexpr float kRoadY = kDiagY + kDiagH + 6.0F;
constexpr float kRoadH = 16.0F;
constexpr float kHintY = 318.0F;
constexpr float kPanelY = 346.0F;
constexpr float kPanelH = 136.0F;
constexpr float kChipY = 492.0F;
constexpr float kChipH = 32.0F;

constexpr float kTrackX = 140.0F;
constexpr float kTrackW = 260.0F;
constexpr float kTrackCy = 365.0F;
constexpr double kDensityLo = 0.05;
constexpr double kDensityHi = 0.95;
constexpr std::size_t kStartCars = 108;  // density 0.6
constexpr std::size_t kKeyStepCars = 2;

constexpr float kPlotX = 632.0F;
constexpr float kPlotY = 378.0F;
constexpr float kPlotW = 282.0F;
constexpr float kPlotH = 70.0F;

constexpr std::array<double, 3> kStepsPerSecond{{8.0, 15.0, 30.0}};
constexpr int kMaxStepsPerFrame = 12;
constexpr std::size_t kWindowSteps = 30;
constexpr std::size_t kMinWindowSteps = 10;
constexpr gd::Color kInkSoft{17, 24, 39, 45};

enum class Action { PlayPause, Step, Random, Queue, AllAtOnce, InPlace, Speed };
enum class Update { AllAtOnce, InPlace };

struct Chip {
    Action action;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    r184::Road road;
    std::deque<r184::Road> history;  // oldest first, the last row is the road as it is now
    r184::Rng rng{1};
    r184::SpeedWindow speed{kWindowSteps};
    std::vector<std::pair<double, double>> measured;  // density, speed
    Update update = Update::AllAtOnce;
    bool paused = false;
    bool dragSlider = false;
    std::size_t speedIndex = 1;
    double accumulator = 0.0;
    double lastMs = 0.0;
    float mouseX = -100.0F;
    float mouseY = -100.0F;
    long maxFrames = -1;
    long frames = 0;
};

double nowMs() {
#ifdef __EMSCRIPTEN__
    return emscripten_get_now();  // SDL_GetPerformanceCounter only has millisecond steps in the browser
#else
    return static_cast<double>(SDL_GetPerformanceCounter()) * 1000.0 / static_cast<double>(SDL_GetPerformanceFrequency());
#endif
}

std::size_t cars(const App& app) { return static_cast<std::size_t>(r184::carCount(app.road)); }
double density(const App& app) { return static_cast<double>(cars(app)) / static_cast<double>(kCells); }

void recordRow(App& app) {
    app.history.push_back(app.road);
    while (app.history.size() > kRows) app.history.pop_front();
}

void stepOnce(App& app) {
    const int hops = app.update == Update::AllAtOnce ? r184::stepParallel(app.road) : r184::stepInPlace(app.road);
    app.speed.push(hops, r184::carCount(app.road));
    recordRow(app);
}

/// A new road: the picture starts empty and fills from the top, the measurement starts over.
void restart(App& app, r184::Road road) {
    app.road = std::move(road);
    app.history.clear();
    recordRow(app);
    app.speed.clear();
    app.accumulator = 0.0;
}

/// The road was edited by hand: show it in the newest row at once, and measure again from here.
void edited(App& app) {
    app.history.back() = app.road;
    app.speed.clear();
}

void setCars(App& app, std::size_t target) {
    if (target == cars(app)) return;
    r184::setCarCount(app.road, target, app.rng);
    edited(app);
}

void setSliderTo(App& app, float mouseX) {
    const double t = std::clamp(static_cast<double>(mouseX - kTrackX) / static_cast<double>(kTrackW), 0.0, 1.0);
    const double d = kDensityLo + t * (kDensityHi - kDensityLo);
    setCars(app, static_cast<std::size_t>(std::lround(d * static_cast<double>(kCells))));
}

bool onSlider(float x, float y) { return x >= kTrackX - 10.0F && x <= kTrackX + kTrackW + 10.0F && y >= kTrackCy - 12.0F && y <= kTrackCy + 12.0F; }

/// Column of the road under the mouse, or -1 when the mouse is not over the picture or the road strip.
int hoverColumn(const App& app) {
    if (app.mouseX < kDiagX || app.mouseX >= kDiagX + kDiagW || app.mouseY < kDiagY || app.mouseY >= kRoadY + kRoadH) return -1;
    return std::min(static_cast<int>(kCells) - 1, static_cast<int>((app.mouseX - kDiagX) / kCell));
}

std::string labelFor(const App& app, Action action) {
    char speedLabel[24];
    switch (action) {
        case Action::PlayPause: return app.paused ? "Space Play" : "Space Pause";
        case Action::Step: return ". Step";
        case Action::Random: return "R Random";
        case Action::Queue: return "B Queue";
        case Action::AllAtOnce: return "1 All at once";
        case Action::InPlace: return "2 In place";
        case Action::Speed:
            SDL_snprintf(speedLabel, sizeof speedLabel, "S Speed %d/s", static_cast<int>(kStepsPerSecond[app.speedIndex]));
            return speedLabel;
    }
    return {};
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::PlayPause: return app.paused;
        case Action::AllAtOnce: return app.update == Update::AllAtOnce;
        case Action::InPlace: return app.update == Update::InPlace;
        default: return false;
    }
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* widest;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::PlayPause, "Space Pause", 0.0F},
                                   {Action::Step, ". Step", 6.0F},
                                   {Action::Random, "R Random", 14.0F},
                                   {Action::Queue, "B Queue", 6.0F},
                                   {Action::AllAtOnce, "1 All at once", 14.0F},
                                   {Action::InPlace, "2 In place", 6.0F},
                                   {Action::Speed, "S Speed 30/s", 14.0F}}};
    float x = kDiagX;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.widest, gd::Hud::Style::Bold) + 16.0F;
        app.chips.push_back({d.action, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void setUpdate(App& app, Update update) {
    if (app.update == update) return;
    app.update = update;
    app.speed.clear();
    app.accumulator = 0.0;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::PlayPause:
            app.paused = !app.paused;
            app.accumulator = 0.0;
            break;
        case Action::Step:
            app.paused = true;
            app.accumulator = 0.0;
            stepOnce(app);
            break;
        case Action::Random: restart(app, r184::randomRoad(kCells, cars(app), app.rng)); break;
        case Action::Queue: restart(app, r184::blockRoad(kCells, cars(app), kCells / 4)); break;
        case Action::AllAtOnce: setUpdate(app, Update::AllAtOnce); break;
        case Action::InPlace: setUpdate(app, Update::InPlace); break;
        case Action::Speed:
            app.speedIndex = (app.speedIndex + 1) % kStepsPerSecond.size();
            app.accumulator = 0.0;
            break;
    }
}

/// Fills the picture and the speed window with a few steps so the very first frame already shows traffic.
void warmUp(App& app) {
    for (std::size_t i = 0; i + 1 < kRows; ++i) stepOnce(app);
}

void measureChart(App& app) {
    for (std::size_t k = 18; k <= 171; k += 9) {
        const double d = static_cast<double>(k) / static_cast<double>(kCells);
        app.measured.emplace_back(d, r184::measureSpeed(kCells, k, 1, 4 * static_cast<int>(kCells), 100));
    }
}

/// Horizontal runs of the same color become one rectangle, a full road would be thousands of calls otherwise.
void drawRow(SDL_Renderer* r, const r184::Road& row, float y) {
    const std::size_t n = row.size();
    std::size_t x = 0;
    while (x < n) {
        if (row[x] == 0) {
            ++x;
            continue;
        }
        const bool moving = row[(x + 1) % n] == 0;
        std::size_t end = x + 1;
        while (end < n && row[end] != 0 && (row[(end + 1) % n] == 0) == moving) ++end;
        gd::fillRect(r, kDiagX + static_cast<float>(x) * kCell, y, static_cast<float>(end - x) * kCell, kCell, moving ? gd::kTeal : gd::kOrange);
        x = end;
    }
}

void drawRoadStrip(SDL_Renderer* r, const r184::Road& road) {
    gd::fillRect(r, kDiagX, kRoadY, kDiagW, kRoadH, gd::kWhite);
    const std::size_t n = road.size();
    for (std::size_t x = 0; x < n; ++x) {
        if (road[x] == 0) continue;
        const bool moving = road[(x + 1) % n] == 0;
        gd::fillRect(r, kDiagX + static_cast<float>(x) * kCell, kRoadY + 3.0F, kCell, kRoadH - 6.0F, moving ? gd::kTeal : gd::kOrange);
    }
    gd::strokeRect(r, kDiagX, kRoadY, kDiagW, kRoadH, gd::kInk);
}

void drawSlider(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, 44.0F, 354.0F, "Density", gd::kInk, gd::Hud::Style::Bold);
    const double span = kDensityHi - kDensityLo;
    const double t = std::clamp((density(app) - kDensityLo) / span, 0.0, 1.0);
    const float knobX = kTrackX + static_cast<float>(t) * kTrackW;
    gd::fillRect(r, kTrackX, kTrackCy - 2.0F, kTrackW, 4.0F, gd::kGrid);
    gd::fillRect(r, kTrackX, kTrackCy - 2.0F, knobX - kTrackX, 4.0F, gd::kTeal);
    const float half = kTrackX + static_cast<float>((0.5 - kDensityLo) / span) * kTrackW;
    gd::fillRect(r, half - 1.0F, kTrackCy - 9.0F, 2.0F, 18.0F, gd::kOrange);  // one half: where the jams stop dissolving
    gd::card(r, knobX - 6.0F, kTrackCy - 9.0F, 12.0F, 18.0F, app.dragSlider ? gd::kTeal : gd::kWhite, gd::kInk);
    char value[16];
    SDL_snprintf(value, sizeof value, "%.2f", density(app));
    app.hud.text(r, kTrackX + kTrackW + 16.0F, 354.0F, value, gd::kTealDark, gd::Hud::Style::Bold);
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const int k = r184::carCount(app.road);
    const int moving = r184::movingCars(app.road);
    const bool inPlace = app.update == Update::InPlace;

    const char* verdict = k == 0 ? "EMPTY ROAD" : moving == k ? "ALL CARS DRIVE" : "STOP AND GO";
    const gd::Color verdictColor = k == 0 ? gd::kMuted : moving == k ? gd::kTeal : gd::kOrange;
    app.hud.text(r, 44.0F, 380.0F, verdict, verdictColor, gd::Hud::Style::Large);

    char line[96];
    SDL_snprintf(line, sizeof line, "cars %d of %d   driving %d   waiting %d", k, static_cast<int>(kCells), moving, k - moving);
    app.hud.text(r, 44.0F, 430.0F, line, gd::kInk);

    // speed, measured over the last steps next to the formula
    float x = 44.0F;
    const float y = 454.0F;
    auto put = [&](const char* s, gd::Color c, gd::Hud::Style style) {
        app.hud.text(r, x, y, s, c, style);
        x += app.hud.width(s, style);
    };
    put("speed ", gd::kInk, gd::Hud::Style::Regular);
    if (k == 0) {
        put("-", gd::kMuted, gd::Hud::Style::Bold);
    } else if (!app.speed.ready(kMinWindowSteps)) {
        put("…", gd::kMuted, gd::Hud::Style::Bold);
    } else {
        SDL_snprintf(line, sizeof line, "%.2f", app.speed.mean());
        put(line, inPlace ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);
    }
    put(" measured   formula ", gd::kInk, gd::Hud::Style::Regular);
    SDL_snprintf(line, sizeof line, "%.2f", r184::theorySpeed(density(app)));
    put(line, gd::kTealDark, gd::Hud::Style::Bold);
}

void drawChart(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 594.0F, kPanelY, 336.0F, kPanelH, gd::kWhite, gd::kInk);
    app.hud.text(r, 606.0F, 352.0F, "Speed over density", gd::kMuted);
    gd::strokeRect(r, kPlotX, kPlotY, kPlotW, kPlotH, gd::kMuted);
    gd::fillRect(r, kPlotX + kPlotW * 0.5F, kPlotY, 1.0F, kPlotH, gd::Color{234, 88, 12, 120});
    app.hud.text(r, kPlotX - 16.0F, kPlotY - 7.0F, "1", gd::kMuted);
    app.hud.text(r, kPlotX - 16.0F, kPlotY + kPlotH - 15.0F, "0", gd::kMuted);
    app.hud.text(r, kPlotX - 5.0F, kPlotY + kPlotH + 4.0F, "0", gd::kMuted);
    app.hud.text(r, kPlotX + kPlotW * 0.5F - 15.0F, kPlotY + kPlotH + 4.0F, "0.5", gd::kMuted);
    app.hud.text(r, kPlotX + kPlotW - 5.0F, kPlotY + kPlotH + 4.0F, "1", gd::kMuted);

    auto px = [](double d) { return kPlotX + static_cast<float>(std::clamp(d, 0.0, 1.0)) * kPlotW; };
    auto py = [](double s) { return kPlotY + kPlotH - static_cast<float>(std::clamp(s, 0.0, 1.0)) * kPlotH; };

    // The formula, two lines side by side so it is not hair thin on a scaled canvas.
    gd::setColor(r, gd::kTeal);
    for (int i = 0; i < 100; ++i) {
        const double d0 = i / 100.0;
        const double d1 = (i + 1) / 100.0;
        for (float o : {0.0F, 1.0F}) {
            SDL_RenderLine(r, px(d0), py(r184::theorySpeed(d0)) + o, px(d1), py(r184::theorySpeed(d1)) + o);
        }
    }
    for (const auto& [d, s] : app.measured) {
        gd::disc(r, px(d), py(s), 3.0F, gd::kInk);
    }

    if (cars(app) > 0 && app.speed.ready(kMinWindowSteps)) {
        const gd::Color c = app.update == Update::InPlace ? gd::kOrange : gd::kTeal;
        gd::disc(r, px(density(app)), py(app.speed.mean()), 7.0F, c);
        gd::ring(r, px(density(app)), py(app.speed.mean()), 7.0F, gd::kInk);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ RULE 184 TRAFFIC ]", "ring road, 180 cells, one row per step");

    gd::fillRect(r, kDiagX, kDiagY, kDiagW, kDiagH, gd::kWhite);
    for (std::size_t y = 0; y < app.history.size(); ++y) drawRow(r, app.history[y], kDiagY + static_cast<float>(y) * kCell);
    gd::strokeRect(r, kDiagX, kDiagY, kDiagW, kDiagH, gd::kInk);
    drawRoadStrip(r, app.road);

    const int col = app.dragSlider ? -1 : hoverColumn(app);
    if (col >= 0) {
        gd::fillRect(r, kDiagX + static_cast<float>(col) * kCell, kDiagY, kCell, kRoadY + kRoadH - kDiagY, kInkSoft);
    }

    if (app.update == Update::InPlace) {
        app.hud.text(r, kDiagX, kHintY, "in place: a moved car is met again and moves again. Only visible below 0.5", gd::kOrange);
    } else {
        app.hud.text(r, kDiagX, kHintY, "click: car or gap   Left/Right: density   cars drive right, jams drift left", gd::kMuted);
    }

    gd::card(r, kDiagX, kPanelY, 540.0F, kPanelH, gd::kWhite, gd::kInk);
    drawSlider(app);
    drawStats(app);
    drawChart(app);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const std::string label = labelFor(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("rule-184-traffic", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    measureChart(*app);
    restart(*app, r184::randomRoad(kCells, kStartCars, app->rng));
    warmUp(*app);
    app->lastMs = nowMs();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouseX = event->motion.x;
            app.mouseY = event->motion.y;
            if (app.dragSlider) setSliderTo(app, app.mouseX);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouseX = p.x;
            app.mouseY = p.y;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (onSlider(p.x, p.y)) {
                app.dragSlider = true;
                setSliderTo(app, p.x);
                break;
            }
            const int col = hoverColumn(app);
            if (col >= 0) {
                app.road[static_cast<std::size_t>(col)] ^= 1U;
                edited(app);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragSlider = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.mouseX = -100.0F;
            app.mouseY = -100.0F;
            app.dragSlider = false;
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragSlider = false; break;
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode key = event->key.key;
            const bool repeatable = key == SDLK_LEFT || key == SDLK_RIGHT || key == SDLK_PERIOD;  // scrubbing keys, the rest toggle or reroll
            if (event->key.repeat && !repeatable) break;
            switch (key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_SPACE: apply(app, Action::PlayPause); break;
                case SDLK_PERIOD: apply(app, Action::Step); break;
                case SDLK_R: apply(app, Action::Random); break;
                case SDLK_B: apply(app, Action::Queue); break;
                case SDLK_1: apply(app, Action::AllAtOnce); break;
                case SDLK_2: apply(app, Action::InPlace); break;
                case SDLK_S: apply(app, Action::Speed); break;
                case SDLK_LEFT: setCars(app, cars(app) > kKeyStepCars ? cars(app) - kKeyStepCars : 0); break;
                case SDLK_RIGHT: setCars(app, cars(app) + kKeyStepCars); break;
                default: break;
            }
            break;
        }
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const double now = nowMs();
    const double dt = std::clamp((now - app.lastMs) / 1000.0, 0.0, 0.25);
    app.lastMs = now;
    if (!app.paused) {
        const double interval = 1.0 / kStepsPerSecond[app.speedIndex];
        app.accumulator += dt;
        for (int n = 0; app.accumulator >= interval && n < kMaxStepsPerFrame; ++n) {
            stepOnce(app);
            app.accumulator -= interval;
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
