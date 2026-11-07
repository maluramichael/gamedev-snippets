// Mood and breakdown with SDL3 main callbacks.
//
// Eight colonists in a row, each with a mood bar that chases a target (the dark triangle). Small events lower or raise the
// target for a few hours, the bar follows at a limited speed, and below a threshold a colonist can break. A break scares the
// two neighbours, which is how one failure turns into several. The second model needs a slow long-term counter to be high
// as well, so a bad day alone does not break anybody. All rules live in logic.hpp and are covered by test.cpp, this file
// only draws and handles input.

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

constexpr float kColX0 = 40.0F;     // left edge of the first column
constexpr float kColW = 80.0F;      // pitch of a column
constexpr float kBarOffset = 12.0F;  // bar inside the column
constexpr float kBarW = 56.0F;
constexpr float kBarTop = 66.0F;
constexpr float kBarH = 220.0F;
constexpr float kColTop = 34.0F;
constexpr float kColBottom = 362.0F;
constexpr float kPanelX = 704.0F;
constexpr float kPanelY = 44.0F;
constexpr float kPanelW = 232.0F;
constexpr float kPanelH = 322.0F;

constexpr float kChipH = 34.0F;
constexpr std::array<float, 3> kChipRows{400.0F, 442.0F, 484.0F};

constexpr float kSliderX = 170.0F;
constexpr float kSliderW = 160.0F;
constexpr float kSliderY = 504.0F;
constexpr double kBaseMin = 10.0;
constexpr double kBaseMax = 60.0;

constexpr double kStepHours = 0.05;
constexpr double kMaxFrameSeconds = 0.25;
constexpr int kMaxStepsPerFrame = 400;
constexpr std::array<double, 3> kSpeeds{1.0, 4.0, 16.0};  // game hours per real second
constexpr std::array<const char*, mb::kColonists> kNames{"Ann", "Bo", "Cy", "Dee", "Eli", "Fay", "Gus", "Hal"};

enum class Action { Corpse, BadMeal, Funeral, ModelRim, ModelDwarf, Reset, Pause, Speed0, Speed1, Speed2 };

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
    std::uint64_t seed = 2026;
    mb::Sim sim = mb::makeSim(2026);
    int selected = 4;
    int speed = 1;  // index into kSpeeds
    bool paused = false;
    bool draggingBase = false;
    double accumulator = 0.0;  // game hours not yet simulated
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

float barX(int i) { return kColX0 + kColW * static_cast<float>(i) + kBarOffset; }
float moodY(double mood) { return kBarTop + kBarH - static_cast<float>(mb::clampMood(mood)) * kBarH / 100.0F; }

int rounded(double v) { return static_cast<int>(std::lround(std::clamp(v, -1000.0, 1000.0))); }

const char* severityName(mb::Severity s) {
    switch (s) {
        case mb::Severity::Minor: return "minor";
        case mb::Severity::Major: return "major";
        case mb::Severity::Extreme: return "extreme";
        case mb::Severity::None: break;
    }
    return "none";
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        int row;
        float gapBefore;
    };
    const std::array<Def, 10> defs{{{Action::Corpse, "1 Corpse (all)", 0, 0.0F},
                                    {Action::BadMeal, "2 Bad meal (selected)", 0, 8.0F},
                                    {Action::Funeral, "3 Funeral (all)", 0, 8.0F},
                                    {Action::ModelRim, "R RimWorld: one bar", 1, 0.0F},
                                    {Action::ModelDwarf, "D Dwarf Fortress: two counters", 1, 8.0F},
                                    {Action::Reset, "X Reset", 1, 24.0F},
                                    {Action::Pause, "Space Pause", 2, 42.0F},
                                    {Action::Speed0, "1 h/s", 2, 8.0F},
                                    {Action::Speed1, "4 h/s", 2, 8.0F},
                                    {Action::Speed2, "16 h/s", 2, 8.0F}}};
    std::array<float, 3> x{40.0F, 40.0F, kSliderX + kSliderW};
    for (const Def& d : defs) {
        float& cx = x[static_cast<std::size_t>(d.row)];
        cx += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{cx, kChipRows[static_cast<std::size_t>(d.row)], w, kChipH}});
        cx += w;
    }
}

void resetSim(App& app) {
    const mb::Model model = app.sim.model;
    app.sim = mb::makeSim(++app.seed);
    app.sim.model = model;
    app.accumulator = 0.0;
    app.draggingBase = false;
}

void setBase(App& app, double base) { app.sim.base = std::clamp(std::round(base), kBaseMin, kBaseMax); }

void setSpeed(App& app, int speed) {
    app.speed = std::clamp(speed, 0, static_cast<int>(kSpeeds.size()) - 1);
    app.accumulator = 0.0;
}

void setPaused(App& app, bool paused) {
    app.paused = paused;
    app.accumulator = 0.0;
}

void setModel(App& app, mb::Model model) {
    app.sim.model = model;
    app.accumulator = 0.0;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Corpse: mb::applyEvent(app.sim, mb::Event::Corpse, app.selected); break;
        case Action::BadMeal: mb::applyEvent(app.sim, mb::Event::BadMeal, app.selected); break;
        case Action::Funeral: mb::applyEvent(app.sim, mb::Event::Funeral, app.selected); break;
        case Action::ModelRim: setModel(app, mb::Model::Rim); break;
        case Action::ModelDwarf: setModel(app, mb::Model::Dwarf); break;
        case Action::Reset: resetSim(app); break;
        case Action::Pause: setPaused(app, !app.paused); break;
        case Action::Speed0: setSpeed(app, 0); break;
        case Action::Speed1: setSpeed(app, 1); break;
        case Action::Speed2: setSpeed(app, 2); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ModelRim: return app.sim.model == mb::Model::Rim;
        case Action::ModelDwarf: return app.sim.model == mb::Model::Dwarf;
        case Action::Pause: return app.paused;
        case Action::Speed0: return app.speed == 0;
        case Action::Speed1: return app.speed == 1;
        case Action::Speed2: return app.speed == 2;
        default: return false;
    }
}

float centeredX(const App& app, float x, float w, const char* s, gd::Hud::Style style) { return x + (w - app.hud.width(s, style)) / 2.0F; }

void triangleLeft(SDL_Renderer* r, float x, float y, float size, gd::Color c) {
    const SDL_FColor f = gd::fcolor(c);
    const SDL_Vertex v[3] = {{{x, y}, f, {0, 0}}, {{x + size, y - size * 0.7F}, f, {0, 0}}, {{x + size, y + size * 0.7F}, f, {0, 0}}};
    SDL_RenderGeometry(r, nullptr, v, 3, nullptr, 0);
}

void drawColumn(App& app, int i) {
    SDL_Renderer* r = app.renderer;
    const mb::Colonist& c = app.sim.c[static_cast<std::size_t>(i)];
    const mb::Thresholds th = mb::thresholds(c.minorThreshold);
    const mb::Severity risk = mb::riskOf(app.sim.model, c.mood, c.longStress, th);
    const bool isSelected = app.selected == i;
    const float colX = kColX0 + kColW * static_cast<float>(i);
    const float bx = barX(i);

    if (isSelected) gd::fillRect(r, colX + 2.0F, kColTop, kColW - 4.0F, kColBottom - kColTop, gd::Color{204, 251, 241, 150});

    // Name, and a dot while the colonist carries the "witnessed a break" thought.
    app.hud.text(r, centeredX(app, bx, kBarW, kNames[static_cast<std::size_t>(i)], gd::Hud::Style::Bold), 40.0F, kNames[static_cast<std::size_t>(i)], isSelected ? gd::kTealDark : gd::kInk, gd::Hud::Style::Bold);
    if (c.left[mb::index(mb::Thought::Witnessed)] > 0.0) gd::disc(r, bx + kBarW - 2.0F, 49.0F, 5.0F, gd::kOrange);

    // Bar with its three threshold lines.
    gd::fillRect(r, bx, kBarTop, kBarW, kBarH, gd::kWhite);
    const float top = moodY(c.mood);
    const gd::Color fill = c.broken ? gd::kMuted : (risk != mb::Severity::None ? gd::kOrange : gd::kTeal);
    gd::fillRect(r, bx, top, kBarW, kBarTop + kBarH - top, fill);
    gd::strokeRect(r, bx, kBarTop, kBarW, kBarH, gd::kInk);
    for (const double line : {th.minor, th.major, th.extreme}) {
        gd::fillRect(r, bx - 4.0F, moodY(line) - 1.0F, kBarW + 8.0F, 2.0F, gd::Color{17, 24, 39, 200});
    }
    triangleLeft(r, bx + kBarW + 2.0F, moodY(mb::targetOf(app.sim, c)), 9.0F, gd::kInk);

    // Value and state.
    char value[16];
    SDL_snprintf(value, sizeof value, "%d", rounded(c.mood));
    app.hud.text(r, centeredX(app, bx, kBarW, value, gd::Hud::Style::Bold), 292.0F, value, gd::kInk, gd::Hud::Style::Bold);
    const char* state = "calm";
    gd::Color stateColor = gd::kMuted;
    if (c.broken) {
        state = "BREAK";
        stateColor = gd::kOrange;
    } else if (risk != mb::Severity::None) {
        state = "RISK";
        stateColor = gd::kOrange;
    } else if (mb::severityOf(c.mood, th) != mb::Severity::None) {
        state = "low";  // below a line, but the long-term counter holds the break back
        stateColor = gd::kTealDark;
    }
    app.hud.text(r, centeredX(app, bx, kBarW, state, gd::Hud::Style::Regular), 316.0F, state, stateColor);

    // Long-term counter, only in the two-counter model.
    if (app.sim.model == mb::Model::Dwarf) {
        gd::fillRect(r, bx, 344.0F, kBarW, 8.0F, gd::kWhite);
        gd::fillRect(r, bx, 344.0F, kBarW * static_cast<float>(c.longStress / 100.0), 8.0F, c.longStress >= mb::kLongGate ? gd::kOrange : gd::kTeal);
        gd::strokeRect(r, bx, 344.0F, kBarW, 8.0F, gd::kInk);
        gd::fillRect(r, bx + kBarW * static_cast<float>(mb::kLongGate / 100.0) - 1.0F, 341.0F, 2.0F, 14.0F, gd::kInk);
    }
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    const mb::Sim& sim = app.sim;
    const mb::Colonist& c = sim.c[static_cast<std::size_t>(app.selected)];
    const mb::Thresholds th = mb::thresholds(c.minorThreshold);
    const mb::Severity risk = mb::riskOf(sim.model, c.mood, c.longStress, th);
    const float x = kPanelX + 14.0F;
    gd::card(r, kPanelX, kPanelY, kPanelW, kPanelH, gd::kWhite, gd::kInk);

    char line[64];
    app.hud.text(r, x, 52.0F, kNames[static_cast<std::size_t>(app.selected)], gd::kInk, gd::Hud::Style::Bold);
    if (c.broken) {
        SDL_snprintf(line, sizeof line, "BREAK %dh", rounded(std::ceil(c.brokenLeft)));
        app.hud.text(r, kPanelX + kPanelW - 14.0F - app.hud.width(line), 52.0F, line, gd::kOrange);
    }
    SDL_snprintf(line, sizeof line, "mood %d  target %d", rounded(c.mood), rounded(mb::targetOf(sim, c)));
    app.hud.text(r, x, 74.0F, line, gd::kInk);
    if (sim.model == mb::Model::Dwarf) {
        SDL_snprintf(line, sizeof line, "short %d  long %d/%d", rounded(mb::shortStress(c.mood)), rounded(c.longStress), rounded(mb::kLongGate));
    } else {
        SDL_snprintf(line, sizeof line, "line at %d / %d / %d", rounded(th.minor), rounded(th.major), rounded(th.extreme));
    }
    app.hud.text(r, x, 96.0F, line, gd::kMuted);
    if (risk != mb::Severity::None) {
        SDL_snprintf(line, sizeof line, "%s: 1 per %.1f d", severityName(risk), mb::mtbHours(risk) / 24.0);
        app.hud.text(r, x, 118.0F, line, gd::kOrange);
    } else if (mb::severityOf(c.mood, th) != mb::Severity::None) {
        app.hud.text(r, x, 118.0F, "low, long-term too low", gd::kTealDark);
    } else {
        app.hud.text(r, x, 118.0F, "no break risk", gd::kMuted);
    }

    SDL_snprintf(line, sizeof line, "base %d  trait %+d", rounded(sim.base), rounded(c.trait));
    app.hud.text(r, x, 144.0F, line, gd::kInk);
    float y = 166.0F;
    for (int k = 0; k < mb::kThoughtKinds; ++k) {
        if (!(c.left[static_cast<std::size_t>(k)] > 0.0)) continue;
        const mb::ThoughtDef& def = mb::kThoughts[static_cast<std::size_t>(k)];
        SDL_snprintf(line, sizeof line, "%+d %s %dh", rounded(def.value), def.name, rounded(std::ceil(c.left[static_cast<std::size_t>(k)])));
        app.hud.text(r, x, y, line, def.value < 0.0 ? gd::kOrange : gd::kTealDark);
        y += 22.0F;
    }

    gd::fillRect(r, x, 284.0F, kPanelW - 28.0F, 1.0F, gd::kMuted);
    bool any = false;
    float ly = 290.0F;
    for (const mb::BreakLog& e : sim.recent) {
        if (e.who < 0) continue;
        any = true;
        const int hours = static_cast<int>(std::floor(e.time));
        SDL_snprintf(line, sizeof line, "d%d %02dh %s %s", hours / 24 + 1, hours % 24, kNames[static_cast<std::size_t>(e.who)], severityName(e.severity));
        app.hud.text(r, x, ly, line, e.chained ? gd::kOrange : gd::kInk);
        if (e.chained) gd::disc(r, kPanelX + kPanelW - 20.0F, ly + 11.0F, 5.0F, gd::kOrange);  // same dot as "saw a break"
        ly += 22.0F;
    }
    if (!any) app.hud.text(r, x, ly, "no breaks yet", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const mb::Sim& sim = app.sim;
    gd::backdrop(r, kWidth, kHeight);

    char right[96];
    const int hours = static_cast<int>(std::floor(sim.clock));
    SDL_snprintf(right, sizeof right, "%s | day %d %02d:00 | breaks %d (%d chained)", sim.model == mb::Model::Rim ? "RimWorld" : "Dwarf Fortress", hours / 24 + 1, hours % 24, sim.breaks, sim.chained);
    app.hud.titleBar(r, kWidth, "[ MOOD AND BREAKDOWN ]", right);

    for (int i = 0; i < mb::kColonists; ++i) drawColumn(app, i);

    // Threshold values for the standard colonist, left of the first bar.
    const mb::Thresholds std35 = mb::thresholds(mb::kDefaultMinor);
    for (const double line : {std35.minor, std35.major, std35.extreme}) {
        char label[8];
        SDL_snprintf(label, sizeof label, "%d", rounded(line));
        app.hud.text(r, barX(0) - 8.0F - app.hud.width(label), moodY(line) - 11.0F, label, gd::kMuted);
    }
    if (sim.model == mb::Model::Dwarf) app.hud.text(r, 6.0F, 340.0F, "long", gd::kMuted);

    drawPanel(app);

    app.hud.text(r, 40.0F, 374.0F, "click colonist: select  1-3: events  R/D: model  </>: base  Up/Down: speed", gd::kMuted);

    app.hud.text(r, 656.0F, 400.0F, "triangle: target", gd::kMuted);
    app.hud.text(r, 656.0F, 420.0F, "dot: saw a break", gd::kMuted);

    for (const Chip& ch : app.chips) {
        const bool on = active(app, ch.action);
        gd::card(r, ch.rect.x, ch.rect.y, ch.rect.w, ch.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, ch.rect.x + (ch.rect.w - app.hud.width(ch.label, gd::Hud::Style::Bold)) / 2.0F, ch.rect.y + 6.0F, ch.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }

    // Base mood slider.
    char base[32];
    SDL_snprintf(base, sizeof base, "Base mood %d", rounded(sim.base));
    app.hud.text(r, 40.0F, 490.0F, base, gd::kInk, gd::Hud::Style::Bold);
    gd::fillRect(r, kSliderX, kSliderY, kSliderW, 6.0F, gd::kWhite);
    gd::strokeRect(r, kSliderX, kSliderY, kSliderW, 6.0F, gd::kInk);
    const float knobX = kSliderX + kSliderW * static_cast<float>((sim.base - kBaseMin) / (kBaseMax - kBaseMin));
    gd::card(r, knobX - 7.0F, kSliderY - 9.0F, 14.0F, 24.0F, app.draggingBase ? gd::kTeal : gd::kWhite, gd::kInk);
}

double baseFromX(float x) { return kBaseMin + static_cast<double>((x - kSliderX) / kSliderW) * (kBaseMax - kBaseMin); }

bool onSlider(const SDL_FPoint& p) { return p.x >= kSliderX - 12.0F && p.x <= kSliderX + kSliderW + 12.0F && p.y >= kSliderY - 16.0F && p.y <= kSliderY + 22.0F; }

void simulate(App& app, double realSeconds) {
    if (app.paused) return;
    app.accumulator += std::min(realSeconds, kMaxFrameSeconds) * kSpeeds[static_cast<std::size_t>(app.speed)];
    int steps = 0;
    while (app.accumulator >= kStepHours && steps < kMaxStepsPerFrame) {
        mb::step(app.sim, kStepHours);
        app.accumulator -= kStepHours;
        ++steps;
    }
    if (steps == kMaxStepsPerFrame) app.accumulator = std::min(app.accumulator, kStepHours);
}

}  // namespace

SDL_AppResult SDL_AppInit(void** appstate, int argc, char** argv) {
    auto* app = new App();
    *appstate = app;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) app->maxFrames = std::atol(argv[i + 1]);
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("mood-breakdown", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.draggingBase) setBase(app, baseFromX(event->motion.x));
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
            if (onSlider(p)) {
                app.draggingBase = true;
                setBase(app, baseFromX(p.x));
            } else if (p.y >= kColTop && p.y <= kColBottom && p.x >= kColX0 && p.x < kColX0 + kColW * static_cast<float>(mb::kColonists)) {
                app.selected = std::clamp(static_cast<int>((p.x - kColX0) / kColW), 0, mb::kColonists - 1);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.draggingBase = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.draggingBase = false; break;
        case SDL_EVENT_KEY_DOWN:
            // Left and right scrub the base mood and may repeat, everything else toggles or fires an event once.
            if (event->key.key == SDLK_LEFT) {
                setBase(app, app.sim.base - 1.0);
                break;
            }
            if (event->key.key == SDLK_RIGHT) {
                setBase(app, app.sim.base + 1.0);
                break;
            }
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Corpse); break;
                case SDLK_2: apply(app, Action::BadMeal); break;
                case SDLK_3: apply(app, Action::Funeral); break;
                case SDLK_R: apply(app, Action::ModelRim); break;
                case SDLK_D: apply(app, Action::ModelDwarf); break;
                case SDLK_X: apply(app, Action::Reset); break;
                case SDLK_SPACE: apply(app, Action::Pause); break;
                case SDLK_UP: setSpeed(app, app.speed + 1); break;
                case SDLK_DOWN: setSpeed(app, app.speed - 1); break;
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
    const double dt = static_cast<double>(now - app.lastTicks) * 1.0e-9;
    app.lastTicks = now;
    simulate(app, dt);
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
