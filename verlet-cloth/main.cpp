// Cloth and ropes with Verlet integration, SDL3 main callbacks.
//
// A point only knows where it is and where it was one step ago; the speed is the difference. Links between points are
// pushed back to their rest length again and again, and the more often that happens per step, the stiffer the cloth:
// one pass is rubber, thirty passes are cloth. "Stored velocity" is the naive way: the same corrections are applied to
// the positions, but the speed lives in its own variable and never hears about them. A hanging point keeps gaining
// speed that the links cancel every step, and when you cut the thread it shoots away. The probe at the bottom of the
// cloth (or the point under the mouse) shows the number in both modes.
//
// The simulation is in logic.hpp (no SDL, covered by test.cpp). The loop is the fixed timestep from the earlier demos:
// frame time goes into an accumulator and the world steps in slices of exactly 1/120 s.

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
constexpr float kFieldTop = 34.0F;
constexpr float kFloorY = 404.0F;
constexpr float kStep = 1.0F / 120.0F;
constexpr double kMaxFrameTime = 0.25;  // clamp: no spiral of death after a stall
constexpr int kMaxStepsPerFrame = 8;
constexpr int kWarmupSteps = 150;       // the scene is already swaying when you see it
constexpr float kGrabRadius = 22.0F;
constexpr float kPinRadius = 16.0F;
constexpr float kCutRadius = 6.0F;
constexpr float kMaxHoldMove = 10.0F;   // px per step a held point may carry as speed
constexpr float kTearFactor = 2.5F;
constexpr float kProbeScale = 12.0F;    // the speed arrow shows 12 steps of motion
constexpr float kProbeMax = 90.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kSliderY = 434.0F;      // vertical center of both sliders
constexpr float kSliderW = 200.0F;
constexpr float kIterLabelX = 40.0F;
constexpr float kIterTrackX = 150.0F;
constexpr float kWindLabelX = 520.0F;
constexpr float kWindTrackX = 580.0F;
constexpr int kMinIterations = 1;
constexpr int kSliderMaxIterations = 30;
constexpr float kStrainGood = 0.02F;    // below this a link is drawn teal
constexpr float kStrainBad = 0.30F;     // from this on fully orange
constexpr int kDefaultIterations = 12;
constexpr float kDefaultWind = 0.3F;
constexpr float kClothFillAlpha = 0.7F;

enum class Tool { Grab, Pin, Cut };
enum class Action { ToolGrab, ToolPin, ToolCut, Verlet, Stored, Tear, Reset };

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
    vc::World world;
    vc::Params params;
    Tool tool = Tool::Grab;
    bool tearOn = false;
    double simTime = 0.0;
    double accumulator = 0.0;
    Uint64 lastTicks = 0;
    vc::Vec2 mouse{-100.0F, -100.0F};
    std::optional<std::size_t> grab;
    vc::Vec2 grabTarget;
    bool cutting = false;
    int dragSlider = -1;  // 0 iterations, 1 wind
    std::vector<SDL_Vertex> fillVerts;
    std::vector<int> fillIdx;
    long maxFrames = -1;
    long frames = 0;
};

vc::Vec2 toVec(SDL_FPoint p) { return {p.x, p.y}; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::ToolGrab, "1 Grab", 0.0F},
                                   {Action::ToolPin, "2 Pin", 8.0F},
                                   {Action::ToolCut, "3 Cut", 8.0F},
                                   {Action::Verlet, "V Verlet", 24.0F},
                                   {Action::Stored, "N Stored velocity", 8.0F},
                                   {Action::Tear, "T Tear", 24.0F},
                                   {Action::Reset, "R Reset", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

/// Lets go of everything the mouse holds: the carried point, the cut swipe, a slider.
void releaseAll(App& app) {
    if (app.grab && *app.grab < app.world.pts.size()) app.world.pts[*app.grab].held = false;
    app.grab.reset();
    app.cutting = false;
    app.dragSlider = -1;
}

void syncParams(App& app) { app.params.tear = app.tearOn ? kTearFactor : 0.0F; }

/// Fresh scene, then a short run so the cloth is already moving. Accumulator and clock restart with it.
void resetScene(App& app) {
    releaseAll(app);
    app.world = vc::makeScene();
    syncParams(app);
    app.simTime = 0.0;
    for (int i = 0; i < kWarmupSteps; ++i) {
        vc::step(app.world, app.params, app.simTime, kStep);
        app.simTime += static_cast<double>(kStep);
    }
    app.accumulator = 0.0;
    app.lastTicks = SDL_GetTicksNS();
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ToolGrab: app.tool = Tool::Grab; break;
        case Action::ToolPin: app.tool = Tool::Pin; break;
        case Action::ToolCut: app.tool = Tool::Cut; break;
        case Action::Verlet:
        case Action::Stored: {
            const bool verlet = action == Action::Verlet;
            if (app.params.verlet != verlet) {
                app.params.verlet = verlet;
                resetScene(app);  // positions and the two kinds of velocity do not translate, start over
            }
            break;
        }
        case Action::Tear:
            app.tearOn = !app.tearOn;
            syncParams(app);
            break;
        case Action::Reset: resetScene(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ToolGrab: return app.tool == Tool::Grab;
        case Action::ToolPin: return app.tool == Tool::Pin;
        case Action::ToolCut: return app.tool == Tool::Cut;
        case Action::Verlet: return app.params.verlet;
        case Action::Stored: return !app.params.verlet;
        case Action::Tear: return app.tearOn;
        case Action::Reset: return false;
    }
    return false;
}

SDL_FRect sliderHit(float trackX) { return SDL_FRect{trackX - 8.0F, kSliderY - 14.0F, kSliderW + 16.0F, 28.0F}; }

void setSlider(App& app, int which, float mouseX) {
    if (which == 0) {
        const float v = vc::sliderValue(mouseX, kIterTrackX, kSliderW, static_cast<float>(kMinIterations), static_cast<float>(kSliderMaxIterations));
        app.params.iterations = static_cast<int>(std::lround(v));
    } else {
        const float v = vc::sliderValue(mouseX, kWindTrackX, kSliderW, 0.0F, 1.0F);
        app.params.wind = std::round(v * 20.0F) / 20.0F;  // steps of 5 percent
    }
}

bool inField(vc::Vec2 p) { return p.y >= kFieldTop && p.y <= kFloorY + 6.0F; }

vc::Vec2 clampTarget(vc::Vec2 p) {
    return {std::clamp(p.x, 0.0F, static_cast<float>(kWidth)), std::clamp(p.y, kFieldTop + 6.0F, kFloorY)};
}

/// The point the speed read-out is about: under the mouse, else the middle of the cloth's bottom row.
std::size_t probeIndex(const App& app) {
    if (app.grab) return *app.grab;
    const float radius = app.tool == Tool::Pin ? kPinRadius : kGrabRadius;
    if (const auto hit = vc::nearestPoint(app.world, app.mouse, radius)) return *hit;
    return static_cast<std::size_t>((app.world.clothRows - 1) * app.world.clothCols + app.world.clothCols / 2);
}

// ---------------------------------------------------------------------------------------------------------------- drawing

gd::Color strainColor(float strain) {
    const float t = std::clamp((strain - kStrainGood) / (kStrainBad - kStrainGood), 0.0F, 1.0F);
    const auto mix = [t](Uint8 a, Uint8 b) { return static_cast<Uint8>(std::lround(static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * t)); };
    return gd::Color{mix(gd::kTealDark.r, gd::kOrange.r), mix(gd::kTealDark.g, gd::kOrange.g), mix(gd::kTealDark.b, gd::kOrange.b)};
}

float linkStrain(const vc::World& w, const vc::Link& l) {
    const float len = vc::length(w.pts[static_cast<std::size_t>(l.b)].pos - w.pts[static_cast<std::size_t>(l.a)].pos);
    return l.rest > 0.0F ? std::fabs(len / l.rest - 1.0F) : 0.0F;
}

/// 1 px lines are thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, vc::Vec2 a, vc::Vec2 b, gd::Color c, int half) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, a.x + (flat ? 0.0F : off), a.y + (flat ? off : 0.0F), b.x + (flat ? 0.0F : off), b.y + (flat ? off : 0.0F));
    }
}

void drawClothFill(App& app) {
    const vc::World& w = app.world;
    const int cols = w.clothCols;
    const int rows = w.clothRows;
    app.fillVerts.clear();
    app.fillIdx.clear();
    const SDL_FColor fill = gd::fcolor(gd::kTealSoft, kClothFillAlpha);
    for (int j = 0; j + 1 < rows; ++j) {
        for (int i = 0; i + 1 < cols; ++i) {
            const auto alive = [&w](int index) { return w.links[static_cast<std::size_t>(index)].alive; };
            if (!(alive(vc::hLinkIndex(cols, i, j)) && alive(vc::hLinkIndex(cols, i, j + 1)) && alive(vc::vLinkIndex(cols, rows, i, j)) &&
                  alive(vc::vLinkIndex(cols, rows, i + 1, j)))) {
                continue;
            }
            const int base = static_cast<int>(app.fillVerts.size());
            for (const int k : {j * cols + i, j * cols + i + 1, (j + 1) * cols + i + 1, (j + 1) * cols + i}) {
                const vc::Vec2 p = w.pts[static_cast<std::size_t>(k)].pos;
                app.fillVerts.push_back(SDL_Vertex{{p.x, p.y}, fill, {0, 0}});
            }
            app.fillIdx.insert(app.fillIdx.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        }
    }
    if (!app.fillIdx.empty()) {
        SDL_RenderGeometry(app.renderer, nullptr, app.fillVerts.data(), static_cast<int>(app.fillVerts.size()), app.fillIdx.data(), static_cast<int>(app.fillIdx.size()));
    }
}

void drawLinks(App& app) {
    SDL_Renderer* r = app.renderer;
    const vc::World& w = app.world;
    const std::size_t clothLinks = w.clothCols > 0 ? static_cast<std::size_t>((w.clothCols - 1) * w.clothRows + w.clothCols * (w.clothRows - 1)) : 0;
    for (std::size_t i = 0; i < w.links.size(); ++i) {
        const vc::Link& l = w.links[i];
        if (!l.alive) continue;
        const vc::Vec2 a = w.pts[static_cast<std::size_t>(l.a)].pos;
        const vc::Vec2 b = w.pts[static_cast<std::size_t>(l.b)].pos;
        const gd::Color c = strainColor(linkStrain(w, l));
        if (i < clothLinks) {
            gd::setColor(r, c);
            SDL_RenderLine(r, a.x, a.y, b.x, b.y);
        } else {
            thickLine(r, a, b, c, 1);
        }
    }
}

void drawPoints(App& app) {
    SDL_Renderer* r = app.renderer;
    for (std::size_t i = 0; i < app.world.pts.size(); ++i) {
        const vc::Point& p = app.world.pts[i];
        if (p.pinned) {
            gd::fillRect(r, p.pos.x - 4.0F, p.pos.y - 4.0F, 8.0F, 8.0F, gd::kInk);
        } else if (p.invMass < 0.5F) {
            gd::disc(r, p.pos.x, p.pos.y, 8.0F, gd::kInk);  // the weight on the second rope
        }
    }
}

/// The read-out for one point: where it was (ring), where it is (disc) and the speed as an arrow.
void drawProbe(App& app, std::size_t index) {
    SDL_Renderer* r = app.renderer;
    const vc::Point& p = app.world.pts[index];
    // Verlet: the speed is pos - prev. Stored velocity: the variable itself, per step it moves by vel * dt.
    const vc::Vec2 perStep = app.params.verlet ? p.pos - p.prev : p.vel * kStep;
    const float speed = vc::length(perStep) / kStep;
    const vc::Vec2 arrow = vc::clampLength(perStep * kProbeScale, kProbeMax);
    const gd::Color tone = app.params.verlet ? gd::kTeal : gd::kOrange;
    if (app.params.verlet) gd::ring(r, p.prev.x, p.prev.y, 3.0F, gd::kInk, 16);
    gd::disc(r, p.pos.x, p.pos.y, 5.0F, tone);
    gd::ring(r, p.pos.x, p.pos.y, 5.0F, gd::kInk, 20);
    const vc::Vec2 tip = p.pos + arrow;
    if (vc::length(arrow) > 2.0F) {
        thickLine(r, p.pos, tip, tone, 1);
        const vc::Vec2 dir = arrow * (1.0F / vc::length(arrow));
        const vc::Vec2 side{-dir.y, dir.x};
        thickLine(r, tip, tip - dir * 9.0F + side * 5.0F, tone, 0);
        thickLine(r, tip, tip - dir * 9.0F - side * 5.0F, tone, 0);
    }
    char line[64];
    SDL_snprintf(line, sizeof line, app.params.verlet ? "pos - prev = %d px/s" : "stored vel = %d px/s", static_cast<int>(std::lround(speed)));
    const float tw = app.hud.width(line, gd::Hud::Style::Bold) + 16.0F;
    float tx = p.pos.x + 14.0F;
    float ty = p.pos.y - 34.0F;
    if (tx + tw > static_cast<float>(kWidth) - 8.0F) tx = p.pos.x - 14.0F - tw;
    ty = std::clamp(ty, kFieldTop + 6.0F, kFloorY - 28.0F);
    gd::card(r, tx, ty, tw, 24.0F, gd::kWhite, tone);
    app.hud.text(r, tx + 8.0F, ty + 1.0F, line, gd::kInk, gd::Hud::Style::Bold);
}

void drawSlider(App& app, int which, const char* label, float labelX, float trackX, float fraction, const char* valueText) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, labelX, kSliderY - 11.0F, label, gd::kInk, gd::Hud::Style::Bold);
    gd::fillRect(r, trackX, kSliderY - 2.0F, kSliderW, 4.0F, gd::kGrid);
    gd::strokeRect(r, trackX, kSliderY - 2.0F, kSliderW, 4.0F, gd::kMuted);
    gd::fillRect(r, trackX, kSliderY - 2.0F, kSliderW * fraction, 4.0F, gd::kTeal);
    const bool held = app.dragSlider == which;
    gd::card(r, trackX + kSliderW * fraction - 7.0F, kSliderY - 12.0F, 14.0F, 24.0F, held ? gd::kTeal : gd::kWhite, gd::kInk);
    app.hud.text(r, trackX + kSliderW + 16.0F, kSliderY - 11.0F, valueText, gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const vc::Strain strain = vc::strain(app.world);
    const bool verlet = app.params.verlet;

    gd::backdrop(r, kWidth, kHeight);
    char maxText[16];
    if (strain.maximum < 9.995F) {
        SDL_snprintf(maxText, sizeof maxText, "%.0f%%", static_cast<double>(strain.maximum * 100.0F));
    } else {
        SDL_snprintf(maxText, sizeof maxText, ">999%%");
    }
    char right[96];
    SDL_snprintf(right, sizeof right, "strain avg %.1f%% max %s  links %d/%d", static_cast<double>(strain.average * 100.0F), maxText, strain.alive, static_cast<int>(app.world.links.size()));
    app.hud.titleBar(r, kWidth, verlet ? "[ VERLET CLOTH ]" : "[ STORED VELOCITY ]", right);

    // Floor.
    gd::fillRect(r, 0.0F, kFloorY + 1.0F, static_cast<float>(kWidth), 3.0F, gd::kInk);

    drawClothFill(app);
    drawLinks(app);
    drawPoints(app);
    drawProbe(app, probeIndex(app));

    // Cursor: grab and pin rings, cut radius.
    if (inField(app.mouse)) {
        if (app.tool == Tool::Cut) {
            gd::ring(r, app.mouse.x, app.mouse.y, kCutRadius, gd::kOrange, 20);
        } else if (!app.grab) {
            const auto hit = vc::nearestPoint(app.world, app.mouse, app.tool == Tool::Pin ? kPinRadius : kGrabRadius);
            if (hit) gd::ring(r, app.world.pts[*hit].pos.x, app.world.pts[*hit].pos.y, 10.0F, gd::kOrange, 24);
        }
    }

    // Sliders.
    char value[16];
    SDL_snprintf(value, sizeof value, "%d", app.params.iterations);
    drawSlider(app, 0, "Iterations", kIterLabelX, kIterTrackX,
               vc::sliderFraction(static_cast<float>(app.params.iterations), static_cast<float>(kMinIterations), static_cast<float>(kSliderMaxIterations)), value);
    SDL_snprintf(value, sizeof value, "%d%%", static_cast<int>(std::lround(app.params.wind * 100.0F)));
    drawSlider(app, 1, "Wind", kWindLabelX, kWindTrackX, vc::sliderFraction(app.params.wind, 0.0F, 1.0F), value);

    const char* hint = app.tool == Tool::Grab  ? "drag a point to pull the cloth   up/down: iterations   left/right: wind"
                       : app.tool == Tool::Pin ? "click a point to pin it, click again to let go   up/down: iterations"
                                               : "swipe across the threads to cut them   up/down: iterations";
    app.hud.text(r, 40.0F, 452.0F, hint, gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const gd::Color onColor = c.action == Action::Stored ? gd::kOrange : gd::kTeal;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? onColor : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

// ---------------------------------------------------------------------------------------------------------------- simulation

void advance(App& app) {
    const Uint64 now = SDL_GetTicksNS();
    const double frame = std::min(static_cast<double>(now - app.lastTicks) / 1.0e9, kMaxFrameTime);
    app.lastTicks = now;
    app.accumulator += frame;
    int steps = 0;
    while (app.accumulator >= static_cast<double>(kStep) && steps < kMaxStepsPerFrame) {
        if (app.grab) vc::moveHeld(app.world.pts[*app.grab], app.grabTarget, kMaxHoldMove, kStep);
        vc::step(app.world, app.params, app.simTime, kStep);
        app.simTime += static_cast<double>(kStep);
        app.accumulator -= static_cast<double>(kStep);
        ++steps;
    }
    if (steps == kMaxStepsPerFrame) app.accumulator = std::fmod(app.accumulator, static_cast<double>(kStep));  // drop the backlog
}

void adjustIterations(App& app, int delta) { app.params.iterations = std::clamp(app.params.iterations + delta, kMinIterations, kSliderMaxIterations); }
void adjustWind(App& app, float delta) { app.params.wind = std::clamp(std::round((app.params.wind + delta) * 20.0F) / 20.0F, 0.0F, 1.0F); }

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
    if (!SDL_CreateWindowAndRenderer("verlet-cloth", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->params.iterations = kDefaultIterations;
    app->params.wind = kDefaultWind;
    layoutChips(*app);
    resetScene(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            const vc::Vec2 from = app.mouse;
            app.mouse = toVec({event->motion.x, event->motion.y});
            if (app.dragSlider >= 0) {
                setSlider(app, app.dragSlider, app.mouse.x);
            } else if (app.grab) {
                app.grabTarget = clampTarget(app.mouse);
            } else if (app.cutting) {
                vc::cutAlong(app.world, from, app.mouse, kCutRadius);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = toVec(p);
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            const std::array<float, 2> tracks{kIterTrackX, kWindTrackX};
            for (std::size_t i = 0; i < tracks.size(); ++i) {
                const SDL_FRect hit = sliderHit(tracks[i]);
                if (SDL_PointInRectFloat(&p, &hit)) {
                    app.dragSlider = static_cast<int>(i);
                    setSlider(app, app.dragSlider, p.x);
                    return SDL_APP_CONTINUE;
                }
            }
            if (!inField(app.mouse)) break;
            if (app.tool == Tool::Grab) {
                if (const auto hit = vc::nearestPoint(app.world, app.mouse, kGrabRadius)) {
                    app.grab = hit;
                    app.world.pts[*hit].held = true;
                    app.grabTarget = clampTarget(app.mouse);
                }
            } else if (app.tool == Tool::Pin) {
                if (const auto hit = vc::nearestPoint(app.world, app.mouse, kPinRadius)) vc::togglePin(app.world, *hit);
            } else {
                app.cutting = true;
                vc::cutAlong(app.world, app.mouse, app.mouse, kCutRadius);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) releaseAll(app);
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: releaseAll(app); break;
        case SDL_EVENT_KEY_DOWN:
            // Up/down/left/right may repeat (hold to sweep a slider), everything else is a toggle or a button.
            switch (event->key.key) {
                case SDLK_UP: adjustIterations(app, 1); break;
                case SDLK_DOWN: adjustIterations(app, -1); break;
                case SDLK_RIGHT: adjustWind(app, 0.05F); break;
                case SDLK_LEFT: adjustWind(app, -0.05F); break;
                default: break;
            }
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::ToolGrab); break;
                case SDLK_2: apply(app, Action::ToolPin); break;
                case SDLK_3: apply(app, Action::ToolCut); break;
                case SDLK_V: apply(app, Action::Verlet); break;
                case SDLK_N: apply(app, Action::Stored); break;
                case SDLK_T: apply(app, Action::Tear); break;
                case SDLK_R: apply(app, Action::Reset); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    advance(app);
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
