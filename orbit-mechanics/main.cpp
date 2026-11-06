// Orbit mechanics with SDL3 main callbacks.
//
// One sun, one body, and four answers to "where is it after t seconds": the exact Kepler orbit (white), explicit
// Euler (orange), semi-implicit Euler (teal) and velocity Verlet (blue). All of them start from the same position and
// velocity, the three numerical ones advance in steps of h. Explicit Euler moves along the tangent before it turns the
// velocity, so every step lands a bit too far out and the orbit spirals outward; swapping the two lines keeps the
// orbit. The table shows what each method does to energy and angular momentum, which the real orbit keeps constant.
//
// Click in space for a circular orbit there, press and drag to set the launch velocity (the exact ellipse follows the
// mouse while you drag). The shaded sectors are the exact orbit cut into eight equal times: equal times, equal areas.
// All the maths is in logic.hpp (no SDL, covered by test.cpp), this file owns input, the fixed step loop and the picture.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;

// The stage is a dark window onto space, drawn inside the usual paper frame.
constexpr float kStageX = 24.0F;
constexpr float kStageY = 44.0F;
constexpr float kStageW = 912.0F;
constexpr float kStageH = 372.0F;
constexpr float kCx = kStageX + 376.0F;  // the sun sits left of centre, the readouts take the right third
constexpr float kCy = kStageY + kStageH / 2.0F;

constexpr double kMu = 602300.0;         // chosen so that a circular orbit at 130 px takes 12 s
constexpr double kSimSpeed = 2.0;        // simulated seconds per real second
constexpr double kMaxFrameTime = 0.25;   // clamp: no burst of steps after a stalled tab
constexpr int kMaxStepsPerFrame = 2000;  // and a hard cap, whatever h is
constexpr double kSunRadius = 12.0;
constexpr double kMinStartDistance = 28.0;  // clicks closer to the sun than this do nothing
constexpr double kMinPeriapsis = 20.0;      // orbits that dive deeper are refused
constexpr double kLostMin = 3.0;            // closer than this a numerical body is "lost" (1/r^2 explodes)
constexpr double kLostMax = 4000.0;         // and so is one that flew this far
constexpr double kSpeedPerPx = 0.6;         // launch speed per pixel of mouse drag
constexpr double kClickDrag = 6.0;          // a drag shorter than this is a click
constexpr std::size_t kTrailCap = 6000;
constexpr int kWedgeCount = 8;
constexpr int kWedgeSamples = 24;
constexpr float kReadoutW = 388.0F;  // width of the two panels on the right
constexpr float kChipH = 32.0F;
constexpr float kRowOneY = 446.0F;
constexpr float kRowTwoY = 488.0F;

constexpr std::array<double, 5> kSteps{0.01, 0.02, 0.05, 0.1, 0.25};
constexpr std::array<const char*, 5> kStepLabels{"0.01", "0.02", "0.05", "0.1", "0.25"};

constexpr gd::Color kLight{232, 238, 248};
constexpr gd::Color kDim{148, 163, 184};
constexpr gd::Color kSky0{9, 13, 31};
constexpr gd::Color kSky1{22, 30, 62};

enum class Method { Euler, Semi, Verlet };
constexpr std::array<const char*, 3> kMethodNames{"Euler", "Semi-impl.", "Verlet"};
constexpr std::array<gd::Color, 3> kMethodStage{gd::Color{255, 122, 40}, gd::Color{45, 212, 191}, gd::Color{147, 155, 255}};
constexpr std::array<gd::Color, 3> kMethodChip{gd::kOrange, gd::kTeal, gd::Color{99, 102, 241}};

enum class Action { Preset, Restart, Pause, Areas, StepSize, Show };

struct Chip {
    Action action;
    int value;
    std::string label;
    SDL_FRect rect;
};

struct Label {
    float x;
    float y;
    const char* text;
};

/// The last kTrailCap positions of one body, oldest overwritten first.
struct Trail {
    std::vector<orb::Vec2> pts;
    std::size_t head = 0;

    void clear() {
        pts.clear();
        head = 0;
    }
    void push(orb::Vec2 p) {
        if (pts.size() < kTrailCap) {
            pts.push_back(p);
        } else {
            pts[head] = p;
            head = (head + 1) % kTrailCap;
        }
    }
    std::size_t size() const { return pts.size(); }
    orb::Vec2 at(std::size_t i) const { return pts[(head + i) % pts.size()]; }
};

struct Body {
    orb::State s;
    Trail trail;
    bool lost = false;
};

struct Star {
    float x, y, size, base, phase;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<Label> labels;
    std::vector<Star> stars;

    orb::State start;
    std::optional<orb::Conic> conic;
    std::vector<orb::Vec2> orbitPts;                 // the exact curve, computed once per launch
    std::vector<std::vector<orb::Vec2>> wedges;      // eight equal-time sectors of the exact orbit
    std::array<Body, 3> bodies;
    orb::State exact;                                // the exact state at the current simulation time
    double energy0 = 0.0;
    double momentum0 = 0.0;
    double simTime = 0.0;
    double accumulator = 0.0;

    int preset = 1;  // 0 circle, 1 ellipse, 2 comet, -1 set by hand
    std::size_t stepIndex = 1;
    bool paused = false;
    bool areas = true;
    std::array<bool, 3> show{true, true, true};

    bool aiming = false;
    orb::Vec2 aimStart;  // world coordinates (relative to the sun)
    orb::Vec2 mouse;     // world coordinates
    SDL_FPoint mouseScreen{0.0F, 0.0F};

    std::string flash;
    Uint64 flashUntil = 0;

    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

SDL_FPoint toScreen(orb::Vec2 p) { return {kCx + static_cast<float>(p.x), kCy + static_cast<float>(p.y)}; }

orb::Vec2 toWorld(SDL_FPoint p) { return {static_cast<double>(p.x - kCx), static_cast<double>(p.y - kCy)}; }

bool inStage(SDL_FPoint p) { return p.x >= kStageX && p.x <= kStageX + kStageW && p.y >= kStageY && p.y <= kStageY + kStageH; }

orb::State presetState(int preset) {
    switch (preset) {
        case 0: return orb::circularState({0.0, -130.0}, kMu);
        case 2: return orb::apsisState(340.0, 0.85, orb::kPi, kMu);
        default: return orb::apsisState(170.0, 0.5, orb::kPi, kMu);
    }
}

orb::State stepWith(Method m, const orb::State& s, double h) {
    switch (m) {
        case Method::Euler: return orb::stepExplicitEuler(s, kMu, h);
        case Method::Semi: return orb::stepSemiImplicit(s, kMu, h);
        case Method::Verlet: return orb::stepVelocityVerlet(s, kMu, h);
    }
    return s;
}

void flashMessage(App& app, const char* text) {
    app.flash = text;
    app.flashUntil = SDL_GetTicks() + 2200;
}

/// Starts over from `s`: every body at the same place, trails and clocks cleared, exact curve and sectors rebuilt.
void launch(App& app, const orb::State& s) {
    app.start = s;
    app.conic = orb::conicFromState(s, kMu);
    app.exact = s;
    app.energy0 = orb::energy(s, kMu);
    app.momentum0 = orb::angularMomentum(s);
    app.simTime = 0.0;
    app.accumulator = 0.0;
    app.aiming = false;
    for (Body& b : app.bodies) {
        b.s = s;
        b.lost = false;
        b.trail.clear();
        b.trail.push(s.pos);
    }
    app.orbitPts.clear();
    app.wedges.clear();
    if (app.conic) {
        app.orbitPts = orb::conicPoints(*app.conic, 240, 3000.0);
        if (app.conic->bound) {
            for (int k = 0; k < kWedgeCount; ++k) {
                std::vector<orb::Vec2> rim;
                for (int j = 0; j <= kWedgeSamples; ++j) {
                    const double t = app.conic->period * (static_cast<double>(k) + static_cast<double>(j) / kWedgeSamples) / kWedgeCount;
                    rim.push_back(orb::propagate(s, kMu, t).pos);
                }
                app.wedges.push_back(std::move(rim));
            }
        }
    }
}

void setPreset(App& app, int preset) {
    app.preset = preset;
    launch(app, presetState(preset));
}

void setStepIndex(App& app, std::size_t index) {
    app.stepIndex = std::min(index, kSteps.size() - 1);
    app.accumulator = 0.0;  // a leftover of the old step size would fire too early or in a burst
}

void apply(App& app, Action action, int value) {
    switch (action) {
        case Action::Preset: setPreset(app, value); break;
        case Action::Restart: launch(app, app.start); break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Areas: app.areas = !app.areas; break;
        case Action::StepSize: setStepIndex(app, static_cast<std::size_t>(value)); break;
        case Action::Show: app.show[static_cast<std::size_t>(value)] = !app.show[static_cast<std::size_t>(value)]; break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Preset: return app.preset == c.value;
        case Action::Restart: return false;
        case Action::Pause: return app.paused;
        case Action::Areas: return app.areas;
        case Action::StepSize: return app.stepIndex == static_cast<std::size_t>(c.value);
        case Action::Show: return app.show[static_cast<std::size_t>(c.value)];
    }
    return false;
}

void layoutChips(App& app) {
    float x = 40.0F;
    float y = kRowOneY;
    auto label = [&](const char* text) {
        app.labels.push_back({x, y + 5.0F, text});
        x += app.hud.width(text) + 14.0F;
    };
    auto chip = [&](Action action, int value, const std::string& text) {
        const float w = app.hud.width(text, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({action, value, text, SDL_FRect{x, y, w, kChipH}});
        x += w + 8.0F;
    };
    label("Orbit");
    chip(Action::Preset, 0, "1 Circle");
    chip(Action::Preset, 1, "2 Ellipse");
    chip(Action::Preset, 2, "3 Comet");
    x += 16.0F;
    chip(Action::Restart, 0, "R Restart");
    chip(Action::Pause, 0, "Space Pause");
    x += 16.0F;
    chip(Action::Areas, 0, "A Areas");

    x = 40.0F;
    y = kRowTwoY;
    label("h (s)");
    for (std::size_t i = 0; i < kSteps.size(); ++i) chip(Action::StepSize, static_cast<int>(i), kStepLabels[i]);
    x += 16.0F;
    chip(Action::Show, 0, "E Euler");
    chip(Action::Show, 1, "S Semi-implicit");
    chip(Action::Show, 2, "V Verlet");
}

/// The state a drag from `aimStart` to the mouse would launch.
orb::State aimedState(const App& app) {
    const orb::Vec2 drag = app.mouse - app.aimStart;
    if (orb::length(drag) < kClickDrag) return orb::circularState(app.aimStart, kMu);
    return orb::launchState(app.aimStart, drag, kSpeedPerPx);
}

void finishAim(App& app) {
    const orb::State s = aimedState(app);
    app.aiming = false;
    const auto c = orb::conicFromState(s, kMu);
    if (!c) {
        flashMessage(app, "That is a straight fall, aim sideways");
        return;
    }
    if (c->rPeri < kMinPeriapsis) {
        flashMessage(app, "That orbit would hit the sun");
        return;
    }
    app.preset = -1;
    launch(app, s);
}

/// One frame's worth of fixed steps. The exact state is looked up once, at the time the steps have reached.
void advance(App& app, double frameDt) {
    if (app.paused || app.aiming) return;
    app.accumulator += std::min(frameDt, kMaxFrameTime) * kSimSpeed;
    const double h = kSteps[app.stepIndex];
    int steps = 0;
    while (app.accumulator >= h && steps < kMaxStepsPerFrame) {
        for (std::size_t i = 0; i < app.bodies.size(); ++i) {
            Body& b = app.bodies[i];
            if (b.lost) continue;
            b.s = stepWith(static_cast<Method>(i), b.s, h);
            if (!orb::isUsable(b.s, kLostMin, kLostMax)) {
                b.lost = true;
                continue;
            }
            b.trail.push(b.s.pos);
        }
        app.simTime += h;
        app.accumulator -= h;
        ++steps;
    }
    if (app.accumulator >= h) app.accumulator = 0.0;  // hit the cap: drop the rest instead of running behind forever
    app.exact = orb::propagate(app.start, kMu, app.simTime);
}

// ---------------------------------------------------------------------------------------------------------------- drawing

void line(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, float width = 1.0F) {
    gd::setColor(r, c);
    SDL_RenderLine(r, a.x, a.y, b.x, b.y);
    if (width > 1.0F) SDL_RenderLine(r, a.x + 0.8F, a.y + 0.8F, b.x + 0.8F, b.y + 0.8F);
}

void fillFan(SDL_Renderer* r, SDL_FPoint center, const std::vector<SDL_FPoint>& rim, gd::Color c) {
    if (rim.size() < 2) return;
    const SDL_FColor col = gd::fcolor(c);
    std::vector<SDL_Vertex> v;
    std::vector<int> idx;
    v.push_back({center, col, {0, 0}});
    for (const SDL_FPoint& p : rim) v.push_back({p, col, {0, 0}});
    for (int i = 1; i + 1 < static_cast<int>(v.size()); ++i) idx.insert(idx.end(), {0, i, i + 1});
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

std::vector<Star> makeStars() {
    std::vector<Star> out;
    unsigned seed = 20241106U;
    auto next = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return static_cast<float>((seed >> 8) & 0xFFFFU) / 65535.0F;
    };
    for (int i = 0; i < 170; ++i) {
        const float x = kStageX + 4.0F + next() * (kStageW - 8.0F);
        const float y = kStageY + 4.0F + next() * (kStageH - 8.0F);
        const float size = next() > 0.93F ? 2.0F : 1.0F;
        out.push_back({x, y, size, 0.25F + next() * 0.6F, next() * 6.2831853F});
    }
    return out;
}

void drawSky(App& app, double clock) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kStageX, kStageY, kStageW, kStageH, kSky0, gd::kInk);
    gd::gradient(r, kStageX + 1.0F, kStageY + 1.0F, kStageW - 2.0F, kStageH - 2.0F, kSky0, kSky1);
    // A soft nebula glow off to one side keeps the dark from looking flat.
    gd::disc(r, kStageX + 760.0F, kStageY + 90.0F, 230.0F, gd::Color{70, 60, 160, 26}, gd::Color{70, 60, 160, 0}, 56);
    gd::disc(r, kStageX + 130.0F, kStageY + 300.0F, 200.0F, gd::Color{13, 148, 136, 22}, gd::Color{13, 148, 136, 0}, 56);
    for (const Star& s : app.stars) {
        const double twinkle = 0.65 + 0.35 * std::sin(clock * 1.6 + static_cast<double>(s.phase));
        const auto a = static_cast<Uint8>(255.0 * static_cast<double>(s.base) * twinkle);
        gd::fillRect(r, s.x, s.y, s.size, s.size, gd::Color{235, 240, 255, a});
    }
    // Distance rings every 100 px, so speed and size have a scale.
    for (int ring = 100; ring <= 500; ring += 100) gd::ring(r, kCx, kCy, static_cast<float>(ring), gd::Color{255, 255, 255, 14}, 96);
}

void drawSun(SDL_Renderer* r, double clock) {
    const float pulse = 1.0F + 0.04F * static_cast<float>(std::sin(clock * 2.0));
    gd::disc(r, kCx, kCy, 78.0F * pulse, gd::Color{255, 170, 60, 70}, gd::Color{255, 140, 40, 0}, 56);
    gd::disc(r, kCx, kCy, 38.0F * pulse, gd::Color{255, 200, 90, 150}, gd::Color{255, 150, 40, 0}, 48);
    gd::disc(r, kCx, kCy, static_cast<float>(kSunRadius), gd::Color{255, 247, 214, 255}, gd::Color{255, 190, 70, 255}, 40);
}

void drawWedges(App& app) {
    if (!app.areas || app.wedges.empty() || !app.conic) return;
    SDL_Renderer* r = app.renderer;
    const double phase = std::fmod(app.simTime, app.conic->period) / app.conic->period;
    const int current = std::min(static_cast<int>(phase * kWedgeCount), kWedgeCount - 1);
    for (int k = 0; k < kWedgeCount; ++k) {
        std::vector<SDL_FPoint> rim;
        for (const orb::Vec2& p : app.wedges[static_cast<std::size_t>(k)]) rim.push_back(toScreen(p));
        const bool now = k == current;
        const gd::Color fill = now ? gd::Color{255, 255, 255, 62} : (k % 2 == 0 ? gd::Color{110, 150, 255, 30} : gd::Color{255, 255, 255, 14});
        fillFan(r, {kCx, kCy}, rim, fill);
        line(r, {kCx, kCy}, rim.front(), gd::Color{255, 255, 255, 46});
    }
}

/// The orbit as a polyline. Dashes are measured along the curve in pixels, so they keep their length wherever the
/// samples are far apart (the far ends of a hyperbola).
void drawCurve(SDL_Renderer* r, const std::vector<orb::Vec2>& pts, gd::Color c, bool dashed) {
    constexpr float kDashOn = 8.0F;
    constexpr float kDashPeriod = 14.0F;
    float along = 0.0F;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const SDL_FPoint a = toScreen(pts[i]);
        const SDL_FPoint b = toScreen(pts[i + 1]);
        const float len = std::hypot(b.x - a.x, b.y - a.y);
        if (!dashed) {
            line(r, a, b, c);
            continue;
        }
        // Long segments are cut into pieces of about 4 px so the dash pattern stays exact.
        const int pieces = std::max(1, static_cast<int>(std::ceil(len / 4.0F)));
        for (int k = 0; k < pieces; ++k) {
            const float t0 = static_cast<float>(k) / static_cast<float>(pieces);
            const float t1 = static_cast<float>(k + 1) / static_cast<float>(pieces);
            if (std::fmod(along + len * t0, kDashPeriod) < kDashOn) {
                line(r, {a.x + (b.x - a.x) * t0, a.y + (b.y - a.y) * t0}, {a.x + (b.x - a.x) * t1, a.y + (b.y - a.y) * t1}, c);
            }
        }
        along += len;
    }
}

void drawTrail(App& app, const Body& b, gd::Color c) {
    SDL_Renderer* r = app.renderer;
    const std::size_t n = b.trail.size();
    if (n < 2) return;
    constexpr int kChunks = 8;
    std::vector<SDL_FPoint> pts;
    for (int k = 0; k < kChunks; ++k) {
        const std::size_t from = n * static_cast<std::size_t>(k) / kChunks;
        const std::size_t to = std::min(n, n * static_cast<std::size_t>(k + 1) / kChunks + 1);
        if (to - from < 2) continue;
        pts.clear();
        for (std::size_t i = from; i < to; ++i) pts.push_back(toScreen(b.trail.at(i)));
        const float fade = static_cast<float>(k + 1) / kChunks;
        // A faint wide pass for the glow, then the line itself.
        gd::setColor(r, gd::Color{c.r, c.g, c.b, static_cast<Uint8>(40.0F * fade)});
        for (const float o : {-1.0F, 1.0F}) {
            for (SDL_FPoint& p : pts) p.y += o;
            SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
            for (SDL_FPoint& p : pts) p.y -= o;
        }
        gd::setColor(r, gd::Color{c.r, c.g, c.b, static_cast<Uint8>(70.0F + 185.0F * fade)});
        SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
        for (SDL_FPoint& p : pts) p.x += 0.8F;
        SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
    }
    // With big steps the polyline is the whole story: show where each step landed.
    if (kSteps[app.stepIndex] >= 0.1) {
        for (std::size_t i = n > 70 ? n - 70 : 0; i < n; ++i) {
            const SDL_FPoint p = toScreen(b.trail.at(i));
            gd::fillRect(r, p.x - 1.5F, p.y - 1.5F, 3.0F, 3.0F, gd::Color{c.r, c.g, c.b, 200});
        }
    }
}

void drawMarker(SDL_Renderer* r, orb::Vec2 pos, gd::Color c, bool lost) {
    const SDL_FPoint p = toScreen(pos);
    if (lost) {
        // A cross where it was lost.
        line(r, {p.x - 5.0F, p.y - 5.0F}, {p.x + 5.0F, p.y + 5.0F}, c, 2.0F);
        line(r, {p.x - 5.0F, p.y + 5.0F}, {p.x + 5.0F, p.y - 5.0F}, c, 2.0F);
        return;
    }
    gd::disc(r, p.x, p.y, 11.0F, gd::Color{c.r, c.g, c.b, 90}, gd::Color{c.r, c.g, c.b, 0}, 24);
    gd::disc(r, p.x, p.y, 4.5F, c, 20);
    gd::ring(r, p.x, p.y, 4.5F, gd::Color{255, 255, 255, 230}, 20);
}

void drawExactMarker(SDL_Renderer* r, orb::Vec2 pos) {
    const SDL_FPoint p = toScreen(pos);
    gd::ring(r, p.x, p.y, 9.0F, gd::Color{255, 255, 255, 235}, 28);
    gd::ring(r, p.x, p.y, 10.0F, gd::Color{255, 255, 255, 110}, 28);
    gd::disc(r, p.x, p.y, 2.0F, gd::Color{255, 255, 255, 255}, 12);
}

void drawArrow(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c) {
    line(r, a, b, c, 2.0F);
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 2.0F) return;
    const float ux = dx / len;
    const float uy = dy / len;
    line(r, b, {b.x - 11.0F * ux + 5.5F * uy, b.y - 11.0F * uy - 5.5F * ux}, c, 2.0F);
    line(r, b, {b.x - 11.0F * ux - 5.5F * uy, b.y - 11.0F * uy + 5.5F * ux}, c, 2.0F);
}

void drawAim(App& app) {
    SDL_Renderer* r = app.renderer;
    const orb::State s = aimedState(app);
    const auto c = orb::conicFromState(s, kMu);
    const bool refused = !c || c->rPeri < kMinPeriapsis;
    const gd::Color tone = refused ? gd::Color{255, 122, 40, 255} : (c->bound ? gd::Color{255, 255, 255, 255} : gd::Color{255, 190, 90, 255});
    if (c) drawCurve(r, orb::conicPoints(*c, 240, 3000.0), gd::Color{tone.r, tone.g, tone.b, 190}, false);
    const SDL_FPoint a = toScreen(app.aimStart);
    if (orb::length(app.mouse - app.aimStart) >= kClickDrag) drawArrow(r, a, app.mouseScreen, gd::Color{255, 255, 255, 230});
    gd::disc(r, a.x, a.y, 5.0F, tone, 20);
    gd::ring(r, a.x, a.y, 7.0F, gd::Color{255, 255, 255, 180}, 24);

    char text[64];
    const double speed = orb::length(s.vel);
    if (!c) {
        SDL_snprintf(text, sizeof text, "straight fall");
    } else if (refused) {
        SDL_snprintf(text, sizeof text, "hits the sun");
    } else if (c->bound) {
        SDL_snprintf(text, sizeof text, "v %.0f  e %.2f  T %.1f s", speed, c->e, c->period);
    } else {
        SDL_snprintf(text, sizeof text, "v %.0f  e %.2f  escapes", speed, c->e);
    }
    const float w = app.hud.width(text);
    float tx = std::clamp(a.x + 14.0F, kStageX + 8.0F, kStageX + kStageW - w - 8.0F);
    float ty = std::clamp(a.y + 12.0F, kStageY + 8.0F, kStageY + kStageH - 30.0F);
    if (orb::length(app.mouse - app.aimStart) >= kClickDrag) {
        tx = std::clamp(app.mouseScreen.x + 14.0F, kStageX + 8.0F, kStageX + kStageW - w - 8.0F);
        ty = std::clamp(app.mouseScreen.y + 12.0F, kStageY + 8.0F, kStageY + kStageH - 30.0F);
    }
    gd::fillRect(r, tx - 6.0F, ty - 3.0F, w + 12.0F, 26.0F, gd::Color{6, 10, 24, 215});
    app.hud.text(r, tx, ty, text, tone);
}

void panel(SDL_Renderer* r, float x, float y, float w, float h) {
    gd::fillRect(r, x, y, w, h, gd::Color{6, 10, 24, 200});
    gd::strokeRect(r, x, y, w, h, gd::Color{255, 255, 255, 46});
}

void drawTable(App& app) {
    SDL_Renderer* r = app.renderer;
    constexpr float kX = kStageX + kStageW - kReadoutW - 12.0F;
    constexpr float kY = 54.0F;
    panel(r, kX, kY, kReadoutW, 144.0F);
    app.hud.text(r, kX + 12.0F, kY + 8.0F, "DRIFT SINCE LAUNCH", kLight, gd::Hud::Style::Bold);
    const float textX = kX + 30.0F;
    char row[96];
    std::snprintf(row, sizeof row, "%-10s%8s%8s%8s", "", "dE", "dL", "dist");
    app.hud.text(r, textX, kY + 32.0F, row, kDim);
    float y = kY + 56.0F;

    // Exact orbit: the same numbers, computed from the closed form. They stay at zero up to rounding.
    std::snprintf(row, sizeof row, "%-10s%8s%8s%8s", "Kepler", orb::formatPercent(orb::relativeDrift(orb::energy(app.exact, kMu), app.energy0)).c_str(),
                 orb::formatPercent(orb::relativeDrift(orb::angularMomentum(app.exact), app.momentum0)).c_str(), "-");
    gd::disc(app.renderer, kX + 18.0F, y + 11.0F, 5.0F, gd::kWhite, 16);
    app.hud.text(r, textX, y, row, kLight);
    y += 22.0F;

    for (std::size_t i = 0; i < app.bodies.size(); ++i) {
        const Body& b = app.bodies[i];
        const gd::Color c = kMethodStage[i];
        char dist[16];
        if (b.lost) {
            std::snprintf(row, sizeof row, "%-10s%8s%8s%8s", kMethodNames[i], "lost", "lost", "-");
        } else {
            const double d = orb::length(b.s.pos - app.exact.pos);
            if (d >= 10000.0) {
                SDL_snprintf(dist, sizeof dist, ">9999");
            } else {
                SDL_snprintf(dist, sizeof dist, "%.1f", d);
            }
            std::snprintf(row, sizeof row, "%-10s%8s%8s%8s", kMethodNames[i], orb::formatPercent(orb::relativeDrift(orb::energy(b.s, kMu), app.energy0)).c_str(),
                         orb::formatPercent(orb::relativeDrift(orb::angularMomentum(b.s), app.momentum0)).c_str(), dist);
        }
        const gd::Color text = app.show[i] ? c : gd::Color{c.r, c.g, c.b, 90};
        gd::disc(r, kX + 18.0F, y + 11.0F, 5.0F, text, 16);
        app.hud.text(r, textX, y, row, text);
        y += 22.0F;
    }
}

void drawInfo(App& app) {
    SDL_Renderer* r = app.renderer;
    if (!app.conic) return;
    const orb::Conic& c = *app.conic;
    constexpr float kW = kReadoutW;
    constexpr float kY = 54.0F + 144.0F + 8.0F;
    const float x = kStageX + kStageW - kW - 12.0F;
    char text[64];
    std::vector<std::pair<std::string, gd::Color>> lines;
    if (c.bound) {
        std::snprintf(text, sizeof text, "peri %.0f  apo %.0f px", c.rPeri, c.rApo);
        lines.emplace_back(text, kDim);
        std::snprintf(text, sizeof text, "t %.1f s  (%.1f orbits)", app.simTime, app.simTime / c.period);
        lines.emplace_back(text, kLight);
        std::snprintf(text, sizeof text, "period %.1f s", c.period);
        lines.emplace_back(text, kDim);
    } else {
        std::snprintf(text, sizeof text, "peri %.0f px, escapes", c.rPeri);
        lines.emplace_back(text, kDim);
        std::snprintf(text, sizeof text, "t %.1f s", app.simTime);
        lines.emplace_back(text, kLight);
    }
    std::snprintf(text, sizeof text, "step h %.2f s", kSteps[app.stepIndex]);
    lines.emplace_back(text, kDim);

    panel(r, x, kY, kW, 40.0F + 24.0F * static_cast<float>(lines.size()));
    const char* shape = c.e < 0.01 ? "CIRCLE" : (c.bound ? "ELLIPSE" : (c.e < 1.0 + 1e-6 ? "PARABOLA" : "HYPERBOLA"));
    std::snprintf(text, sizeof text, "%s  e %.2f", shape, c.e);
    app.hud.text(r, x + 12.0F, kY + 8.0F, text, kLight, gd::Hud::Style::Bold);
    float y = kY + 32.0F;
    for (const auto& [line, color] : lines) {
        app.hud.text(r, x + 12.0F, y, line, color);
        y += 24.0F;
    }
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Label& l : app.labels) app.hud.text(r, l.x, l.y, l.text, gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        gd::Color fill = on ? gd::kTeal : gd::kWhite;
        if (c.action == Action::Show && on) fill = kMethodChip[static_cast<std::size_t>(c.value)];
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, c.label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const double clock = static_cast<double>(SDL_GetTicks()) / 1000.0;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ ORBIT MECHANICS ]", "exact orbit vs three step methods");
    drawSky(app, clock);

    const SDL_Rect clip{static_cast<int>(kStageX) + 1, static_cast<int>(kStageY) + 1, static_cast<int>(kStageW) - 2, static_cast<int>(kStageH) - 2};
    SDL_SetRenderClipRect(r, &clip);
    if (app.aiming) {
        drawSun(r, clock);
        drawAim(app);
    } else {
        drawWedges(app);
        drawCurve(r, app.orbitPts, gd::Color{255, 255, 255, 120}, true);
        drawSun(r, clock);
        for (std::size_t i = 0; i < app.bodies.size(); ++i) {
            if (app.show[i]) drawTrail(app, app.bodies[i], kMethodStage[i]);
        }
        for (std::size_t i = 0; i < app.bodies.size(); ++i) {
            if (app.show[i]) drawMarker(r, app.bodies[i].s.pos, kMethodStage[i], app.bodies[i].lost);
        }
        drawExactMarker(r, app.exact.pos);
    }
    SDL_SetRenderClipRect(r, nullptr);

    drawTable(app);
    drawInfo(app);

    if (SDL_GetTicks() < app.flashUntil) {
        const float w = app.hud.width(app.flash, gd::Hud::Style::Bold) + 28.0F;
        const float x = kStageX + kStageW / 2.0F - w / 2.0F;
        gd::card(r, x, kStageY + kStageH - 50.0F, w, 32.0F, gd::kWhite, gd::kOrange);
        app.hud.text(r, x + 14.0F, kStageY + kStageH - 45.0F, app.flash, gd::kOrange, gd::Hud::Style::Bold);
    }

    app.hud.text(r, 40.0F, 421.0F, "click: circular orbit   drag: launch velocity   right click / Esc: cancel   - / +: step h", gd::kMuted);
    drawChips(app);
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
    if (!SDL_CreateWindowAndRenderer("orbit-mechanics", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->stars = makeStars();
    layoutChips(*app);
    setPreset(*app, 1);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouseScreen = {event->motion.x, event->motion.y};
            app.mouse = toWorld(app.mouseScreen);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouseScreen = p;
            app.mouse = toWorld(p);
            if (event->button.button == SDL_BUTTON_RIGHT) {
                app.aiming = false;
                break;
            }
            if (event->button.button != SDL_BUTTON_LEFT) break;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action, c.value);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inStage(p) && orb::length(app.mouse) >= kMinStartDistance) {
                app.aiming = true;
                app.aimStart = app.mouse;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT && app.aiming) {
                app.mouseScreen = {event->button.x, event->button.y};
                app.mouse = toWorld(app.mouseScreen);
                finishAim(app);
            }
            break;
        // Letting go outside the window never reaches us, so leaving or losing focus cancels the aim.
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.aiming = false; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE:
                    if (app.aiming) {
                        app.aiming = false;
                    } else {
                        return SDL_APP_SUCCESS;
                    }
                    break;
                case SDLK_1: apply(app, Action::Preset, 0); break;
                case SDLK_2: apply(app, Action::Preset, 1); break;
                case SDLK_3: apply(app, Action::Preset, 2); break;
                case SDLK_R: apply(app, Action::Restart, 0); break;
                case SDLK_SPACE: apply(app, Action::Pause, 0); break;
                case SDLK_A: apply(app, Action::Areas, 0); break;
                case SDLK_E: apply(app, Action::Show, 0); break;
                case SDLK_S: apply(app, Action::Show, 1); break;
                case SDLK_V: apply(app, Action::Show, 2); break;
                case SDLK_MINUS:
                case SDLK_KP_MINUS:
                    if (app.stepIndex > 0) setStepIndex(app, app.stepIndex - 1);
                    break;
                case SDLK_PLUS:
                case SDLK_EQUALS:
                case SDLK_KP_PLUS: setStepIndex(app, app.stepIndex + 1); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetPerformanceCounter();
    const double dt = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    advance(app, dt);
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
