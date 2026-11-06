// Hohmann transfer with SDL3 main callbacks.
//
// A ship circles a planet on a low orbit and has to meet a target on a higher one. The cheap way is two short burns along
// the direction of travel: the first stretches the orbit into an ellipse whose far point touches the target orbit, the
// second, at that far point, rounds it off. The catch is timing. While the ship flies half the ellipse the target keeps
// moving, so the burn has to wait until the target is just far enough ahead (the phase angle) to arrive at the far point
// together with the ship. The dashed ellipse shows where a burn right now would lead, the hollow ring where the target
// will be by then; when they meet, the window is open.
//
// The other aim ("At target") points the thrust at the target and fires with the same fuel. The ship is on another orbit
// afterwards, not on the way to the target. The autopilot fires at the exact moment inside a frame, so it is the
// reference for the timing a player can only approximate.
// All the maths is in logic.hpp (no SDL, covered by test.cpp), this file owns input, the clock and the picture.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <optional>
#include <string>
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
constexpr float kCx = kStageX + 330.0F;  // the planet sits left of centre, the readouts take the right third
constexpr float kCy = kStageY + kStageH / 2.0F;
constexpr float kPanelW = 340.0F;
constexpr float kPanelX = kStageX + kStageW - kPanelW - 12.0F;
constexpr float kChipH = 32.0F;
constexpr float kRowOneY = 446.0F;
constexpr float kRowTwoY = 488.0F;

constexpr double kMu = 600000.0;  // a circle at 80 px takes 5.8 s, at 170 px 18 s
constexpr double kR1 = 80.0;
constexpr double kPlanetR = 20.0;
constexpr std::array<double, 3> kTargetRadii{120.0, 150.0, 170.0};
constexpr std::array<double, 4> kSpeeds{0.25, 0.5, 1.0, 2.0};
constexpr std::array<const char*, 4> kSpeedLabels{"0.25x", "0.5x", "1x", "2x"};
constexpr double kLeadSeconds = 2.5;       // a restart puts the window this far away
constexpr double kMaxFrameTime = 0.25;     // clamp: no jump after a stalled tab
constexpr double kChunk = 1.0 / 90.0;      // longest piece of time the mission is advanced in (trail and burn effects)
constexpr double kReplayDelay = 6.5;       // real seconds the autopilot shows its result before it starts over
constexpr double kMissTol = 14.0;          // closer than this after the second burn is a rendezvous
constexpr double kApsisTol = 0.15;         // seconds around the end of the ellipse that count as "now"
constexpr double kFxTime = 0.7;            // flame and pulse after a burn
constexpr std::size_t kShipTrailCap = 420;
constexpr std::size_t kTargetTrailCap = 150;

constexpr gd::Color kLight{232, 238, 248};
constexpr gd::Color kDim{148, 163, 184};
constexpr gd::Color kSky0{9, 13, 31};
constexpr gd::Color kSky1{22, 30, 62};
constexpr gd::Color kShipColor{45, 212, 191};
constexpr gd::Color kTargetColor{255, 140, 60};

enum class Action { SetAim, Burn, Autopilot, Restart, Pause, TargetOrbit, Speed };

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

struct Star {
    float x, y, size, base, phase;
};

/// The last positions of a body, oldest dropped first. Points closer than 2 px to the previous one are skipped.
struct Trail {
    std::deque<hoh::Vec2> pts;
    std::size_t cap = 100;

    void clear() { pts.clear(); }
    void push(hoh::Vec2 p) {
        if (!pts.empty() && hoh::length(p - pts.back()) < 2.0) return;
        pts.push_back(p);
        if (pts.size() > cap) pts.pop_front();
    }
};

/// Where a burn (now, or the coast to the far point) would lead, for the dashed preview.
struct Ghost {
    bool valid = false;
    std::vector<hoh::Vec2> curve;
    std::optional<hoh::Vec2> apsis;       // where the ship is at the end of the ellipse
    std::optional<hoh::Vec2> targetThen;  // where the target is at that moment
    double seconds = 0.0;
    double rPeri = 0.0;
    double rApo = 0.0;
    bool bound = false;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<Label> labels;
    std::vector<Star> stars;

    hoh::Mission m;
    hoh::Aim aim = hoh::Aim::Prograde;
    bool autopilot = true;
    bool paused = false;
    std::size_t targetIndex = 2;
    std::size_t speedIndex = 2;
    Trail shipTrail;
    Trail targetTrail;

    // Reused every frame so the loop does not allocate: the two orbit circles (rebuilt on restart), the preview and scratch lists.
    std::vector<hoh::Vec2> ringStart;
    std::vector<hoh::Vec2> ringTarget;
    Ghost ghost;
    std::vector<hoh::Vec2> arc;
    std::vector<SDL_FPoint> rim;

    double clock = 0.0;      // real seconds since the start, for glints and effects only
    double fxStart = -100.0;
    double idleSince = 0.0;  // the autopilot has nothing to do and the player did not touch anything since then
    double crashClock = -100.0;
    bool wasCrashed = false;

    std::string flash;
    Uint64 flashUntil = 0;

    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

// ---------------------------------------------------------------------------------------------------------------- mapping

SDL_FPoint toScreen(hoh::Vec2 p) { return {kCx + static_cast<float>(p.x), kCy - static_cast<float>(p.y)}; }

bool inStage(SDL_FPoint p) { return p.x >= kStageX && p.x <= kStageX + kStageW && p.y >= kStageY && p.y <= kStageY + kStageH; }

/// The phase error (radians) that still puts the ship within kMissTol of the target at the far end: the chord is about r2 * error.
double windowTol(const hoh::Mission& m) { return m.sc.r2 > 1.0 ? kMissTol / m.sc.r2 : 0.0; }

hoh::Scenario scenario(const App& app) { return hoh::Scenario{kMu, kR1, kTargetRadii[app.targetIndex], kPlanetR}; }

void circlePoints(std::vector<hoh::Vec2>& out, double radius, int n);

void flashMessage(App& app, const std::string& text) {
    app.flash = text;
    app.flashUntil = SDL_GetTicks() + 2400;
}

void restart(App& app) {
    const hoh::Scenario sc = scenario(app);
    app.m = hoh::startMission(sc, app.aim, hoh::phaseForWait(sc.mu, sc.r1, sc.r2, kLeadSeconds));
    circlePoints(app.ringStart, sc.r1, 160);
    circlePoints(app.ringTarget, sc.r2, 200);
    app.shipTrail.clear();
    app.targetTrail.clear();
    app.shipTrail.push(app.m.ship.pos);
    app.targetTrail.push(hoh::targetState(app.m).pos);
    app.fxStart = -100.0;
    app.crashClock = -100.0;
    app.wasCrashed = false;
    app.idleSince = app.clock;
    app.flashUntil = 0;
}

void noteBurn(App& app) {
    app.fxStart = app.clock;
    char text[64];
    std::snprintf(text, sizeof text, "Burn %d: %+.1f px/s", app.m.burns, app.m.lastBurnDv);
    flashMessage(app, text);
}

void userBurn(App& app) {
    app.idleSince = app.clock;
    if (app.m.crashed) {
        flashMessage(app, "The ship is gone, press R");
        return;
    }
    if (!hoh::burn(app.m)) {
        flashMessage(app, "Both burns are done, press R");
        return;
    }
    app.shipTrail.push(app.m.ship.pos);
    noteBurn(app);
}

/// One frame's worth of mission time, cut into short chunks so burns and trails stay smooth.
void update(App& app, double frameDt) {
    app.clock += frameDt;
    if (app.paused) return;
    const double dt = std::min(frameDt, kMaxFrameTime) * kSpeeds[app.speedIndex];
    if (!(dt > 0.0)) return;
    const int chunks = std::max(1, static_cast<int>(std::ceil(dt / kChunk)));
    const double h = dt / chunks;
    for (int i = 0; i < chunks; ++i) {
        const int burnsBefore = app.m.burns;
        hoh::advance(app.m, h, app.autopilot);
        if (app.m.burns != burnsBefore) {
            noteBurn(app);
            app.idleSince = app.clock;
        }
        app.shipTrail.push(app.m.ship.pos);
        app.targetTrail.push(hoh::targetState(app.m).pos);
    }
    if (app.m.crashed && !app.wasCrashed) {
        app.wasCrashed = true;
        app.crashClock = app.clock;
    }
    // The autopilot shows its result for a while, then starts over (any touch of the controls resets the wait).
    if (!app.autopilot || std::isfinite(hoh::autoEventTime(app.m))) {
        app.idleSince = app.clock;
    } else if (app.clock - app.idleSince > kReplayDelay) {
        restart(app);
    }
}

void apply(App& app, Action action, int value) {
    app.idleSince = app.clock;
    switch (action) {
        case Action::SetAim:
            app.aim = value == 0 ? hoh::Aim::Prograde : hoh::Aim::AtTarget;
            restart(app);
            break;
        case Action::Burn: userBurn(app); break;
        case Action::Autopilot: app.autopilot = !app.autopilot; break;
        case Action::Restart: restart(app); break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::TargetOrbit:
            app.targetIndex = static_cast<std::size_t>(std::clamp(value, 0, 2));
            restart(app);
            break;
        case Action::Speed: app.speedIndex = static_cast<std::size_t>(std::clamp(value, 0, static_cast<int>(kSpeeds.size()) - 1)); break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::SetAim: return (c.value == 0) == (app.aim == hoh::Aim::Prograde);
        case Action::Burn: return false;
        case Action::Autopilot: return app.autopilot;
        case Action::Restart: return false;
        case Action::Pause: return app.paused;
        case Action::TargetOrbit: return app.targetIndex == static_cast<std::size_t>(c.value);
        case Action::Speed: return app.speedIndex == static_cast<std::size_t>(c.value);
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
    label("Aim");
    chip(Action::SetAim, 0, "P Prograde");
    chip(Action::SetAim, 1, "T At target");
    x += 16.0F;
    chip(Action::Burn, 0, "B Burn");
    chip(Action::Autopilot, 0, "A Autopilot");
    x += 16.0F;
    chip(Action::Restart, 0, "R Restart");

    x = 40.0F;
    y = kRowTwoY;
    label("Target orbit");
    for (int i = 0; i < 3; ++i) chip(Action::TargetOrbit, i, i == 0 ? "1 Near" : (i == 1 ? "2 Mid" : "3 Far"));
    x += 16.0F;
    label("Speed");
    for (int i = 0; i < static_cast<int>(kSpeeds.size()); ++i) chip(Action::Speed, i, kSpeedLabels[static_cast<std::size_t>(i)]);
    x += 16.0F;
    chip(Action::Pause, 0, "Space Pause");
}

/// Where a burn now would lead (stage 0), or where the coast ends (stage 1). Nothing once both burns are done.
void makeGhost(const hoh::Mission& m, Ghost& g) {
    g.valid = false;
    g.curve.clear();
    g.apsis.reset();
    g.targetThen.reset();
    if (m.crashed || m.burns >= 2) return;
    hoh::State s = m.ship;
    // The textbook second burn is far away (the coast to the far end is the preview then); a burn toward the target is
    // always previewed for "now".
    if (m.burns == 0 || m.aim == hoh::Aim::AtTarget) {
        const double dv = hoh::plannedDv(m);
        s = m.aim == hoh::Aim::Prograde ? hoh::progradeBurn(s, dv) : hoh::aimBurn(s, hoh::targetState(m).pos, dv);
    }
    const auto c = hoh::conicFromState(s, kMu);
    if (!c) return;
    g.valid = true;
    g.bound = c->bound;
    g.rPeri = c->rPeri;
    g.rApo = c->rApo;
    g.curve = hoh::conicPoints(*c, 220, 3000.0);
    if (m.aim == hoh::Aim::Prograde) {
        const hoh::Apsis end = hoh::arrivalApsis(m);
        if (const auto t = hoh::timeToApsis(s, kMu, end)) {
            g.seconds = *t;
            g.apsis = hoh::conicPoint(*c, end == hoh::Apsis::Far ? hoh::kPi : 0.0);
            const double n2 = hoh::meanMotion(kMu, m.sc.r2);
            g.targetThen = hoh::circularState(m.sc.r2, m.targetAngle0 + n2 * (m.time + *t), kMu).pos;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------- drawing

void line(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, float width = 1.0F) {
    gd::setColor(r, c);
    SDL_RenderLine(r, a.x, a.y, b.x, b.y);
    if (width > 1.0F) SDL_RenderLine(r, a.x + 0.8F, a.y + 0.8F, b.x + 0.8F, b.y + 0.8F);
}

void fillTriangle(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, SDL_FPoint c, SDL_FColor ca, SDL_FColor cb, SDL_FColor cc) {
    const SDL_Vertex v[3] = {{a, ca, {0, 0}}, {b, cb, {0, 0}}, {c, cc, {0, 0}}};
    SDL_RenderGeometry(r, nullptr, v, 3, nullptr, 0);
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
    unsigned seed = 20261106U;
    auto next = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return static_cast<float>((seed >> 8) & 0xFFFFU) / 65535.0F;
    };
    for (int i = 0; i < 190; ++i) {
        const float x = kStageX + 4.0F + next() * (kStageW - 8.0F);
        const float y = kStageY + 4.0F + next() * (kStageH - 8.0F);
        const float size = next() > 0.94F ? 2.0F : 1.0F;
        out.push_back({x, y, size, 0.25F + next() * 0.6F, next() * 6.2831853F});
    }
    return out;
}

void drawSky(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kStageX, kStageY, kStageW, kStageH, kSky0, gd::kInk);
    gd::gradient(r, kStageX + 1.0F, kStageY + 1.0F, kStageW - 2.0F, kStageH - 2.0F, kSky0, kSky1);
    gd::disc(r, kStageX + 770.0F, kStageY + 80.0F, 240.0F, gd::Color{70, 60, 160, 24}, gd::Color{70, 60, 160, 0}, 56);
    gd::disc(r, kStageX + 90.0F, kStageY + 310.0F, 200.0F, gd::Color{13, 148, 136, 22}, gd::Color{13, 148, 136, 0}, 56);
    for (const Star& s : app.stars) {
        const double twinkle = 0.65 + 0.35 * std::sin(app.clock * 1.6 + static_cast<double>(s.phase));
        const auto a = static_cast<Uint8>(255.0 * static_cast<double>(s.base) * twinkle);
        gd::fillRect(r, s.x, s.y, s.size, s.size, gd::Color{235, 240, 255, a});
    }
}

/// A polyline in world coordinates, drawn solid or dashed. Dashes are measured along the curve in pixels.
void drawCurve(SDL_Renderer* r, const std::vector<hoh::Vec2>& pts, gd::Color c, bool dashed) {
    constexpr float kDashOn = 7.0F;
    constexpr float kDashPeriod = 13.0F;
    float along = 0.0F;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const SDL_FPoint a = toScreen(pts[i]);
        const SDL_FPoint b = toScreen(pts[i + 1]);
        const float len = std::hypot(b.x - a.x, b.y - a.y);
        if (!dashed) {
            line(r, a, b, c);
            continue;
        }
        const int pieces = std::max(1, static_cast<int>(std::ceil(len / 3.0F)));
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

void circlePoints(std::vector<hoh::Vec2>& out, double radius, int n) {
    out.clear();
    for (int i = 0; i <= n; ++i) {
        const double a = 2.0 * hoh::kPi * i / n;
        out.push_back({radius * std::cos(a), radius * std::sin(a)});
    }
}

/// An arc of radius `radius` from angle `a0` to `a1` (radians, either way round).
void arcPoints(std::vector<hoh::Vec2>& out, double radius, double a0, double a1, int n) {
    out.clear();
    for (int i = 0; i <= n; ++i) {
        const double a = a0 + (a1 - a0) * i / n;
        out.push_back({radius * std::cos(a), radius * std::sin(a)});
    }
}

void drawPlanet(SDL_Renderer* r, double clock) {
    gd::disc(r, kCx, kCy, 38.0F, gd::Color{110, 180, 255, 70}, gd::Color{110, 180, 255, 0}, 56);                // atmosphere haze
    gd::disc(r, kCx, kCy, static_cast<float>(kPlanetR), gd::Color{44, 120, 170}, gd::Color{16, 48, 96}, 48);      // body
    // Land masses that turn with the planet: a blob is seen on the near half only and squeezed toward the rim.
    struct Land {
        double lon, lat, size;
    };
    static constexpr Land kLand[] = {{0.3, 0.35, 5.5}, {2.1, -0.4, 6.5}, {3.6, 0.15, 5.0}, {5.0, -0.2, 4.5}, {1.2, -0.75, 3.5}};
    for (const Land& l : kLand) {
        const double lon = l.lon + clock * 0.22;
        const double facing = std::cos(lon);
        if (facing <= 0.05) continue;
        const float x = kCx + static_cast<float>(std::sin(lon) * std::cos(l.lat) * (kPlanetR - 3.0));
        const float y = kCy - static_cast<float>(std::sin(l.lat) * (kPlanetR - 3.0));
        gd::disc(r, x, y, static_cast<float>(l.size * (0.55 + 0.45 * facing)), gd::Color{74, 160, 120, static_cast<Uint8>(150.0 * facing)}, gd::Color{74, 160, 120, 0}, 20);
    }
    gd::disc(r, kCx - 5.0F, kCy - 5.0F, 14.0F, gd::Color{150, 220, 235, 130}, gd::Color{150, 220, 235, 0}, 32);  // lit side
    gd::disc(r, kCx, kCy, static_cast<float>(kPlanetR), gd::Color{4, 8, 24, 0}, gd::Color{4, 8, 24, 170}, 48);    // limb darkening
    gd::ring(r, kCx, kCy, static_cast<float>(kPlanetR), gd::Color{150, 210, 255, 140}, 48);
}

void drawTrail(SDL_Renderer* r, std::vector<SDL_FPoint>& pts, const Trail& t, gd::Color c, float strength) {
    const std::size_t n = t.pts.size();
    if (n < 2) return;
    constexpr std::size_t kChunks = 8;
    for (std::size_t k = 0; k < kChunks; ++k) {
        const std::size_t from = n * k / kChunks;
        const std::size_t to = std::min(n, n * (k + 1) / kChunks + 1);
        if (to - from < 2) continue;
        pts.clear();
        for (std::size_t i = from; i < to; ++i) pts.push_back(toScreen(t.pts[i]));
        const float fade = static_cast<float>(k + 1) / static_cast<float>(kChunks);
        gd::setColor(r, gd::Color{c.r, c.g, c.b, static_cast<Uint8>(46.0F * fade * strength)});
        for (const float o : {-1.0F, 1.0F}) {
            for (SDL_FPoint& p : pts) p.y += o;
            SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
            for (SDL_FPoint& p : pts) p.y -= o;
        }
        gd::setColor(r, gd::Color{c.r, c.g, c.b, static_cast<Uint8>((60.0F + 195.0F * fade) * strength)});
        SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
        for (SDL_FPoint& p : pts) p.x += 0.8F;
        SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
    }
}

SDL_FPoint unitScreen(hoh::Vec2 worldDir) { return {static_cast<float>(worldDir.x), -static_cast<float>(worldDir.y)}; }

void drawShip(SDL_Renderer* r, const hoh::Mission& m, double clock, double fxStart, bool ready) {
    const SDL_FPoint p = toScreen(m.ship.pos);
    const hoh::Vec2 dirW = hoh::unit(m.ship.vel);
    const SDL_FPoint d = unitScreen(dirW);
    const SDL_FPoint n{-d.y, d.x};

    if (ready) {
        const float pulse = 1.0F + 0.25F * static_cast<float>(std::sin(clock * 9.0));
        gd::ring(r, p.x, p.y, 17.0F * pulse, gd::Color{45, 255, 220, 220}, 36);
        gd::ring(r, p.x, p.y, 20.0F * pulse, gd::Color{45, 255, 220, 100}, 36);
    }

    // The burn: a flame against the thrust and a ring that spreads from the place of the burn.
    const double age = clock - fxStart;
    if (age >= 0.0 && age < kFxTime) {
        const float k = static_cast<float>(1.0 - age / kFxTime);
        const SDL_FPoint bd = unitScreen(m.lastBurnDir);
        const float sign = m.lastBurnDv >= 0.0 ? -1.0F : 1.0F;  // the exhaust leaves against the push
        const float flicker = 0.85F + 0.15F * static_cast<float>(std::sin(clock * 70.0));
        const float len = 30.0F * k * flicker;
        const SDL_FPoint root{p.x - d.x * 5.0F, p.y - d.y * 5.0F};
        const SDL_FPoint tip{p.x + bd.x * sign * (6.0F + len), p.y + bd.y * sign * (6.0F + len)};
        const SDL_FPoint nb{-bd.y, bd.x};
        fillTriangle(r, {root.x + nb.x * 4.0F, root.y + nb.y * 4.0F}, {root.x - nb.x * 4.0F, root.y - nb.y * 4.0F}, tip, gd::fcolor(gd::Color{255, 235, 170}, 0.95F),
                     gd::fcolor(gd::Color{255, 235, 170}, 0.95F), gd::fcolor(gd::Color{255, 80, 10}, 0.0F));
        const SDL_FPoint at = toScreen(m.lastBurnPos);
        gd::ring(r, at.x, at.y, 6.0F + 46.0F * (1.0F - k), gd::Color{255, 220, 150, static_cast<Uint8>(200.0F * k)}, 40);
    }

    gd::disc(r, p.x, p.y, 15.0F, gd::Color{kShipColor.r, kShipColor.g, kShipColor.b, 80}, gd::Color{kShipColor.r, kShipColor.g, kShipColor.b, 0}, 24);
    const SDL_FPoint tip{p.x + d.x * 10.0F, p.y + d.y * 10.0F};
    const SDL_FPoint left{p.x - d.x * 6.0F + n.x * 5.5F, p.y - d.y * 6.0F + n.y * 5.5F};
    const SDL_FPoint right{p.x - d.x * 6.0F - n.x * 5.5F, p.y - d.y * 6.0F - n.y * 5.5F};
    const SDL_FPoint notch{p.x - d.x * 3.0F, p.y - d.y * 3.0F};
    const SDL_FColor body = gd::fcolor(kShipColor);
    fillTriangle(r, tip, left, notch, body, body, body);
    fillTriangle(r, tip, notch, right, body, body, body);
    const gd::Color edge{235, 255, 250, 255};
    line(r, tip, left, edge);
    line(r, left, notch, edge);
    line(r, notch, right, edge);
    line(r, right, tip, edge);
}

void drawTarget(SDL_Renderer* r, hoh::Vec2 pos, double clock, bool docked) {
    const SDL_FPoint p = toScreen(pos);
    gd::disc(r, p.x, p.y, 16.0F, gd::Color{kTargetColor.r, kTargetColor.g, kTargetColor.b, 80}, gd::Color{kTargetColor.r, kTargetColor.g, kTargetColor.b, 0}, 24);
    const float s = 7.0F;
    const SDL_FColor body = gd::fcolor(kTargetColor);
    fillTriangle(r, {p.x, p.y - s}, {p.x + s, p.y}, {p.x, p.y + s}, body, body, body);
    fillTriangle(r, {p.x, p.y - s}, {p.x, p.y + s}, {p.x - s, p.y}, body, body, body);
    const gd::Color edge{255, 245, 230, 255};
    line(r, {p.x, p.y - s}, {p.x + s, p.y}, edge);
    line(r, {p.x + s, p.y}, {p.x, p.y + s}, edge);
    line(r, {p.x, p.y + s}, {p.x - s, p.y}, edge);
    line(r, {p.x - s, p.y}, {p.x, p.y - s}, edge);
    if (docked) {
        const float pulse = 1.0F + 0.1F * static_cast<float>(std::sin(clock * 6.0));
        gd::ring(r, p.x, p.y, 19.0F * pulse, gd::Color{120, 255, 230, 230}, 36);
    }
}

void pill(App& app, float x, float y, const std::string& text, gd::Color color) {
    const float w = app.hud.width(text);
    gd::fillRect(app.renderer, x - 5.0F, y - 1.0F, w + 10.0F, 21.0F, gd::Color{6, 10, 24, 200});
    app.hud.text(app.renderer, x, y - 2.0F, text, color);
}

void drawMarkerLabel(App& app, hoh::Vec2 pos, const char* text, gd::Color color, float offset) {
    const SDL_FPoint p = toScreen(pos);
    const float dx = p.x - kCx;
    const float dy = p.y - kCy;
    const float len = std::max(1.0F, std::hypot(dx, dy));
    const float w = app.hud.width(text);
    const float x = std::clamp(p.x + dx / len * offset - w / 2.0F, kStageX + 8.0F, kStageX + kStageW - w - 8.0F);
    const float y = std::clamp(p.y + dy / len * offset - 10.0F, kStageY + 6.0F, kStageY + kStageH - 28.0F);
    pill(app, x, y, text, color);
}

/// The preview of the next burn: the dashed ellipse, the end of it, and where the target will be by then.
void drawGhost(App& app, const Ghost& g, bool ready) {
    if (!g.valid) return;
    SDL_Renderer* r = app.renderer;
    const bool textbook = app.m.aim == hoh::Aim::Prograde;
    const gd::Color tone = textbook ? gd::Color{110, 240, 220} : gd::Color{255, 160, 90};
    drawCurve(r, g.curve, gd::Color{tone.r, tone.g, tone.b, 175}, true);
    if (!g.apsis || !g.targetThen) return;

    const SDL_FPoint a = toScreen(*g.apsis);
    const SDL_FPoint t = toScreen(*g.targetThen);
    const double gap = hoh::length(*g.apsis - *g.targetThen);
    const gd::Color gapColor = gap < kMissTol ? gd::Color{110, 255, 225, 255} : gd::Color{255, 170, 90, 255};
    line(r, a, t, gd::Color{gapColor.r, gapColor.g, gapColor.b, 150});
    // The end of the ellipse: a small teal diamond. The target by then: a hollow orange ring.
    const float s = 5.0F;
    gd::setColor(r, tone);
    const SDL_FPoint pts[5] = {{a.x, a.y - s}, {a.x + s, a.y}, {a.x, a.y + s}, {a.x - s, a.y}, {a.x, a.y - s}};
    SDL_RenderLines(r, pts, 5);
    gd::ring(r, t.x, t.y, 9.0F, gd::Color{kTargetColor.r, kTargetColor.g, kTargetColor.b, 230}, 28);
    gd::ring(r, t.x, t.y, 10.0F, gd::Color{kTargetColor.r, kTargetColor.g, kTargetColor.b, 110}, 28);
    if (ready) gd::ring(r, t.x, t.y, 14.0F, gd::Color{110, 255, 225, 230}, 28);

    char text[48];
    std::snprintf(text, sizeof text, "gap %.0f px", gap);
    const float w = app.hud.width(text);
    pill(app, std::clamp((a.x + t.x) / 2.0F - w / 2.0F, kStageX + 8.0F, kStageX + kStageW - w - 8.0F),
         std::clamp((a.y + t.y) / 2.0F - 9.0F, kStageY + 6.0F, kStageY + kStageH - 28.0F), text, gapColor);
}

/// The phase angle: a fan from the ship to the target, and a teal tick where the target would have to be to burn now.
void drawPhase(App& app) {
    const hoh::Mission& m = app.m;
    if (m.aim != hoh::Aim::Prograde || m.burns != 0 || m.crashed) return;
    SDL_Renderer* r = app.renderer;
    const double shipAngle = std::atan2(m.ship.pos.y, m.ship.pos.x);
    const double phase = hoh::phaseNow(m);
    const double needed = hoh::hohmannPhase(kMu, m.sc.r1, m.sc.r2);
    constexpr double kFanR = 46.0;

    arcPoints(app.arc, kFanR, shipAngle, shipAngle + phase, 28);
    app.rim.clear();
    for (const hoh::Vec2& p : app.arc) app.rim.push_back(toScreen(p));
    fillFan(r, {kCx, kCy}, app.rim, gd::Color{255, 255, 255, 24});
    line(r, {kCx, kCy}, app.rim.front(), gd::Color{255, 255, 255, 60});
    line(r, {kCx, kCy}, app.rim.back(), gd::Color{255, 255, 255, 60});

    arcPoints(app.arc, kFanR + 6.0, shipAngle, shipAngle + needed, 28);
    drawCurve(r, app.arc, gd::Color{110, 240, 220, 210}, true);
    const SDL_FPoint t0 = toScreen({(kFanR - 2.0) * std::cos(shipAngle + needed), (kFanR - 2.0) * std::sin(shipAngle + needed)});
    const SDL_FPoint t1 = toScreen({(kFanR + 14.0) * std::cos(shipAngle + needed), (kFanR + 14.0) * std::sin(shipAngle + needed)});
    line(r, t0, t1, gd::Color{110, 240, 220, 255}, 2.0F);

    if (std::fabs(phase) > 0.35) {
        char text[16];
        std::snprintf(text, sizeof text, "%.0f°", std::fabs(phase) * 180.0 / hoh::kPi);
        const SDL_FPoint c = toScreen({(kFanR + 22.0) * std::cos(shipAngle + phase / 2.0), (kFanR + 22.0) * std::sin(shipAngle + phase / 2.0)});
        const float w = app.hud.width(text);
        app.hud.text(r, c.x - w / 2.0F, c.y - 11.0F, text, gd::Color{255, 255, 255, 190});
    }
}

/// The thrust the next burn would give: an arrow from the ship along the direction of travel or toward the target.
void drawThrustArrow(App& app, bool ready) {
    const hoh::Mission& m = app.m;
    if (m.burns >= 2 || m.crashed) return;
    // The textbook second burn happens at the far end, so the arrow only shows up when the ship is there.
    if (m.aim == hoh::Aim::Prograde && m.burns == 1 && !ready) return;
    SDL_Renderer* r = app.renderer;
    const hoh::Vec2 dir = m.aim == hoh::Aim::Prograde ? hoh::unit(m.ship.vel) : hoh::unit(hoh::targetState(m).pos - m.ship.pos);
    const SDL_FPoint a = toScreen(m.ship.pos);
    const SDL_FPoint d = unitScreen(dir);
    const float len = 20.0F + static_cast<float>(std::fabs(hoh::plannedDv(m))) * 1.6F;
    const SDL_FPoint b{a.x + d.x * len, a.y + d.y * len};
    const gd::Color c = m.aim == hoh::Aim::Prograde ? gd::Color{110, 240, 220, 230} : gd::Color{255, 170, 100, 230};
    if (m.aim == hoh::Aim::AtTarget) {
        // The line of sight to the target, where the thrust points.
        const SDL_FPoint t = toScreen(hoh::targetState(m).pos);
        line(r, a, t, gd::Color{255, 170, 100, 80});
    }
    line(r, a, b, c, 2.0F);
    line(r, b, {b.x - 9.0F * d.x + 4.5F * d.y, b.y - 9.0F * d.y - 4.5F * d.x}, c, 2.0F);
    line(r, b, {b.x - 9.0F * d.x - 4.5F * d.y, b.y - 9.0F * d.y + 4.5F * d.x}, c, 2.0F);
}

void drawCrash(SDL_Renderer* r, const App& app) {
    if (!app.m.crashed) return;
    const SDL_FPoint p = toScreen(app.m.ship.pos);
    const double age = std::min(app.clock - app.crashClock, 1.2);
    const float k = static_cast<float>(age / 1.2);
    gd::ring(r, p.x, p.y, 6.0F + 34.0F * k, gd::Color{255, 190, 90, static_cast<Uint8>(230.0F * (1.0F - k))}, 36);
    gd::disc(r, p.x, p.y, 12.0F * (1.0F - 0.5F * k), gd::Color{255, 240, 200, 220}, gd::Color{255, 90, 20, 0}, 24);
    line(r, {p.x - 6.0F, p.y - 6.0F}, {p.x + 6.0F, p.y + 6.0F}, gd::Color{255, 120, 40, 255}, 2.0F);
    line(r, {p.x - 6.0F, p.y + 6.0F}, {p.x + 6.0F, p.y - 6.0F}, gd::Color{255, 120, 40, 255}, 2.0F);
}

void panel(SDL_Renderer* r, float x, float y, float w, float h) {
    gd::fillRect(r, x, y, w, h, gd::Color{6, 10, 24, 205});
    gd::strokeRect(r, x, y, w, h, gd::Color{255, 255, 255, 46});
}

struct Verdict {
    const char* text;
    gd::Color color;
};

Verdict verdictFor(const App& app, bool ready) {
    const hoh::Mission& m = app.m;
    if (m.crashed) return {"LOST IN THE PLANET", gd::Color{255, 120, 60}};
    if (m.burns >= 2) {
        return hoh::rendezvous(m, kMissTol) ? Verdict{"RENDEZVOUS", gd::Color{110, 255, 225}} : Verdict{"MISSED THE TARGET", gd::Color{255, 150, 80}};
    }
    if (m.aim == hoh::Aim::AtTarget) {
        return m.burns == 0 ? Verdict{"POINT AND BURN", gd::Color{255, 170, 100}} : Verdict{"OFF COURSE", gd::Color{255, 150, 80}};
    }
    if (m.burns == 0) return ready ? Verdict{"WINDOW OPEN, BURN", gd::Color{110, 255, 225}} : Verdict{"WAIT FOR THE WINDOW", kLight};
    return ready ? Verdict{"END OF ELLIPSE, BURN", gd::Color{110, 255, 225}} : Verdict{"COASTING", kLight};
}

void drawPanels(App& app, const Ghost& g, bool ready) {
    SDL_Renderer* r = app.renderer;
    const hoh::Mission& m = app.m;
    const double dv1 = hoh::hohmannDv1(kMu, m.sc.r1, m.sc.r2);
    const double dv2 = hoh::hohmannDv2(kMu, m.sc.r1, m.sc.r2);
    char text[96];

    // Plan: what the textbook transfer costs, fixed for the chosen target orbit.
    float y = 54.0F;
    panel(r, kPanelX, y, kPanelW, 84.0F);
    app.hud.text(r, kPanelX + 12.0F, y + 7.0F, "TRANSFER PLAN", kLight, gd::Hud::Style::Bold);
    std::snprintf(text, sizeof text, "burn 1 %+.1f   burn 2 %+.1f", dv1, dv2);
    app.hud.text(r, kPanelX + 12.0F, y + 32.0F, text, kLight);
    std::snprintf(text, sizeof text, "total %.1f px/s  flight %.1f s", hoh::hohmannTotalDv(kMu, m.sc.r1, m.sc.r2), hoh::hohmannTime(kMu, m.sc.r1, m.sc.r2));
    app.hud.text(r, kPanelX + 12.0F, y + 54.0F, text, kDim);

    // Phase angle (textbook aim) or the aim itself (naive aim).
    y += 92.0F;
    panel(r, kPanelX, y, kPanelW, 118.0F);
    if (m.aim == hoh::Aim::Prograde) {
        const double needed = hoh::hohmannPhase(kMu, m.sc.r1, m.sc.r2);
        app.hud.text(r, kPanelX + 12.0F, y + 7.0F, "PHASE ANGLE", kLight, gd::Hud::Style::Bold);
        std::snprintf(text, sizeof text, "target ahead  %4.0f°", std::fmod(std::round(hoh::wrap2Pi(hoh::phaseNow(m)) * 180.0 / hoh::kPi), 360.0));
        app.hud.text(r, kPanelX + 12.0F, y + 32.0F, text, kLight);
        std::snprintf(text, sizeof text, "needed        %4.0f°", std::round(hoh::wrap2Pi(needed) * 180.0 / hoh::kPi));
        app.hud.text(r, kPanelX + 12.0F, y + 54.0F, text, gd::Color{110, 240, 220});
        if (m.burns == 0 && !m.crashed) {
            const double wait = hoh::timeToWindow(hoh::phaseNow(m), needed, hoh::meanMotion(kMu, m.sc.r1), hoh::meanMotion(kMu, m.sc.r2));
            if (ready) {
                std::snprintf(text, sizeof text, "window       open now");
            } else if (std::isfinite(wait)) {
                std::snprintf(text, sizeof text, "window in    %5.1f s", wait);
            } else {
                std::snprintf(text, sizeof text, "window in         -");
            }
        } else if (g.valid && g.apsis) {
            std::snprintf(text, sizeof text, "far end in   %5.1f s", g.seconds);
        } else {
            std::snprintf(text, sizeof text, "burns done");
        }
        app.hud.text(r, kPanelX + 12.0F, y + 76.0F, text, kDim);
        // Gauge: the phase error from -90 to +90 degrees; the teal block in the middle is the window.
        if (m.burns == 0) {
            const float gx = kPanelX + 14.0F;
            const float gw = kPanelW - 28.0F;
            const float gy = y + 104.0F;
            gd::fillRect(r, gx, gy, gw, 6.0F, gd::Color{255, 255, 255, 28});
            const float half = static_cast<float>(windowTol(m) / (hoh::kPi / 2.0)) * gw / 2.0F;
            gd::fillRect(r, gx + gw / 2.0F - half, gy - 2.0F, 2.0F * half, 10.0F, gd::Color{45, 212, 191, static_cast<Uint8>(ready ? 255 : 120)});
            const double err = std::clamp(hoh::windowError(m), -hoh::kPi / 2.0, hoh::kPi / 2.0);
            const float mx = gx + gw / 2.0F + static_cast<float>(err / (hoh::kPi / 2.0)) * gw / 2.0F;
            gd::fillRect(r, mx - 2.0F, gy - 4.0F, 4.0F, 14.0F, gd::kWhite);
        }
    } else {
        app.hud.text(r, kPanelX + 12.0F, y + 7.0F, "AIM AT TARGET", kLight, gd::Hud::Style::Bold);
        const hoh::Vec2 toTarget = hoh::unit(hoh::targetState(m).pos - m.ship.pos);
        const hoh::Vec2 prograde = hoh::unit(m.ship.vel);
        const double off = std::atan2(hoh::cross(prograde, toTarget), hoh::dot(prograde, toTarget)) * 180.0 / hoh::kPi;
        std::snprintf(text, sizeof text, "off prograde %+4.0f°", off);
        app.hud.text(r, kPanelX + 12.0F, y + 32.0F, text, kLight);
        if (g.valid && g.bound) {
            std::snprintf(text, sizeof text, "if burned %3.0f-%3.0f px", g.rPeri, g.rApo);
        } else if (g.valid) {
            std::snprintf(text, sizeof text, "if burned: escapes");
        } else {
            std::snprintf(text, sizeof text, "burns done");
        }
        app.hud.text(r, kPanelX + 12.0F, y + 54.0F, text, gd::Color{255, 170, 100});
        std::snprintf(text, sizeof text, "target orbit  %3.0f px", m.sc.r2);
        app.hud.text(r, kPanelX + 12.0F, y + 76.0F, text, kDim);
    }

    // Ship: status, orbit, fuel, distance.
    y += 126.0F;
    panel(r, kPanelX, y, kPanelW, 124.0F);
    const Verdict v = verdictFor(app, ready);
    app.hud.text(r, kPanelX + 12.0F, y + 7.0F, v.text, v.color, gd::Hud::Style::Bold);
    const auto c = hoh::conicFromState(m.ship, kMu);
    if (m.crashed) {
        std::snprintf(text, sizeof text, "orbit  -");
    } else if (c && c->bound) {
        std::snprintf(text, sizeof text, "orbit %3.0f-%3.0f px  e %.2f", c->rPeri, c->rApo, c->e);
    } else {
        std::snprintf(text, sizeof text, "orbit  open, escapes");
    }
    app.hud.text(r, kPanelX + 12.0F, y + 34.0F, text, kLight);
    std::snprintf(text, sizeof text, "dv spent %.1f of %.1f", m.dvSpent, hoh::hohmannTotalDv(kMu, m.sc.r1, m.sc.r2));
    app.hud.text(r, kPanelX + 12.0F, y + 56.0F, text, kDim);
    if (m.burns >= 2) {
        std::snprintf(text, sizeof text, "miss at burn 2 %.0f px", m.missAtBurn2);
        app.hud.text(r, kPanelX + 12.0F, y + 78.0F, text, hoh::rendezvous(m, kMissTol) ? gd::Color{110, 255, 225} : gd::Color{255, 150, 80});
    } else {
        std::snprintf(text, sizeof text, "target %3.0f px away", hoh::length(m.ship.pos - hoh::targetState(m).pos));
        app.hud.text(r, kPanelX + 12.0F, y + 78.0F, text, kDim);
    }
    std::snprintf(text, sizeof text, "burns %d of 2   t %.1f s", m.burns, m.time);
    app.hud.text(r, kPanelX + 12.0F, y + 100.0F, text, kDim);
}

void drawChips(App& app, bool ready) {
    SDL_Renderer* r = app.renderer;
    for (const Label& l : app.labels) app.hud.text(r, l.x, l.y, l.text, gd::kMuted);
    const bool burnsLeft = app.m.burns < 2 && !app.m.crashed;
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const bool dim = c.action == Action::Burn && !burnsLeft;
        gd::Color fill = on ? gd::kTeal : gd::kWhite;
        if (c.action == Action::Burn && ready) fill = gd::Color{20, 184, 166};
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, dim ? gd::kMuted : gd::kInk);
        const gd::Color textColor = dim ? gd::kMuted : ((on || (c.action == Action::Burn && ready)) ? gd::kWhite : gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, c.label, textColor, gd::Hud::Style::Bold);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const hoh::Mission& m = app.m;
    const bool ready = hoh::burnReady(m, windowTol(m), kApsisTol);
    makeGhost(m, app.ghost);
    const Ghost& ghost = app.ghost;
    const hoh::State target = hoh::targetState(m);
    const bool docked = m.burns >= 2 && hoh::rendezvous(m, kMissTol) && hoh::length(m.ship.pos - target.pos) < kMissTol;

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ HOHMANN TRANSFER ]", "two burns, one meeting");
    drawSky(app);

    const SDL_Rect clip{static_cast<int>(kStageX) + 1, static_cast<int>(kStageY) + 1, static_cast<int>(kStageW) - 2, static_cast<int>(kStageH) - 2};
    SDL_SetRenderClipRect(r, &clip);
    drawCurve(r, app.ringStart, gd::Color{kShipColor.r, kShipColor.g, kShipColor.b, 95}, true);
    drawCurve(r, app.ringTarget, gd::Color{kTargetColor.r, kTargetColor.g, kTargetColor.b, 95}, true);
    drawPlanet(r, app.clock);
    drawPhase(app);
    drawGhost(app, ghost, ready);
    drawTrail(r, app.rim, app.targetTrail, kTargetColor, 0.55F);
    drawTrail(r, app.rim, app.shipTrail, kShipColor, 1.0F);
    drawThrustArrow(app, ready);
    drawTarget(r, target.pos, app.clock, docked);
    if (m.crashed) {
        drawCrash(r, app);
    } else {
        drawShip(r, m, app.clock, app.fxStart, ready);
        drawMarkerLabel(app, m.ship.pos, "SHIP", kShipColor, -30.0F);
    }
    drawMarkerLabel(app, target.pos, "TARGET", kTargetColor, 30.0F);
    SDL_SetRenderClipRect(r, nullptr);

    drawPanels(app, ghost, ready);

    if (SDL_GetTicks() < app.flashUntil) {
        const float w = app.hud.width(app.flash, gd::Hud::Style::Bold) + 28.0F;
        const float x = kStageX + 330.0F - w / 2.0F;
        gd::card(r, x, kStageY + kStageH - 46.0F, w, 32.0F, gd::kWhite, gd::kTeal);
        app.hud.text(r, x + 14.0F, kStageY + kStageH - 41.0F, app.flash, gd::kTealDark, gd::Hud::Style::Bold);
    }

    app.hud.text(r, 40.0F, 421.0F, "click in space or B: burn   P / T: aim   A: autopilot   1 2 3: target orbit   - / +: speed", gd::kMuted);
    drawChips(app, ready);
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
    if (!SDL_CreateWindowAndRenderer("hohmann-transfer", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->shipTrail.cap = kShipTrailCap;
    app->targetTrail.cap = kTargetTrailCap;
    app->stars = makeStars();
    layoutChips(*app);
    restart(*app);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action, c.value);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inStage(p)) apply(app, Action::Burn, 0);
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            if (event->key.repeat) break;  // every key here is a toggle or a one-shot
            switch (event->key.key) {
                case SDLK_B:
                case SDLK_RETURN: apply(app, Action::Burn, 0); break;
                case SDLK_P: apply(app, Action::SetAim, 0); break;
                case SDLK_T: apply(app, Action::SetAim, 1); break;
                case SDLK_A: apply(app, Action::Autopilot, 0); break;
                case SDLK_R: apply(app, Action::Restart, 0); break;
                case SDLK_SPACE: apply(app, Action::Pause, 0); break;
                case SDLK_1: apply(app, Action::TargetOrbit, 0); break;
                case SDLK_2: apply(app, Action::TargetOrbit, 1); break;
                case SDLK_3: apply(app, Action::TargetOrbit, 2); break;
                case SDLK_MINUS: apply(app, Action::Speed, static_cast<int>(app.speedIndex) - 1); break;
                case SDLK_EQUALS:
                case SDLK_PLUS: apply(app, Action::Speed, static_cast<int>(app.speedIndex) + 1); break;
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
    const Uint64 now = SDL_GetPerformanceCounter();
    const double dt = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    update(app, std::min(dt, kMaxFrameTime));
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    if (app.maxFrames >= 0 && ++app.frames >= app.maxFrames) return SDL_APP_SUCCESS;
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app == nullptr) return;
    app->hud.destroy();
    if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
    if (app->window != nullptr) SDL_DestroyWindow(app->window);
    delete app;
}
