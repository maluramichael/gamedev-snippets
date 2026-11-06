// Gravity assist with SDL3 main callbacks.
//
// A probe flies past a planet that moves at a constant speed. Seen from the planet, the probe comes in at some speed,
// swings around and leaves with exactly the same speed, only the direction has changed. Seen from the sun (where the
// planet moves) the same flight ends faster or slower, because the planet's velocity is added to both the incoming and
// the outgoing velocity: v = V + u. Passing behind the planet gains speed, passing in front loses it.
//
// Click the frame chips (or press F) to switch the point of view; the stage slides smoothly between the two. The
// trail is coloured by the probe's speed in the frame you are looking from, so the gain shows up as a colour change in
// the sun's frame and as a constant colour in the planet's. The flight is computed in one go (logic.hpp) and then only
// played back, so the dashed prediction while you drag and the animation afterwards are the same data. Drag in space
// to set the start and the velocity, click the time bar to scrub.
// All the maths is in logic.hpp (no SDL, covered by test.cpp), this file owns input, the playback and the picture.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;

// The stage is a dark window onto space on the left, the readouts sit in a column on the right.
constexpr float kStageX = 24.0F;
constexpr float kStageY = 44.0F;
constexpr float kStageW = 600.0F;
constexpr float kStageH = 350.0F;
constexpr float kCx = kStageX + kStageW / 2.0F;  // world (0, 0) is the middle of the stage in the sun's frame
constexpr float kCy = kStageY + kStageH / 2.0F;
constexpr ga::Vec2 kAnchor{-60.0, 0.0};  // where the planet sits on the stage in its own frame

constexpr float kBarY = 402.0F;
constexpr float kBarH = 8.0F;
constexpr float kSideX = 640.0F;
constexpr float kSideW = 296.0F;
constexpr float kTableY = 44.0F;
constexpr float kTableH = 170.0F;
constexpr float kDiagY = 222.0F;
constexpr float kDiagH = 172.0F;
constexpr float kChipH = 32.0F;
constexpr float kRowOneY = 446.0F;
constexpr float kRowTwoY = 488.0F;

constexpr double kMaxFrameTime = 0.25;  // clamp: no jump after a stalled tab
constexpr double kHold = 2.2;           // the finished flight stays on screen this long, then it replays
constexpr double kFrameFade = 0.55;     // seconds for the slide between the two points of view
constexpr double kSpeedPerPx = 0.5;     // launch speed per pixel of mouse drag
constexpr double kClickDrag = 6.0;      // a drag shorter than this is a click (probe at rest in the shown frame)
constexpr double kSpeedMax = 160.0;     // speed that gets the hottest trail colour
constexpr float kArrowPerSpeed = 0.55F; // pixels of arrow per px/s on the stage
constexpr double kPotentialStep = 900.0;  // the rings around the planet are equal steps of potential

constexpr gd::Color kLight{232, 238, 248};
constexpr gd::Color kDim{148, 163, 184};
constexpr gd::Color kSky0{9, 13, 31};
constexpr gd::Color kSky1{22, 30, 62};
constexpr gd::Color kIn{45, 212, 191};
constexpr gd::Color kOut{255, 122, 40};

enum class Action { Preset, Restart, Pause, Frame, Speed, Mass };

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

struct Dust {
    float x, y, size, alpha;
};

constexpr float kDustW = 640.0F;
constexpr float kDustH = 420.0F;

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<Label> labels;
    std::vector<Star> stars;
    std::vector<Dust> dust;

    ga::World world;
    std::size_t speedIndex = 2;
    std::size_t massIndex = 1;
    int presetIndex = 0;  // -1 = set by hand
    ga::Launch launch = ga::preset(0);
    ga::Flight flight;
    ga::Encounter enc;

    bool aiming = false;
    bool aimOk = true;
    SDL_FPoint aimStart{0.0F, 0.0F};
    SDL_FPoint mouse{0.0F, 0.0F};
    ga::Launch aimLaunch;
    ga::Flight aimFlight;
    ga::Encounter aimEnc;

    double time = 0.0;
    double hold = 0.0;
    bool paused = false;
    bool seeking = false;

    bool planetFrame = false;
    double blendRaw = 0.0;
    double blend = 0.0;  // eased, 0 = the sun's frame, 1 = the planet's

    std::string flash;
    Uint64 flashUntil = 0;

    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

double smooth(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

/// How far the view is shifted at time t: nothing in the sun's frame, the planet's travel in its own.
ga::Vec2 viewOrigin(const App& app, double t) {
    const ga::Vec2 p = ga::planetPosition(app.world, t);
    return {(p.x - kAnchor.x) * app.blend, (p.y - kAnchor.y) * app.blend};
}

SDL_FPoint toScreen(const App& app, ga::Vec2 p, double t) {
    const ga::Vec2 o = viewOrigin(app, t);
    return {kCx + static_cast<float>(p.x - o.x), kCy + static_cast<float>(p.y - o.y)};
}

/// A velocity as the shown frame sees it.
ga::Vec2 viewVelocity(const App& app, ga::Vec2 v) { return v - ga::planetVelocity(app.world) * app.blend; }

bool inStage(SDL_FPoint p) { return p.x >= kStageX && p.x <= kStageX + kStageW && p.y >= kStageY && p.y <= kStageY + kStageH; }

bool onBar(SDL_FPoint p) { return p.x >= kStageX && p.x <= kStageX + kStageW && p.y >= kBarY - 6.0F && p.y <= kBarY + kBarH + 6.0F; }

void flashMessage(App& app, const char* text) {
    app.flash = text;
    app.flashUntil = SDL_GetTicks() + 2400;
}

/// Recomputes the flight after anything that changes it, and starts the playback over.
void refresh(App& app) {
    app.world.mu = ga::kMasses[app.massIndex];
    app.world.planetSpeed = ga::kSpeeds[app.speedIndex];
    ga::simulate(app.world, ga::worldState(app.world, app.launch), app.flight);
    app.enc = ga::analyze(app.world, app.flight);
    app.time = 0.0;
    app.hold = 0.0;
}

void setPreset(App& app, int index) {
    app.presetIndex = index;
    app.launch = ga::preset(index);
    refresh(app);
}

void apply(App& app, Action action, int value) {
    switch (action) {
        case Action::Preset: setPreset(app, value); break;
        case Action::Restart:
            app.time = 0.0;
            app.hold = 0.0;
            break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Frame: app.planetFrame = value == 1; break;
        case Action::Speed:
            app.speedIndex = static_cast<std::size_t>(value);
            refresh(app);
            break;
        case Action::Mass:
            app.massIndex = static_cast<std::size_t>(value);
            refresh(app);
            break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Preset: return app.presetIndex == c.value;
        case Action::Restart: return false;
        case Action::Pause: return app.paused;
        case Action::Frame: return app.planetFrame == (c.value == 1);
        case Action::Speed: return app.speedIndex == static_cast<std::size_t>(c.value);
        case Action::Mass: return app.massIndex == static_cast<std::size_t>(c.value);
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
    label("Launch");
    chip(Action::Preset, 0, "1 Behind");
    chip(Action::Preset, 1, "2 In front");
    chip(Action::Preset, 2, "3 From rest");
    x += 16.0F;
    chip(Action::Restart, 0, "R Replay");
    chip(Action::Pause, 0, "Space Pause");

    x = 40.0F;
    y = kRowTwoY;
    label("View");
    chip(Action::Frame, 0, "Sun");
    chip(Action::Frame, 1, "Planet");
    x += 16.0F;
    label("Planet speed");
    for (std::size_t i = 0; i < ga::kSpeeds.size(); ++i) chip(Action::Speed, static_cast<int>(i), std::to_string(static_cast<int>(ga::kSpeeds[i])));
    x += 16.0F;
    label("Mass");
    chip(Action::Mass, 0, "Low");
    chip(Action::Mass, 1, "Mid");
    chip(Action::Mass, 2, "High");
}

/// The launch a drag from aimStart to the mouse would give, in whatever frame is on screen right now.
void updateAim(App& app) {
    const ga::Vec2 o0 = viewOrigin(app, 0.0);
    const ga::Vec2 pos{static_cast<double>(app.aimStart.x - kCx) + o0.x, static_cast<double>(app.aimStart.y - kCy) + o0.y};
    ga::Vec2 drag{static_cast<double>(app.mouse.x - app.aimStart.x), static_cast<double>(app.mouse.y - app.aimStart.y)};
    if (ga::length(drag) < kClickDrag) drag = {};
    // A velocity dragged in the planet's frame is relative to the planet, in the sun's frame it is absolute.
    const ga::Vec2 vel = drag * kSpeedPerPx + ga::planetVelocity(app.world) * app.blend;
    app.aimLaunch = ga::relativeLaunch(app.world, pos, vel);
    app.aimOk = ga::startAllowed(app.world, app.aimLaunch.rel);
    if (app.aimOk) {
        ga::simulate(app.world, ga::worldState(app.world, app.aimLaunch), app.aimFlight);
        app.aimEnc = ga::analyze(app.world, app.aimFlight);
    }
}

void finishAim(App& app) {
    app.aiming = false;
    if (!app.aimOk) {
        flashMessage(app, "Start outside the dashed circle");
        return;
    }
    app.launch = app.aimLaunch;
    app.presetIndex = -1;
    refresh(app);
}

void seekTo(App& app, float mouseX) {
    const double dur = ga::duration(app.flight);
    app.time = std::clamp(static_cast<double>(mouseX - kStageX) / static_cast<double>(kStageW), 0.0, 1.0) * dur;
    app.hold = 0.0;
}

void advance(App& app, double dt) {
    dt = std::min(dt, kMaxFrameTime);
    const double target = app.planetFrame ? 1.0 : 0.0;
    const double step = dt / kFrameFade;
    app.blendRaw = target > app.blendRaw ? std::min(target, app.blendRaw + step) : std::max(target, app.blendRaw - step);
    app.blend = smooth(app.blendRaw);
    if (app.aiming) {
        updateAim(app);
        return;
    }
    if (app.paused || app.seeking) return;
    const double dur = ga::duration(app.flight);
    if (app.time < dur) {
        app.time = std::min(dur, app.time + dt);
        app.hold = 0.0;
    } else {
        app.hold += dt;
        if (app.hold >= kHold) {
            app.time = 0.0;
            app.hold = 0.0;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------- drawing

void line(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, float width = 1.0F) {
    gd::setColor(r, c);
    SDL_RenderLine(r, a.x, a.y, b.x, b.y);
    if (width > 1.0F) SDL_RenderLine(r, a.x + 0.8F, a.y + 0.8F, b.x + 0.8F, b.y + 0.8F);
}

void drawArrow(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, float head = 9.0F) {
    line(r, a, b, c, 2.0F);
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 3.0F) return;
    const float ux = dx / len;
    const float uy = dy / len;
    line(r, b, {b.x - head * ux + head * 0.5F * uy, b.y - head * uy - head * 0.5F * ux}, c, 2.0F);
    line(r, b, {b.x - head * ux - head * 0.5F * uy, b.y - head * uy + head * 0.5F * ux}, c, 2.0F);
}

void fillTriangle(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, SDL_FPoint c, gd::Color col) {
    const SDL_FColor f = gd::fcolor(col);
    const SDL_Vertex v[3] = {{a, f, {0, 0}}, {b, f, {0, 0}}, {c, f, {0, 0}}};
    SDL_RenderGeometry(r, nullptr, v, 3, nullptr, 0);
}

void dashedCircle(SDL_Renderer* r, SDL_FPoint c, float radius, gd::Color col, int segments) {
    gd::setColor(r, col);
    for (int i = 0; i < segments; i += 2) {
        const float a = 6.2831853F * static_cast<float>(i) / static_cast<float>(segments);
        const float b = 6.2831853F * static_cast<float>(i + 1) / static_cast<float>(segments);
        SDL_RenderLine(r, c.x + std::cos(a) * radius, c.y + std::sin(a) * radius, c.x + std::cos(b) * radius, c.y + std::sin(b) * radius);
    }
}

gd::Color lerpColor(gd::Color a, gd::Color b, double t) {
    t = std::clamp(t, 0.0, 1.0);
    const auto mix = [t](Uint8 x, Uint8 y) { return static_cast<Uint8>(static_cast<double>(x) + (static_cast<double>(y) - static_cast<double>(x)) * t); };
    return gd::Color{mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b), 255};
}

/// Cold blue for slow, teal, yellow, hot orange for fast. s is the speed over kSpeedMax.
gd::Color heat(double s) {
    constexpr gd::Color c0{80, 120, 255};
    constexpr gd::Color c1{45, 212, 191};
    constexpr gd::Color c2{255, 214, 80};
    constexpr gd::Color c3{255, 96, 40};
    s = std::clamp(s, 0.0, 1.0);
    if (s < 0.33) return lerpColor(c0, c1, s / 0.33);
    if (s < 0.66) return lerpColor(c1, c2, (s - 0.33) / 0.33);
    return lerpColor(c2, c3, (s - 0.66) / 0.34);
}

std::vector<Star> makeStars() {
    std::vector<Star> out;
    unsigned seed = 20271029U;
    auto next = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return static_cast<float>((seed >> 8) & 0xFFFFU) / 65535.0F;
    };
    for (int i = 0; i < 150; ++i) {
        const float x = kStageX + 4.0F + next() * (kStageW - 8.0F);
        const float y = kStageY + 4.0F + next() * (kStageH - 8.0F);
        const float size = next() > 0.93F ? 2.0F : 1.0F;
        out.push_back({x, y, size, 0.25F + next() * 0.6F, next() * 6.2831853F});
    }
    return out;
}

std::vector<Dust> makeDust() {
    std::vector<Dust> out;
    unsigned seed = 77123U;
    auto next = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return static_cast<float>((seed >> 8) & 0xFFFFU) / 65535.0F;
    };
    for (int i = 0; i < 90; ++i) out.push_back({next() * kDustW, next() * kDustH, next() > 0.8F ? 2.0F : 1.0F, 0.28F + next() * 0.4F});
    return out;
}

void drawSky(App& app, double clock) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kStageX, kStageY, kStageW, kStageH, kSky0, gd::kInk);
    gd::gradient(r, kStageX + 1.0F, kStageY + 1.0F, kStageW - 2.0F, kStageH - 2.0F, kSky0, kSky1);
    gd::disc(r, kStageX + 470.0F, kStageY + 70.0F, 200.0F, gd::Color{70, 60, 160, 24}, gd::Color{70, 60, 160, 0}, 56);
    gd::disc(r, kStageX + 90.0F, kStageY + 160.0F, 170.0F, gd::Color{13, 148, 136, 20}, gd::Color{13, 148, 136, 0}, 56);
    // Far stars do not move when the point of view moves, that is what far means.
    for (const Star& s : app.stars) {
        const double twinkle = 0.65 + 0.35 * std::sin(clock * 1.6 + static_cast<double>(s.phase));
        const auto a = static_cast<Uint8>(255.0 * static_cast<double>(s.base) * twinkle);
        gd::fillRect(r, s.x, s.y, s.size, s.size, gd::Color{235, 240, 255, a});
    }
}

/// The sun is far below the stage. It, and the dust that stands still in its frame, slide past when the planet's frame is shown.
void drawSunAndDust(App& app, double t) {
    SDL_Renderer* r = app.renderer;
    const SDL_FPoint sun = toScreen(app, {0.0, 560.0}, t);
    gd::disc(r, sun.x, sun.y, 560.0F, gd::Color{255, 170, 60, 120}, gd::Color{255, 130, 30, 0}, 72);
    const SDL_FPoint tag = toScreen(app, {-14.0, 150.0}, t);
    app.hud.text(r, tag.x, tag.y, "sun", gd::Color{255, 205, 140, 170});

    const ga::Vec2 o = viewOrigin(app, t);
    for (const Dust& d : app.dust) {
        float sx = std::fmod(d.x - static_cast<float>(o.x), kDustW);
        float sy = std::fmod(d.y - static_cast<float>(o.y), kDustH);
        if (sx < 0.0F) sx += kDustW;
        if (sy < 0.0F) sy += kDustH;
        gd::fillRect(r, kStageX - 20.0F + sx, kStageY - 35.0F + sy, d.size, d.size, gd::Color{210, 220, 240, static_cast<Uint8>(255.0F * d.alpha)});
    }
}

void drawPlanetRing(SDL_Renderer* r, SDL_FPoint c, bool front) {
    constexpr int kSeg = 40;
    constexpr float kA = 17.0F;
    constexpr float kB = 5.0F;
    const float tilt = -0.35F;
    const float ct = std::cos(tilt);
    const float st = std::sin(tilt);
    gd::setColor(r, gd::Color{190, 215, 225, front ? Uint8{230} : Uint8{120}});
    for (int i = 0; i < kSeg; ++i) {
        const float a0 = 6.2831853F * static_cast<float>(i) / static_cast<float>(kSeg);
        const float a1 = 6.2831853F * static_cast<float>(i + 1) / static_cast<float>(kSeg);
        if ((std::sin(a0) > 0.0F) != front) continue;
        const float x0 = std::cos(a0) * kA;
        const float y0 = std::sin(a0) * kB;
        const float x1 = std::cos(a1) * kA;
        const float y1 = std::sin(a1) * kB;
        SDL_RenderLine(r, c.x + x0 * ct - y0 * st, c.y + x0 * st + y0 * ct, c.x + x1 * ct - y1 * st, c.y + x1 * st + y1 * ct);
    }
}

void drawPlanet(App& app, double t, bool ghost) {
    SDL_Renderer* r = app.renderer;
    const SDL_FPoint c = toScreen(app, ga::planetPosition(app.world, t), t);
    const float radius = static_cast<float>(app.world.planetRadius);
    const float alpha = ghost ? 0.45F : 1.0F;
    auto a = [alpha](int v) { return static_cast<Uint8>(static_cast<float>(v) * alpha); };

    // Equal steps of potential: the rings crowd together toward the planet, the field gets steep.
    for (int k = 2; k < 40; ++k) {
        const double rr = app.world.mu / (kPotentialStep * static_cast<double>(k));
        if (rr < app.world.planetRadius + 3.0) break;
        if (rr > app.world.zone) continue;
        gd::ring(r, c.x, c.y, static_cast<float>(rr), gd::Color{150, 190, 255, a(30)}, 56);
    }
    dashedCircle(r, c, static_cast<float>(app.world.zone), gd::Color{255, 255, 255, a(120)}, 90);
    app.hud.text(r, c.x - 20.0F, c.y + static_cast<float>(app.world.zone) + 4.0F, "zone", gd::Color{255, 255, 255, a(110)});

    gd::disc(r, c.x, c.y, radius * 3.4F, gd::Color{45, 212, 191, a(70)}, gd::Color{45, 212, 191, 0}, 40);
    drawPlanetRing(r, c, false);
    // Lit from the direction of the sun.
    const SDL_FPoint sun = toScreen(app, {0.0, 560.0}, t);
    float lx = sun.x - c.x;
    float ly = sun.y - c.y;
    const float ll = std::sqrt(lx * lx + ly * ly);
    if (ll > 1e-3F) {
        lx /= ll;
        ly /= ll;
    } else {
        lx = 0.0F;
        ly = 1.0F;
    }
    gd::disc(r, c.x, c.y, radius, gd::Color{16, 58, 82, a(255)}, gd::Color{12, 40, 62, a(255)}, 36);
    gd::disc(r, c.x + lx * radius * 0.35F, c.y + ly * radius * 0.35F, radius * 0.95F, gd::Color{150, 235, 215, a(255)}, gd::Color{40, 140, 140, 0}, 36);
    gd::disc(r, c.x + lx * radius * 0.45F, c.y + ly * radius * 0.45F, radius * 0.3F, gd::Color{255, 255, 255, a(150)}, gd::Color{255, 255, 255, 0}, 16);
    drawPlanetRing(r, c, true);
}

/// The trail of a flight, coloured by the speed in the shown frame. What has been flown is solid, the rest dashed.
void drawPath(App& app, const ga::Flight& f, double tNow, Uint8 futureAlpha) {
    SDL_Renderer* r = app.renderer;
    const auto& s = f.samples;
    if (s.size() < 2) return;
    SDL_FPoint prev = toScreen(app, s[0].pos, s[0].t);
    int dash = 0;
    for (std::size_t i = 1; i < s.size(); ++i) {
        const SDL_FPoint p = toScreen(app, s[i].pos, s[i].t);
        const float dx = p.x - prev.x;
        const float dy = p.y - prev.y;
        if (dx * dx + dy * dy < 16.0F && i + 1 != s.size()) continue;
        const gd::Color c = heat(ga::length(viewVelocity(app, s[i].vel)) / kSpeedMax);
        if (s[i].t <= tNow + 1e-9) {
            gd::setColor(r, gd::Color{c.r, c.g, c.b, 46});
            SDL_RenderLine(r, prev.x, prev.y - 1.0F, p.x, p.y - 1.0F);
            SDL_RenderLine(r, prev.x, prev.y + 1.0F, p.x, p.y + 1.0F);
            gd::setColor(r, c);
            SDL_RenderLine(r, prev.x, prev.y, p.x, p.y);
            SDL_RenderLine(r, prev.x + 0.8F, prev.y, p.x + 0.8F, p.y);
        } else if (dash++ % 2 == 0) {
            gd::setColor(r, gd::Color{c.r, c.g, c.b, futureAlpha});
            SDL_RenderLine(r, prev.x, prev.y, p.x, p.y);
        }
        prev = p;
    }
}

/// Equal-time ticks: a dot every second on the probe's path, joined by a faint line to where the planet was at that moment.
void drawTicks(App& app, const ga::Flight& f, double tNow) {
    SDL_Renderer* r = app.renderer;
    const int count = static_cast<int>(std::floor(ga::duration(f)));
    for (int k = 0; k <= count; ++k) {
        const double t = static_cast<double>(k);
        const SDL_FPoint p = toScreen(app, ga::sampleAt(f, t).pos, t);
        const SDL_FPoint q = toScreen(app, ga::planetPosition(app.world, t), t);
        const bool past = t <= tNow + 1e-9;
        line(r, p, q, gd::Color{255, 255, 255, static_cast<Uint8>(past ? 34 : 18)});
        if (app.blend < 0.9) gd::ring(r, q.x, q.y, 3.0F, gd::Color{255, 255, 255, static_cast<Uint8>(past ? 60 : 34)}, 12);
        gd::fillRect(r, p.x - 1.5F, p.y - 1.5F, 3.0F, 3.0F, gd::Color{255, 255, 255, static_cast<Uint8>(past ? 230 : 110)});
    }
}

void drawEncounterMarkers(App& app, const ga::Flight& f, const ga::Encounter& e) {
    if (!e.valid) return;
    SDL_Renderer* r = app.renderer;
    struct Mark {
        double t;
        ga::Vec2 vel;
        gd::Color color;
        const char* text;
    };
    const Mark marks[2] = {{e.tIn, e.vIn, kIn, "in"}, {e.tOut, e.vOut, kOut, "out"}};
    for (const Mark& m : marks) {
        const SDL_FPoint p = toScreen(app, ga::sampleAt(f, m.t).pos, m.t);
        const ga::Vec2 v = viewVelocity(app, m.vel);
        const float len = static_cast<float>(ga::length(v));
        const ga::Vec2 dir = ga::normalized(v);
        const SDL_FPoint tip{p.x + static_cast<float>(dir.x) * len * kArrowPerSpeed, p.y + static_cast<float>(dir.y) * len * kArrowPerSpeed};
        gd::ring(r, p.x, p.y, 5.0F, gd::Color{m.color.r, m.color.g, m.color.b, 255}, 20);
        if (len > 4.0F) drawArrow(r, p, tip, gd::Color{m.color.r, m.color.g, m.color.b, 235});
        // The word goes to the side of the arrow, away from the path.
        const float nx = -static_cast<float>(dir.y);
        const float ny = static_cast<float>(dir.x);
        app.hud.text(r, p.x + nx * 16.0F - 8.0F, p.y + ny * 16.0F - 11.0F, m.text, m.color);
    }
}

void drawProbe(App& app, SDL_FPoint p, ga::Vec2 viewVel, bool crashed) {
    SDL_Renderer* r = app.renderer;
    if (crashed) {
        gd::disc(r, p.x, p.y, 20.0F, gd::Color{255, 150, 60, 150}, gd::Color{255, 90, 20, 0}, 28);
        line(r, {p.x - 7.0F, p.y - 7.0F}, {p.x + 7.0F, p.y + 7.0F}, kOut, 2.0F);
        line(r, {p.x - 7.0F, p.y + 7.0F}, {p.x + 7.0F, p.y - 7.0F}, kOut, 2.0F);
        return;
    }
    ga::Vec2 dir = ga::normalized(viewVel);
    if (dir.x == 0.0 && dir.y == 0.0) dir = {0.0, -1.0};
    const float dx = static_cast<float>(dir.x);
    const float dy = static_cast<float>(dir.y);
    gd::disc(r, p.x, p.y, 15.0F, gd::Color{255, 255, 255, 80}, gd::Color{255, 255, 255, 0}, 24);
    const SDL_FPoint tip{p.x + dx * 10.0F, p.y + dy * 10.0F};
    const SDL_FPoint left{p.x - dx * 6.0F + dy * 5.5F, p.y - dy * 6.0F - dx * 5.5F};
    const SDL_FPoint right{p.x - dx * 6.0F - dy * 5.5F, p.y - dy * 6.0F + dx * 5.5F};
    fillTriangle(r, tip, left, right, gd::Color{245, 247, 255, 255});
    line(r, tip, left, gd::kTealDark);
    line(r, tip, right, gd::kTealDark);
    line(r, left, right, gd::kTealDark);
}

void drawAimMarker(App& app) {
    SDL_Renderer* r = app.renderer;
    const gd::Color tone = app.aimOk ? gd::Color{255, 255, 255, 255} : kOut;
    const bool dragged = ga::length({static_cast<double>(app.mouse.x - app.aimStart.x), static_cast<double>(app.mouse.y - app.aimStart.y)}) >= kClickDrag;
    if (dragged) drawArrow(r, app.aimStart, app.mouse, gd::Color{255, 255, 255, 230});
    gd::disc(r, app.aimStart.x, app.aimStart.y, 5.0F, tone, 20);
    gd::ring(r, app.aimStart.x, app.aimStart.y, 8.0F, gd::Color{tone.r, tone.g, tone.b, 180}, 24);
    char text[64];
    if (!app.aimOk) {
        std::snprintf(text, sizeof text, "too close to the planet");
    } else if (dragged) {
        const double speed = ga::length({static_cast<double>(app.mouse.x - app.aimStart.x), static_cast<double>(app.mouse.y - app.aimStart.y)}) * kSpeedPerPx;
        std::snprintf(text, sizeof text, "%.0f px/s", speed);
    } else {
        std::snprintf(text, sizeof text, "drag to set the velocity");
    }
    const float w = app.hud.width(text);
    const float tx = std::clamp(app.mouse.x + 14.0F, kStageX + 8.0F, kStageX + kStageW - w - 14.0F);
    const float ty = std::clamp(app.mouse.y + 12.0F, kStageY + 8.0F, kStageY + kStageH - 32.0F);
    gd::fillRect(r, tx - 6.0F, ty - 3.0F, w + 12.0F, 26.0F, gd::Color{6, 10, 24, 215});
    app.hud.text(r, tx, ty, text, tone);
}

void darkCard(SDL_Renderer* r, float x, float y, float w, float h) {
    gd::card(r, x, y, w, h, kSky0, gd::kInk);
    gd::gradient(r, x + 1.0F, y + 1.0F, w - 2.0F, h - 2.0F, kSky0, kSky1);
}

const char* outcomeText(ga::Outcome o) {
    switch (o) {
        case ga::Outcome::Flyby: return "FLYBY";
        case ga::Outcome::Crash: return "CRASH";
        case ga::Outcome::Bound: return "BOUND";
        case ga::Outcome::Miss: return "NO ENCOUNTER";
    }
    return "";
}

void drawStageLabels(App& app, const ga::Flight& f) {
    SDL_Renderer* r = app.renderer;
    const bool planet = app.blend > 0.5;
    char sub[96];
    if (app.world.planetSpeed == 0.0) {
        std::snprintf(sub, sizeof sub, "the planet stands still, both frames agree");
    } else if (planet) {
        std::snprintf(sub, sizeof sub, "sun and dust drift past at %.0f px/s", app.world.planetSpeed);
    } else {
        std::snprintf(sub, sizeof sub, "planet flies at %.0f px/s", app.world.planetSpeed);
    }
    app.hud.text(r, kStageX + 12.0F, kStageY + 8.0F, planet ? "PLANET FRAME" : "SUN FRAME", kLight, gd::Hud::Style::Bold);
    app.hud.text(r, kStageX + 12.0F, kStageY + 30.0F, sub, kDim);
    const char* out = outcomeText(f.outcome);
    const gd::Color tone = f.outcome == ga::Outcome::Flyby ? kIn : kOut;
    app.hud.text(r, kStageX + kStageW - app.hud.width(out, gd::Hud::Style::Bold) - 12.0F, kStageY + 8.0F, out, tone, gd::Hud::Style::Bold);
}

void drawBar(App& app, const ga::Flight& f, const ga::Encounter& e, double tNow) {
    SDL_Renderer* r = app.renderer;
    const double dur = std::max(ga::duration(f), 1e-9);
    gd::fillRect(r, kStageX, kBarY, kStageW, kBarH, gd::Color{206, 203, 200, 255});
    if (e.valid) {
        const float x0 = kStageX + static_cast<float>(e.tIn / dur) * kStageW;
        const float x1 = kStageX + static_cast<float>(e.tOut / dur) * kStageW;
        gd::fillRect(r, x0, kBarY, x1 - x0, kBarH, gd::Color{13, 148, 136, 120});
    }
    const float fill = static_cast<float>(std::clamp(tNow / dur, 0.0, 1.0)) * kStageW;
    gd::fillRect(r, kStageX, kBarY, fill, kBarH, gd::kTeal);
    gd::strokeRect(r, kStageX, kBarY, kStageW, kBarH, gd::kInk);
    gd::card(r, kStageX + fill - 4.0F, kBarY - 3.0F, 8.0F, kBarH + 6.0F, gd::kWhite, gd::kInk);
    char text[64];
    std::snprintf(text, sizeof text, "t %.1f / %.1f s", tNow, ga::duration(f));
    app.hud.text(r, kSideX, kBarY - 7.0F, text, gd::kMuted);
}

void numberCell(App& app, float x, float y, double v, gd::Color c) {
    char text[16];
    std::snprintf(text, sizeof text, "%5.1f", v);
    app.hud.text(app.renderer, x, y, text, c);
}

void drawTable(App& app, const ga::Flight& f, const ga::Encounter& e, double tNow) {
    SDL_Renderer* r = app.renderer;
    darkCard(r, kSideX, kTableY, kSideW, kTableH);
    const float x = kSideX;
    const float y = kTableY;
    constexpr float kColIn = 104.0F;
    constexpr float kColNow = 164.0F;
    constexpr float kColOut = 224.0F;
    app.hud.text(r, x + 12.0F, y + 8.0F, "SPEED", kLight, gd::Hud::Style::Bold);
    app.hud.text(r, x + 68.0F, y + 8.0F, "px/s", kDim);
    app.hud.text(r, x + kColIn + 20.0F, y + 8.0F, "in", kIn);
    app.hud.text(r, x + kColNow + 10.0F, y + 8.0F, "now", kLight);
    app.hud.text(r, x + kColOut + 14.0F, y + 8.0F, "out", kOut);

    const ga::Sample now = ga::sampleAt(f, tNow);
    const ga::Vec2 planetV = ga::planetVelocity(app.world);
    const bool sunShown = app.blend < 0.5;
    struct Row {
        const char* name;
        gd::Color dot;
        double in, out, cur;
    };
    const Row rows[2] = {{"sun", gd::kWhite, e.sunIn, e.sunOut, ga::length(now.vel)},
                         {"planet", gd::Color{150, 235, 215, 255}, e.planetIn, e.planetOut, ga::length(now.vel - planetV)}};
    for (int i = 0; i < 2; ++i) {
        const float ry = y + 34.0F + 24.0F * static_cast<float>(i);
        if ((i == 0) == sunShown) gd::fillRect(r, x + 6.0F, ry - 2.0F, kSideW - 12.0F, 24.0F, gd::Color{255, 255, 255, 22});
        gd::disc(r, x + 18.0F, ry + 11.0F, 4.5F, rows[i].dot, 16);
        app.hud.text(r, x + 30.0F, ry, rows[i].name, kLight);
        if (e.valid) {
            numberCell(app, x + kColIn, ry, rows[i].in, kIn);
            numberCell(app, x + kColOut, ry, rows[i].out, kOut);
        } else {
            app.hud.text(r, x + kColIn + 30.0F, ry, "-", kDim);
            app.hud.text(r, x + kColOut + 30.0F, ry, "-", kDim);
        }
        numberCell(app, x + kColNow, ry, rows[i].cur, kLight);
    }

    float ry = y + 90.0F;
    app.hud.text(r, x + 12.0F, ry, "gain", kDim);
    char text[64];
    if (e.valid) {
        const gd::Color tone = e.gain >= 0.0 ? kIn : kOut;
        std::snprintf(text, sizeof text, "%+.1f px/s", e.gain);
        app.hud.text(r, x + 90.0F, ry, text, tone, gd::Hud::Style::Bold);
        if (e.sunIn >= 20.0) {
            std::snprintf(text, sizeof text, "x%.2f", e.sunOut / e.sunIn);
            app.hud.text(r, x + 220.0F, ry, text, tone);
        }
    } else {
        app.hud.text(r, x + 90.0F, ry, f.outcome == ga::Outcome::Bound ? "bound to the planet" : "no flyby", kDim);
    }
    ry += 24.0F;
    app.hud.text(r, x + 12.0F, ry, "turn", kDim);
    if (e.valid) {
        std::snprintf(text, sizeof text, "%.0f\xC2\xB0", std::fabs(e.turn) * 180.0 / ga::kPi);
        app.hud.text(r, x + 90.0F, ry, text, kLight);
    } else {
        app.hud.text(r, x + 90.0F, ry, "-", kDim);
    }
    ry += 24.0F;
    app.hud.text(r, x + 12.0F, ry, "closest", kDim);
    if (f.outcome == ga::Outcome::Crash) {
        app.hud.text(r, x + 90.0F, ry, "crashed", kOut);
    } else {
        std::snprintf(text, sizeof text, "%.0f px", f.closest);
        app.hud.text(r, x + 90.0F, ry, text, kLight);
    }
}

void drawDiagram(App& app, const ga::Encounter& e) {
    SDL_Renderer* r = app.renderer;
    darkCard(r, kSideX, kDiagY, kSideW, kDiagH);
    app.hud.text(r, kSideX + 12.0F, kDiagY + 8.0F, "VELOCITIES v = V + u", kLight, gd::Hud::Style::Bold);
    if (!e.valid) {
        app.hud.text(r, kSideX + 12.0F, kDiagY + 70.0F, "no flyby, nothing to add up", kDim);
        return;
    }
    constexpr float kAreaX = kSideX + 8.0F;
    constexpr float kAreaY = kDiagY + 34.0F;
    constexpr float kAreaW = kSideW - 16.0F;
    constexpr float kAreaH = kDiagH - 40.0F;
    const ga::Vec2 planetV = ga::planetVelocity(app.world);
    const double uLen = 0.5 * (e.planetIn + e.planetOut);
    const ga::DiagramFit fit = ga::fitDiagram(planetV, uLen, kAreaW, kAreaH, 14.0);
    const auto at = [&](ga::Vec2 p) {
        const ga::Vec2 m = fit.map(p);
        return SDL_FPoint{kAreaX + static_cast<float>(m.x), kAreaY + static_cast<float>(m.y)};
    };
    const SDL_FPoint o = at({0.0, 0.0});
    const SDL_FPoint tip = at(planetV);
    const SDL_FPoint inEnd = at(planetV + e.uIn);
    const SDL_FPoint outEnd = at(planetV + e.uOut);

    // Every velocity the probe could leave with after some turn lies on this circle: its radius never changes.
    dashedCircle(r, tip, static_cast<float>(uLen * fit.scale), gd::Color{255, 255, 255, 80}, 72);
    // The sun's view: where the probe comes in and goes out, from the origin.
    drawArrow(r, o, inEnd, gd::Color{kIn.r, kIn.g, kIn.b, 150}, 7.0F);
    drawArrow(r, o, outEnd, gd::Color{kOut.r, kOut.g, kOut.b, 150}, 7.0F);
    // The planet's view: the same circle, from the tip of the planet's velocity.
    drawArrow(r, tip, inEnd, kIn, 8.0F);
    drawArrow(r, tip, outEnd, kOut, 8.0F);
    if (planetV.x != 0.0 || planetV.y != 0.0) drawArrow(r, o, tip, gd::Color{210, 215, 230, 255}, 8.0F);
    gd::disc(r, o.x, o.y, 3.5F, gd::kWhite, 12);

    char text[48];
    std::snprintf(text, sizeof text, "v %.0f", e.sunIn);
    const SDL_FPoint inLabel{std::clamp(inEnd.x + (inEnd.x >= tip.x ? 6.0F : -6.0F - app.hud.width(text)), kSideX + 6.0F, kSideX + kSideW - app.hud.width(text) - 6.0F),
                             std::clamp(inEnd.y + (inEnd.y >= tip.y ? 4.0F : -24.0F), kDiagY + 30.0F, kDiagY + kDiagH - 26.0F)};
    app.hud.text(r, inLabel.x, inLabel.y, text, kIn);
    std::snprintf(text, sizeof text, "v %.0f", e.sunOut);
    const SDL_FPoint outLabel{std::clamp(outEnd.x + (outEnd.x >= tip.x ? 6.0F : -6.0F - app.hud.width(text)), kSideX + 6.0F, kSideX + kSideW - app.hud.width(text) - 6.0F),
                              std::clamp(outEnd.y + (outEnd.y >= tip.y ? 4.0F : -24.0F), kDiagY + 30.0F, kDiagY + kDiagH - 26.0F)};
    app.hud.text(r, outLabel.x, outLabel.y, text, kOut);
    std::snprintf(text, sizeof text, "u %.0f", uLen);
    app.hud.text(r, kSideX + kSideW - app.hud.width(text) - 12.0F, kDiagY + 8.0F, text, gd::Color{150, 235, 215, 255});
    std::snprintf(text, sizeof text, "V %.0f", app.world.planetSpeed);
    app.hud.text(r, std::clamp((o.x + tip.x) / 2.0F - 14.0F, kSideX + 6.0F, kSideX + kSideW - 60.0F), std::clamp(o.y + 6.0F, kDiagY + 30.0F, kDiagY + kDiagH - 26.0F), text,
                 gd::Color{210, 215, 230, 255});
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const double clock = static_cast<double>(SDL_GetTicks()) / 1000.0;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ GRAVITY ASSIST ]", "borrowing speed from a planet");
    drawSky(app, clock);

    const bool aim = app.aiming && app.aimOk;
    const ga::Flight& f = aim ? app.aimFlight : app.flight;
    const ga::Encounter& e = aim ? app.aimEnc : app.enc;
    const double tNow = app.aiming ? 0.0 : app.time;

    const SDL_Rect clip{static_cast<int>(kStageX) + 1, static_cast<int>(kStageY) + 1, static_cast<int>(kStageW) - 2, static_cast<int>(kStageH) - 2};
    SDL_SetRenderClipRect(r, &clip);
    drawSunAndDust(app, tNow);
    if (!app.aiming || app.aimOk) {
        drawTicks(app, f, tNow);
        drawPath(app, f, tNow, app.aiming ? 190 : 120);
    }
    drawPlanet(app, tNow, false);
    if (!app.aiming || app.aimOk) {
        drawEncounterMarkers(app, f, e);
        const ga::Sample s = ga::sampleAt(f, tNow);
        const bool crashed = f.outcome == ga::Outcome::Crash && tNow >= ga::duration(f) - 1e-9;
        if (!app.aiming) drawProbe(app, toScreen(app, s.pos, tNow), viewVelocity(app, s.vel), crashed);
    }
    if (app.aiming) drawAimMarker(app);
    SDL_SetRenderClipRect(r, nullptr);

    drawStageLabels(app, f);
    drawBar(app, f, e, tNow);
    drawTable(app, f, e, tNow);
    drawDiagram(app, e);

    if (SDL_GetTicks() < app.flashUntil) {
        const float w = app.hud.width(app.flash, gd::Hud::Style::Bold) + 28.0F;
        const float x = kStageX + kStageW / 2.0F - w / 2.0F;
        gd::card(r, x, kStageY + kStageH - 56.0F, w, 32.0F, gd::kWhite, gd::kOrange);
        app.hud.text(r, x + 14.0F, kStageY + kStageH - 51.0F, app.flash, gd::kOrange, gd::Hud::Style::Bold);
    }

    app.hud.text(r, 40.0F, 417.0F, "drag: start + velocity   right click / Esc: cancel   bar: scrub   keys F V M", gd::kMuted);
    for (const Label& l : app.labels) app.hud.text(r, l.x, l.y, l.text, gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const gd::Color fill = on ? gd::kTeal : gd::kWhite;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, c.label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
    }
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
    if (!SDL_CreateWindowAndRenderer("gravity-assist", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->stars = makeStars();
    app->dust = makeDust();
    layoutChips(*app);
    setPreset(*app, 0);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {event->motion.x, event->motion.y};
            if (app.seeking) seekTo(app, app.mouse.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
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
            if (onBar(p)) {
                app.seeking = true;
                seekTo(app, p.x);
            } else if (inStage(p)) {
                app.aiming = true;
                app.aimStart = p;
                updateAim(app);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) {
                app.mouse = {event->button.x, event->button.y};
                if (app.aiming) {
                    updateAim(app);
                    finishAim(app);
                }
                app.seeking = false;
            }
            break;
        // Letting go outside the window never reaches us, so leaving or losing focus cancels a drag.
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.aiming = false;
            app.seeking = false;
            break;
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
                case SDLK_F: apply(app, Action::Frame, app.planetFrame ? 0 : 1); break;
                case SDLK_V: apply(app, Action::Speed, static_cast<int>((app.speedIndex + 1) % ga::kSpeeds.size())); break;
                case SDLK_M: apply(app, Action::Mass, static_cast<int>((app.massIndex + 1) % ga::kMasses.size())); break;
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
