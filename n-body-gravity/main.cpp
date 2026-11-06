// N-body gravity: a handful of bodies pull on each other, and you can watch what a bad integrator or a missing
// softening does to them.
//
// The physics lives in logic.hpp (no SDL, covered by test.cpp). This file owns the loop, the input and the picture.
// The loop is a fixed timestep: frame time, scaled to simulated seconds, goes into an accumulator and the system is
// stepped in slices of exactly the chosen dt, so the numbers do not depend on the frame rate. Frame time is clamped
// and the steps per frame are capped, otherwise a stalled tab would trigger thousands of catch-up steps.
//
// Orange marks the naive choice (explicit Euler, no softening), teal the one that holds up. Energy and momentum are
// printed live: momentum survives every integrator because the pair forces are symmetric, energy is the one that
// tells them apart. Dragging in space launches a new body, the dotted line is where the whole system would carry it.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kViewTop = 30.0F;
constexpr float kViewHeight = 402.0F;
constexpr float kCenterX = 480.0F;
constexpr float kCenterY = kViewTop + kViewHeight / 2.0F;

constexpr double kMaxFrameTime = 0.25;       // real seconds, clamp against a stalled tab
constexpr int kMaxStepsPerFrame = 600;       // more than this and the rest of the backlog is dropped
constexpr std::size_t kTrailPoints = 260;
constexpr double kSampleEvery = 0.05;        // simulated seconds between two points of the energy graph
constexpr std::size_t kHistory = 220;
constexpr double kDriftScale = 1e-6;         // symlog threshold: drifts below this look like zero
constexpr double kDriftDecades = 8.0;        // the graph spans +-8 decades above that
constexpr double kPreviewStep = 0.02;
constexpr std::size_t kPreviewPoints = 200;
constexpr float kMaxDragPixels = 300.0F;
constexpr std::size_t kStars = 120;
constexpr double kTwoPi = 6.283185307179586;

struct Tint {
    Uint8 r, g, b;
};

// Bright versions of the demo colors, readable on the dark sky.
constexpr std::array<Tint, 6> kTints{{{251, 146, 60}, {45, 212, 191}, {226, 244, 240}, {253, 186, 116}, {94, 234, 212}, {252, 211, 170}}};
constexpr gd::Color kSkyTop{7, 10, 24};
constexpr gd::Color kSkyBottom{15, 22, 44};
constexpr gd::Color kText{226, 232, 240};
constexpr gd::Color kDim{148, 163, 184};
constexpr gd::Color kGood{45, 212, 191};
constexpr gd::Color kWarn{251, 191, 36};
constexpr gd::Color kBad{251, 113, 36};

SDL_FColor fcol(Tint t, float alpha) { return SDL_FColor{t.r / 255.0F, t.g / 255.0F, t.b / 255.0F, alpha}; }
SDL_FColor fcol(gd::Color c, float alpha) { return SDL_FColor{c.r / 255.0F, c.g / 255.0F, c.b / 255.0F, alpha}; }

enum class Act { Scene, Method, Step, Softening, Trails, Pause, Reset };

struct Chip {
    SDL_FRect rect;
    const char* label;
    Act act;
    int value;
    bool naive;   // active state is drawn in orange: this is the choice the demo warns about
    bool accent;  // orange edge
};

struct Label {
    float x;
    float y;
    const char* text;
};

struct Launch {
    bool active = false;
    float startX = 0.0F;
    float startY = 0.0F;
    float curX = 0.0F;
    float curY = 0.0F;
};

struct Star {
    float x, y;
    float size;
    float alpha;
    float speed;   // twinkle speed, a whole number so the phase can be wrapped
    float offset;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;

    nb::Scene scene = nb::Scene::Figure8;
    nb::Method method = nb::Method::Leapfrog;
    std::size_t stepIndex = 1;
    bool softening = false;
    bool trailsOn = true;
    bool paused = false;
    nb::System sys;
    double massRef = 1.0;

    double simTime = 0.0;
    double accumulator = 0.0;
    double trailClock = 0.0;
    double sampleClock = 0.0;
    double e0 = 0.0;

    std::array<std::array<nb::Vec2, kTrailPoints>, nb::kMaxBodies> trail{};
    std::array<std::size_t, nb::kMaxBodies> trailCount{};
    std::size_t trailHead = 0;

    std::array<float, kHistory> history{};
    std::size_t historyHead = 0;
    std::size_t historyCount = 0;

    std::array<Chip, 24> chips{};
    std::size_t chipCount = 0;
    std::array<Label, 6> labels{};
    std::size_t labelCount = 0;

    // Buffers for the trail ribbons and the launch preview, allocated once.
    std::array<SDL_Vertex, 2 * (kTrailPoints + 1)> ribbon{};
    std::array<int, 6 * kTrailPoints> ribbonIndex{};
    std::array<nb::Vec2, kPreviewPoints> preview{};
    std::size_t previewCount = 0;
    std::array<Star, kStars> stars{};

    Launch launch;
    double wallClock = 0.0;     // real seconds modulo one twinkle period
    double noticeLeft = 0.0;
    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

double step(const App& app) { return nb::kStepSizes[app.stepIndex]; }

const char* methodName(nb::Method m) {
    switch (m) {
        case nb::Method::Euler: return "euler";
        case nb::Method::Leapfrog: return "leapfrog";
        case nb::Method::Rk4: return "rk4";
    }
    return "?";
}

// ---- coordinates ----

SDL_FPoint toScreen(const App& app, nb::Vec2 w) {
    const double s = nb::info(app.scene).pxPerUnit;
    // Clamped, so a body flung to 1e300 stays a finite float.
    const double x = std::clamp(w.x * s, -1e5, 1e5);
    const double y = std::clamp(w.y * s, -1e5, 1e5);
    return SDL_FPoint{kCenterX + static_cast<float>(x), kCenterY - static_cast<float>(y)};
}

nb::Vec2 toWorld(const App& app, float px, float py) {
    const double s = nb::info(app.scene).pxPerUnit;
    return {static_cast<double>(px - kCenterX) / s, -static_cast<double>(py - kCenterY) / s};
}

// ---- simulation state ----

void rebaseline(App& app) {
    app.e0 = nb::totalEnergy(app.sys);
    app.historyHead = 0;
    app.historyCount = 0;
    app.sampleClock = 0.0;
    app.history[app.historyHead] = 0.0F;
    app.historyHead = (app.historyHead + 1) % kHistory;
    app.historyCount = 1;
}

void pushTrail(App& app) {
    for (std::size_t i = 0; i < app.sys.count; ++i) {
        app.trail[i][app.trailHead] = app.sys.bodies[i].pos;
        app.trailCount[i] = std::min(app.trailCount[i] + 1, kTrailPoints);
    }
    app.trailHead = (app.trailHead + 1) % kTrailPoints;
}

void pushSample(App& app) {
    const double drift = nb::relativeDrift(nb::totalEnergy(app.sys), app.e0);
    app.history[app.historyHead] = static_cast<float>(nb::symlog(drift, kDriftScale));
    app.historyHead = (app.historyHead + 1) % kHistory;
    app.historyCount = std::min(app.historyCount + 1, kHistory);
}

/// One fixed step plus the bookkeeping. Returns false if the numbers left the finite range.
bool stepOnce(App& app, double dt) {
    nb::step(app.sys, app.method, dt);
    app.simTime += dt;
    if (!nb::allFinite(app.sys)) return false;

    const double trailEvery = nb::info(app.scene).trailSeconds / static_cast<double>(kTrailPoints);
    app.trailClock += dt;
    if (app.trailClock >= trailEvery) {
        pushTrail(app);
        app.trailClock = std::fmod(app.trailClock, trailEvery);
    }
    app.sampleClock += dt;
    if (app.sampleClock >= kSampleEvery) {
        pushSample(app);
        app.sampleClock = std::fmod(app.sampleClock, kSampleEvery);
    }
    return true;
}

void loadScene(App& app, nb::Scene scene, bool applySceneStep) {
    const nb::SceneInfo& si = nb::info(scene);
    app.scene = scene;
    if (applySceneStep) app.stepIndex = static_cast<std::size_t>(si.defaultStep);
    app.sys = nb::makeScene(scene);
    app.sys.softening = app.softening ? si.softening : 0.0;
    app.massRef = 0.0;
    for (std::size_t i = 0; i < app.sys.count; ++i) app.massRef = std::max(app.massRef, app.sys.bodies[i].mass);
    app.simTime = 0.0;
    app.accumulator = 0.0;
    app.trailClock = 0.0;
    app.trailHead = 0;
    app.trailCount.fill(0);
    app.launch.active = false;
    app.previewCount = 0;
    rebaseline(app);
    // Run a few seconds before the first frame so the picture opens with trails instead of a bare start position.
    pushTrail(app);
    const double dt = step(app);
    for (int n = 0; app.simTime < si.prerollSeconds && n < 100000; ++n) {
        if (!stepOnce(app, dt)) break;
    }
}

void advance(App& app, double simDelta) {
    const double dt = step(app);
    app.accumulator += simDelta;
    int steps = 0;
    while (app.accumulator >= dt && steps < kMaxStepsPerFrame) {
        if (!stepOnce(app, dt)) {
            loadScene(app, app.scene, false);
            app.noticeLeft = 3.0;
            return;
        }
        app.accumulator -= dt;
        ++steps;
    }
    if (app.accumulator >= dt) app.accumulator = std::fmod(app.accumulator, dt);  // drop what the cap left over
}

/// Method, step and softening restart the scene: every run starts from the same bodies, so two settings can be
/// compared by changing exactly one thing. The pre-roll then runs with the new setting.
void changeSetting(App& app) { loadScene(app, app.scene, false); }

bool spawnBody(App& app, nb::Vec2 pos, nb::Vec2 vel) {
    if (!app.sys.add(nb::Body{pos, vel, nb::info(app.scene).spawnMass})) return false;
    app.trailCount[app.sys.count - 1] = 0;
    rebaseline(app);  // the new body brings energy of its own
    return true;
}

// ---- input ----

bool isActive(const App& app, const Chip& c) {
    switch (c.act) {
        case Act::Scene: return static_cast<int>(app.scene) == c.value;
        case Act::Method: return static_cast<int>(app.method) == c.value;
        case Act::Step: return static_cast<int>(app.stepIndex) == c.value;
        case Act::Softening: return app.softening == (c.value == 1);
        case Act::Trails: return app.trailsOn;
        case Act::Pause: return app.paused;
        case Act::Reset: return false;
    }
    return false;
}

void apply(App& app, Act act, int value) {
    switch (act) {
        case Act::Scene: loadScene(app, static_cast<nb::Scene>(value), true); break;
        case Act::Method:
            app.method = static_cast<nb::Method>(value);
            changeSetting(app);
            break;
        case Act::Step:
            app.stepIndex = static_cast<std::size_t>(value);
            changeSetting(app);
            break;
        case Act::Softening:
            app.softening = value == 1;
            changeSetting(app);
            break;
        case Act::Trails: app.trailsOn = !app.trailsOn; break;
        case Act::Pause: app.paused = !app.paused; break;
        case Act::Reset: loadScene(app, app.scene, true); break;
    }
}

void buildLayout(App& app) {
    constexpr float kChipH = 30.0F;
    constexpr float kGap = 8.0F;
    float x = 40.0F;
    float y = 441.0F;

    auto label = [&](const char* text) {
        app.labels[app.labelCount++] = Label{x, y + 4.0F, text};
        x += app.hud.width(text) + 12.0F;
    };
    auto chip = [&](const char* text, Act act, int value, bool naive = false, bool accent = false, const char* widest = nullptr) {
        const float w = app.hud.width(widest != nullptr ? widest : text, gd::Hud::Style::Bold) + 24.0F;
        app.chips[app.chipCount++] = Chip{SDL_FRect{x, y, w, kChipH}, text, act, value, naive, accent};
        x += w + kGap;
    };

    label("scene");
    for (int i = 0; i < nb::kSceneCount; ++i) chip(nb::info(static_cast<nb::Scene>(i)).label, Act::Scene, i);
    x += 12.0F;
    label("method");
    chip("euler", Act::Method, 0, true);
    chip("leapfrog", Act::Method, 1);
    chip("rk4", Act::Method, 2);

    x = 40.0F;
    y = 477.0F;
    label("dt");
    chip("0.05", Act::Step, 0);
    chip("0.01", Act::Step, 1);
    chip("0.002", Act::Step, 2);
    x += 12.0F;
    label("softening");
    chip("off", Act::Softening, 0, true);
    chip("on", Act::Softening, 1);
    x += 12.0F;
    chip("trails", Act::Trails, 0);
    chip("pause", Act::Pause, 0, false, false, "resume");
    chip("reset", Act::Reset, 0, false, true);
}

void cancelLaunch(App& app) {
    app.launch.active = false;
    app.previewCount = 0;
}

/// Velocity of a launch: the drag arrow is the distance the body would cover in one simulated second.
nb::Vec2 launchVelocity(const App& app) {
    float dx = app.launch.curX - app.launch.startX;
    float dy = app.launch.curY - app.launch.startY;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len > kMaxDragPixels) {
        dx *= kMaxDragPixels / len;
        dy *= kMaxDragPixels / len;
    }
    const double s = nb::info(app.scene).pxPerUnit;
    return {static_cast<double>(dx) / s, -static_cast<double>(dy) / s};
}

void computePreview(App& app) {
    app.previewCount = 0;
    if (!app.launch.active) return;
    const nb::Vec2 pos = toWorld(app, app.launch.startX, app.launch.startY);
    const nb::Vec2 vel = launchVelocity(app);
    nb::System copy = app.sys;
    if (!copy.add(nb::Body{pos, vel, nb::info(app.scene).spawnMass})) return;
    const std::size_t me = copy.count - 1;
    for (std::size_t k = 0; k < kPreviewPoints; ++k) {
        nb::stepLeapfrog(copy, kPreviewStep);
        if (!nb::allFinite(copy)) break;
        app.preview[k] = copy.bodies[me].pos;
        app.previewCount = k + 1;
    }
}

bool inViewport(float x, float y) { return x >= 0.0F && x < kWidth && y > kViewTop && y < kViewTop + kViewHeight; }

// ---- drawing ----

const std::array<SDL_FPoint, 29>& unitCircle() {
    static const std::array<SDL_FPoint, 29> circle = [] {
        std::array<SDL_FPoint, 29> c{};
        for (std::size_t i = 0; i < c.size(); ++i) {
            const double a = kTwoPi * static_cast<double>(i) / 28.0;
            c[i] = SDL_FPoint{static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a))};
        }
        return c;
    }();
    return circle;
}

/// Triangle fan with a color in the middle and another on the rim: a disc or, with a transparent rim, a glow.
void fan(SDL_Renderer* r, float cx, float cy, float radius, SDL_FColor center, SDL_FColor rim) {
    const auto& circle = unitCircle();
    std::array<SDL_Vertex, 30> v{};
    std::array<int, 28 * 3> idx{};
    v[0] = SDL_Vertex{{cx, cy}, center, {0, 0}};
    for (std::size_t i = 0; i < circle.size(); ++i) {
        v[i + 1] = SDL_Vertex{{cx + circle[i].x * radius, cy + circle[i].y * radius}, rim, {0, 0}};
    }
    for (std::size_t i = 0; i < 28; ++i) {
        idx[i * 3] = 0;
        idx[i * 3 + 1] = static_cast<int>(i + 1);
        idx[i * 3 + 2] = static_cast<int>(i + 2);
    }
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

void line(SDL_Renderer* r, gd::Color c, float x0, float y0, float x1, float y1) {
    gd::setColor(r, c);
    SDL_RenderLine(r, x0, y0, x1, y1);
}

float bodyRadius(const App& app, const nb::Body& b) {
    const nb::SceneInfo& si = nb::info(app.scene);
    const double ratio = std::min(b.mass / std::max(app.massRef, 1e-12), 1.6);
    return static_cast<float>(2.4 + (si.bodyScale - 2.4) * std::cbrt(ratio));
}

void drawSky(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::gradient(r, 0.0F, kViewTop, kWidth, kViewHeight, kSkyTop, kSkyBottom);
    // Two soft clouds of color, far behind everything else.
    fan(r, 240.0F, 150.0F, 260.0F, fcol(kGood, 0.07F), fcol(kGood, 0.0F));
    fan(r, 760.0F, 360.0F, 300.0F, fcol(kBad, 0.06F), fcol(kBad, 0.0F));
    for (const Star& s : app.stars) {
        const float tw = 0.7F + 0.3F * std::sin(static_cast<float>(app.wallClock) * s.speed + s.offset);
        const Uint8 a = static_cast<Uint8>(std::clamp(s.alpha * tw * 255.0F, 0.0F, 255.0F));
        gd::fillRect(r, s.x, s.y, s.size, s.size, gd::Color{210, 225, 255, a});
    }
}

void drawTrail(App& app, std::size_t i) {
    const std::size_t count = app.trailCount[i];
    if (count < 1) return;
    // The ribbon runs from the oldest stored point to the live position, thin and faint to wide and bright.
    const std::size_t points = count + 1;
    std::array<SDL_FPoint, kTrailPoints + 1> pts{};
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t slot = (app.trailHead + kTrailPoints - count + k) % kTrailPoints;
        pts[k] = toScreen(app, app.trail[i][slot]);
    }
    pts[count] = toScreen(app, app.sys.bodies[i].pos);

    const Tint tint = kTints[i % kTints.size()];
    float nx = 0.0F;
    float ny = 1.0F;
    for (std::size_t k = 0; k < points; ++k) {
        const SDL_FPoint a = pts[k == 0 ? 0 : k - 1];
        const SDL_FPoint b = pts[k + 1 < points ? k + 1 : k];
        const float tx = b.x - a.x;
        const float ty = b.y - a.y;
        const float len = std::sqrt(tx * tx + ty * ty);
        if (len > 1e-4F) {
            nx = -ty / len;
            ny = tx / len;
        }
        const float t = static_cast<float>(k) / static_cast<float>(points - 1 > 0 ? points - 1 : 1);
        const float half = 0.35F + 1.25F * t;
        const float alpha = 0.9F * t * t;
        app.ribbon[2 * k] = SDL_Vertex{{pts[k].x + nx * half, pts[k].y + ny * half}, fcol(tint, alpha), {0, 0}};
        app.ribbon[2 * k + 1] = SDL_Vertex{{pts[k].x - nx * half, pts[k].y - ny * half}, fcol(tint, alpha), {0, 0}};
    }
    SDL_RenderGeometry(app.renderer, nullptr, app.ribbon.data(), static_cast<int>(2 * points), app.ribbonIndex.data(),
                       static_cast<int>(6 * (points - 1)));
}

void drawBody(App& app, std::size_t i) {
    SDL_Renderer* r = app.renderer;
    const nb::Body& b = app.sys.bodies[i];
    const SDL_FPoint p = toScreen(app, b.pos);
    const float rad = bodyRadius(app, b);
    const Tint tint = kTints[i % kTints.size()];

    const float hw = (kWidth - 1.0F) / 2.0F - 10.0F;
    const float hh = kViewHeight / 2.0F - 10.0F;
    const bool outside = std::fabs(p.x - kCenterX) > hw + 10.0F || std::fabs(p.y - kCenterY) > hh + 10.0F;
    if (outside) {
        // Off screen: a small arrow at the edge shows which way it left.
        const nb::Vec2 e = nb::edgePoint(hw, hh, static_cast<double>(p.x - kCenterX), static_cast<double>(p.y - kCenterY));
        const float ex = kCenterX + static_cast<float>(e.x);
        const float ey = kCenterY + static_cast<float>(e.y);
        const float dx = p.x - kCenterX;
        const float dy = p.y - kCenterY;
        const float dl = std::max(std::sqrt(dx * dx + dy * dy), 1e-3F);
        const float ux = dx / dl;
        const float uy = dy / dl;
        const SDL_Vertex tri[3] = {{{ex + ux * 8.0F, ey + uy * 8.0F}, fcol(tint, 0.9F), {0, 0}},
                                   {{ex - ux * 5.0F - uy * 5.0F, ey - uy * 5.0F + ux * 5.0F}, fcol(tint, 0.9F), {0, 0}},
                                   {{ex - ux * 5.0F + uy * 5.0F, ey - uy * 5.0F - ux * 5.0F}, fcol(tint, 0.9F), {0, 0}}};
        SDL_RenderGeometry(r, nullptr, tri, 3, nullptr, 0);
        return;
    }
    fan(r, p.x, p.y, rad * 4.2F, fcol(tint, 0.26F), fcol(tint, 0.0F));
    fan(r, p.x, p.y, rad * 1.9F, fcol(tint, 0.38F), fcol(tint, 0.0F));
    const SDL_FColor hot{std::min(1.0F, tint.r / 255.0F * 0.35F + 0.65F), std::min(1.0F, tint.g / 255.0F * 0.35F + 0.65F),
                         std::min(1.0F, tint.b / 255.0F * 0.35F + 0.65F), 1.0F};
    fan(r, p.x, p.y, rad, hot, fcol(tint, 1.0F));
}

void drawLaunch(App& app) {
    if (!app.launch.active) return;
    SDL_Renderer* r = app.renderer;
    const nb::SceneInfo& si = nb::info(app.scene);

    // Dotted path of the new body through the field of all the others.
    for (std::size_t k = 0; k < app.previewCount; k += 3) {
        const SDL_FPoint p = toScreen(app, app.preview[k]);
        const float fade = 1.0F - static_cast<float>(k) / static_cast<float>(kPreviewPoints);
        const Uint8 a = static_cast<Uint8>(255.0F * (0.25F + 0.6F * fade));
        gd::fillRect(r, p.x - 1.5F, p.y - 1.5F, 3.0F, 3.0F, gd::Color{251, 146, 60, a});
    }

    // Ghost body at the start and the velocity arrow.
    const nb::Vec2 v = launchVelocity(app);
    const float s = static_cast<float>(si.pxPerUnit);
    const float ex = app.launch.startX + static_cast<float>(v.x) * s;
    const float ey = app.launch.startY - static_cast<float>(v.y) * s;
    const float len = std::sqrt((ex - app.launch.startX) * (ex - app.launch.startX) + (ey - app.launch.startY) * (ey - app.launch.startY));
    fan(r, app.launch.startX, app.launch.startY, 9.0F, fcol(kBad, 0.45F), fcol(kBad, 0.0F));
    gd::ring(r, app.launch.startX, app.launch.startY, 4.0F, kBad);
    if (len > 3.0F) {
        const float ux = (ex - app.launch.startX) / len;
        const float uy = (ey - app.launch.startY) / len;
        line(r, kBad, app.launch.startX, app.launch.startY, ex, ey);
        line(r, kBad, ex, ey, ex - ux * 9.0F - uy * 5.0F, ey - uy * 9.0F + ux * 5.0F);
        line(r, kBad, ex, ey, ex - ux * 9.0F + uy * 5.0F, ey - uy * 9.0F - ux * 5.0F);
        char text[32];
        SDL_snprintf(text, sizeof text, "v %.2f", nb::length(v));
        app.hud.text(r, std::clamp(ex + 8.0F, 4.0F, kWidth - 80.0F), std::clamp(ey - 8.0F, kViewTop + 4.0F, kViewTop + kViewHeight - 26.0F), text, kBad);
    }
}

void formatEnergy(char* out, std::size_t n, double e) {
    if (!std::isfinite(e)) {
        SDL_snprintf(out, n, "n/a");
    } else if (std::fabs(e) >= 1e5) {
        SDL_snprintf(out, n, "%.2e", e);
    } else {
        SDL_snprintf(out, n, "%.4f", e);
    }
}

void drawOverlay(App& app) {
    SDL_Renderer* r = app.renderer;
    const double energy = nb::totalEnergy(app.sys);
    const double drift = nb::relativeDrift(energy, app.e0);
    const nb::Vec2 p = nb::momentum(app.sys);
    const double minDist = nb::minDistance(app.sys);

    // Left: the numbers. Every line is a quantity the physics promises (or fails) to keep.
    char a[48], b[48], c[48], d[48], e[48];
    SDL_snprintf(a, sizeof a, "t      %.2f", app.simTime);
    char energyText[32];
    formatEnergy(energyText, sizeof energyText, energy);
    SDL_snprintf(b, sizeof b, "E      %s", energyText);
    const double percent = drift * 100.0;
    if (!std::isfinite(drift)) {
        SDL_snprintf(c, sizeof c, "dE/E0  n/a");
    } else if (percent != 0.0 && (std::fabs(percent) < 1e-3 || std::fabs(percent) >= 1000.0)) {
        SDL_snprintf(c, sizeof c, "dE/E0  %+.1e %%", percent);  // tiny or huge: scientific
    } else {
        SDL_snprintf(c, sizeof c, "dE/E0  %+.4f %%", percent);
    }
    SDL_snprintf(d, sizeof d, "|P|    %.1e", nb::length(p));
    if (std::isinf(minDist)) {
        SDL_snprintf(e, sizeof e, "r_min  -");
    } else {
        SDL_snprintf(e, sizeof e, "r_min  %.4f", minDist);
    }
    const float x = 14.0F;
    float y = kViewTop + 10.0F;
    app.hud.text(r, x, y, a, kDim);
    app.hud.text(r, x, y + 22.0F, b, kText);
    const double mag = std::fabs(drift);
    const gd::Color driftColor = mag < 1e-3 ? kGood : (mag < 1e-1 ? kWarn : kBad);
    app.hud.text(r, x, y + 44.0F, c, driftColor, gd::Hud::Style::Bold);
    app.hud.text(r, x, y + 66.0F, d, kDim);
    app.hud.text(r, x, y + 88.0F, e, kDim);

    // Right: the energy graph on a symmetric log axis, so 1e-6 and 1e+2 fit on one scale.
    const float gx = 724.0F;
    const float gy = kViewTop + 14.0F;
    const float gw = 220.0F;
    const float gh = 116.0F;
    gd::fillRect(r, gx, gy, gw, gh, gd::Color{4, 7, 18, 150});
    gd::strokeRect(r, gx, gy, gw, gh, gd::Color{148, 163, 184, 70});
    const float mid = gy + gh / 2.0F;
    const float half = gh / 2.0F - 3.0F;
    auto axisY = [&](double decades) { return mid - static_cast<float>(decades / kDriftDecades) * half; };
    line(r, gd::Color{148, 163, 184, 110}, gx, mid, gx + gw, mid);
    for (double dec : {3.0, 6.0}) {
        line(r, gd::Color{148, 163, 184, 45}, gx, axisY(dec), gx + gw, axisY(dec));
        line(r, gd::Color{148, 163, 184, 45}, gx, axisY(-dec), gx + gw, axisY(-dec));
    }
    app.hud.text(r, gx + 4.0F, axisY(6.0) - 11.0F, "100%", gd::Color{148, 163, 184, 170});
    app.hud.text(r, gx + 4.0F, axisY(3.0) - 11.0F, "0.1%", gd::Color{148, 163, 184, 170});
    app.hud.text(r, gx + 4.0F, mid - 11.0F, "0", gd::Color{148, 163, 184, 170});
    app.hud.text(r, gx, gy + gh + 2.0F, "energy drift", kDim);

    if (app.historyCount >= 2) {
        // Newest sample on the right edge, the history runs off to the left.
        const float step = (gw - 2.0F) / static_cast<float>(kHistory - 1);
        float prevX = 0.0F;
        float prevY = 0.0F;
        for (std::size_t k = 0; k < app.historyCount; ++k) {
            const std::size_t slot = (app.historyHead + kHistory - app.historyCount + k) % kHistory;
            const float v = std::clamp(app.history[slot] / static_cast<float>(kDriftDecades), -1.0F, 1.0F);
            const float px = gx + gw - 1.0F - static_cast<float>(app.historyCount - 1 - k) * step;
            const float py = mid - v * half;
            if (k > 0) line(r, driftColor, prevX, prevY, px, py);
            prevX = px;
            prevY = py;
        }
    }

    // Bottom: what to do, and what is in the sky.
    app.hud.text(r, 14.0F, kViewTop + kViewHeight - 26.0F, "drag: launch a body, arrow = 1 s of flight", kDim);
    char count[48];
    SDL_snprintf(count, sizeof count, "bodies %d/%d", static_cast<int>(app.sys.count), static_cast<int>(nb::kMaxBodies));
    const bool full = app.sys.count >= nb::kMaxBodies;
    app.hud.text(r, kWidth - 14.0F - app.hud.width(count), kViewTop + kViewHeight - 26.0F, count, full ? kBad : kDim);
    if (app.noticeLeft > 0.0) {
        const char* note = "numbers overflowed, scene restarted";
        app.hud.text(r, (kWidth - app.hud.width(note)) / 2.0F, kViewTop + 12.0F, note, kBad);
    }
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    for (std::size_t i = 0; i < app.labelCount; ++i) app.hud.text(r, app.labels[i].x, app.labels[i].y, app.labels[i].text, gd::kMuted);
    for (std::size_t i = 0; i < app.chipCount; ++i) {
        const Chip& c = app.chips[i];
        const bool on = isActive(app, c);
        const gd::Color fill = on ? (c.naive ? gd::kOrange : gd::kTeal) : gd::kWhite;
        const gd::Color edge = (c.accent || c.naive) ? gd::kOrange : gd::kInk;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, edge);
        const char* text = c.label;
        if (c.act == Act::Pause && app.paused) text = "resume";
        const gd::Color ink = on ? gd::kWhite : (c.accent || c.naive ? gd::kOrange : gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(text, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, text, ink, gd::Hud::Style::Bold);
    }
}

void render(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);

    const SDL_Rect clip{0, static_cast<int>(kViewTop), kWidth, static_cast<int>(kViewHeight)};
    SDL_SetRenderClipRect(r, &clip);
    drawSky(app);
    if (app.trailsOn) {
        for (std::size_t i = 0; i < app.sys.count; ++i) drawTrail(app, i);
    }
    // Center of mass: it stays put under every integrator, because momentum does.
    const SDL_FPoint com = toScreen(app, nb::centerOfMass(app.sys));
    line(r, gd::Color{148, 163, 184, 90}, com.x - 5.0F, com.y, com.x + 5.0F, com.y);
    line(r, gd::Color{148, 163, 184, 90}, com.x, com.y - 5.0F, com.x, com.y + 5.0F);
    for (std::size_t i = 0; i < app.sys.count; ++i) drawBody(app, i);
    drawLaunch(app);
    drawOverlay(app);
    SDL_SetRenderClipRect(r, nullptr);
    gd::strokeRect(r, 0.0F, kViewTop, kWidth, kViewHeight, gd::kInk);

    char right[96];
    SDL_snprintf(right, sizeof right, "%s \xC2\xB7 %s \xC2\xB7 dt %.3g%s", nb::info(app.scene).label, methodName(app.method), step(app),
                 app.paused ? " \xC2\xB7 paused" : "");
    app.hud.titleBar(r, kWidth, "[ N-BODY GRAVITY ]", right);
    drawChips(app);
    const char* keys = "keys: 1-4 scene  M method  S softening  T trails  R reset  space pause";
    app.hud.text(r, 40.0F, 513.0F, keys, gd::kMuted);
    SDL_RenderPresent(r);
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
    if (!SDL_CreateWindowAndRenderer("n-body-gravity", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX)) {
        SDL_Log("logical presentation failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);

    // The ribbon index pattern never changes: two triangles per segment of the strip.
    for (std::size_t k = 0; k < kTrailPoints; ++k) {
        const int base = static_cast<int>(2 * k);
        const std::size_t o = 6 * k;
        app->ribbonIndex[o] = base;
        app->ribbonIndex[o + 1] = base + 1;
        app->ribbonIndex[o + 2] = base + 2;
        app->ribbonIndex[o + 3] = base + 1;
        app->ribbonIndex[o + 4] = base + 3;
        app->ribbonIndex[o + 5] = base + 2;
    }
    nb::Rng rng{987654321};
    for (Star& s : app->stars) {
        s.x = static_cast<float>(rng.uniform()) * kWidth;
        s.y = kViewTop + static_cast<float>(rng.uniform()) * kViewHeight;
        const bool big = rng.uniform() < 0.12;
        s.size = big ? 2.0F : 1.0F;
        s.alpha = big ? 0.9F : 0.25F + 0.5F * static_cast<float>(rng.uniform());
        s.speed = static_cast<float>(1 + static_cast<int>(rng.uniform() * 3.0));
        s.offset = static_cast<float>(rng.uniform() * kTwoPi);
    }

    buildLayout(*app);
    loadScene(*app, nb::Scene::Figure8, true);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    SDL_AppResult result = SDL_APP_CONTINUE;
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);

    switch (event->type) {
        case SDL_EVENT_QUIT: result = SDL_APP_SUCCESS; break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: cancelLaunch(app); break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (event->button.button != SDL_BUTTON_LEFT) break;
            {
                const SDL_FPoint p{event->button.x, event->button.y};
                for (std::size_t i = 0; i < app.chipCount; ++i) {
                    if (SDL_PointInRectFloat(&p, &app.chips[i].rect)) {
                        apply(app, app.chips[i].act, app.chips[i].value);
                        return SDL_APP_CONTINUE;
                    }
                }
                if (inViewport(p.x, p.y)) {
                    app.launch = Launch{true, p.x, p.y, p.x, p.y};
                    computePreview(app);
                }
            }
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.launch.active) {
                app.launch.curX = event->motion.x;
                app.launch.curY = event->motion.y;
                computePreview(app);
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
            // Released anywhere, also outside the picture: the body is placed, the launch never hangs.
            if (event->button.button == SDL_BUTTON_LEFT && app.launch.active) {
                app.launch.curX = event->button.x;
                app.launch.curY = event->button.y;
                spawnBody(app, toWorld(app, app.launch.startX, app.launch.startY), launchVelocity(app));
                cancelLaunch(app);
            }
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.key == SDLK_ESCAPE) {
                result = SDL_APP_SUCCESS;
            } else if (!event->key.repeat) {
                switch (event->key.key) {
                    case SDLK_SPACE: apply(app, Act::Pause, 0); break;
                    case SDLK_R: apply(app, Act::Reset, 0); break;
                    case SDLK_T: apply(app, Act::Trails, 0); break;
                    case SDLK_S: apply(app, Act::Softening, app.softening ? 0 : 1); break;
                    case SDLK_M: apply(app, Act::Method, (static_cast<int>(app.method) + 1) % 3); break;
                    case SDLK_1: apply(app, Act::Scene, 0); break;
                    case SDLK_2: apply(app, Act::Scene, 1); break;
                    case SDLK_3: apply(app, Act::Scene, 2); break;
                    case SDLK_4: apply(app, Act::Scene, 3); break;
                    default: break;
                }
            }
            break;
        default: break;
    }
    return result;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    double frameTime = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    frameTime = std::min(frameTime, kMaxFrameTime);

    app.wallClock = std::fmod(app.wallClock + frameTime, kTwoPi);
    app.noticeLeft = std::max(0.0, app.noticeLeft - frameTime);
    if (!app.paused) advance(app, frameTime * nb::info(app.scene).timeScale);
    render(app);

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
