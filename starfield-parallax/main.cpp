// A starfield with depth, SDL3 main callbacks.
//
// Two ways to fake a flight through stars. The layered one (naive) keeps every star as a 2D point on one of three
// layers, each moving away from the centre at its own pixel speed. It reads as three sheets sliding over each other:
// near the centre and at the edge a star of one layer moves equally fast, and nothing speeds up as it comes closer.
// The perspective one keeps stars as 3D points, projects x / z, and moves the camera. Speed on the screen, size,
// brightness and the streaks all fall out of the same division, and steering moves the vanishing point.
//
// The world is a periodic box, a star that passes the camera comes back at the far end with fresh x and y. The
// bug chips show the two classic mistakes: respawning at a random depth (stars appear in mid-air) and dividing by z
// without a near test (stars flash across the screen). Everything that can be wrong is in logic.hpp (no SDL,
// covered by test.cpp), this file owns input, the loop and the picture.

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

// The stage is a dark window onto space, drawn inside the usual paper frame.
constexpr float kStageX = 24.0F;
constexpr float kStageY = 44.0F;
constexpr float kStageW = 912.0F;
constexpr float kStageH = 372.0F;
constexpr sf::Vec2 kCentre{kStageX + kStageW / 2.0F, kStageY + kStageH / 2.0F};
constexpr sf::Vec2 kHalf{kStageW / 2.0F, kStageH / 2.0F};

constexpr sf::Lens kLens{330.0, 0.04, 1.0};
constexpr sf::Box kBox{1.5, 0.62, 1.0};  // the far end of the view is 456 / 330 = 1.38 wide, 186 / 330 = 0.56 high
constexpr std::size_t kStarCount = 5200;
constexpr std::size_t kFlatCount = 800;
constexpr double kFlatDepth[sf::kFlatLayerCount] = {0.85, 0.55, 0.25};  // only for the top view of the layers

constexpr std::array<double, 3> kSpeeds{0.12, 0.35, 1.0};  // box depths per second
constexpr double kCruise = 0.35;
constexpr double kMinSpeed = 0.04;
constexpr double kMaxSpeed = 2.0;
constexpr double kMaxFrameTime = 0.25;  // clamp: no burst after a stalled tab
constexpr double kExposure = 0.08;      // seconds of flight a streak shows
constexpr double kSteerLambda = 2.5;    // how fast the vanishing point follows its target (1 / s)
constexpr double kMouseReach = 0.8;     // the vanishing point goes at most this far toward the edge
constexpr sf::Vec2 kAutoAmplitude{160.0, 55.0};
constexpr double kPopLife = 0.45;       // seconds a pop-in ring stays
constexpr double kPopMaxDepth = 0.5;    // only nearer stars get a ring, the far ones are dots anyway
constexpr std::size_t kMaxPops = 160;
constexpr float kClip = 6000.0F;        // renderer sanity for the near-test bug, which sends points to the moon

constexpr gd::Color kSky0{9, 13, 31};
constexpr gd::Color kSky1{22, 30, 62};
constexpr std::array<gd::Color, sf::kTints> kTint{{{196, 218, 255}, {255, 252, 244}, {255, 236, 188}, {255, 204, 150}, {255, 168, 140}}};

constexpr float kChipH = 32.0F;
constexpr float kRowOneY = 450.0F;
constexpr float kRowTwoY = 490.0F;

constexpr float kViewX = 684.0F;  // the top view inset
constexpr float kViewY = 292.0F;
constexpr float kViewW = 240.0F;
constexpr float kViewH = 112.0F;

enum class Action { Model, Speed, Streaks, Steer, TopView, PopIn, SkipNear, Restart };

struct Chip {
    Action action;
    int value;
    std::string label;
    SDL_FRect rect;
};

struct SkyStar {
    float x, y, alpha, phase;
};

/// A star that just appeared in the middle of the view (only the pop-in bug causes this), shown as a fading ring.
struct Pop {
    sf::Vec2 at;
    double age;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<SkyStar> sky;
    std::vector<Pop> pops;

    sf::Rng rng{20271126};
    std::vector<sf::Star> stars;
    std::vector<sf::FlatStar> flat;

    bool perspective = true;
    double speed = kSpeeds[1];
    bool streaks = true;
    bool mouseSteer = false;  // false: autopilot, true: the vanishing point follows the mouse
    bool topView = true;
    bool popIn = false;       // the bug: respawn at a random depth
    bool skipNear = false;    // the bug: no near test before the divide
    sf::Vec2 mouseTarget = kCentre;
    sf::Vec2 mouse{-1.0, -1.0};
    sf::Vec2 foe = kCentre;   // the vanishing point on the screen, eased toward its target

    double clock = 0.0;
    int visible = 0;
    sf::Vec3 velocity{};

    std::vector<SDL_Vertex> verts;
    std::vector<int> idx;

    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

// ------------------------------------------------------------------ state

bool inStage(sf::Vec2 p) {
    return p.x >= static_cast<double>(kStageX) && p.x <= static_cast<double>(kStageX + kStageW) && p.y >= static_cast<double>(kStageY) &&
           p.y <= static_cast<double>(kStageY + kStageH);
}

sf::Vec2 steerTarget(const App& app) {
    if (app.mouseSteer) return sf::clampHeading(app.mouseTarget, kCentre, kHalf, kMouseReach);
    const sf::Vec2 a = sf::autopilot(app.clock, kAutoAmplitude);
    return {kCentre.x + a.x, kCentre.y + a.y};
}

void restart(App& app) {
    app.rng = sf::Rng(20271126 + static_cast<std::uint64_t>(app.frames));
    app.stars = sf::makeStars(kStarCount, kBox, app.rng);
    app.flat = sf::makeFlatStars(kFlatCount, kHalf, app.rng);
    app.foe = steerTarget(app);
    app.pops.clear();
}

void apply(App& app, Action action, int value) {
    switch (action) {
        case Action::Model: app.perspective = value == 1; break;
        case Action::Speed: app.speed = kSpeeds[static_cast<std::size_t>(value)]; break;
        case Action::Streaks: app.streaks = !app.streaks; break;
        case Action::Steer: app.mouseSteer = value == 1; break;
        case Action::TopView: app.topView = !app.topView; break;
        case Action::PopIn:
            if (app.perspective) app.popIn = !app.popIn;
            break;
        case Action::SkipNear:
            if (app.perspective) app.skipNear = !app.skipNear;
            break;
        case Action::Restart: restart(app); break;
    }
}

bool active(const App& app, Action action, int value) {
    switch (action) {
        case Action::Model: return app.perspective == (value == 1);
        case Action::Speed: return app.speed == kSpeeds[static_cast<std::size_t>(value)];
        case Action::Streaks: return app.streaks;
        case Action::Steer: return app.mouseSteer == (value == 1);
        case Action::TopView: return app.topView;
        case Action::PopIn: return app.popIn && app.perspective;
        case Action::SkipNear: return app.skipNear && app.perspective;
        case Action::Restart: return false;
    }
    return false;
}

bool enabled(const App& app, Action action) {
    return (action != Action::PopIn && action != Action::SkipNear) || app.perspective;
}

void update(App& app, double rawDt) {
    const double dt = std::min(rawDt, kMaxFrameTime);
    app.clock += dt;
    const sf::Vec2 target = steerTarget(app);
    app.foe = {sf::approach(app.foe.x, target.x, kSteerLambda, dt), sf::approach(app.foe.y, target.y, kSteerLambda, dt)};
    app.velocity = sf::velocityToward(app.foe, kCentre, kLens.focal, app.speed);

    const sf::Respawn mode = app.popIn ? sf::Respawn::Anywhere : sf::Respawn::Far;
    for (Pop& p : app.pops) p.age += dt;
    app.pops.erase(std::remove_if(app.pops.begin(), app.pops.end(), [](const Pop& p) { return p.age > kPopLife; }), app.pops.end());
    for (sf::Star& s : app.stars) {
        const int laps = sf::advance(s, app.velocity, dt, kBox, mode, app.rng);
        if (laps > 0 && app.popIn && app.pops.size() < kMaxPops && s.p.z < kPopMaxDepth && sf::inFrustum(s.p, kHalf, kLens)) {
            if (const auto at = sf::project(s.p, kCentre, kLens)) app.pops.push_back({*at, 0.0});
        }
    }

    // The layered stars run on pixel speeds scaled with the speed setting, steering shifts whole layers.
    const sf::Vec2 steer{app.foe.x - kCentre.x, app.foe.y - kCentre.y};
    for (sf::FlatStar& f : app.flat) sf::advanceFlat(f, dt * app.speed / kCruise, kCentre, steer, kHalf, app.rng);
}

// ------------------------------------------------------------------ batched geometry

float clampPx(double v) { return static_cast<float>(std::clamp(v, -static_cast<double>(kClip), static_cast<double>(kClip))); }

SDL_FPoint pt(sf::Vec2 p) { return {clampPx(p.x), clampPx(p.y)}; }

void addVertex(App& app, SDL_FPoint p, SDL_FColor c) { app.verts.push_back({p, c, {0.0F, 0.0F}}); }

void addTriangle(App& app, int a, int b, int c) { app.idx.insert(app.idx.end(), {a, b, c}); }

void addQuad(App& app, sf::Vec2 c, float half, SDL_FColor col) {
    const auto base = static_cast<int>(app.verts.size());
    const auto x = static_cast<float>(c.x);
    const auto y = static_cast<float>(c.y);
    addVertex(app, {x - half, y - half}, col);
    addVertex(app, {x + half, y - half}, col);
    addVertex(app, {x + half, y + half}, col);
    addVertex(app, {x - half, y + half}, col);
    addTriangle(app, base, base + 1, base + 2);
    addTriangle(app, base, base + 2, base + 3);
}

/// A soft round glow: bright in the middle, nothing at the rim.
void addGlow(App& app, sf::Vec2 c, float radius, SDL_FColor col) {
    constexpr int kSegments = 12;
    const auto base = static_cast<int>(app.verts.size());
    SDL_FColor rim = col;
    rim.a = 0.0F;
    addVertex(app, pt(c), col);
    for (int i = 0; i <= kSegments; ++i) {
        const float a = 6.2831853F * static_cast<float>(i) / static_cast<float>(kSegments);
        addVertex(app, {static_cast<float>(c.x) + std::cos(a) * radius, static_cast<float>(c.y) + std::sin(a) * radius}, rim);
        if (i > 0) addTriangle(app, base, base + i, base + i + 1);
    }
}

/// A comet: full colour at the head, fading to nothing at the tail, as wide as the star.
void addStreak(App& app, sf::Vec2 head, sf::Vec2 tail, float halfWidth, SDL_FColor col, bool fadeTail = true) {
    const double dx = head.x - tail.x;
    const double dy = head.y - tail.y;
    const double len = std::hypot(dx, dy);
    if (len < 0.75) {
        addQuad(app, head, halfWidth, col);
        return;
    }
    const auto nx = static_cast<float>(-dy / len) * halfWidth;
    const auto ny = static_cast<float>(dx / len) * halfWidth;
    const SDL_FPoint h = pt(head);
    SDL_FColor none = col;
    if (fadeTail) none.a = 0.0F;
    const auto base = static_cast<int>(app.verts.size());
    addVertex(app, {h.x + nx, h.y + ny}, col);
    addVertex(app, {h.x - nx, h.y - ny}, col);
    addVertex(app, pt(tail), none);
    addTriangle(app, base, base + 1, base + 2);
    addQuad(app, head, halfWidth, col);
}

void flush(App& app) {
    if (!app.verts.empty()) {
        SDL_RenderGeometry(app.renderer, nullptr, app.verts.data(), static_cast<int>(app.verts.size()), app.idx.data(), static_cast<int>(app.idx.size()));
    }
    app.verts.clear();
    app.idx.clear();
}

SDL_FColor starColor(int tint, float alpha) { return gd::fcolor(kTint[static_cast<std::size_t>(tint)], std::clamp(alpha, 0.0F, 1.0F)); }

// ------------------------------------------------------------------ the scene

std::vector<SkyStar> makeSky() {
    std::vector<SkyStar> out;
    sf::Rng rng(77);
    for (int i = 0; i < 170; ++i) {
        out.push_back({static_cast<float>(rng.range(kStageX + 3.0, kStageX + kStageW - 3.0)), static_cast<float>(rng.range(kStageY + 3.0, kStageY + kStageH - 3.0)),
                       static_cast<float>(rng.range(0.12, 0.4)), static_cast<float>(rng.range(0.0, 6.28))});
    }
    return out;
}

/// A sphere lit from the upper left: the fan's hub sits off centre, so the colour gradient does too.
void drawPlanet(App& app, float cx, float cy, float radius, gd::Color lit, gd::Color dark) {
    constexpr int kSegments = 40;
    const SDL_FColor hub = gd::fcolor(lit);
    const SDL_FColor rim = gd::fcolor(dark);
    const auto base = static_cast<int>(app.verts.size());
    addVertex(app, {cx - radius * 0.32F, cy - radius * 0.32F}, hub);
    for (int i = 0; i <= kSegments; ++i) {
        const float a = 6.2831853F * static_cast<float>(i) / static_cast<float>(kSegments);
        addVertex(app, {cx + std::cos(a) * radius, cy + std::sin(a) * radius}, rim);
        if (i > 0) addTriangle(app, base, base + i, base + i + 1);
    }
    flush(app);
}

void drawSky(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kStageX, kStageY, kStageW, kStageH, kSky0, gd::kInk);
    gd::gradient(r, kStageX + 1.0F, kStageY + 1.0F, kStageW - 2.0F, kStageH - 2.0F, kSky0, kSky1);
    gd::disc(r, kStageX + 770.0F, kStageY + 80.0F, 240.0F, gd::Color{70, 60, 160, 26}, gd::Color{70, 60, 160, 0}, 56);
    gd::disc(r, kStageX + 110.0F, kStageY + 290.0F, 220.0F, gd::Color{13, 148, 136, 24}, gd::Color{13, 148, 136, 0}, 56);
    // Stars at infinity do not move when the camera translates, so they stay put and only twinkle.
    for (const SkyStar& s : app.sky) {
        const double tw = 0.7 + 0.3 * std::sin(app.clock * 1.4 + static_cast<double>(s.phase));
        gd::fillRect(r, s.x, s.y, 1.0F, 1.0F, gd::Color{225, 232, 255, static_cast<Uint8>(255.0 * static_cast<double>(s.alpha) * tw)});
    }
    // One planet at infinity: no parallax either, however fast we fly.
    gd::disc(r, kStageX + 112.0F, kStageY + 296.0F, 74.0F, gd::Color{90, 130, 230, 34}, gd::Color{90, 130, 230, 0}, 48);
    drawPlanet(app, kStageX + 112.0F, kStageY + 296.0F, 38.0F, gd::Color{176, 204, 255}, gd::Color{14, 22, 64});
    // The vanishing point glows a little, more the faster we go.
    const float boost = static_cast<float>(std::clamp(app.speed / kMaxSpeed, 0.0, 1.0));
    const auto glow = static_cast<Uint8>(14.0F + 52.0F * boost);
    gd::disc(r, static_cast<float>(app.foe.x), static_cast<float>(app.foe.y), 110.0F + 90.0F * boost, gd::Color{150, 205, 255, glow}, gd::Color{150, 205, 255, 0}, 48);
}

void drawPerspective(App& app) {
    const sf::Vec3 v = app.velocity;
    const bool guard = !app.skipNear;
    int seen = 0;
    for (const sf::Star& s : app.stars) {
        if (guard ? s.p.z <= kLens.near : s.p.z <= 0.0) continue;
        const sf::Vec2 head = guard ? *sf::project(s.p, kCentre, kLens) : sf::projectUnguarded(s.p, kCentre, kLens);
        const double dx = std::fabs(head.x - kCentre.x);
        const double dy = std::fabs(head.y - kCentre.y);
        const bool nearBug = !guard && s.p.z <= kLens.near;
        if (!nearBug && (dx > kHalf.x + 80.0 || dy > kHalf.y + 80.0)) continue;
        if (dx <= kHalf.x && dy <= kHalf.y) ++seen;

        // Without the near test there is no fade at the near end either, the star just keeps growing.
        const double fade = nearBug ? 1.0 : sf::depthFade(s.p.z, kLens);
        const double bright = 0.4 + 0.6 * (1.0 - std::min(s.p.z, 1.0));
        const auto alpha = static_cast<float>(std::clamp(static_cast<double>(s.mag) * fade * bright * 1.3, 0.0, 1.0));
        if (alpha <= 0.01F) continue;
        const double radius = sf::apparentRadius(s.p.z, kLens);
        // The broken ones are drawn solid, thick and orange, so a line that shoots across the window cannot be missed.
        const SDL_FColor col = nearBug ? gd::fcolor(gd::Color{255, 120, 50}) : starColor(s.tint, alpha);

        if (app.streaks) {
            const sf::Vec3 old{s.p.x + v.x * kExposure, s.p.y + v.y * kExposure, s.p.z + v.z * kExposure};
            std::optional<sf::Streak> st;
            if (guard) {
                st = sf::streak(s.p, v, kExposure, kCentre, kLens);
            } else {
                st = sf::Streak{head, sf::projectUnguarded(old, kCentre, kLens)};
            }
            if (st) addStreak(app, st->head, st->tail, nearBug ? 2.2F : static_cast<float>(radius) * 0.8F, col, !nearBug);
        } else {
            addQuad(app, head, std::max(0.5F, static_cast<float>(radius) * 0.8F), col);
        }
        if (radius > 1.8) addGlow(app, head, static_cast<float>(radius) * 3.4F, starColor(s.tint, alpha * 0.32F));
    }
    app.visible = seen;
}

void drawLayers(App& app) {
    const double factor = app.speed / kCruise;
    for (const sf::FlatStar& f : app.flat) {
        const sf::FlatLayer& layer = sf::kFlatLayers[f.layer];
        const sf::Vec2 steer{app.foe.x - kCentre.x, app.foe.y - kCentre.y};
        const sf::Vec2 p = sf::flatPosition(f, kCentre, steer);
        // Fade in near the middle where the star was just reborn.
        const double born = sf::smoothstep(f.radius / 36.0);
        const auto alpha = static_cast<float>(static_cast<double>(layer.brightness) * static_cast<double>(f.mag) * born);
        if (alpha <= 0.01F) continue;
        const SDL_FColor col = starColor(f.tint, alpha);
        const float half = layer.size * 0.6F;
        if (app.streaks) {
            const double len = layer.speed * factor * kExposure;
            const sf::Vec2 tail{p.x - std::cos(f.angle) * len, p.y - std::sin(f.angle) * len};
            addStreak(app, p, tail, half, col);
        } else {
            addQuad(app, p, std::max(0.5F, half), col);
        }
    }
    app.visible = static_cast<int>(app.flat.size());
}

void drawPops(App& app) {
    for (const Pop& p : app.pops) {
        const double t = p.age / kPopLife;
        const auto a = static_cast<Uint8>(230.0 * (1.0 - t));
        gd::ring(app.renderer, static_cast<float>(p.at.x), static_cast<float>(p.at.y), static_cast<float>(5.0 + 16.0 * t), gd::Color{255, 120, 50, a}, 16);
    }
}

void drawTopView(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, kViewX, kViewY, kViewW, kViewH, gd::Color{9, 13, 31, 205});
    gd::strokeRect(r, kViewX, kViewY, kViewW, kViewH, gd::Color{255, 255, 255, 60});
    app.hud.text(r, kViewX + 8.0F, kViewY + 1.0F, "TOP VIEW", gd::kWhite, gd::Hud::Style::Bold);
    app.hud.text(r, kViewX + kViewW - 8.0F - app.hud.width("x / z"), kViewY + 1.0F, "x / z", gd::kMuted);

    const float scale = kViewW / static_cast<float>(2.0 * kBox.halfX);
    const float camX = kViewX + kViewW / 2.0F;
    const float camY = kViewY + kViewH - 8.0F;
    // The window's pyramid: its edges from the camera to the far plane.
    const float reach = static_cast<float>(kHalf.x / kLens.focal * kLens.far) * scale;
    const float top = camY - static_cast<float>(kLens.far) * scale;
    gd::setColor(r, gd::Color{255, 255, 255, 80});
    SDL_RenderLine(r, camX, camY, camX - reach, top);
    SDL_RenderLine(r, camX, camY, camX + reach, top);
    SDL_RenderLine(r, camX - reach, top, camX + reach, top);

    if (app.perspective) {
        for (const sf::Star& s : app.stars) {
            const bool in = sf::inFrustum(s.p, kHalf, kLens);
            const SDL_FColor col = in ? starColor(s.tint, 0.95F) : gd::fcolor(gd::Color{160, 170, 200}, 0.2F);
            addQuad(app, {camX + s.p.x * static_cast<double>(scale), camY - s.p.z * static_cast<double>(scale)}, in ? 0.9F : 0.6F, col);
        }
    } else {
        // The layers drawn as what they are: flat sheets, each at one depth.
        for (const sf::FlatStar& f : app.flat) {
            const sf::Vec2 p = sf::flatPosition(f, kCentre, {0.0, 0.0});
            const double depth = kFlatDepth[f.layer];
            addQuad(app, {camX + (p.x - kCentre.x) / kLens.focal * depth * static_cast<double>(scale), camY - depth * static_cast<double>(scale)}, 0.9F, starColor(f.tint, 0.95F));
        }
    }
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_ADD);
    flush(app);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);

    // The flight direction.
    const double norm = std::hypot(app.velocity.x, app.velocity.z);
    if (norm > 1e-9) {
        const float ax = camX + static_cast<float>(app.velocity.x / norm) * 34.0F;
        const float ay = camY - static_cast<float>(app.velocity.z / norm) * 34.0F;
        gd::setColor(r, gd::kOrange);
        SDL_RenderLine(r, camX, camY, ax, ay);
        SDL_RenderLine(r, camX + 1.0F, camY, ax + 1.0F, ay);
        gd::disc(r, ax, ay, 3.0F, gd::kOrange, 12);
    }
    gd::fillRect(r, camX - 3.0F, camY - 3.0F, 7.0F, 7.0F, gd::kWhite);
}

void drawReadout(App& app) {
    SDL_Renderer* r = app.renderer;
    std::array<std::string, 8> lines;
    std::array<gd::Color, 8> colors{};
    int n = 0;
    char buf[64];
    auto add = [&](const char* text, gd::Color c) {
        lines[static_cast<std::size_t>(n)] = text;
        colors[static_cast<std::size_t>(n)] = c;
        ++n;
    };
    if (app.perspective) {
        add("PERSPECTIVE", gd::kTeal);
        add("3D points, x / z", gd::kMuted);
        SDL_snprintf(buf, sizeof buf, "stars %d  seen %d", static_cast<int>(app.stars.size()), app.visible);
        add(buf, gd::kWhite);
    } else {
        add("LAYERS (NAIVE)", gd::kOrange);
        add("no depth, 3 speeds", gd::kMuted);
        SDL_snprintf(buf, sizeof buf, "stars %d in 3 layers", static_cast<int>(app.flat.size()));
        add(buf, gd::kWhite);
    }
    SDL_snprintf(buf, sizeof buf, "speed %.2f /s", app.speed);
    add(buf, gd::kWhite);
    if (app.perspective) {
        const double h = app.velocity.z > 1e-9 ? app.velocity.x / app.velocity.z : 0.0;
        const double w = app.velocity.z > 1e-9 ? app.velocity.y / app.velocity.z : 0.0;
        SDL_snprintf(buf, sizeof buf, "heading %+.2f %+.2f", h, w);
        add(buf, gd::kMuted);
    } else {
        const double f = app.speed / kCruise;
        SDL_snprintf(buf, sizeof buf, "%.0f/%.0f/%.0f px/s", sf::kFlatLayers[0].speed * f, sf::kFlatLayers[1].speed * f, sf::kFlatLayers[2].speed * f);
        add(buf, gd::kMuted);
    }
    if (app.perspective && app.popIn) add("BUG: pop-in (rings)", gd::Color{255, 140, 80});
    if (app.perspective && app.skipNear) add("BUG: no near test", gd::Color{255, 140, 80});

    const float x = kStageX + 12.0F;
    const float y = kStageY + 12.0F;
    const float h = 14.0F + 22.0F * static_cast<float>(n);
    gd::fillRect(r, x, y, 238.0F, h, gd::Color{9, 13, 31, 190});
    gd::strokeRect(r, x, y, 238.0F, h, gd::Color{255, 255, 255, 60});
    for (int i = 0; i < n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        app.hud.text(r, x + 10.0F, y + 7.0F + 22.0F * static_cast<float>(i), lines[k], colors[k], i == 0 ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
    }
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, 24.0F, kRowTwoY + 5.0F, "Steer", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action, c.value);
        const bool ok = enabled(app, c.action);
        const bool bug = c.action == Action::PopIn || c.action == Action::SkipNear || (c.action == Action::Model && c.value == 0);
        const float tx = c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F;
        if (!ok) {
            gd::fillRect(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, gd::kPaper);
            gd::strokeRect(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, gd::kGrid);
            app.hud.text(r, tx, c.rect.y + 6.0F, c.label, gd::kMuted, gd::Hud::Style::Bold);
            continue;
        }
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? (bug ? gd::kOrange : gd::kTeal) : gd::kWhite, gd::kInk);
        app.hud.text(r, tx, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ STARFIELD PARALLAX ]", app.perspective ? "3D points, perspective divide" : "three layers, fixed speeds");
    drawSky(app);

    const SDL_Rect clip{static_cast<int>(kStageX) + 1, static_cast<int>(kStageY) + 1, static_cast<int>(kStageW) - 2, static_cast<int>(kStageH) - 2};
    SDL_SetRenderClipRect(r, &clip);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_ADD);
    if (app.perspective) {
        drawPerspective(app);
    } else {
        drawLayers(app);
    }
    flush(app);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    drawPops(app);
    SDL_SetRenderClipRect(r, nullptr);

    drawReadout(app);
    if (app.topView) drawTopView(app);

    app.hud.text(r, 24.0F, 420.0F, "mouse: steer   wheel: speed   keys: 1 to 5, S, M, A, T, P, Z, R", gd::kMuted);
    drawChips(app);
}

void layoutChips(App& app) {
    float x = 24.0F;
    auto add = [&](Action action, int value, const char* label, float gapBefore) {
        x += gapBefore;
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
        const float y = (action == Action::Steer || action == Action::TopView || action == Action::PopIn || action == Action::SkipNear || action == Action::Restart) ? kRowTwoY : kRowOneY;
        app.chips.push_back({action, value, label, SDL_FRect{x, y, w, kChipH}});
        x += w;
    };
    add(Action::Model, 0, "1 Layers (naive)", 0.0F);
    add(Action::Model, 1, "2 Perspective", 8.0F);
    add(Action::Speed, 0, "3 Slow", 24.0F);
    add(Action::Speed, 1, "4 Cruise", 8.0F);
    add(Action::Speed, 2, "5 Warp", 8.0F);
    add(Action::Streaks, 0, "S Streaks", 24.0F);
    x = 80.0F;
    add(Action::Steer, 1, "M Mouse", 0.0F);
    add(Action::Steer, 0, "A Auto", 8.0F);
    add(Action::TopView, 0, "T Top view", 20.0F);
    add(Action::PopIn, 0, "P Pop-in respawn", 20.0F);
    add(Action::SkipNear, 0, "Z Skip near test", 8.0F);
    add(Action::Restart, 0, "R Restart", 20.0F);
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
    if (!SDL_CreateWindowAndRenderer("starfield-parallax", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->sky = makeSky();
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
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            // The first move over the stage hands the steering to the mouse. It keeps the last point when the
            // pointer leaves, so the ship does not swerve back.
            if (inStage(app.mouse)) {
                app.mouseTarget = app.mouse;
                app.mouseSteer = true;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action, c.value);
                    return SDL_APP_CONTINUE;
                }
            }
            break;
        }
        case SDL_EVENT_MOUSE_WHEEL:
            if (inStage(app.mouse)) app.speed = std::clamp(app.speed * std::pow(1.15, static_cast<double>(event->wheel.y)), kMinSpeed, kMaxSpeed);
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Model, 0); break;
                case SDLK_2: apply(app, Action::Model, 1); break;
                case SDLK_3: apply(app, Action::Speed, 0); break;
                case SDLK_4: apply(app, Action::Speed, 1); break;
                case SDLK_5: apply(app, Action::Speed, 2); break;
                case SDLK_S: apply(app, Action::Streaks, 0); break;
                case SDLK_M: apply(app, Action::Steer, 1); break;
                case SDLK_A: apply(app, Action::Steer, 0); break;
                case SDLK_T: apply(app, Action::TopView, 0); break;
                case SDLK_P: apply(app, Action::PopIn, 0); break;
                case SDLK_Z: apply(app, Action::SkipNear, 0); break;
                case SDLK_R: apply(app, Action::Restart, 0); break;
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
    update(app, dt);
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
