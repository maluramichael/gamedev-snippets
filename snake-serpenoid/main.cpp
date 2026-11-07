// A snake from one sine wave, SDL3 main callbacks.
//
// Twelve links, eleven joints, every joint driven by the same sine wave a little later than its neighbour
// (Hirose's serpenoid curve). Nothing steers the body and nothing aims it: the joint angles fix the shape, the ground
// decides where the body goes. logic.hpp models the ground as friction at the middle of every link, with a different
// strength across the link than along it (the chips). With the same friction everywhere ("Ice") the snake wiggles in
// place, with a strong side grip ("Wheels") the wave turns into forward motion. The orange bars are the sideways speed
// of each link, the thing the wheels forbid.
//
// The sliders are the three parameters of the source: a (bend), b (waves on the body) and c (steering bias). They glide
// to their new value instead of jumping, otherwise the body would pop into a new shape. The physics runs in fixed
// steps of 1/120 s, the stride readout is measured over exactly one wave period of simulated time and starts over
// whenever something it depends on changes.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kScale = 26.0F;  // pixels per link length
constexpr float kCenterX = 500.0F;
constexpr float kCenterY = 238.0F;
constexpr float kFieldTop = 34.0F;
constexpr float kFieldBottom = 436.0F;

constexpr double kStep = 1.0 / 120.0;
constexpr double kMaxFrameTime = 0.25;
constexpr int kMaxStepsPerFrame = 12;
constexpr double kOmega = serp::kPi;  // one wave period = 2 s
constexpr double kWindow = serp::kTwoPi / kOmega;
constexpr int kWarmupSteps = 600;  // 5 s of crawling before the first frame, so the trail and the stride are already there
constexpr double kGlide = 9.0;     // 1 / s, how fast the sliders' effect follows them
constexpr std::size_t kTrailMax = 240;
constexpr int kTrailEvery = 3;     // steps between trail points
constexpr float kSlipScale = 12.0F;  // pixels per (link length / s) of sideways speed

constexpr gd::Color kInkSoft{17, 24, 39, 120};
constexpr gd::Color kTrail{107, 114, 128, 150};
constexpr gd::Color kPebble{168, 162, 158, 255};

enum class Ground { Ice, Scales, Wheels };
constexpr std::array<double, 3> kRatios{1.0, 10.0, 1000.0};

enum class Action { Ice, Scales, Wheels, Pause, Reverse, Slip, Reset };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct SliderDef {
    const char* label;
    double lo;
    double hi;
    double initial;
};

constexpr std::array<SliderDef, 3> kSliders{{{"a  bend", 0.0, 1.2, 0.6}, {"b  waves", 0.5, 2.0, 1.0}, {"c  steer (arrows)", -2.0, 2.0, 0.0}}};
constexpr float kSliderW = 250.0F;
constexpr std::array<float, 3> kSliderX{40.0F, 355.0F, 670.0F};
constexpr float kSliderLabelY = 442.0F;
constexpr float kSliderTrackY = 478.0F;
constexpr float kChipY = 496.0F;
constexpr float kChipH = 32.0F;

/// What the stride readout depends on. When it changes, the old measurement is not about this setup any more.
struct Setup {
    std::array<double, 3> target{};
    Ground ground = Ground::Wheels;
    bool reversed = false;
    bool operator==(const Setup&) const = default;
};

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

    std::array<double, 3> target{kSliders[0].initial, kSliders[1].initial, kSliders[2].initial};  // what the sliders say
    std::array<double, 3> applied = target;                                                       // what the body follows
    Ground ground = Ground::Wheels;
    bool paused = false;
    bool reversed = false;
    bool showSlip = true;
    int dragSlider = -1;

    serp::State state;
    serp::SpeedMeter meter{kWindow};
    std::deque<serp::Vec2> trail;
    int trailCounter = 0;
    Setup lastSetup;

    double accumulator = 0.0;
    double lastMs = 0.0;
    SDL_FPoint mouse{-100.0F, -100.0F};

    long maxFrames = -1;
    long frames = 0;
};

/// -0.0 would print as "-0.00".
double noNegativeZero(double v) { return v == 0.0 ? 0.0 : v; }

double ratioOf(const App& app) { return kRatios[static_cast<std::size_t>(app.ground)]; }
double omegaOf(const App& app) { return app.reversed ? -kOmega : kOmega; }

serp::Gait gaitOf(const std::array<double, 3>& p) { return serp::gaitFrom(p[0], serp::kTwoPi * p[1], p[2]); }

Setup setupOf(const App& app) { return Setup{app.target, app.ground, app.reversed}; }

bool settled(const App& app) {
    for (std::size_t i = 0; i < 3; ++i) {
        if (std::fabs(app.applied[i] - app.target[i]) > 1e-3) return false;
    }
    return true;
}

void stepOnce(App& app) {
    // Frame-rate independent glide: the same fraction of the remaining distance per second whatever the step.
    const double k = 1.0 - std::exp(-kGlide * kStep);
    for (std::size_t i = 0; i < 3; ++i) {
        app.applied[i] += (app.target[i] - app.applied[i]) * k;
        if (std::fabs(app.target[i] - app.applied[i]) < 1e-4) app.applied[i] = app.target[i];
    }
    serp::advance(app.state, gaitOf(app.applied), omegaOf(app), ratioOf(app), kStep);
    const serp::Shape shape = serp::shapeAt(gaitOf(app.applied), app.state.phase, omegaOf(app));
    const serp::Vec2 com = serp::centerOfMass(shape, app.state.pose);
    if (!settled(app)) {
        app.meter.reset();  // the body is still changing shape, a stride measured now belongs to neither setup
    } else {
        app.meter.push(app.state.time, com);
    }
    if (++app.trailCounter >= kTrailEvery) {
        app.trailCounter = 0;
        app.trail.push_back(serp::toWorld(app.state.pose, shape.center[serp::kLinks - 1]));
        if (app.trail.size() > kTrailMax) app.trail.pop_front();
    }
}

/// Fresh snake heading right, then a few seconds of crawling so the first frame already shows the trail and a stride.
void restart(App& app) {
    app.applied = app.target;
    app.state = serp::State{};
    const serp::Shape s0 = serp::shapeAt(gaitOf(app.applied), 0.0, omegaOf(app));
    app.state.pose = serp::poseFor(s0, {0.0, 0.0}, 0.0);
    app.meter.reset();
    app.trail.clear();
    app.trailCounter = 0;
    for (int i = 0; i < kWarmupSteps; ++i) stepOnce(app);
    app.accumulator = 0.0;
    app.lastSetup = setupOf(app);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::Ice, "1 Ice", 0.0F},
                                   {Action::Scales, "2 Scales", 8.0F},
                                   {Action::Wheels, "3 Wheels", 8.0F},
                                   {Action::Pause, "Space Pause", 22.0F},
                                   {Action::Reverse, "W Reverse", 8.0F},
                                   {Action::Slip, "S Slip", 8.0F},
                                   {Action::Reset, "R Restart", 8.0F}}};
    float x = 104.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 20.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::Ice: return app.ground == Ground::Ice;
        case Action::Scales: return app.ground == Ground::Scales;
        case Action::Wheels: return app.ground == Ground::Wheels;
        case Action::Pause: return app.paused;
        case Action::Reverse: return app.reversed;
        case Action::Slip: return app.showSlip;
        case Action::Reset: return false;
    }
    return false;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::Ice: app.ground = Ground::Ice; break;
        case Action::Scales: app.ground = Ground::Scales; break;
        case Action::Wheels: app.ground = Ground::Wheels; break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Reverse: app.reversed = !app.reversed; break;
        case Action::Slip: app.showSlip = !app.showSlip; break;
        case Action::Reset: restart(app); break;
    }
}

SDL_FRect sliderHit(std::size_t i) { return SDL_FRect{kSliderX[i] - 10.0F, kSliderTrackY - 13.0F, kSliderW + 20.0F, 26.0F}; }

void setSlider(App& app, std::size_t i, float mouseX) {
    const SliderDef& d = kSliders[i];
    const double v = serp::sliderValue(static_cast<double>(mouseX), static_cast<double>(kSliderX[i]), static_cast<double>(kSliderW), d.lo, d.hi);
    app.target[i] = std::round(v * 20.0) / 20.0;  // steps of 0.05
}

SDL_FPoint toScreen(serp::Vec2 camera, serp::Vec2 p) {
    return SDL_FPoint{kCenterX + static_cast<float>(p.x - camera.x) * kScale, kCenterY - static_cast<float>(p.y - camera.y) * kScale};
}

void thickSegment(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, float width, gd::Color c) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-4F) return;
    const float nx = -dy / len * width / 2.0F;
    const float ny = dx / len * width / 2.0F;
    const SDL_FColor col = gd::fcolor(c);
    const SDL_Vertex v[4] = {{{a.x + nx, a.y + ny}, col, {0, 0}}, {{b.x + nx, b.y + ny}, col, {0, 0}}, {{b.x - nx, b.y - ny}, col, {0, 0}}, {{a.x - nx, a.y - ny}, col, {0, 0}}};
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
}

std::uint32_t hash2(std::int32_t x, std::int32_t y) {
    auto h = static_cast<std::uint32_t>(x) * 73856093U ^ static_cast<std::uint32_t>(y) * 19349663U;
    h ^= h >> 13;
    h *= 0x5bd1e995U;
    return h ^ (h >> 15);
}

/// The ground scrolls with the camera: the same 30 px grid as the other demos plus a few pebbles, so the motion is visible.
void drawGround(App& app, serp::Vec2 camera) {
    SDL_Renderer* r = app.renderer;
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    gd::setColor(r, gd::kPaper);
    SDL_RenderClear(r);
    const float ox = std::fmod(kCenterX - static_cast<float>(camera.x) * kScale, 30.0F);
    const float oy = std::fmod(kCenterY + static_cast<float>(camera.y) * kScale, 30.0F);
    gd::setColor(r, gd::kGrid);
    for (float x = ox - 30.0F; x < static_cast<float>(kWidth); x += 30.0F) SDL_RenderLine(r, x, 0.0F, x, static_cast<float>(kHeight));
    for (float y = oy - 30.0F; y < static_cast<float>(kHeight); y += 30.0F) SDL_RenderLine(r, 0.0F, y, static_cast<float>(kWidth), y);

    constexpr double kCell = 3.0;  // world units per pebble cell
    const double halfW = kWidth / 2.0 / static_cast<double>(kScale) + kCell;
    const double halfH = kHeight / 2.0 / static_cast<double>(kScale) + kCell;
    const auto ix0 = static_cast<std::int32_t>(std::floor((camera.x - halfW) / kCell));
    const auto ix1 = static_cast<std::int32_t>(std::floor((camera.x + halfW) / kCell));
    const auto iy0 = static_cast<std::int32_t>(std::floor((camera.y - halfH) / kCell));
    const auto iy1 = static_cast<std::int32_t>(std::floor((camera.y + halfH) / kCell));
    for (std::int32_t ix = ix0; ix <= ix1; ++ix) {
        for (std::int32_t iy = iy0; iy <= iy1; ++iy) {
            const std::uint32_t h = hash2(ix, iy);
            if (h % 5U != 0U) continue;
            const double wx = (static_cast<double>(ix) + static_cast<double>(h >> 8 & 255U) / 256.0) * kCell;
            const double wy = (static_cast<double>(iy) + static_cast<double>(h >> 16 & 255U) / 256.0) * kCell;
            const SDL_FPoint p = toScreen(camera, {wx, wy});
            if (p.y < kFieldTop || p.y > kFieldBottom) continue;
            const float size = 3.0F + static_cast<float>(h >> 24 & 3U);
            gd::fillRect(r, p.x, p.y, size * 1.6F, size, kPebble);
        }
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const serp::Gait applied = gaitOf(app.applied);
    const double omega = omegaOf(app);
    const double ratio = ratioOf(app);
    const serp::Shape shape = serp::shapeAt(applied, app.state.phase, omega);
    const serp::Twist twist = serp::solveTwist(shape, app.state.pose, ratio);
    const serp::Vec2 camera = serp::centerOfMass(shape, app.state.pose);

    drawGround(app, camera);

    // Where the head has been.
    for (std::size_t i = 0; i < app.trail.size(); ++i) {
        const SDL_FPoint p = toScreen(camera, app.trail[i]);
        if (p.y < kFieldTop || p.y > kFieldBottom) continue;
        const float t = static_cast<float>(i + 1) / static_cast<float>(app.trail.size());
        gd::Color c = kTrail;
        c.a = static_cast<Uint8>(150.0F * t);
        gd::fillRect(r, p.x - 2.0F, p.y - 2.0F, 4.0F, 4.0F, c);
    }

    std::array<SDL_FPoint, serp::kLinks + 1> joint{};
    for (std::size_t k = 0; k <= static_cast<std::size_t>(serp::kLinks); ++k) joint[k] = toScreen(camera, serp::toWorld(app.state.pose, shape.joint[k]));
    std::array<SDL_FPoint, serp::kLinks> mid{};
    std::array<float, serp::kLinks> linkAngle{};
    for (std::size_t k = 0; k < static_cast<std::size_t>(serp::kLinks); ++k) {
        mid[k] = toScreen(camera, serp::toWorld(app.state.pose, shape.center[k]));
        // Screen y points down, so the world angle flips its sign.
        linkAngle[k] = -static_cast<float>(app.state.pose.theta + shape.angle[k]);
    }

    // Body: ink outline first, teal fill on top, so the joints blend into one tube.
    for (std::size_t k = 0; k < static_cast<std::size_t>(serp::kLinks); ++k) thickSegment(r, joint[k], joint[k + 1], 18.0F, gd::kInk);
    for (std::size_t k = 0; k <= static_cast<std::size_t>(serp::kLinks); ++k) gd::disc(r, joint[k].x, joint[k].y, 9.0F, gd::kInk, 16);
    for (std::size_t k = 0; k < static_cast<std::size_t>(serp::kLinks); ++k) {
        thickSegment(r, joint[k], joint[k + 1], 13.0F, (k % 2 == 0) ? gd::kTeal : gd::kTealDark);
    }
    for (std::size_t k = 0; k <= static_cast<std::size_t>(serp::kLinks); ++k) gd::disc(r, joint[k].x, joint[k].y, 6.5F, gd::kTealDark, 16);

    // Head: a bigger disc at the front end with two eyes.
    {
        const SDL_FPoint h = joint[static_cast<std::size_t>(serp::kLinks)];
        const float a = linkAngle[static_cast<std::size_t>(serp::kLinks) - 1];
        gd::disc(r, h.x, h.y, 11.0F, gd::kInk, 20);
        gd::disc(r, h.x, h.y, 8.5F, gd::kTeal, 20);
        for (const float side : {-1.0F, 1.0F}) {
            const float ex = h.x + std::cos(a) * 2.0F - std::sin(a) * side * 4.5F;
            const float ey = h.y + std::sin(a) * 2.0F + std::cos(a) * side * 4.5F;
            gd::disc(r, ex, ey, 2.6F, gd::kWhite, 10);
            gd::disc(r, ex + std::cos(a) * 1.0F, ey + std::sin(a) * 1.0F, 1.2F, gd::kInk, 8);
        }
    }

    // Wheels: a pair at the middle of every link, turning along the link. Only they forbid sliding sideways.
    if (app.ground == Ground::Wheels) {
        for (std::size_t k = 0; k < static_cast<std::size_t>(serp::kLinks); ++k) {
            const float ca = std::cos(linkAngle[k]);
            const float sa = std::sin(linkAngle[k]);
            for (const float side : {-1.0F, 1.0F}) {
                const SDL_FPoint c{mid[k].x - sa * side * 10.0F, mid[k].y + ca * side * 10.0F};
                thickSegment(r, {c.x - ca * 6.0F, c.y - sa * 6.0F}, {c.x + ca * 6.0F, c.y + sa * 6.0F}, 5.0F, gd::kInk);
            }
        }
    } else if (app.ground == Ground::Scales) {
        // Scales: short ridges along the sides, they grip across the body and let it glide along.
        for (std::size_t k = 0; k < static_cast<std::size_t>(serp::kLinks); ++k) {
            const float ca = std::cos(linkAngle[k]);
            const float sa = std::sin(linkAngle[k]);
            for (const float side : {-1.0F, 1.0F}) {
                const SDL_FPoint c{mid[k].x - sa * side * 9.5F, mid[k].y + ca * side * 9.5F};
                thickSegment(r, {c.x - ca * 4.0F, c.y - sa * 4.0F}, {c.x + ca * 4.0F, c.y + sa * 4.0F}, 2.0F, gd::kInk);
            }
        }
    }

    // Sideways speed of every link center: what the ground has to take away.
    if (app.showSlip) {
        for (std::size_t k = 0; k < static_cast<std::size_t>(serp::kLinks); ++k) {
            const double across = serp::slipOf(shape, app.state.pose, twist, static_cast<int>(k)).across;
            const float len = static_cast<float>(across) * kSlipScale;
            if (std::fabs(len) < 1.5F) continue;
            // Left normal of the link in world space is (-sin, cos); on screen y is flipped.
            const float nx = std::sin(linkAngle[k]);
            const float ny = -std::cos(linkAngle[k]);
            const SDL_FPoint tip{mid[k].x + nx * len, mid[k].y + ny * len};
            thickSegment(r, mid[k], tip, 3.0F, gd::kOrange);
            gd::disc(r, tip.x, tip.y, 3.5F, gd::kOrange, 10);
        }
    }

    // Readout.
    const serp::Gait gait = gaitOf(app.target);
    gd::card(r, 24.0F, 44.0F, 400.0F, 104.0F, gd::kWhite, gd::kInk);
    char line[96];
    SDL_snprintf(line, sizeof line, "alpha %.3f  beta %.1f\xC2\xB0  gamma %+.3f", gait.alpha, gait.beta * 180.0 / serp::kPi, noNegativeZero(gait.gamma));
    app.hud.text(r, 36.0F, 52.0F, line, gd::kMuted);
    SDL_snprintf(line, sizeof line, "friction across/along: %.0f : 1", ratio);
    app.hud.text(r, 36.0F, 74.0F, line, gd::kMuted);
    const auto d = app.meter.displacement();
    if (d) {
        const double h = serp::heading(shape, app.state.pose);
        const double along = d->x * std::cos(h) + d->y * std::sin(h);
        const double stride = std::copysign(std::hypot(d->x, d->y), along) / static_cast<double>(serp::kLinks);
        const bool stuck = std::fabs(stride) < 0.05;
        SDL_snprintf(line, sizeof line, "%+.2f", noNegativeZero(std::round(stride * 100.0) / 100.0));
        app.hud.text(r, 36.0F, 98.0F, line, stuck ? gd::kOrange : gd::kTeal, gd::Hud::Style::Large);
        app.hud.text(r, 36.0F + app.hud.width(line, gd::Hud::Style::Large) + 10.0F, 100.0F, "body lengths", gd::kInk, gd::Hud::Style::Bold);
        app.hud.text(r, 36.0F + app.hud.width(line, gd::Hud::Style::Large) + 10.0F, 122.0F, "per wave cycle", gd::kMuted);
    } else {
        app.hud.text(r, 36.0F, 106.0F, "measuring one wave cycle ...", gd::kMuted, gd::Hud::Style::Bold);
    }

    app.hud.titleBar(r, kWidth, "[ SNAKE FROM ONE SINE WAVE ]", "joint_i = 2*alpha*sin(w*t + i*beta) + gamma");

    // Sliders.
    for (std::size_t i = 0; i < kSliders.size(); ++i) {
        const SliderDef& def = kSliders[i];
        const float x = kSliderX[i];
        char label[64];
        SDL_snprintf(label, sizeof label, "%s", def.label);
        app.hud.text(r, x, kSliderLabelY, label, gd::kInk, gd::Hud::Style::Bold);
        char value[32];
        SDL_snprintf(value, sizeof value, i == 2 ? "%+.2f" : "%.2f", noNegativeZero(app.target[i]));
        app.hud.text(r, x + kSliderW - app.hud.width(value, gd::Hud::Style::Bold), kSliderLabelY, value, gd::kTealDark, gd::Hud::Style::Bold);
        const float t = static_cast<float>(serp::sliderFraction(app.target[i], def.lo, def.hi));
        gd::fillRect(r, x, kSliderTrackY - 2.0F, kSliderW, 4.0F, gd::kGrid);
        gd::strokeRect(r, x, kSliderTrackY - 2.0F, kSliderW, 4.0F, gd::kMuted);
        gd::fillRect(r, x, kSliderTrackY - 2.0F, kSliderW * t, 4.0F, gd::kTeal);
        const bool held = app.dragSlider == static_cast<int>(i);
        gd::card(r, x + kSliderW * t - 6.0F, kSliderTrackY - 11.0F, 12.0F, 22.0F, held ? gd::kTeal : gd::kWhite, gd::kInk);
    }
    app.hud.text(r, 40.0F, kChipY + 8.0F, "ground", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("snake-serpenoid", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    restart(*app);
    app->lastMs = nowMs();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = SDL_FPoint{event->motion.x, event->motion.y};
            if (app.dragSlider >= 0) setSlider(app, static_cast<std::size_t>(app.dragSlider), app.mouse.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c.action);
            }
            for (std::size_t i = 0; i < kSliders.size(); ++i) {
                const SDL_FRect hit = sliderHit(i);
                if (SDL_PointInRectFloat(&p, &hit)) {
                    app.dragSlider = static_cast<int>(i);
                    setSlider(app, i, p.x);
                }
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragSlider = -1;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragSlider = -1; break;
        case SDL_EVENT_KEY_DOWN:
            switch (event->key.key) {
                case SDLK_LEFT:  // steering may repeat while held
                    app.target[2] = std::max(kSliders[2].lo, std::round((app.target[2] - 0.1) * 20.0) / 20.0);
                    break;
                case SDLK_RIGHT: app.target[2] = std::min(kSliders[2].hi, std::round((app.target[2] + 0.1) * 20.0) / 20.0); break;
                default: break;
            }
            if (event->key.repeat) break;  // everything below toggles or restarts, none of it should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Ice); break;
                case SDLK_2: apply(app, Action::Scales); break;
                case SDLK_3: apply(app, Action::Wheels); break;
                case SDLK_SPACE: apply(app, Action::Pause); break;
                case SDLK_W: apply(app, Action::Reverse); break;
                case SDLK_S: apply(app, Action::Slip); break;
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

    const double now = nowMs();
    const double frameTime = std::min((now - app.lastMs) / 1000.0, kMaxFrameTime);
    app.lastMs = now;

    // A changed setup starts a new stride measurement.
    const Setup setup = setupOf(app);
    if (!(setup == app.lastSetup)) {
        app.meter.reset();
        app.lastSetup = setup;
    }

    if (!app.paused) {
        app.accumulator += frameTime;
        int steps = 0;
        while (app.accumulator >= kStep && steps < kMaxStepsPerFrame) {
            stepOnce(app);
            app.accumulator -= kStep;
            ++steps;
        }
        if (steps == kMaxStepsPerFrame) app.accumulator = 0.0;  // slow machine: run slower instead of piling up debt
    }

    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app == nullptr) return;
    app->hud.destroy();
    if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
    if (app->window != nullptr) SDL_DestroyWindow(app->window);
    delete app;
}
