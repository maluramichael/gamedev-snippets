// Juice with SDL3 main callbacks: a hero hits a dummy, and seven small reactions can be switched on one by one.
//
// The point is the contrast. With everything off the hit is only a number going down. With everything on, the same
// 0.1 s carries a freeze, a camera kick, a white flash, a squash, sparks, an impact ring and a damage number.
// Two details matter more than the effects themselves: the hit stop pauses GAME time only (the camera, the flash and
// the input keep running on real time, see the timeline), and shaking on routine actions drowns the real hits
// (chip "Overdo"). All rules live in logic.hpp and are covered by test.cpp.
//
// Reduced motion: shake, flash, squash and sparks go off with the "Reduce motion" chip. At most one flash per hit and
// a hit takes at least 0.42 s, so nothing blinks faster than 3 times a second.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kStageTop = 34.0F;
constexpr float kStageBottom = 300.0F;
constexpr float kFloorY = 262.0F;
constexpr float kHeroX = 400.0F;
constexpr float kDummyX = 560.0F;
constexpr float kDummyW = 64.0F;
constexpr float kDummyH = 116.0F;
constexpr double kMaxShakePx = 24.0;
constexpr double kTraumaDecay = 1.6;      // trauma per second
constexpr double kBufferWindow = 0.35;    // s a click waits for the next swing
constexpr double kFlashSeconds = 0.14;
constexpr double kImpactSeconds = 0.14;
constexpr double kNumberSeconds = 0.8;
constexpr double kSparkGravity = 700.0;
constexpr double kTimelineSpan = 2.0;
constexpr double kSlowFactor = 0.25;
constexpr float kTimelineX = 40.0F;
constexpr float kTimelineW = 880.0F;
constexpr float kTimelineY = 348.0F;
constexpr float kChipH = 34.0F;
constexpr int kHpMax = 100;
constexpr gd::Color kWood{205, 176, 128};
constexpr gd::Color kWoodDark{150, 118, 74};
constexpr gd::Color kSpark{250, 176, 5};
constexpr gd::Color kFloor{232, 227, 216};

enum class Act { Stop, Shake, Flash, Squash, Sparks, Impact, Number, None, All, Overdo, Input, Slow, Reduce };

struct Chip {
    Act act;
    std::string label;
    SDL_FRect rect;
};

struct Mark {
    double t;
    bool accepted;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> effectChips;
    std::vector<Chip> modeChips;

    jc::Effects fx;
    bool overdo = false;
    bool slow = false;
    bool reduced = false;
    jc::InputPolicy policy = jc::InputPolicy::Buffered;

    jc::Clock clock;
    jc::InputBuffer buffer;
    jc::Attack attack;
    int kind = jc::kLightKind;
    int hp = kHpMax;

    double time = 0.0;  // app time (real time, slowed down in slow motion)
    double trauma = 0.0;
    double flashAge = 99.0;   // real time
    double impactAge = 99.0;  // real time
    double squashAge = 99.0;  // game time
    double squashStrength = 0.0;
    double kickDistance = 0.0;
    double numberAge = 99.0;  // game time
    int numberValue = 0;
    bool numberHeavy = false;
    std::vector<jc::Particle> sparks;
    jc::Rng rng{20240607};

    std::vector<jc::Sample> history;
    std::vector<Mark> marks;

    bool attract = true;  // the demo plays itself until the first input
    double attractTimer = 0.1;
    int attractStep = 0;

    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

jc::Effects effective(const App& app) { return jc::withReducedMotion(app.fx, app.reduced); }

void layoutChips(App& app) {
    struct Def {
        Act act;
        const char* label;
        float gapBefore;
    };
    constexpr std::array<Def, 7> effects{{{Act::Stop, "1 Hit stop", 0.0F},
                                           {Act::Shake, "2 Shake", 8.0F},
                                           {Act::Flash, "3 Flash", 8.0F},
                                           {Act::Squash, "4 Squash", 8.0F},
                                           {Act::Sparks, "5 Sparks", 8.0F},
                                           {Act::Impact, "6 Impact", 8.0F},
                                           {Act::Number, "7 Number", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : effects) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.effectChips.push_back({d.act, d.label, SDL_FRect{x, 396.0F, w, kChipH}});
        x += w;
    }
    constexpr std::array<Def, 6> modes{{{Act::None, "0 None", 0.0F},
                                         {Act::All, "A All", 8.0F},
                                         {Act::Overdo, "O Overdo shake", 24.0F},
                                         {Act::Input, "I Stop keeps input", 8.0F},
                                         {Act::Slow, "T Slow-mo", 24.0F},
                                         {Act::Reduce, "M Reduce motion", 8.0F}}};
    x = 40.0F;
    for (const Def& d : modes) {
        x += d.gapBefore;
        // The input chip changes its label, so reserve the wider one.
        const char* widest = d.act == Act::Input ? "I Stop keeps input" : d.label;
        const float w = app.hud.width(widest, gd::Hud::Style::Bold) + 24.0F;
        app.modeChips.push_back({d.act, d.label, SDL_FRect{x, 442.0F, w, kChipH}});
        x += w;
    }
}

bool* effectFlag(App& app, Act act) {
    switch (act) {
        case Act::Stop: return &app.fx.stop;
        case Act::Shake: return &app.fx.shake;
        case Act::Flash: return &app.fx.flash;
        case Act::Squash: return &app.fx.squash;
        case Act::Sparks: return &app.fx.sparks;
        case Act::Impact: return &app.fx.impact;
        case Act::Number: return &app.fx.number;
        default: return nullptr;
    }
}

void setAllEffects(App& app, bool on) {
    app.fx.stop = app.fx.shake = app.fx.flash = app.fx.squash = app.fx.sparks = app.fx.impact = app.fx.number = on;
}

void apply(App& app, Act act) {
    app.attract = false;
    if (bool* flag = effectFlag(app, act)) {
        *flag = !*flag;
        return;
    }
    switch (act) {
        case Act::None: setAllEffects(app, false); break;
        case Act::All: setAllEffects(app, true); break;
        case Act::Overdo: app.overdo = !app.overdo; break;
        case Act::Input:
            app.policy = app.policy == jc::InputPolicy::Buffered ? jc::InputPolicy::Dropped : jc::InputPolicy::Buffered;
            break;
        case Act::Slow: app.slow = !app.slow; break;
        case Act::Reduce: app.reduced = !app.reduced; break;
        default: break;
    }
}

bool active(const App& app, Act act) {
    switch (act) {
        case Act::Stop: return app.fx.stop;
        case Act::Shake: return app.fx.shake;
        case Act::Flash: return app.fx.flash;
        case Act::Squash: return app.fx.squash;
        case Act::Sparks: return app.fx.sparks;
        case Act::Impact: return app.fx.impact;
        case Act::Number: return app.fx.number;
        case Act::None: return jc::effectCount(app.fx) == 0;
        case Act::All: return jc::effectCount(app.fx) == 7;
        case Act::Overdo: return app.overdo;
        case Act::Input: return app.policy == jc::InputPolicy::Buffered;
        case Act::Slow: return app.slow;
        case Act::Reduce: return app.reduced;
    }
    return false;
}

/// True for a reaction that is on in the chip but silenced by reduced motion.
bool silenced(const App& app, Act act) {
    if (!app.reduced) return false;
    return (act == Act::Shake && app.fx.shake) || (act == Act::Flash && app.fx.flash) || (act == Act::Squash && app.fx.squash) ||
           (act == Act::Sparks && app.fx.sparks);
}

void pressAttack(App& app, int kind) {
    const bool accepted = jc::acceptsInput(app.clock.stopped(), app.policy);
    app.marks.push_back({app.time, accepted});
    if (accepted) app.buffer.press(app.time, kind);
}

void startSwing(App& app, int kind) {
    const jc::Weight& w = kind == jc::kHeavyKind ? jc::kHeavy : jc::kLight;
    if (app.hp <= 0) app.hp = kHpMax;
    if (app.attack.start({w.windup, w.recovery})) {
        app.kind = kind;
        app.trauma = jc::addTrauma(app.trauma, jc::routineTrauma(app.overdo, effective(app)));
    }
}

void onHit(App& app) {
    const jc::Weight& w = app.kind == jc::kHeavyKind ? jc::kHeavy : jc::kLight;
    const jc::Reaction r = jc::resolveHit(w, app.overdo, effective(app));
    app.clock.requestStop(r.stop);
    app.trauma = jc::addTrauma(app.trauma, r.trauma);
    if (r.flash) app.flashAge = 0.0;
    if (r.impact) app.impactAge = 0.0;
    app.squashAge = 0.0;
    app.squashStrength = r.squash;
    app.kickDistance = r.kick;
    if (r.sparks > 0) {
        jc::burst(app.sparks, app.rng, {static_cast<double>(kDummyX - kDummyW / 2.0F), static_cast<double>(kFloorY - 66.0F)}, {-0.5, -0.85}, r.sparks,
                  app.kind == jc::kHeavyKind ? 460.0 : 340.0, 0.9, 0.55);
    }
    app.hp = jc::takeDamage(app.hp, w.damage);
    if (r.number) {
        app.numberAge = 0.0;
        app.numberValue = w.damage;
        app.numberHeavy = app.kind == jc::kHeavyKind;
    }
}

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

gd::Color withAlpha(gd::Color c, double a) {
    c.a = static_cast<Uint8>(std::clamp(a, 0.0, 1.0) * 255.0);
    return c;
}

void drawStage(App& app, jc::Vec2 cam) {
    SDL_Renderer* r = app.renderer;
    const auto cx = static_cast<float>(cam.x);
    const auto cy = static_cast<float>(cam.y);

    const SDL_Rect clip{0, static_cast<int>(kStageTop), kWidth, static_cast<int>(kStageBottom - kStageTop)};
    SDL_SetRenderClipRect(r, &clip);
    gd::fillRect(r, 0.0F, kStageTop, static_cast<float>(kWidth), kStageBottom - kStageTop, gd::kPaper);
    gd::setColor(r, gd::kGrid);
    for (float gx = std::fmod(cx, 30.0F) - 30.0F; gx <= static_cast<float>(kWidth); gx += 30.0F) SDL_RenderLine(r, gx, kStageTop, gx, kStageBottom);
    for (float gy = std::fmod(cy, 30.0F) - 30.0F + 30.0F; gy <= kStageBottom; gy += 30.0F) SDL_RenderLine(r, 0.0F, gy, static_cast<float>(kWidth), gy);

    // Floor.
    gd::fillRect(r, -60.0F, kFloorY + cy, static_cast<float>(kWidth) + 120.0F, 80.0F, kFloor);
    thickLine(r, -60.0F, kFloorY + cy, static_cast<float>(kWidth) + 60.0F, kFloorY + cy, gd::kInk, 1);

    // Hero.
    const jc::Pose pose = jc::pose(app.attack.phase, app.attack.progress());
    const float hx = kHeroX + static_cast<float>(pose.dx) + cx;
    gd::card(r, hx - 26.0F, kFloorY - 92.0F + cy, 52.0F, 92.0F, gd::kTeal, gd::kInk);
    gd::fillRect(r, hx + 4.0F, kFloorY - 78.0F + cy, 14.0F, 14.0F, gd::kWhite);
    gd::fillRect(r, hx + 10.0F, kFloorY - 74.0F + cy, 6.0F, 6.0F, gd::kInk);
    const float handX = hx + 22.0F;
    const float handY = kFloorY - 56.0F + cy;
    const double rad = pose.angle * 3.14159265358979323846 / 180.0;
    const auto dirX = static_cast<float>(std::sin(rad));
    const auto dirY = static_cast<float>(-std::cos(rad));
    thickLine(r, handX - dirX * 14.0F, handY - dirY * 14.0F, handX, handY, gd::kTealDark, 2);
    thickLine(r, handX, handY, handX + dirX * 78.0F, handY + dirY * 78.0F, gd::kInk, 2);
    thickLine(r, handX + dirX * 4.0F, handY + dirY * 4.0F, handX + dirX * 74.0F, handY + dirY * 74.0F, gd::kWhite, 0);

    // Dummy.
    const jc::Scale sq = jc::squash(app.squashAge, app.squashStrength);
    const float dx = kDummyX + static_cast<float>(jc::kick(app.squashAge, app.kickDistance)) + cx;
    const float w = kDummyW * static_cast<float>(sq.x);
    const float h = kDummyH * static_cast<float>(sq.y);
    const float top = kFloorY - h + cy;
    gd::fillRect(r, dx - 30.0F, kFloorY - 6.0F + cy, 60.0F, 6.0F, kWoodDark);
    gd::card(r, dx - w / 2.0F, top, w, h - 6.0F, kWood, gd::kInk);
    const float ty = top + (h - 6.0F) * 0.4F;
    gd::disc(r, dx, ty, 20.0F * static_cast<float>(sq.x), gd::kWhite);
    gd::ring(r, dx, ty, 20.0F * static_cast<float>(sq.x), gd::kInk);
    gd::disc(r, dx, ty, 9.0F * static_cast<float>(sq.x), gd::kOrange);
    const double flash = app.fx.flash && !app.reduced ? jc::flashAmount(app.flashAge, kFlashSeconds) : 0.0;
    if (flash > 0.0) gd::fillRect(r, dx - w / 2.0F, top, w, h - 6.0F, withAlpha(gd::kWhite, flash * 0.92));

    // Health bar above the dummy. It does not squash with the body.
    const float barX = kDummyX - 40.0F + cx;
    const float barY = kFloorY - kDummyH - 30.0F + cy;
    gd::fillRect(r, barX, barY, 80.0F, 10.0F, gd::kGrid);
    gd::fillRect(r, barX, barY, 80.0F * static_cast<float>(app.hp) / static_cast<float>(kHpMax), 10.0F, app.hp > 25 ? gd::kTeal : gd::kOrange);
    gd::strokeRect(r, barX, barY, 80.0F, 10.0F, gd::kInk);

    // Impact ring and rays at the point of contact.
    if (app.impactAge < kImpactSeconds) {
        const auto u = static_cast<float>(app.impactAge / kImpactSeconds);
        const float px = kDummyX - kDummyW / 2.0F + cx;
        const float py = kFloorY - 66.0F + cy;
        const gd::Color c = withAlpha(gd::kOrange, 1.0 - static_cast<double>(u));
        gd::ring(r, px, py, 6.0F + 40.0F * u, c);
        gd::ring(r, px, py, 7.0F + 40.0F * u, c);
        for (int i = 0; i < 8; ++i) {
            const float a = 0.3927F + 0.7854F * static_cast<float>(i);
            const float r0 = 12.0F + 26.0F * u;
            thickLine(r, px + std::cos(a) * r0, py + std::sin(a) * r0, px + std::cos(a) * (r0 + 16.0F), py + std::sin(a) * (r0 + 16.0F), c, 1);
        }
    }

    // Sparks.
    for (std::size_t i = 0; i < app.sparks.size(); ++i) {
        const jc::Particle& p = app.sparks[i];
        const gd::Color c = withAlpha(i % 3 == 0 ? gd::kOrange : kSpark, p.maxLife > 0.0 ? p.life / p.maxLife * 1.6 : 0.0);
        gd::fillRect(r, static_cast<float>(p.pos.x) + cx - 2.5F, static_cast<float>(p.pos.y) + cy - 2.5F, 5.0F, 5.0F, c);
    }

    // Damage number.
    if (app.numberAge < kNumberSeconds) {
        char text[16];
        SDL_snprintf(text, sizeof text, "%d", app.numberValue);
        const gd::Hud::Style style = app.numberHeavy ? gd::Hud::Style::Large : gd::Hud::Style::Bold;
        const float tw = app.hud.width(text, style);
        const float ny = barY - 30.0F - static_cast<float>(app.numberAge) * 50.0F;
        const double fade = std::min(1.0, (kNumberSeconds - app.numberAge) / 0.3);
        app.hud.text(r, kDummyX - tw / 2.0F + cx, ny + cy, text, withAlpha(app.numberHeavy ? gd::kOrange : gd::kInk, fade), style);
    }

    SDL_SetRenderClipRect(r, nullptr);
}

void drawChip(App& app, const Chip& c) {
    SDL_Renderer* r = app.renderer;
    const bool on = active(app, c.act);
    const bool naive = (c.act == Act::Overdo && on) || (c.act == Act::Input && !on);
    gd::Color fill = on ? gd::kTeal : gd::kWhite;
    gd::Color ink = on ? gd::kWhite : gd::kInk;
    if (naive) {
        fill = gd::kOrange;
        ink = gd::kWhite;
    }
    if (silenced(app, c.act)) {
        fill = gd::kGrid;
        ink = gd::kMuted;
    }
    const std::string_view label = c.act == Act::Input && !on ? std::string_view{"I Stop eats input"} : std::string_view{c.label};
    gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
    app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, label, ink, gd::Hud::Style::Bold);
}

float timelineX(const App& app, double t) {
    const double u = (t - (app.time - kTimelineSpan)) / kTimelineSpan;
    return kTimelineX + static_cast<float>(std::clamp(u, 0.0, 1.0)) * kTimelineW;
}

void drawTimeline(App& app) {
    SDL_Renderer* r = app.renderer;
    // Caption with colored words as the legend.
    float x = kTimelineX;
    const auto word = [&](const char* s, gd::Color c) {
        app.hud.text(r, x, 308.0F, s, c);
        x += app.hud.width(s) + 10.0F;
    };
    word("last 2 s:", gd::kMuted);
    word("game runs", gd::kTealDark);
    word("hit stop", gd::kOrange);
    word("ticks = your clicks (orange = lost)", gd::kMuted);

    gd::fillRect(r, kTimelineX, kTimelineY, kTimelineW, 18.0F, gd::kGrid);
    for (std::size_t i = 1; i < app.history.size(); ++i) {
        const float a = timelineX(app, app.history[i - 1].t);
        const float b = timelineX(app, app.history[i].t);
        gd::fillRect(r, a, kTimelineY, std::max(b - a, 1.0F), 18.0F, app.history[i].stopped ? gd::kOrange : gd::kTeal);
    }
    gd::strokeRect(r, kTimelineX, kTimelineY, kTimelineW, 18.0F, gd::kInk);
    for (const Mark& m : app.marks) {
        const float mx = timelineX(app, m.t);
        gd::fillRect(r, mx - 1.0F, kTimelineY - 16.0F, 3.0F, 16.0F, m.accepted ? gd::kTealDark : gd::kOrange);
    }
    app.hud.text(r, kTimelineX + kTimelineW - app.hud.width("now"), 308.0F, "now", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const jc::Effects fx = effective(app);
    const jc::Vec2 cam = fx.shake ? jc::shakeOffset(app.trauma, app.time, kMaxShakePx) : jc::Vec2{};

    gd::backdrop(r, kWidth, kHeight);
    char right[48];
    SDL_snprintf(right, sizeof right, "%d of 7 reactions%s", jc::effectCount(fx), app.reduced ? " (reduced motion)" : "");
    app.hud.titleBar(r, kWidth, "[ JUICE ]", right);

    drawStage(app, cam);
    gd::strokeRect(r, 0.5F, kStageTop, static_cast<float>(kWidth) - 1.0F, kStageBottom - kStageTop, gd::kInk);

    // Readout. It stays put while the stage shakes.
    gd::card(r, 24.0F, 44.0F, 230.0F, 86.0F, gd::kWhite, gd::kInk);
    char line[48];
    app.hud.text(r, 38.0F, 52.0F, "trauma", gd::kMuted);
    gd::fillRect(r, 118.0F, 54.0F, 120.0F, 12.0F, gd::kGrid);
    gd::fillRect(r, 118.0F, 54.0F, static_cast<float>(120.0 * app.trauma), 12.0F, gd::kTeal);
    gd::strokeRect(r, 118.0F, 54.0F, 120.0F, 12.0F, gd::kInk);
    SDL_snprintf(line, sizeof line, "shake   %3d %%", static_cast<int>(std::lround(jc::shakeStrength(app.trauma) * 100.0)));
    app.hud.text(r, 38.0F, 76.0F, line, gd::kInk, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "freeze  %3d ms", static_cast<int>(std::lround(app.clock.stopRemaining * 1000.0)));
    app.hud.text(r, 38.0F, 100.0F, line, app.clock.stopped() ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);

    drawTimeline(app);

    app.hud.text(r, 40.0F, 376.0F, "reactions", gd::kMuted);
    for (const Chip& c : app.effectChips) drawChip(app, c);
    for (const Chip& c : app.modeChips) drawChip(app, c);

    const char* hint = app.attract ? "the demo plays itself: click or press Space to take over" : "click / Space: light hit    right click / Enter: heavy hit";
    app.hud.text(r, 40.0F, 492.0F, hint, gd::kMuted);
}

void update(App& app, double realDt) {
    const double dt = realDt * (app.slow ? kSlowFactor : 1.0);
    app.time += dt;
    app.history.push_back({app.time, app.clock.stopped()});
    jc::trimHistory(app.history, app.time, kTimelineSpan);
    app.marks.erase(app.marks.begin(), std::find_if(app.marks.begin(), app.marks.end(), [&](const Mark& m) { return m.t >= app.time - kTimelineSpan; }));

    // Real time: the camera, the flash and the impact ring keep running during a hit stop.
    app.trauma = jc::decayTrauma(app.trauma, dt, kTraumaDecay);
    app.flashAge += dt;
    app.impactAge += dt;

    if (app.attract) {
        app.attractTimer -= dt;
        if (app.attractTimer <= 0.0) {
            constexpr std::array<int, 3> pattern{jc::kLightKind, jc::kLightKind, jc::kHeavyKind};
            pressAttack(app, pattern[static_cast<std::size_t>(app.attractStep++) % pattern.size()]);
            app.attractTimer = 1.15;
        }
    }

    // Game time: the clock turns real time into game time, a hit stop makes it zero.
    const double gdt = app.clock.advance(dt);
    const bool wasBusy = !app.attack.ready();
    if (app.attack.ready()) {
        if (const auto k = app.buffer.take(app.time, kBufferWindow)) startSwing(app, *k);
    }
    if (app.attack.advance(gdt)) onHit(app);
    if (wasBusy && app.attack.ready()) {
        app.trauma = jc::addTrauma(app.trauma, jc::routineTrauma(app.overdo, effective(app)));  // landing, only in overdo
    }
    app.squashAge += gdt;
    app.numberAge += gdt;
    jc::stepParticles(app.sparks, gdt, kSparkGravity);
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
    if (!SDL_CreateWindowAndRenderer("juice", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            if (event->button.button == SDL_BUTTON_LEFT) {
                for (const auto* chips : {&app.effectChips, &app.modeChips}) {
                    for (const Chip& c : *chips) {
                        if (SDL_PointInRectFloat(&p, &c.rect)) {
                            apply(app, c.act);
                            return SDL_APP_CONTINUE;
                        }
                    }
                }
            }
            if (p.y >= kStageTop && p.y < kStageBottom) {
                if (event->button.button == SDL_BUTTON_LEFT || event->button.button == SDL_BUTTON_RIGHT) {
                    app.attract = false;
                    pressAttack(app, event->button.button == SDL_BUTTON_RIGHT ? jc::kHeavyKind : jc::kLightKind);
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_SPACE:
                    app.attract = false;
                    pressAttack(app, jc::kLightKind);
                    break;
                case SDLK_RETURN:
                    app.attract = false;
                    pressAttack(app, jc::kHeavyKind);
                    break;
                case SDLK_1: apply(app, Act::Stop); break;
                case SDLK_2: apply(app, Act::Shake); break;
                case SDLK_3: apply(app, Act::Flash); break;
                case SDLK_4: apply(app, Act::Squash); break;
                case SDLK_5: apply(app, Act::Sparks); break;
                case SDLK_6: apply(app, Act::Impact); break;
                case SDLK_7: apply(app, Act::Number); break;
                case SDLK_0: apply(app, Act::None); break;
                case SDLK_A: apply(app, Act::All); break;
                case SDLK_O: apply(app, Act::Overdo); break;
                case SDLK_I: apply(app, Act::Input); break;
                case SDLK_T: apply(app, Act::Slow); break;
                case SDLK_M: apply(app, Act::Reduce); break;
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
    const double frameTime = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    update(app, std::clamp(frameTime, 0.0, 0.25));

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
