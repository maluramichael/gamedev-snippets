// Easing and tweening with SDL3 main callbacks.
//
// An easing function bends time, not the path: the object always travels a + (b - a) * f(t), only f changes. The
// first view is a gallery of all of them. Every card plots f(t) (teal), the orange dot is "now" on that curve and
// the square below runs between two points driven by the same f. In / out / inOut choose the variant for all cards,
// a click picks one for the detail view: big plot, an object with position, scale and rotation, and a ghost trail
// whose spacing shows the speed. The second view (Tab) is the Tween<T> from logic.hpp at work: three tweens in a row
// (move, grow, tint), each handing the time it did not use to the next one. Pick a smaller frame rate and watch that
// the chain still ends at the same time. All the math lives in logic.hpp and is covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

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

// Gallery: 4 x 3 cards, the 12th cell is the legend.
constexpr int kCols = 4;
constexpr float kCardW = 130.0F;
constexpr float kCardH = 130.0F;
constexpr float kCardGap = 8.0F;
constexpr float kGalX = 14.0F;
constexpr float kGalY = 40.0F;

// Detail panel on the right.
constexpr float kPanelX = 566.0F;
constexpr float kPanelY = 40.0F;
constexpr float kPanelW = 380.0F;
constexpr float kPanelH = 406.0F;

constexpr float kChipH = 34.0F;
constexpr float kBottomY = 476.0F;

constexpr double kGalleryPeriod = 1.5;  // seconds for one run on a card
constexpr double kGhostGap = 0.04;      // seconds between two ghosts in the trail
constexpr int kGhosts = 12;
constexpr double kChainHold = 1.2;      // seconds the finished chain rests before it starts over
constexpr double kMaxFrame = 0.1;       // clamp after a stall (tab switch)

constexpr std::array<double, 3> kDurations{0.5, 1.0, 2.0};

constexpr gd::Color kFaint{17, 24, 39, 30};
constexpr gd::Color kStrong{17, 24, 39, 90};

enum class View { Easing, Chain };
enum class Step { Real, Fps20, Fps5 };

enum class Action { ViewEasing, ViewChain, VarIn, VarOut, VarInOut, Dur0, Dur1, Dur2, Loop, Once, Play, Restart, StepReal, StepFps20, StepFps5 };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

// The three stages of the chain in the second view.
const ease::Vec2 kStageA{552.0, 236.0};
const ease::Vec2 kStageB{866.0, 128.0};
const ease::Rgb kTealRgb{13.0, 148.0, 136.0};
const ease::Rgb kOrangeRgb{234.0, 88.0, 12.0};
constexpr double kMoveSeconds = 0.8;
constexpr double kGrowSeconds = 0.6;
constexpr double kTintSeconds = 0.6;
constexpr double kChainTotal = kMoveSeconds + kGrowSeconds + kTintSeconds;
constexpr double kGrowTo = 2.2;

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    long maxFrames = -1;
    long frames = 0;
    Uint64 lastTicks = 0;
    View view = View::Easing;

    // Easing view
    int selected = static_cast<int>(ease::Family::Elastic);
    ease::Variant variant = ease::Variant::Out;
    ease::Tween<double> gallery{0.0, 1.0, kGalleryPeriod, ease::linear, ease::Mode::PingPong};
    ease::Tween<double> detail{0.0, 1.0, 1.0, ease::elasticOut, ease::Mode::Loop};
    double clock = 0.0;                           // seconds since start, the time axis of the ghost trail
    std::deque<std::pair<double, double>> trail;  // (clock, eased value) of every frame in the last second

    // Chain view
    ease::Tween<ease::Vec2> move{kStageA, kStageB, kMoveSeconds, ease::backOut};
    ease::Tween<double> grow{1.0, kGrowTo, kGrowSeconds, ease::elasticOut};
    ease::Tween<ease::Rgb> tint{kTealRgb, kOrangeRgb, kTintSeconds, ease::quadInOut};
    double chainTime = 0.0;
    double chainIdle = 0.0;
    ease::ChunkClock stepClock;
    double lastStep = 0.0;
    Step step = Step::Real;
};

ease::Family selectedFamily(const App& app) { return static_cast<ease::Family>(app.selected); }
ease::Fn selectedFn(const App& app) { return ease::lookup(selectedFamily(app), app.variant); }

gd::Color toColor(ease::Rgb c) {
    const auto channel = [](double v) { return static_cast<Uint8>(std::clamp(v, 0.0, 255.0) + 0.5); };
    return gd::Color{channel(c.r), channel(c.g), channel(c.b)};
}

// ---------------------------------------------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------------------------------------------

/// A line with a real width: a quad along the segment. SDL_RenderLine is always 1 px, too thin on a scaled canvas.
void strokeLine(SDL_Renderer* r, float ax, float ay, float bx, float by, float width, gd::Color c) {
    const float dx = bx - ax;
    const float dy = by - ay;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-4F) return;
    const float nx = -dy / len * width / 2.0F;
    const float ny = dx / len * width / 2.0F;
    const SDL_FColor col = gd::fcolor(c);
    const SDL_Vertex v[4] = {{{ax + nx, ay + ny}, col, {0, 0}}, {{ax - nx, ay - ny}, col, {0, 0}}, {{bx - nx, by - ny}, col, {0, 0}}, {{bx + nx, by + ny}, col, {0, 0}}};
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
}

/// A whole polyline in one draw call.
void polyline(SDL_Renderer* r, const std::vector<SDL_FPoint>& pts, float width, gd::Color c) {
    if (pts.size() < 2) return;
    const SDL_FColor col = gd::fcolor(c);
    std::vector<SDL_Vertex> v;
    std::vector<int> idx;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const float dx = pts[i + 1].x - pts[i].x;
        const float dy = pts[i + 1].y - pts[i].y;
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-4F) continue;
        const float nx = -dy / len * width / 2.0F;
        const float ny = dx / len * width / 2.0F;
        const int base = static_cast<int>(v.size());
        v.push_back({{pts[i].x + nx, pts[i].y + ny}, col, {0, 0}});
        v.push_back({{pts[i].x - nx, pts[i].y - ny}, col, {0, 0}});
        v.push_back({{pts[i + 1].x - nx, pts[i + 1].y - ny}, col, {0, 0}});
        v.push_back({{pts[i + 1].x + nx, pts[i + 1].y + ny}, col, {0, 0}});
        idx.insert(idx.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    if (!v.empty()) SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

void dashedH(SDL_Renderer* r, float x0, float x1, float y, gd::Color c) {
    for (float x = x0; x < x1; x += 6.0F) gd::fillRect(r, x, y, std::min(3.0F, x1 - x), 1.0F, c);
}

void dashedV(SDL_Renderer* r, float x, float y0, float y1, gd::Color c) {
    for (float y = y0; y < y1; y += 6.0F) gd::fillRect(r, x, y, 1.0F, std::min(3.0F, y1 - y), c);
}

void clip(SDL_Renderer* r, float x, float y, float w, float h) {
    const SDL_Rect rect{static_cast<int>(x), static_cast<int>(y), static_cast<int>(w), static_cast<int>(h)};
    SDL_SetRenderClipRect(r, &rect);
}

void unclip(SDL_Renderer* r) { SDL_SetRenderClipRect(r, nullptr); }

/// A square turned by `angle`. The orange notch in one corner makes the rotation visible (a square looks the same
/// after every quarter turn).
void turnedSquare(SDL_Renderer* r, float cx, float cy, float size, float angle, gd::Color fill, bool outline) {
    const float h = size / 2.0F;
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    const std::array<std::pair<float, float>, 4> local{{{-h, -h}, {h, -h}, {h, h}, {-h, h}}};
    std::array<SDL_FPoint, 4> p{};
    for (std::size_t i = 0; i < 4; ++i) p[i] = {cx + local[i].first * c - local[i].second * s, cy + local[i].first * s + local[i].second * c};
    const SDL_FColor col = gd::fcolor(fill);
    const SDL_Vertex v[4] = {{p[0], col, {0, 0}}, {p[1], col, {0, 0}}, {p[2], col, {0, 0}}, {p[3], col, {0, 0}}};
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
    if (outline) {
        for (std::size_t i = 0; i < 4; ++i) strokeLine(r, p[i].x, p[i].y, p[(i + 1) % 4].x, p[(i + 1) % 4].y, 2.0F, gd::kInk);
        const float nx = cx + (-h * 0.55F) * c - (-h * 0.55F) * s;
        const float ny = cy + (-h * 0.55F) * s + (-h * 0.55F) * c;
        gd::disc(r, nx, ny, std::max(2.0F, size * 0.09F), gd::kOrange);
    }
}

struct Plot {
    float x, y, w, h;
    double lo, hi;
    float px(double t) const { return x + static_cast<float>(t) * w; }
    float py(double v) const { return y + h - static_cast<float>((v - lo) / (hi - lo)) * h; }
};

std::vector<SDL_FPoint> curvePoints(ease::Fn fn, const Plot& plot, int segments) {
    std::vector<SDL_FPoint> pts;
    pts.reserve(static_cast<std::size_t>(segments) + 1);
    for (int i = 0; i <= segments; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(segments);
        pts.push_back({plot.px(t), plot.py(fn(t))});
    }
    return pts;
}

/// Plot range that always contains 0 and 1 and the whole curve, with a little air.
Plot makePlot(ease::Fn fn, float x, float y, float w, float h) {
    const ease::Range range = ease::sampleRange(fn, 160);
    return Plot{x, y, w, h, std::min(0.0, range.min) - 0.1, std::max(1.0, range.max) + 0.1};
}

float centered(const App& app, std::string_view s, float x, float w, gd::Hud::Style style = gd::Hud::Style::Regular) { return x + (w - app.hud.width(s, style)) / 2.0F; }

// ---------------------------------------------------------------------------------------------------------------
// Chips
// ---------------------------------------------------------------------------------------------------------------

void addChip(App& app, Action action, std::string label, float& x, float y, float gap = 8.0F) {
    const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
    app.chips.push_back({action, std::move(label), SDL_FRect{x, y, w, kChipH}});
    x += w + gap;
}

void layoutChips(App& app) {
    float x = 40.0F;
    addChip(app, Action::ViewEasing, "Easing", x, kBottomY);
    addChip(app, Action::ViewChain, "Tween<T>", x, kBottomY, 24.0F);
    const float afterTabs = x;
    addChip(app, Action::VarIn, "1 in", x, kBottomY);
    addChip(app, Action::VarOut, "2 out", x, kBottomY);
    addChip(app, Action::VarInOut, "3 inOut", x, kBottomY);
    x = afterTabs;
    addChip(app, Action::Restart, "R restart", x, kBottomY);
    addChip(app, Action::StepReal, "1 real dt", x, kBottomY);
    addChip(app, Action::StepFps20, "2 20 fps", x, kBottomY);
    addChip(app, Action::StepFps5, "3 5 fps", x, kBottomY);
    x = kPanelX + 14.0F;
    addChip(app, Action::Dur0, "0.5 s", x, 310.0F);
    addChip(app, Action::Dur1, "1 s", x, 310.0F);
    addChip(app, Action::Dur2, "2 s", x, 310.0F);
    x = kPanelX + 14.0F;
    addChip(app, Action::Loop, "loop", x, 350.0F);
    addChip(app, Action::Once, "once", x, 350.0F);
    addChip(app, Action::Play, "play", x, 350.0F);
}

bool chipVisible(const App& app, Action action) {
    switch (action) {
        case Action::ViewEasing:
        case Action::ViewChain: return true;
        case Action::VarIn:
        case Action::VarOut:
        case Action::VarInOut:
        case Action::Dur0:
        case Action::Dur1:
        case Action::Dur2:
        case Action::Loop:
        case Action::Once:
        case Action::Play: return app.view == View::Easing;
        case Action::Restart:
        case Action::StepReal:
        case Action::StepFps20:
        case Action::StepFps5: return app.view == View::Chain;
    }
    return false;
}

bool chipActive(const App& app, Action action) {
    switch (action) {
        case Action::ViewEasing: return app.view == View::Easing;
        case Action::ViewChain: return app.view == View::Chain;
        case Action::VarIn: return app.variant == ease::Variant::In;
        case Action::VarOut: return app.variant == ease::Variant::Out;
        case Action::VarInOut: return app.variant == ease::Variant::InOut;
        case Action::Dur0: return app.detail.duration == kDurations[0];
        case Action::Dur1: return app.detail.duration == kDurations[1];
        case Action::Dur2: return app.detail.duration == kDurations[2];
        case Action::Loop: return app.detail.mode == ease::Mode::Loop;
        case Action::Once: return app.detail.mode == ease::Mode::Once;
        case Action::StepReal: return app.step == Step::Real;
        case Action::StepFps20: return app.step == Step::Fps20;
        case Action::StepFps5: return app.step == Step::Fps5;
        case Action::Play:
        case Action::Restart: return false;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------
// State changes
// ---------------------------------------------------------------------------------------------------------------

void restartDetail(App& app) {
    app.detail.ease = selectedFn(app);
    app.detail.restart();
    app.trail.clear();
}

void restartChain(App& app) {
    app.move.restart();
    app.grow.restart();
    app.tint.restart();
    app.chainTime = 0.0;
    app.chainIdle = 0.0;
    app.stepClock.reset();
}

void setStep(App& app, Step step) {
    app.step = step;
    app.stepClock.reset();  // the rest collected for the old chunk size must not leak into the new one
}

void select(App& app, int index) {
    app.selected = std::clamp(index, 0, ease::kFamilyCount - 1);
    restartDetail(app);
}

void setVariant(App& app, ease::Variant variant) {
    app.variant = variant;
    restartDetail(app);
}

void setDuration(App& app, std::size_t i) {
    app.detail.duration = kDurations[i];
    restartDetail(app);
}

void setMode(App& app, ease::Mode mode) {
    app.detail.mode = mode;
    restartDetail(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ViewEasing: app.view = View::Easing; break;
        case Action::ViewChain:
            app.view = View::Chain;
            restartChain(app);
            break;
        case Action::VarIn: setVariant(app, ease::Variant::In); break;
        case Action::VarOut: setVariant(app, ease::Variant::Out); break;
        case Action::VarInOut: setVariant(app, ease::Variant::InOut); break;
        case Action::Dur0: setDuration(app, 0); break;
        case Action::Dur1: setDuration(app, 1); break;
        case Action::Dur2: setDuration(app, 2); break;
        case Action::Loop: setMode(app, ease::Mode::Loop); break;
        case Action::Once: setMode(app, ease::Mode::Once); break;
        case Action::Play: restartDetail(app); break;
        case Action::Restart: restartChain(app); break;
        case Action::StepReal: setStep(app, Step::Real); break;
        case Action::StepFps20: setStep(app, Step::Fps20); break;
        case Action::StepFps5: setStep(app, Step::Fps5); break;
    }
}

SDL_FRect cardRect(int index) {
    const int col = index % kCols;
    const int row = index / kCols;
    return SDL_FRect{kGalX + static_cast<float>(col) * (kCardW + kCardGap), kGalY + static_cast<float>(row) * (kCardH + kCardGap), kCardW, kCardH};
}

// ---------------------------------------------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------------------------------------------

void chainStep(App& app, double step) {
    app.lastStep = step;
    if (ease::allFinished(app.move, app.grow, app.tint)) {
        app.chainIdle += step;
        if (app.chainIdle >= kChainHold) restartChain(app);
        return;
    }
    ease::updateChain(step, app.move, app.grow, app.tint);
    app.chainTime = std::min(kChainTotal, app.chainTime + step);
}

void update(App& app, double dt) {
    app.gallery.update(dt);
    app.detail.update(dt);
    app.clock += dt;
    app.trail.emplace_back(app.clock, app.detail.value());
    while (app.trail.size() > 1 && app.trail.front().first < app.clock - 1.0) app.trail.pop_front();

    if (app.view != View::Chain) return;
    if (app.step == Step::Real) {
        chainStep(app, dt);
        return;
    }
    // A fixed, low frame rate: the chain is stepped in big, equal chunks. It must end up in the same places.
    const double chunk = app.step == Step::Fps20 ? 1.0 / 20.0 : 1.0 / 5.0;
    for (int n = app.stepClock.advance(dt, chunk); n > 0; --n) chainStep(app, chunk);
}

// ---------------------------------------------------------------------------------------------------------------
// Easing view
// ---------------------------------------------------------------------------------------------------------------

void drawLegend(App& app, const SDL_FRect& rc) {
    SDL_Renderer* r = app.renderer;
    gd::strokeRect(r, rc.x, rc.y, rc.w, rc.h, gd::kMuted);
    app.hud.text(r, rc.x + 10.0F, rc.y + 4.0F, "legend", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, rc.x + 10.0F, rc.y + 34.0F, "x  time t", gd::kMuted);
    app.hud.text(r, rc.x + 10.0F, rc.y + 56.0F, "y  f(t)", gd::kTeal);
    app.hud.text(r, rc.x + 10.0F, rc.y + 78.0F, "dot  now", gd::kOrange);
    app.hud.text(r, rc.x + 10.0F, rc.y + 100.0F, "box  a to b", gd::kMuted);
}

void drawCard(App& app, int index, const SDL_FRect& rc) {
    SDL_Renderer* r = app.renderer;
    const auto family = static_cast<ease::Family>(index);
    const ease::Fn fn = ease::lookup(family, app.variant);
    const bool selected = index == app.selected;

    gd::card(r, rc.x, rc.y, rc.w, rc.h, selected ? gd::kTealSoft : gd::kWhite, selected ? gd::kTeal : gd::kInk);
    if (selected) gd::strokeRect(r, rc.x + 1.0F, rc.y + 1.0F, rc.w - 2.0F, rc.h - 2.0F, gd::kTeal);
    app.hud.text(r, rc.x + 10.0F, rc.y + 4.0F, ease::name(family), selected ? gd::kTealDark : gd::kInk, gd::Hud::Style::Bold);

    const Plot plot = makePlot(fn, rc.x + 12.0F, rc.y + 34.0F, rc.w - 24.0F, 54.0F);
    dashedH(r, plot.x, plot.x + plot.w, plot.py(0.0), kStrong);
    dashedH(r, plot.x, plot.x + plot.w, plot.py(1.0), kStrong);
    polyline(r, curvePoints(fn, plot, 48), 2.5F, selected ? gd::kTealDark : gd::kTeal);

    const double t = app.gallery.progress();
    const double v = fn(t);
    gd::disc(r, plot.px(t), plot.py(v), 4.0F, gd::kOrange);
    gd::ring(r, plot.px(t), plot.py(v), 4.0F, gd::kInk);

    // The square runs between the two ends of the track, driven by the same value.
    const float trackY = rc.y + 112.0F;
    const float a = rc.x + 34.0F;
    const float b = rc.x + rc.w - 34.0F;
    gd::fillRect(r, a, trackY - 1.0F, b - a, 2.0F, kStrong);
    gd::fillRect(r, a - 1.0F, trackY - 6.0F, 2.0F, 12.0F, kStrong);
    gd::fillRect(r, b - 1.0F, trackY - 6.0F, 2.0F, 12.0F, kStrong);
    clip(r, rc.x, rc.y, rc.w, rc.h);
    const auto x = static_cast<float>(ease::mix(static_cast<double>(a), static_cast<double>(b), v));
    gd::fillRect(r, x - 7.0F, trackY - 7.0F, 14.0F, 14.0F, gd::kOrange);
    gd::strokeRect(r, x - 7.0F, trackY - 7.0F, 14.0F, 14.0F, gd::kInk);
    unclip(r);
}

const char* formulaIn(ease::Family family) {
    switch (family) {
        case ease::Family::Linear: return "t";
        case ease::Family::Sine: return "1 - cos(t * pi / 2)";
        case ease::Family::Quad: return "t^2";
        case ease::Family::Cubic: return "t^3";
        case ease::Family::Quart: return "t^4";
        case ease::Family::Quint: return "t^5";
        case ease::Family::Expo: return "2^(10t - 10)";
        case ease::Family::Circ: return "1 - sqrt(1 - t^2)";
        case ease::Family::Back: return "c3 t^3 - c1 t^2";
        case ease::Family::Elastic: return "-2^(10t-10) sin((10t-10.75) c4)";
        case ease::Family::Bounce: return "1 - bounceOut(1 - t)";
    }
    return "";
}

std::string ruleText(ease::Family family, ease::Variant variant) {
    if (family == ease::Family::Linear) return "same speed all the way";
    switch (variant) {
        case ease::Variant::In: return "in: slow start, fast end";
        case ease::Variant::Out: return "out: 1 - in(1 - t)";
        case ease::Variant::InOut:
            if (family == ease::Family::Back) return "inOut: c2 = c1 * 1.525, mirrored";
            if (family == ease::Family::Elastic) return "inOut: c5 = 2pi / 4.5, mirrored";
            return "inOut: in(2t) / 2, then mirrored";
    }
    return "";
}

void drawDetail(App& app) {
    SDL_Renderer* r = app.renderer;
    const ease::Family family = selectedFamily(app);
    const ease::Fn fn = selectedFn(app);
    gd::card(r, kPanelX, kPanelY, kPanelW, kPanelH, gd::kWhite, gd::kInk);
    const float ix = kPanelX + 14.0F;

    char text[96];
    SDL_snprintf(text, sizeof text, "%s %s", ease::name(family), ease::name(app.variant));
    app.hud.text(r, ix, kPanelY + 6.0F, text, gd::kTealDark, gd::Hud::Style::Bold);
    const double t = app.detail.progress();
    const double v = fn(t);
    SDL_snprintf(text, sizeof text, "t %.2f  f %.2f", t, v);
    app.hud.text(r, kPanelX + kPanelW - 14.0F - app.hud.width(text), kPanelY + 6.0F, text, gd::kMuted);

    // Plot with grid every half unit, the lines at 0 and 1 are the start and the target.
    const Plot plot = makePlot(fn, kPanelX + 52.0F, kPanelY + 36.0F, kPanelW - 52.0F - 14.0F, 140.0F);
    gd::fillRect(r, plot.x, plot.y, plot.w, plot.h, gd::kPaper);
    gd::strokeRect(r, plot.x, plot.y, plot.w, plot.h, kStrong);
    clip(r, plot.x, plot.y, plot.w, plot.h);
    dashedV(r, plot.px(0.5), plot.y, plot.y + plot.h, kFaint);
    for (double g = std::ceil(plot.lo * 2.0) / 2.0; g <= plot.hi; g += 0.5) {
        const bool edge = std::fabs(g) < 1e-9 || std::fabs(g - 1.0) < 1e-9;
        dashedH(r, plot.x, plot.x + plot.w, plot.py(g), edge ? kStrong : kFaint);
    }
    // guides from the dot to the axes: time on the x axis gives the value on the y axis
    dashedH(r, plot.x, plot.px(t), plot.py(v), gd::kOrange);
    dashedV(r, plot.px(t), plot.py(v), plot.y + plot.h, gd::kOrange);
    polyline(r, curvePoints(fn, plot, 160), 3.0F, gd::kTeal);
    gd::disc(r, plot.px(t), plot.py(v), 6.0F, gd::kOrange);
    gd::ring(r, plot.px(t), plot.py(v), 6.0F, gd::kInk);
    unclip(r);
    for (double g = std::ceil(plot.lo * 2.0) / 2.0; g <= plot.hi; g += 0.5) {
        SDL_snprintf(text, sizeof text, std::fabs(g - std::round(g)) < 1e-9 ? "%.0f" : "%.1f", g == 0.0 ? 0.0 : g);
        app.hud.text(r, plot.x - 6.0F - app.hud.width(text), plot.py(g) - 11.0F, text, gd::kMuted);
    }
    app.hud.text(r, plot.x - 5.0F, plot.y + plot.h + 1.0F, "0", gd::kMuted);
    app.hud.text(r, plot.x + plot.w - 5.0F, plot.y + plot.h + 1.0F, "1", gd::kMuted);
    app.hud.text(r, plot.x + plot.w / 2.0F - 5.0F, plot.y + plot.h + 1.0F, "t", gd::kMuted);

    // The tween: one object, position + scale + rotation all driven by the same value.
    const float trackY = kPanelY + 230.0F;
    const float a = kPanelX + kPanelW / 2.0F - 80.0F;
    const float b = kPanelX + kPanelW / 2.0F + 80.0F;
    gd::fillRect(r, a, trackY - 1.0F, b - a, 2.0F, kStrong);
    gd::fillRect(r, a - 1.0F, trackY - 8.0F, 2.0F, 16.0F, kStrong);
    gd::fillRect(r, b - 1.0F, trackY - 8.0F, 2.0F, 16.0F, kStrong);
    clip(r, kPanelX + 2.0F, trackY - 40.0F, kPanelW - 4.0F, 80.0F);
    const auto pose = [&](double value, gd::Color fill, bool outline) {
        const auto size = static_cast<float>(32.0 * std::max(0.05, ease::mix(0.6, 1.2, value)));
        const auto angle = static_cast<float>(ease::mix(0.0, ease::kPi, value));
        const auto x = static_cast<float>(ease::mix(static_cast<double>(a), static_cast<double>(b), value));
        turnedSquare(r, x, trackY, size, angle, fill, outline);
    };
    // Ghosts: where the object was 40 ms, 80 ms ... ago. The wider the gaps, the faster it moves.
    for (int k = kGhosts; k >= 1; --k) {
        const double when = app.clock - static_cast<double>(k) * kGhostGap;
        if (app.trail.empty() || when < app.trail.front().first) continue;
        const auto it = std::upper_bound(app.trail.begin(), app.trail.end(), when, [](double w, const std::pair<double, double>& s) { return w < s.first; });
        const double ghost = (it == app.trail.begin() ? it : it - 1)->second;
        const auto alpha = static_cast<Uint8>(110.0 * (1.0 - static_cast<double>(k) / static_cast<double>(kGhosts + 1)));
        pose(ghost, gd::Color{13, 148, 136, alpha}, false);
    }
    pose(v, gd::kTeal, true);
    unclip(r);

    app.hud.text(r, ix, kPanelY + 352.0F, (std::string("in: ") + formulaIn(family)).c_str(), gd::kInk);
    app.hud.text(r, ix, kPanelY + 374.0F, ruleText(family, app.variant), gd::kMuted);
}

void drawEasing(App& app) {
    for (int i = 0; i < kCols * 3; ++i) {
        const SDL_FRect rc = cardRect(i);
        if (i < ease::kFamilyCount) {
            drawCard(app, i, rc);
        } else {
            drawLegend(app, rc);
        }
    }
    drawDetail(app);
}

// ---------------------------------------------------------------------------------------------------------------
// Chain view
// ---------------------------------------------------------------------------------------------------------------

template <class T>
const char* stateOf(const ease::Tween<T>& tween, bool previousDone) {
    if (tween.finished()) return "done";
    return previousDone ? "running" : "waiting";
}

gd::Color stateColor(const char* state) { return std::strcmp(state, "done") == 0 ? gd::kTeal : std::strcmp(state, "running") == 0 ? gd::kOrange : gd::kMuted; }

void drawRow(App& app, float y, const char* name, const char* type, const char* spec, const char* state, double progress, const char* value) {
    SDL_Renderer* r = app.renderer;
    constexpr float x = 24.0F;
    constexpr float w = 450.0F;
    constexpr float h = 78.0F;
    const bool running = std::strcmp(state, "running") == 0;
    gd::card(r, x, y, w, h, running ? gd::kTealSoft : gd::kWhite, running ? gd::kTeal : gd::kInk);
    app.hud.text(r, x + 12.0F, y + 6.0F, name, gd::kTealDark, gd::Hud::Style::Bold);
    app.hud.text(r, x + 12.0F + app.hud.width(name, gd::Hud::Style::Bold) + 10.0F, y + 6.0F, type, gd::kMuted);
    app.hud.text(r, x + w - 12.0F - app.hud.width(value, gd::Hud::Style::Bold), y + 6.0F, value, gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, x + 12.0F, y + 28.0F, spec, gd::kInk);
    app.hud.text(r, x + w - 12.0F - app.hud.width(state), y + 28.0F, state, stateColor(state));
    gd::fillRect(r, x + 12.0F, y + h - 16.0F, w - 24.0F, 8.0F, gd::kGrid);
    gd::fillRect(r, x + 12.0F, y + h - 16.0F, (w - 24.0F) * static_cast<float>(std::clamp(progress, 0.0, 1.0)), 8.0F, gd::kTeal);
    gd::strokeRect(r, x + 12.0F, y + h - 16.0F, w - 24.0F, 8.0F, gd::kInk);
}

void drawChain(App& app) {
    SDL_Renderer* r = app.renderer;
    char value[48];

    // Stage on the right: a to b, then bigger, then orange.
    constexpr float sx = 490.0F;
    constexpr float sy = 44.0F;
    constexpr float sw = 446.0F;
    constexpr float sh = 246.0F;
    gd::card(r, sx, sy, sw, sh, gd::kWhite, gd::kInk);
    const auto ax = static_cast<float>(kStageA.x);
    const auto ay = static_cast<float>(kStageA.y);
    const auto bx = static_cast<float>(kStageB.x);
    const auto by = static_cast<float>(kStageB.y);
    for (int i = 0; i < 20; i += 2) {
        const float f0 = static_cast<float>(i) / 20.0F;
        const float f1 = static_cast<float>(i + 1) / 20.0F;
        strokeLine(r, ax + (bx - ax) * f0, ay + (by - ay) * f0, ax + (bx - ax) * f1, ay + (by - ay) * f1, 2.0F, kStrong);
    }
    gd::ring(r, ax, ay, 10.0F, gd::kInk);
    gd::ring(r, bx, by, 10.0F, gd::kInk);
    app.hud.text(r, ax - 5.0F, ay + 14.0F, "a", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, bx - 5.0F, by + 14.0F, "b", gd::kMuted, gd::Hud::Style::Bold);
    clip(r, sx + 2.0F, sy + 2.0F, sw - 4.0F, sh - 4.0F);
    const ease::Vec2 pos = app.move.value();
    const auto size = static_cast<float>(36.0 * std::max(0.1, app.grow.value()));
    const gd::Color fill = toColor(app.tint.value());
    gd::fillRect(r, static_cast<float>(pos.x) - size / 2.0F + 4.0F, static_cast<float>(pos.y) - size / 2.0F + 4.0F, size, size, gd::Color{17, 24, 39, 40});
    gd::fillRect(r, static_cast<float>(pos.x) - size / 2.0F, static_cast<float>(pos.y) - size / 2.0F, size, size, fill);
    gd::strokeRect(r, static_cast<float>(pos.x) - size / 2.0F, static_cast<float>(pos.y) - size / 2.0F, size, size, gd::kInk);
    unclip(r);

    // The three tweens.
    const bool moveDone = app.move.finished();
    const bool growDone = app.grow.finished();
    SDL_snprintf(value, sizeof value, "(%.0f, %.0f)", pos.x, pos.y);
    drawRow(app, 44.0F, "move", "Tween<Vec2>", "a to b  0.8 s  backOut", stateOf(app.move, true), app.move.progress(), value);
    SDL_snprintf(value, sizeof value, "%.2f", app.grow.value());
    drawRow(app, 130.0F, "grow", "Tween<double>", "1.0 to 2.2  0.6 s  elasticOut", stateOf(app.grow, moveDone), app.grow.progress(), value);
    const ease::Rgb c = app.tint.value();
    SDL_snprintf(value, sizeof value, "rgb(%d,%d,%d)", static_cast<int>(std::clamp(c.r, 0.0, 255.0)), static_cast<int>(std::clamp(c.g, 0.0, 255.0)), static_cast<int>(std::clamp(c.b, 0.0, 255.0)));
    drawRow(app, 216.0F, "tint", "Tween<Rgb>", "teal to orange  0.6 s  quadInOut", stateOf(app.tint, growDone), app.tint.progress(), value);

    // Timeline: the three tweens end to end, the playhead is the time since the start.
    constexpr float tx = 24.0F;
    constexpr float tw = 912.0F;
    constexpr float ty = 312.0F;
    constexpr float th = 30.0F;
    const std::array<std::pair<const char*, double>, 3> parts{{{"move", kMoveSeconds}, {"grow", kGrowSeconds}, {"tint", kTintSeconds}}};
    float px = tx;
    const float headX = tx + static_cast<float>(app.chainTime / kChainTotal) * tw;
    for (const auto& [label, seconds] : parts) {
        const auto width = static_cast<float>(seconds / kChainTotal) * tw;
        gd::fillRect(r, px, ty, width, th, gd::kWhite);
        gd::fillRect(r, px, ty, std::clamp(headX - px, 0.0F, width), th, gd::kTealSoft);  // the part that is already played
        gd::strokeRect(r, px, ty, width, th, gd::kInk);
        app.hud.text(r, centered(app, label, px, width), ty + 4.0F, label, gd::kTealDark, gd::Hud::Style::Bold);
        px += width;
    }
    gd::fillRect(r, headX - 1.5F, ty - 6.0F, 3.0F, th + 12.0F, gd::kOrange);
    gd::disc(r, headX, ty - 6.0F, 5.0F, gd::kOrange);
    SDL_snprintf(value, sizeof value, "t %.2f s   last dt %.1f ms", app.chainTime, app.lastStep * 1000.0);
    app.hud.text(r, tx, ty + th + 10.0F, value, gd::kMuted);

    // The code that does it.
    gd::card(r, 24.0F, 384.0F, 912.0F, 64.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, 392.0F, "updateChain(dt, move, grow, tint);", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, 38.0F, 416.0F, "update(dt) returns the time it did not use, the next tween gets it", gd::kMuted);
}

// ---------------------------------------------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------------------------------------------

void drawFrame(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    if (app.view == View::Easing) {
        char right[48];
        SDL_snprintf(right, sizeof right, "%s %s", ease::name(selectedFamily(app)), ease::name(app.variant));
        app.hud.titleBar(r, kWidth, "[ EASING ]", right);
        drawEasing(app);
    } else {
        app.hud.titleBar(r, kWidth, "[ TWEEN CHAIN ]", "Tween<T>");
        drawChain(app);
    }

    for (const Chip& chip : app.chips) {
        if (!chipVisible(app, chip.action)) continue;
        const bool on = chipActive(app, chip.action);
        gd::card(r, chip.rect.x, chip.rect.y, chip.rect.w, chip.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, centered(app, chip.label, chip.rect.x, chip.rect.w, gd::Hud::Style::Bold), chip.rect.y + 6.0F, chip.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    if (app.view == View::Easing) {
        app.hud.text(r, 530.0F, kBottomY + 6.0F, "arrows pick  Space play  Tab view", gd::kMuted);
    } else {
        app.hud.text(r, 730.0F, kBottomY + 6.0F, "Tab view", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("easing", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    restartDetail(*app);
    app->lastTicks = SDL_GetTicksNS();
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
            for (const Chip& chip : app.chips) {
                if (chipVisible(app, chip.action) && SDL_PointInRectFloat(&p, &chip.rect)) {
                    apply(app, chip.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (app.view == View::Easing) {
                for (int i = 0; i < ease::kFamilyCount; ++i) {
                    const SDL_FRect rc = cardRect(i);
                    if (SDL_PointInRectFloat(&p, &rc)) select(app, i);
                }
            } else {
                restartChain(app);
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            // Tab, 1-3, Space, R, L and D toggle or trigger and must not fire while held. The arrow keys step through
            // the gallery and may repeat.
            if (event->key.repeat && event->key.key != SDLK_LEFT && event->key.key != SDLK_RIGHT && event->key.key != SDLK_UP && event->key.key != SDLK_DOWN) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_TAB: apply(app, app.view == View::Easing ? Action::ViewChain : Action::ViewEasing); break;
                case SDLK_1: apply(app, app.view == View::Easing ? Action::VarIn : Action::StepReal); break;
                case SDLK_2: apply(app, app.view == View::Easing ? Action::VarOut : Action::StepFps20); break;
                case SDLK_3: apply(app, app.view == View::Easing ? Action::VarInOut : Action::StepFps5); break;
                case SDLK_SPACE: apply(app, app.view == View::Easing ? Action::Play : Action::Restart); break;
                case SDLK_R: apply(app, Action::Restart); break;
                case SDLK_L:
                    if (app.view == View::Easing) setMode(app, app.detail.mode == ease::Mode::Loop ? ease::Mode::Once : ease::Mode::Loop);
                    break;
                case SDLK_D:
                    if (app.view == View::Easing) {
                        const auto it = std::find(kDurations.begin(), kDurations.end(), app.detail.duration);
                        setDuration(app, static_cast<std::size_t>((it - kDurations.begin() + 1) % 3));
                    }
                    break;
                case SDLK_LEFT:
                    if (app.view == View::Easing) select(app, app.selected - 1);
                    break;
                case SDLK_RIGHT:
                    if (app.view == View::Easing) select(app, app.selected + 1);
                    break;
                case SDLK_UP:
                    if (app.view == View::Easing && app.selected >= kCols) select(app, app.selected - kCols);
                    break;
                case SDLK_DOWN:
                    if (app.view == View::Easing && app.selected + kCols < ease::kFamilyCount) select(app, app.selected + kCols);
                    break;
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
    const double dt = std::min(static_cast<double>(now - app.lastTicks) / 1e9, kMaxFrame);
    app.lastTicks = now;

    update(app, dt);
    drawFrame(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
