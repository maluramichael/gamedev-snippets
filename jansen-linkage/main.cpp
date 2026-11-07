// Jansen's linkage (Theo Jansen's Strandbeest leg) with SDL3 main callbacks.
//
// One crank turns, nothing else is driven. Every other joint is the intersection of two circles whose radii are rod
// lengths, so the whole foot path is hidden in eleven numbers. The rod chips pick a rod, the slider stretches or shrinks
// it by up to 20 percent, the faint path is the original one. Some rods tilt the path, some make the crank rock back and
// forth because a pair of circles stops meeting. "Sides" switches between the intersection that was remembered from the
// start pose (the leg Jansen drew) and simply taking the first one at every joint, which assembles into a different leg.
// The math lives in logic.hpp and is covered by test.cpp.

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
constexpr float kFieldBottom = 440.0F;
constexpr float kFieldRight = 600.0F;
constexpr float kScale = 2.4F;  // px per rod unit: every joint of every one-rod change stays in the field (x -87..56, y -95..70)
constexpr float kOriginX = 300.0F;
constexpr float kOriginY = 207.0F;
constexpr float kPanelX = 612.0F;
constexpr float kPanelY = 44.0F;
constexpr float kPanelW = 324.0F;
constexpr float kPanelH = 392.0F;
constexpr float kSliderX = 636.0F;
constexpr float kSliderW = 276.0F;
constexpr float kSliderY = 238.0F;
constexpr float kRowY = 450.0F;
constexpr float kRowH = 30.0F;
constexpr float kRow2Y = 492.0F;
constexpr float kRow2H = 34.0F;
constexpr double kOmega = 1.6;  // rad/s, one turn in about four seconds
constexpr double kForward = -1.0;  // clockwise: the foot moves backwards along the ground, the body walks to the right
constexpr double kStepKey = 0.035;  // radians per scrub key press, 2 degrees
constexpr double kMaxDt = 0.25;
constexpr int kPathSamples = 360;
constexpr double kContactBand = 1.0;  // rod units above the lowest point that still count as "on the ground"
constexpr int kMaxPercent = 20;
constexpr gd::Color kBand{13, 148, 136, 40};
constexpr gd::Color kHighlight{13, 148, 136, 110};
constexpr gd::Color kPathOrange{234, 88, 12, 230};

enum class Action { SelectRod, Pause, Sides, Reset };

struct Chip {
    Action action;
    int value;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    jl::Scale scale = jl::unitScale();
    int selected = 3;  // rod e
    bool naive = false;
    bool paused = false;
    bool sliderDrag = false;
    double angle = jl::referenceAngle();
    double dir = kForward;
    jl::Lengths lengths = jl::standardLengths();
    jl::Sides sides = jl::referenceSides();
    jl::Path path;
    jl::Path ghost;  // the original rods with the remembered sides
    std::optional<double> ground;
    std::optional<double> contact;
    double pathWidth = 0.0;
    double pathHeight = 0.0;
    bool reachable = true;
    jl::Pose pose = jl::referencePose();
    Uint64 lastNs = 0;
    long maxFrames = -1;
    long frames = 0;
};

SDL_FPoint toScreen(jl::Vec2 p) { return {kOriginX + static_cast<float>(p.x) * kScale, kOriginY - static_cast<float>(p.y) * kScale}; }

int percentOf(const App& app, int rod) { return static_cast<int>(std::lround((app.scale[static_cast<std::size_t>(rod)] - 1.0) * 100.0)); }

bool modified(const App& app) {
    return std::any_of(app.scale.begin(), app.scale.end(), [](double s) { return s != 1.0; });
}

void refreshPose(App& app) {
    const jl::Solved s = jl::solve(app.lengths, app.sides, app.angle);
    if (s.ok) app.pose = s.pose;
}

/// Everything that depends on the rods or on the sides: the path, its numbers, and a crank angle that still assembles.
void recompute(App& app) {
    app.lengths = jl::scaled(app.scale);
    app.sides = app.naive ? jl::naiveSides() : jl::referenceSides();
    app.path = jl::footPath(app.lengths, app.sides, kPathSamples);
    app.ground = jl::groundLevel(app.path);
    app.contact = jl::groundContact(app.path, kContactBand);
    double minX = 1e9;
    double maxX = -1e9;
    double minY = 1e9;
    double maxY = -1e9;
    for (const auto& p : app.path) {
        if (!p) continue;
        minX = std::min(minX, p->x);
        maxX = std::max(maxX, p->x);
        minY = std::min(minY, p->y);
        maxY = std::max(maxY, p->y);
    }
    app.pathWidth = app.ground ? maxX - minX : 0.0;
    app.pathHeight = app.ground ? maxY - minY : 0.0;
    // After a change the crank may sit where the new rods cannot assemble: roll it to the nearest angle that works.
    const auto angle = jl::nearestReachable(app.lengths, app.sides, app.angle);
    app.reachable = angle.has_value();
    if (angle) {
        app.angle = *angle;
        refreshPose(app);
    }
}

void setPercent(App& app, int percent) {
    const double s = 1.0 + static_cast<double>(std::clamp(percent, -kMaxPercent, kMaxPercent)) / 100.0;
    double& slot = app.scale[static_cast<std::size_t>(app.selected)];
    if (slot == s) return;
    slot = s;
    recompute(app);
}

void selectRod(App& app, int rod) {
    app.selected = ((rod % jl::kRodCount) + jl::kRodCount) % jl::kRodCount;
    app.sliderDrag = false;
}

void apply(App& app, Action action, int value) {
    switch (action) {
        case Action::SelectRod: selectRod(app, value); break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Sides:
            app.naive = !app.naive;
            app.dir = kForward;
            recompute(app);
            break;
        case Action::Reset:
            app.scale = jl::unitScale();
            app.dir = kForward;
            app.sliderDrag = false;
            recompute(app);
            break;
    }
}

bool active(const App& app, const Chip& chip) {
    switch (chip.action) {
        case Action::SelectRod: return app.selected == chip.value;
        case Action::Pause: return app.paused;
        case Action::Sides: return true;
        case Action::Reset: return false;
    }
    return false;
}

void layoutChips(App& app) {
    float x = 78.0F;
    for (std::size_t i = 0; i < jl::kLinks.size(); ++i) {
        app.chips.push_back({Action::SelectRod, static_cast<int>(i), std::string(1, jl::kLinks[i].name), SDL_FRect{x, kRowY, 34.0F, kRowH}});
        x += 40.0F;
    }
    x = 40.0F;
    app.chips.push_back({Action::Pause, 0, "", SDL_FRect{x, kRow2Y, 130.0F, kRow2H}});
    x += 130.0F + 12.0F;
    app.chips.push_back({Action::Sides, 0, "", SDL_FRect{x, kRow2Y, 226.0F, kRow2H}});
    x += 226.0F + 12.0F;
    app.chips.push_back({Action::Reset, 0, "R Reset rods", SDL_FRect{x, kRow2Y, 144.0F, kRow2H}});
}

std::string chipLabel(const App& app, const Chip& chip) {
    switch (chip.action) {
        case Action::Pause: return app.paused ? "Space Play" : "Space Pause";
        case Action::Sides: return app.naive ? "N Sides: first hit" : "N Sides: remembered";
        default: return chip.label;
    }
}

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side, offset along the normal.
void thickLine(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, float half = 1.0F) {
    gd::setColor(r, c);
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::hypot(dx, dy);
    if (len < 1e-3F) return;
    const float nx = -dy / len;
    const float ny = dx / len;
    for (float o = -half; o <= half + 0.01F; o += 1.0F) SDL_RenderLine(r, a.x + nx * o, a.y + ny * o, b.x + nx * o, b.y + ny * o);
}

void dashedLine(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, float dash = 8.0F, float gap = 6.0F) {
    const float len = std::hypot(b.x - a.x, b.y - a.y);
    if (len < 1e-3F) return;
    const float ux = (b.x - a.x) / len;
    const float uy = (b.y - a.y) / len;
    for (float t = 0.0F; t < len; t += dash + gap) {
        const float e = std::min(t + dash, len);
        thickLine(r, {a.x + ux * t, a.y + uy * t}, {a.x + ux * e, a.y + uy * e}, c, 0.0F);
    }
}

void drawPath(App& app, const jl::Path& path, gd::Color c, float half) {
    const std::size_t n = path.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& p = path[i];
        const auto& q = path[(i + 1) % n];
        if (p && q) thickLine(app.renderer, toScreen(*p), toScreen(*q), c, half);
    }
}

void drawField(App& app) {
    SDL_Renderer* r = app.renderer;
    const SDL_Rect clip{0, static_cast<int>(kFieldTop), static_cast<int>(kFieldRight), static_cast<int>(kFieldBottom - kFieldTop)};
    SDL_SetRenderClipRect(r, &clip);

    // The ground: the lowest point of the current path, with the band that counts as contact.
    if (app.ground) {
        const SDL_FPoint g = toScreen({0.0, *app.ground});
        gd::fillRect(r, 24.0F, g.y - static_cast<float>(kContactBand) * kScale, kFieldRight - 40.0F, static_cast<float>(kContactBand) * kScale, kBand);
        dashedLine(r, {24.0F, g.y}, {kFieldRight - 16.0F, g.y}, gd::kMuted);
        app.hud.text(r, 30.0F, g.y - 26.0F, "ground", gd::kMuted);
    }

    if (!app.reachable) {
        const char* msg = "LINKAGE CANNOT ASSEMBLE";
        app.hud.text(r, 40.0F, 200.0F, msg, gd::kOrange, gd::Hud::Style::Bold);
        app.hud.text(r, 40.0F, 224.0F, "no crank angle works with these rods", gd::kMuted);
        SDL_SetRenderClipRect(r, nullptr);
        return;
    }

    // The frame: pivot and crank center are fixed to the body.
    const SDL_FPoint pivot = toScreen(app.pose.pivot);
    const SDL_FPoint hub = toScreen(app.pose.hub);
    dashedLine(r, pivot, {hub.x, pivot.y}, gd::kMuted, 4.0F, 4.0F);
    dashedLine(r, {hub.x, pivot.y}, hub, gd::kMuted, 4.0F, 4.0F);

    const gd::Color pathColor = app.naive ? kPathOrange : gd::kTeal;
    if (modified(app) || app.naive) drawPath(app, app.ghost, gd::Color{107, 114, 128, 150}, 0.0F);
    drawPath(app, app.path, pathColor, 1.0F);
    if (app.ground) {
        for (const auto& p : app.path) {
            if (p && p->y < *app.ground + kContactBand) {
                const SDL_FPoint s = toScreen(*p);
                gd::disc(r, s.x, s.y, 2.5F, pathColor, 12);
            }
        }
    }

    // Rods. The selected one gets a teal halo, the crank is orange because it is the one thing that is driven.
    const jl::Pose& pose = app.pose;
    for (std::size_t i = 0; i < jl::kLinks.size(); ++i) {
        const jl::Link& link = jl::kLinks[i];
        const SDL_FPoint a = toScreen(pose.*link.from);
        const SDL_FPoint b = toScreen(pose.*link.to);
        if (static_cast<int>(i) == app.selected) thickLine(r, a, b, kHighlight, 5.0F);
        thickLine(r, a, b, link.name == 'm' ? gd::kOrange : gd::kInk, 1.0F);
    }
    for (std::size_t i = 0; i < jl::kLinks.size(); ++i) {
        const jl::Link& link = jl::kLinks[i];
        const SDL_FPoint a = toScreen(pose.*link.from);
        const SDL_FPoint b = toScreen(pose.*link.to);
        float mx = (a.x + b.x) / 2.0F;
        float my = (a.y + b.y) / 2.0F;
        if (link.name == 'm') {  // the crank is short, keep its letter off the rod
            const float len = std::max(std::hypot(b.x - a.x, b.y - a.y), 1e-3F);
            mx += -(b.y - a.y) / len * 14.0F;
            my += (b.x - a.x) / len * 14.0F;
        }
        const bool sel = static_cast<int>(i) == app.selected;
        gd::fillRect(r, mx - 8.0F, my - 10.0F, 16.0F, 20.0F, gd::kPaper);
        const char label[2] = {link.name, '\0'};
        app.hud.text(r, mx - 5.0F, my - 11.0F, label, sel ? gd::kTealDark : link.name == 'm' ? gd::kOrange : gd::kMuted, gd::Hud::Style::Bold);
    }

    // Joints. In "first hit" mode a joint that sits on the other intersection than in Jansen's leg gets an orange ring.
    const std::array<jl::Vec2, 5> joints{pose.b, pose.c, pose.d, pose.e, pose.foot};
    const jl::Sides good = jl::referenceSides();
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const SDL_FPoint s = toScreen(joints[i]);
        const bool foot = i == 4;
        gd::disc(r, s.x, s.y, foot ? 8.0F : 5.0F, foot ? pathColor : gd::kWhite);
        gd::ring(r, s.x, s.y, foot ? 8.0F : 5.0F, gd::kInk);
        if (app.sides[i] != good[i]) gd::ring(r, s.x, s.y, foot ? 12.0F : 10.0F, gd::kOrange);
    }
    const SDL_FPoint tip = toScreen(pose.crank);
    gd::disc(r, tip.x, tip.y, 5.0F, gd::kOrange);
    gd::ring(r, tip.x, tip.y, 5.0F, gd::kInk);
    for (const SDL_FPoint fixed : {pivot, hub}) {
        gd::fillRect(r, fixed.x - 6.0F, fixed.y - 6.0F, 12.0F, 12.0F, gd::kInk);
        gd::fillRect(r, fixed.x - 3.0F, fixed.y - 3.0F, 6.0F, 6.0F, gd::kPaper);
    }
    SDL_SetRenderClipRect(r, nullptr);
}

void row(App& app, float y, const char* label, const std::string& value, gd::Color c = gd::kInk) {
    app.hud.text(app.renderer, kPanelX + 14.0F, y, label, gd::kMuted);
    app.hud.text(app.renderer, kPanelX + kPanelW - 14.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, c, gd::Hud::Style::Bold);
}

std::string formatF(const char* fmt, double a, double b = 0.0) {
    char buf[64];
    SDL_snprintf(buf, sizeof buf, fmt, a, b);
    return buf;
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, kPanelY, kPanelW, kPanelH, gd::kWhite, gd::kInk);

    const int degrees = static_cast<int>(std::lround(jl::wrapAngle(app.angle) * 180.0 / (jl::kTau / 2.0))) % 360;
    row(app, 54.0F, "crank angle", formatF("%.0f°", static_cast<double>(degrees)));
    const double err = app.reachable ? jl::maxRodError(app.lengths, app.pose) : 0.0;
    row(app, 78.0F, "rod error", app.reachable ? formatF("%.0e", err) : "-");
    row(app, 102.0F, "foot path", app.ground ? formatF("%.0f × %.0f", app.pathWidth, app.pathHeight) : "-");
    row(app, 126.0F, "on ground", app.contact ? formatF("%.0f%%", std::round(*app.contact * 100.0)) : "-");
    row(app, 150.0F, "sides", app.naive ? "first hit" : "remembered", app.naive ? gd::kOrange : gd::kTealDark);
    gd::fillRect(r, kPanelX + 14.0F, 180.0F, kPanelW - 28.0F, 2.0F, gd::kGrid);

    const jl::Link& link = jl::kLinks[static_cast<std::size_t>(app.selected)];
    const int pct = percentOf(app, app.selected);
    const double base = jl::standardLengths().*link.length;
    const double now = app.lengths.*link.length;
    const std::string rod = std::string("rod ") + link.name;
    app.hud.text(r, kPanelX + 14.0F, 190.0F, rod, gd::kInk, gd::Hud::Style::Bold);
    const std::string value = formatF("%.1f", base) + " -> " + formatF("%.1f", now) + (pct == 0 ? "" : formatF(" (%+.0f%%)", static_cast<double>(pct)));
    app.hud.text(r, kPanelX + kPanelW - 14.0F - app.hud.width(value), 190.0F, value, pct == 0 ? gd::kMuted : gd::kOrange);

    // Slider with a mark at 0 %.
    gd::fillRect(r, kSliderX, kSliderY - 3.0F, kSliderW, 6.0F, gd::kGrid);
    gd::strokeRect(r, kSliderX, kSliderY - 3.0F, kSliderW, 6.0F, gd::kInk);
    gd::fillRect(r, kSliderX + kSliderW / 2.0F - 1.0F, kSliderY - 10.0F, 2.0F, 20.0F, gd::kInk);
    const float hx = kSliderX + kSliderW * static_cast<float>(pct + kMaxPercent) / static_cast<float>(2 * kMaxPercent);
    gd::card(r, hx - 7.0F, kSliderY - 13.0F, 14.0F, 26.0F, app.sliderDrag ? gd::kTeal : gd::kWhite, gd::kInk);
    app.hud.text(r, kSliderX - 4.0F, kSliderY + 18.0F, "-20%", gd::kMuted);
    app.hud.text(r, kSliderX + kSliderW / 2.0F - 5.0F, kSliderY + 18.0F, "0", gd::kMuted);
    app.hud.text(r, kSliderX + kSliderW - app.hud.width("+20%") + 4.0F, kSliderY + 18.0F, "+20%", gd::kMuted);

    float y = 290.0F;
    if (!app.reachable) {
        app.hud.text(r, kPanelX + 14.0F, y, "CANNOT ASSEMBLE", gd::kOrange, gd::Hud::Style::Bold);
        y += 24.0F;
    } else if (!app.contact) {
        app.hud.text(r, kPanelX + 14.0F, y, "NO FULL TURN", gd::kOrange, gd::Hud::Style::Bold);
        app.hud.text(r, kPanelX + 14.0F, y + 22.0F, "the crank rocks back and forth", gd::kMuted);
        y += 50.0F;
    }
    if (app.naive) {
        const jl::Sides good = jl::referenceSides();
        std::string wrong = "wrong side:";
        const char* names[] = {"B", "C", "D", "E", "F"};
        for (std::size_t i = 0; i < good.size(); ++i) {
            if (app.sides[i] != good[i]) wrong += std::string(" ") + names[i];
        }
        app.hud.text(r, kPanelX + 14.0F, y, wrong, gd::kOrange, gd::Hud::Style::Bold);
    }

    app.hud.text(r, kPanelX + 14.0F, 358.0F, "Left/Right  rod length 1%", gd::kMuted);
    app.hud.text(r, kPanelX + 14.0F, 380.0F, "Up/Down     pick a rod", gd::kMuted);
    app.hud.text(r, kPanelX + 14.0F, 402.0F, ", .         step the crank", gd::kMuted);
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, 40.0F, kRowY + 4.0F, "rod", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const bool orange = c.action == Action::Sides && app.naive;
        const gd::Color fill = orange ? gd::kOrange : on ? gd::kTeal : gd::kWhite;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        const std::string label = chipLabel(app, c);
        const float ty = c.rect.y + (c.rect.h - app.hud.lineHeight(gd::Hud::Style::Bold)) / 2.0F;
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, ty, label, on || orange ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
        if (c.action == Action::SelectRod && percentOf(app, c.value) != 0) gd::disc(r, c.rect.x + c.rect.w - 3.0F, c.rect.y + 3.0F, 4.0F, gd::kOrange, 12);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ JANSEN LINKAGE ]", "one motor, no joint angles");
    drawField(app);
    drawPanel(app);
    drawChips(app);
}

void update(App& app, double dt) {
    if (app.paused || !app.reachable) return;
    if (!jl::advance(app.lengths, app.sides, app.angle, app.dir * kOmega * dt)) app.dir = -app.dir;  // out of reach: rock back
    refreshPose(app);
}

/// Scrubbing pauses, and runs the crank by hand a few degrees. Where it cannot go on it simply stays.
void scrub(App& app, double delta) {
    app.paused = true;
    if (!app.reachable) return;
    jl::advance(app.lengths, app.sides, app.angle, delta);
    refreshPose(app);
}

void sliderTo(App& app, float mouseX) {
    const double t = (static_cast<double>(mouseX) - static_cast<double>(kSliderX)) / static_cast<double>(kSliderW);
    setPercent(app, static_cast<int>(std::lround(t * 2.0 * kMaxPercent - kMaxPercent)));
}

bool onSlider(float x, float y) { return x >= kSliderX - 12.0F && x <= kSliderX + kSliderW + 12.0F && y >= kSliderY - 18.0F && y <= kSliderY + 18.0F; }

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
    if (!SDL_CreateWindowAndRenderer("jansen-linkage", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    app->ghost = jl::footPath(jl::standardLengths(), jl::referenceSides(), kPathSamples);
    recompute(*app);
    app->lastNs = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.sliderDrag) sliderTo(app, event->motion.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action, c.value);
                    return SDL_APP_CONTINUE;
                }
            }
            if (onSlider(p.x, p.y)) {
                app.sliderDrag = true;
                sliderTo(app, p.x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.sliderDrag = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.sliderDrag = false; break;
        case SDL_EVENT_KEY_DOWN:
            // Holding these is the point: they repeat.
            switch (event->key.key) {
                case SDLK_LEFT: setPercent(app, percentOf(app, app.selected) - 1); return SDL_APP_CONTINUE;
                case SDLK_RIGHT: setPercent(app, percentOf(app, app.selected) + 1); return SDL_APP_CONTINUE;
                case SDLK_COMMA: scrub(app, -kForward * kStepKey); return SDL_APP_CONTINUE;
                case SDLK_PERIOD: scrub(app, kForward * kStepKey); return SDL_APP_CONTINUE;
                default: break;
            }
            if (event->key.repeat) break;  // everything below toggles or cycles, none of it should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_SPACE: apply(app, Action::Pause, 0); break;
                case SDLK_N: apply(app, Action::Sides, 0); break;
                case SDLK_R: apply(app, Action::Reset, 0); break;
                case SDLK_UP: selectRod(app, app.selected - 1); break;
                case SDLK_DOWN: selectRod(app, app.selected + 1); break;
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
    const double dt = std::min(static_cast<double>(now - app.lastNs) / 1e9, kMaxDt);
    app.lastNs = now;
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
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
