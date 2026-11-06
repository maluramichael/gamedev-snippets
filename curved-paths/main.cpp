// Curved roads with SDL3 main callbacks.
//
// A road needs two things from its centre line: an edge at a fixed distance on both sides (offsets) and a way to move
// at a constant speed along it (distances). Three handles define the centre line: P0, the direction handle C and P1.
// As a quadratic Bezier curve it passes through P0 and P1 and is pulled towards C. As a circular arc it starts at P0
// heading towards C and ends at P1. Orange is the easy way (shift the control points for the edges, step t evenly for
// the car and the dots), teal is the way that holds up (offset along the normal, equal steps in distance). On the
// arc both ways give the same road, which is the point. The geometry lives in logic.hpp and is covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

using cp::Vec2;

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldTop = 34.0F;
constexpr float kFieldBottom = 440.0F;
constexpr double kGrab = 14.0;        // px around a handle that grabs it
constexpr double kCarSpeed = 200.0;   // px/s of the car that drives at a constant speed
constexpr double kRest = 0.25;        // the cars wait this long at the end, as a fraction of the drive
constexpr double kMinRateLength = 40.0;
constexpr int kDots = 16;
constexpr int kDenseSegments = 256;   // reference polyline for measuring the width of an edge
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr std::array<int, 3> kWidths{40, 80, 120};
constexpr std::array<int, 3> kSegmentChoices{4, 8, 32};

constexpr gd::Color kRoad{17, 24, 39, 24};
constexpr gd::Color kInkSoft{17, 24, 39, 150};
constexpr gd::Color kGhost{107, 114, 128, 150};
constexpr gd::Color kOrangeEdge{234, 88, 12, 235};

enum class Curve { Bezier, Arc };
enum class Action { Bezier, Arc, Seg0, Seg1, Seg2, Width, Reset };

struct Chip {
    Action action;
    SDL_FRect rect;
};

struct Pose {
    Vec2 pos;
    Vec2 dir{1.0, 0.0};
};

/// Everything one frame needs, built from the handles and the options.
struct Scene {
    Curve curve = Curve::Bezier;
    bool ok = true;  // false: no arc fits
    double halfWidth = 40.0;
    double length = 0.0;
    int segments = 8;
    cp::Bezier bez;
    cp::Arc arc;
    cp::LengthTable table;
    std::vector<Vec2> centre, tealLeft, tealRight, naiveLeft, naiveRight, ghost;
    double errTeal = 0.0;
    double errNaive = 0.0;
    double lengthErrPct = 0.0;
    double speedMin = 0.0;  // px/s of the car that steps t evenly
    double speedMax = 0.0;

    /// A car that steps the curve parameter evenly. `p` runs from 0 to 1.
    Pose poseByParameter(double p, double lane) const {
        if (curve == Curve::Bezier) return {cp::bezierOffsetPoint(bez, p, lane), cp::bezierTangent(bez, p)};
        return {cp::arcOffsetPoint(arc, p, lane), cp::arcTangent(arc, p)};
    }

    /// A car that steps the distance evenly. On the arc the parameter already is the fraction of the distance, because
    /// angle = distance / radius: equal steps in the angle are equal steps along the road.
    Pose poseByDistance(double p, double lane) const {
        if (curve == Curve::Bezier) {
            const double t = cp::tAtDistance(table, p * length);
            return {cp::bezierOffsetPoint(bez, t, lane), cp::bezierTangent(bez, t)};
        }
        return {cp::arcOffsetPoint(arc, p, lane), cp::arcTangent(arc, p)};
    }
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::array<Vec2, 3> handles{};  // P0, C, P1
    Curve curve = Curve::Bezier;
    int segmentIndex = 2;
    int widthIndex = 1;
    double progress = 0.3;
    int dragging = -1;
    int hover = -1;
    Vec2 mouse{-100.0, -100.0};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

std::array<Vec2, 3> defaultHandles() { return {{{110.0, 410.0}, {340.0, 80.0}, {560.0, 410.0}}}; }

SDL_FPoint fp(Vec2 v) { return SDL_FPoint{static_cast<float>(v.x), static_cast<float>(v.y)}; }

std::string widthLabel(const App& app) { return "W Width " + std::to_string(kWidths[static_cast<std::size_t>(app.widthIndex)]); }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::Bezier, "1 Bezier", 0.0F},
                                   {Action::Arc, "2 Arc", 8.0F},
                                   {Action::Seg0, "3 N=4", 24.0F},
                                   {Action::Seg1, "4 N=8", 8.0F},
                                   {Action::Seg2, "5 N=32", 8.0F},
                                   {Action::Width, "W Width 120", 24.0F},
                                   {Action::Reset, "R Reset", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

std::string chipLabel(const App& app, Action a) {
    switch (a) {
        case Action::Bezier: return "1 Bezier";
        case Action::Arc: return "2 Arc";
        case Action::Seg0: return "3 N=" + std::to_string(kSegmentChoices[0]);
        case Action::Seg1: return "4 N=" + std::to_string(kSegmentChoices[1]);
        case Action::Seg2: return "5 N=" + std::to_string(kSegmentChoices[2]);
        case Action::Width: return widthLabel(app);
        case Action::Reset: return "R Reset";
    }
    return {};
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::Bezier: return app.curve == Curve::Bezier;
        case Action::Arc: return app.curve == Curve::Arc;
        case Action::Seg0: return app.segmentIndex == 0;
        case Action::Seg1: return app.segmentIndex == 1;
        case Action::Seg2: return app.segmentIndex == 2;
        case Action::Width:
        case Action::Reset: return false;
    }
    return false;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::Bezier: app.curve = Curve::Bezier; break;
        case Action::Arc: app.curve = Curve::Arc; break;
        case Action::Seg0: app.segmentIndex = 0; break;
        case Action::Seg1: app.segmentIndex = 1; break;
        case Action::Seg2: app.segmentIndex = 2; break;
        case Action::Width: app.widthIndex = (app.widthIndex + 1) % static_cast<int>(kWidths.size()); break;
        case Action::Reset:
            app.handles = defaultHandles();
            app.progress = 0.3;
            app.dragging = -1;
            break;
    }
}

int nearestHandle(const App& app, Vec2 p) {
    int best = -1;
    double bestDist = kGrab;
    for (int i = 0; i < 3; ++i) {
        const double d = cp::length(app.handles[static_cast<std::size_t>(i)] - p);
        if (d <= bestDist) {
            best = i;
            bestDist = d;
        }
    }
    return best;
}

Scene buildScene(const App& app) {
    Scene s;
    s.curve = app.curve;
    s.halfWidth = kWidths[static_cast<std::size_t>(app.widthIndex)] / 2.0;
    s.segments = kSegmentChoices[static_cast<std::size_t>(app.segmentIndex)];
    s.bez = {app.handles[0], app.handles[1], app.handles[2]};
    s.arc = cp::makeArc(app.handles[0], app.handles[1] - app.handles[0], app.handles[2]);
    s.table = cp::makeLengthTable(s.bez, s.segments);
    if (s.curve == Curve::Bezier) {
        s.length = s.table.total();
        const double fine = cp::makeLengthTable(s.bez, 2048).total();
        s.lengthErrPct = fine > cp::kEps ? (s.length - fine) / fine * 100.0 : 0.0;
        s.centre = cp::bezierPolyline(s.bez, kDenseSegments);
        s.tealLeft = cp::offsetPolyline(s.bez, s.segments, s.halfWidth);
        s.tealRight = cp::offsetPolyline(s.bez, s.segments, -s.halfWidth);
        s.naiveLeft = cp::bezierPolyline(cp::offsetControlPoints(s.bez, s.halfWidth), 48);
        s.naiveRight = cp::bezierPolyline(cp::offsetControlPoints(s.bez, -s.halfWidth), 48);
        s.errTeal = std::max(cp::maxWidthError(s.tealLeft, s.centre, s.halfWidth), cp::maxWidthError(s.tealRight, s.centre, s.halfWidth));
        s.errNaive = std::max(cp::maxWidthError(s.naiveLeft, s.centre, s.halfWidth), cp::maxWidthError(s.naiveRight, s.centre, s.halfWidth));
        const cp::SpeedRange range = cp::bezierSpeedRange(s.bez, 64);
        const double rate = s.length > cp::kEps ? kCarSpeed / s.length : 0.0;
        s.speedMin = range.min * rate;
        s.speedMax = range.max * rate;
        if (s.arc.valid) s.ghost = cp::arcPolyline(s.arc, 96);
    } else {
        s.ok = s.arc.valid;
        s.length = s.arc.length;
        s.centre = cp::arcPolyline(s.arc, kDenseSegments);
        s.tealLeft = cp::arcPolyline(cp::offsetArc(s.arc, s.halfWidth), 64);
        s.tealRight = cp::arcPolyline(cp::offsetArc(s.arc, -s.halfWidth), 64);
        s.errTeal = std::max(cp::maxWidthError(s.tealLeft, s.centre, s.halfWidth), cp::maxWidthError(s.tealRight, s.centre, s.halfWidth));
        s.speedMin = s.speedMax = s.length > cp::kEps ? kCarSpeed : 0.0;
        s.ghost = cp::bezierPolyline(s.bez, 96);
    }
    return s;
}

void quad(SDL_Renderer* r, Vec2 a, Vec2 b, Vec2 c, Vec2 d, gd::Color color) {
    const SDL_FColor fc = gd::fcolor(color);
    const SDL_Vertex v[4] = {{fp(a), fc, {0, 0}}, {fp(b), fc, {0, 0}}, {fp(c), fc, {0, 0}}, {fp(d), fc, {0, 0}}};
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
}

void stroke(SDL_Renderer* r, Vec2 a, Vec2 b, double width, gd::Color color) {
    const Vec2 d = b - a;
    const double len = cp::length(d);
    if (len < 1e-6) return;
    const Vec2 n = cp::leftNormal(d) * (width / (2.0 * len));
    quad(r, a + n, b + n, b - n, a - n, color);
}

void polyline(SDL_Renderer* r, const std::vector<Vec2>& pts, double width, gd::Color color) {
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) stroke(r, pts[i], pts[i + 1], width, color);
}

void roadFill(SDL_Renderer* r, const std::vector<Vec2>& left, const std::vector<Vec2>& right) {
    if (left.size() != right.size() || left.size() < 2) return;
    for (std::size_t i = 0; i + 1 < left.size(); ++i) quad(r, left[i], right[i], right[i + 1], left[i + 1], kRoad);
}

void drawCar(SDL_Renderer* r, const Pose& p, gd::Color body) {
    const Vec2 f = p.dir;
    const Vec2 s = cp::leftNormal(f);
    const Vec2 fl = p.pos + f * 11.0 + s * 6.0;
    const Vec2 fr = p.pos + f * 11.0 - s * 6.0;
    const Vec2 bl = p.pos - f * 11.0 + s * 6.0;
    const Vec2 br = p.pos - f * 11.0 - s * 6.0;
    quad(r, fl, fr, br, bl, body);
    quad(r, p.pos + f * 3.0 + s * 4.0, p.pos + f * 3.0 - s * 4.0, p.pos + f * 7.0 - s * 4.0, p.pos + f * 7.0 + s * 4.0, gd::kWhite);
    gd::setColor(r, gd::kInk);
    const SDL_FPoint pts[5] = {fp(fl), fp(fr), fp(br), fp(bl), fp(fl)};
    SDL_RenderLines(r, pts, 5);
}

void dot(SDL_Renderer* r, Vec2 p, gd::Color c) {
    gd::disc(r, static_cast<float>(p.x), static_cast<float>(p.y), 4.0F, c);
    gd::ring(r, static_cast<float>(p.x), static_cast<float>(p.y), 4.0F, gd::kInk, 16);
}

void drawScene(App& app, const Scene& s) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ CURVED PATHS ]", s.curve == Curve::Bezier ? "quadratic Bezier" : "circular arc");

    const bool bezier = s.curve == Curve::Bezier;
    const Vec2 p0 = app.handles[0];
    const Vec2 c = app.handles[1];
    const Vec2 p1 = app.handles[2];

    // Everything in the field is clipped to it: the arc's spokes can reach far below the chips.
    const SDL_Rect field{0, static_cast<int>(kFieldTop), kWidth, static_cast<int>(kFieldBottom - kFieldTop)};
    SDL_SetRenderClipRect(r, &field);

    // The road and the other curve for comparison.
    roadFill(r, s.tealLeft, s.tealRight);
    polyline(r, s.ghost, 1.5, kGhost);
    if (bezier) {
        gd::setColor(r, gd::Color{107, 114, 128, 140});
        SDL_RenderLine(r, static_cast<float>(p0.x), static_cast<float>(p0.y), static_cast<float>(c.x), static_cast<float>(c.y));
        SDL_RenderLine(r, static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(p1.x), static_cast<float>(p1.y));
    } else if (s.ok && !s.arc.line && s.arc.radius < 3000.0) {
        gd::setColor(r, gd::Color{107, 114, 128, 140});
        SDL_RenderLine(r, static_cast<float>(s.arc.center.x), static_cast<float>(s.arc.center.y), static_cast<float>(p0.x), static_cast<float>(p0.y));
        SDL_RenderLine(r, static_cast<float>(s.arc.center.x), static_cast<float>(s.arc.center.y), static_cast<float>(p1.x), static_cast<float>(p1.y));
        gd::ring(r, static_cast<float>(s.arc.center.x), static_cast<float>(s.arc.center.y), 4.0F, gd::kMuted, 16);
    } else {
        gd::setColor(r, gd::Color{107, 114, 128, 140});
        SDL_RenderLine(r, static_cast<float>(p0.x), static_cast<float>(p0.y), static_cast<float>(c.x), static_cast<float>(c.y));
    }

    // Edges: orange is the control-point offset, teal the exact one.
    polyline(r, s.naiveLeft, 2.5, kOrangeEdge);
    polyline(r, s.naiveRight, 2.5, kOrangeEdge);
    polyline(r, s.tealLeft, 3.0, gd::kTeal);
    polyline(r, s.tealRight, 3.0, gd::kTeal);
    polyline(r, s.centre, 1.5, kInkSoft);

    // Dots: orange at equal steps of the parameter in the left lane, teal at equal steps of distance in the right lane.
    if (s.ok) {
        const double lane = s.halfWidth / 2.0;
        for (int i = 0; i <= kDots; ++i) {
            const double p = static_cast<double>(i) / kDots;
            dot(r, s.poseByParameter(p, lane).pos, gd::kOrange);
            dot(r, s.poseByDistance(p, -lane).pos, gd::kTeal);
        }
        const double p = std::min(app.progress, 1.0);
        drawCar(r, s.poseByParameter(p, lane), gd::kOrange);
        drawCar(r, s.poseByDistance(p, -lane), gd::kTeal);
    }

    SDL_SetRenderClipRect(r, nullptr);

    // Panel.
    gd::card(r, 640.0F, 42.0F, 296.0F, 172.0F, gd::kWhite, gd::kInk);
    constexpr float kX = 652.0F;
    float y = 50.0F;
    char line[64];
    const auto row = [&](const char* text, gd::Color color, gd::Hud::Style style = gd::Hud::Style::Regular) {
        app.hud.text(r, kX, y, text, color, style);
        y += 22.0F;
    };
    if (bezier) {
        SDL_snprintf(line, sizeof line, "BEZIER  length %.0f px", s.length);
        row(line, gd::kInk, gd::Hud::Style::Bold);
        SDL_snprintf(line, sizeof line, "N=%d segments: %+.1f%%", s.segments, std::fabs(s.lengthErrPct) < 0.05 ? 0.0 : s.lengthErrPct);
        row(line, gd::kMuted);
        row("width error, max in px", gd::kMuted);
        SDL_snprintf(line, sizeof line, " ctrl-point edge %5.1f", s.errNaive);
        row(line, gd::kOrange, gd::Hud::Style::Bold);
        SDL_snprintf(line, sizeof line, " normal edge     %5.1f", s.errTeal);
        row(line, gd::kTealDark, gd::Hud::Style::Bold);
    } else if (s.ok) {
        SDL_snprintf(line, sizeof line, "ARC  r %.0f  turn %.0f°", s.arc.line ? 0.0 : s.arc.radius, std::fabs(s.arc.sweep) * 180.0 / cp::kPi);
        row(line, gd::kInk, gd::Hud::Style::Bold);
        SDL_snprintf(line, sizeof line, "length = r × angle = %.0f", s.length);
        row(line, gd::kMuted);
        row("width error, max in px", gd::kMuted);
        SDL_snprintf(line, sizeof line, " arc edges       %5.1f", s.errTeal);
        row(line, gd::kTealDark, gd::Hud::Style::Bold);
        y += 22.0F;
    } else {
        row("ARC", gd::kInk, gd::Hud::Style::Bold);
        row("no arc: P1 is behind P0", gd::kOrange, gd::Hud::Style::Bold);
        y += 66.0F;
    }
    if (s.ok) {
        if (s.speedMax - s.speedMin < 0.5) {
            SDL_snprintf(line, sizeof line, "t-step car %.0f px/s", s.speedMin);
        } else {
            SDL_snprintf(line, sizeof line, "t-step car %.0f..%.0f px/s", s.speedMin, s.speedMax);
        }
        row(line, gd::kOrange, gd::Hud::Style::Bold);
        SDL_snprintf(line, sizeof line, "dist car   %.0f px/s", kCarSpeed);
        row(line, gd::kTealDark, gd::Hud::Style::Bold);
    }

    // Handles last, so none of them can hide under the panel.
    const std::array<const char*, 3> names{"P0", bezier ? "C" : "dir", "P1"};
    for (int i = 0; i < 3; ++i) {
        const Vec2 h = app.handles[static_cast<std::size_t>(i)];
        const bool hot = app.dragging == i || (app.dragging < 0 && app.hover == i);
        gd::disc(r, static_cast<float>(h.x), static_cast<float>(h.y), 8.0F, hot ? gd::kTeal : gd::kWhite);
        gd::ring(r, static_cast<float>(h.x), static_cast<float>(h.y), 8.0F, gd::kInk);
        gd::ring(r, static_cast<float>(h.x), static_cast<float>(h.y), 9.0F, gd::kInk);
        // Labels sit outside the road: below the end points, above the direction handle.
        const float lx = i == 0 ? -38.0F : 12.0F;
        const float ly = i == 1 ? -28.0F : 12.0F;
        app.hud.text(r, static_cast<float>(h.x) + lx, static_cast<float>(h.y) + ly, names[static_cast<std::size_t>(i)], gd::kInk, gd::Hud::Style::Bold);
    }

    app.hud.text(r, 40.0F, 446.0F, "drag P0, C, P1    N = segments for Bezier length and normal edge", gd::kMuted);

    for (const Chip& chip : app.chips) {
        const bool on = active(app, chip.action);
        const std::string label = chipLabel(app, chip.action);
        gd::card(r, chip.rect.x, chip.rect.y, chip.rect.w, chip.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, chip.rect.x + (chip.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, chip.rect.y + 6.0F, label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
    }
}

}  // namespace

SDL_AppResult SDL_AppInit(void** appstate, int argc, char** argv) {
    auto* app = new App();
    *appstate = app;
    app->handles = defaultHandles();
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) app->maxFrames = std::atol(argv[i + 1]);
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("curved-paths", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            if (app.dragging >= 0) {
                app.handles[static_cast<std::size_t>(app.dragging)] = {std::clamp(app.mouse.x, 12.0, 948.0),
                                                                       std::clamp(app.mouse.y, static_cast<double>(kFieldTop) + 10.0, static_cast<double>(kFieldBottom) - 30.0)};
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = {static_cast<double>(p.x), static_cast<double>(p.y)};
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                }
            }
            if (!onChip && app.mouse.y >= static_cast<double>(kFieldTop) && app.mouse.y <= static_cast<double>(kFieldBottom)) app.dragging = nearestHandle(app, app.mouse);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = -1;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging = -1; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Bezier); break;
                case SDLK_2: apply(app, Action::Arc); break;
                case SDLK_3: apply(app, Action::Seg0); break;
                case SDLK_4: apply(app, Action::Seg1); break;
                case SDLK_5: apply(app, Action::Seg2); break;
                case SDLK_W: apply(app, Action::Width); break;
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
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(static_cast<double>(now - app.lastTicks) / 1e9, 0.25);
    app.lastTicks = now;

    app.hover = nearestHandle(app, app.mouse);
    const Scene scene = buildScene(app);
    if (scene.ok && scene.length > cp::kEps) {
        // The cars drive the whole road in length / speed seconds and wait a moment at the end. Roads shorter than
        // kMinRateLength drive at the rate of that length, so a tiny road does not flicker.
        app.progress = std::fmod(app.progress + kCarSpeed * dt / std::max(scene.length, kMinRateLength), 1.0 + kRest);
    }
    drawScene(app, scene);
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
