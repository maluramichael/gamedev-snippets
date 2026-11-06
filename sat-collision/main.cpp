// Separating axis theorem with SDL3 main callbacks.
//
// Two convex shapes you can drag and turn. Every edge normal is an axis; the selected axis is drawn as a line with the
// shadow of each shape on it (teal bar for A, orange bar for B). A gap between the bars on any axis proves the shapes
// do not touch. If every axis shows an overlap they collide, and the smallest one is the shortest way out: the arrow is
// that minimum translation vector (MTV), the dashed outline is B after the push.
//
// The two chips Normalize and Orient switch the usual mistakes on: raw axes as long as the edge, and an MTV that keeps
// the direction of the axis instead of the side B has to leave through. Both still answer "collision, yes or no"
// correctly, which is why they go unnoticed. The geometry lives in logic.hpp and is covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr double kScale = 40.0;  // px per meter
constexpr double kFieldTop = 34.0;
constexpr double kFieldBottom = 430.0;
constexpr float kChipH = 34.0F;
constexpr float kRowH = 24.0F;
constexpr float kAxisPanelX = 708.0F;
constexpr float kAxisPanelY = 44.0F;
constexpr float kAxisPanelW = 228.0F;
constexpr float kAxisRowsTop = 56.0F;  // first row, below the panel heading
constexpr double kTurnStep = 0.06;   // rad per Q/E press
constexpr double kWheelStep = 0.08;  // rad per wheel notch
constexpr gd::Color kInkSoft{17, 24, 39, 150};
constexpr gd::Color kTealFill{13, 148, 136, 70};
constexpr gd::Color kOrangeFill{234, 88, 12, 70};

enum class Action { Preset0, Preset1, Preset2, Reset, Normalize, Orient, AutoAxis, NextAxis, Push };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    int preset = 0;
    std::array<sat::Shape, 2> shape{};  // 0 = A, 1 = B
    sat::Options options;
    int selected = 1;      // the shape Q/E and the wheel turn
    int dragging = -1;     // index into `shape`, -1 = none
    sat::Vec2 grabOffset{0.0, 0.0};
    int axisPick = -1;     // -1 = the axis with the smallest overlap, else the index in the axis list
    sat::Vec2 mouse{-100.0, -100.0};  // px
    std::size_t axisCount = 0;        // of the last frame, for clicks on the list
    long maxFrames = -1;
    long frames = 0;
};

sat::Vec2 px(sat::Vec2 meters) { return meters * kScale; }

sat::Polygon pxPolygon(const sat::Polygon& meters) {
    sat::Polygon out;
    out.reserve(meters.size());
    for (const sat::Vec2& v : meters) out.push_back(px(v));
    return out;
}

SDL_FPoint fp(sat::Vec2 v) { return SDL_FPoint{static_cast<float>(v.x), static_cast<float>(v.y)}; }

sat::Polygon world(const App& app, int i) { return sat::worldVertices(app.shape[static_cast<std::size_t>(i)]); }

void loadPreset(App& app, int index) {
    const sat::Scene sc = sat::preset(index);
    app.preset = index;
    app.shape = {sc.a, sc.b};
    app.dragging = -1;
    app.axisPick = -1;
}

sat::Vec2 clampToWorld(sat::Vec2 p) {
    return {std::clamp(p.x, 0.4, static_cast<double>(kWidth) / kScale - 0.4), std::clamp(p.y, kFieldTop / kScale + 0.8, kFieldBottom / kScale - 0.4)};
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float row;
        float gapBefore;
    };
    const std::array<Def, 9> defs{{{Action::Preset0, "1 Box + pentagon", 0, 0.0F},
                                   {Action::Preset1, "2 Triangle + hexagon", 0, 8.0F},
                                   {Action::Preset2, "3 Two boxes", 0, 8.0F},
                                   {Action::Reset, "R Reset", 0, 24.0F},
                                   {Action::Normalize, "N Normalize axes", 1, 0.0F},
                                   {Action::Orient, "O Orient MTV", 1, 8.0F},
                                   {Action::AutoAxis, "A Auto axis", 1, 24.0F},
                                   {Action::NextAxis, "< > Axis", 1, 8.0F},
                                   {Action::Push, "Space Push B out", 1, 24.0F}}};
    float x[2] = {40.0F, 40.0F};
    for (const Def& d : defs) {
        const auto row = static_cast<std::size_t>(d.row);
        x[row] += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x[row], 458.0F + d.row * 40.0F, w, kChipH}});
        x[row] += w;
    }
}

/// Axis list in the order of the shapes' edges, the selection is the index into it.
std::size_t shownAxis(const App& app, const sat::Result& res) {
    if (res.axes.empty()) return 0;
    return app.axisPick >= 0 ? std::min(static_cast<std::size_t>(app.axisPick), res.axes.size() - 1) : res.best;
}

void stepAxis(App& app, int direction) {
    const sat::Result res = sat::collide(world(app, 0), world(app, 1), app.options);
    if (res.axes.empty()) return;
    const auto n = static_cast<int>(res.axes.size());
    const int now = static_cast<int>(shownAxis(app, res));
    app.axisPick = (now + direction + n) % n;
}

void push(App& app) {
    const sat::Result res = sat::collide(world(app, 0), world(app, 1), app.options);
    if (!res.colliding) return;
    sat::Shape& b = app.shape[1];
    b.pos = clampToWorld(b.pos + res.mtv);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Preset0: loadPreset(app, 0); break;
        case Action::Preset1: loadPreset(app, 1); break;
        case Action::Preset2: loadPreset(app, 2); break;
        case Action::Reset: loadPreset(app, app.preset); break;
        case Action::Normalize: app.options.normalize = !app.options.normalize; break;
        case Action::Orient: app.options.orient = !app.options.orient; break;
        case Action::AutoAxis: app.axisPick = -1; break;
        case Action::NextAxis: stepAxis(app, 1); break;
        case Action::Push: push(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Preset0: return app.preset == 0;
        case Action::Preset1: return app.preset == 1;
        case Action::Preset2: return app.preset == 2;
        case Action::Normalize: return app.options.normalize;
        case Action::Orient: return app.options.orient;
        case Action::AutoAxis: return app.axisPick < 0;
        default: return false;
    }
}

/// True for the two switches whose "off" state is the mistake: they turn orange instead of white.
bool isMistakeSwitch(Action action) { return action == Action::Normalize || action == Action::Orient; }

bool clipLine(sat::Vec2& a, sat::Vec2& b, double x0, double y0, double x1, double y1) {
    double t0 = 0.0;
    double t1 = 1.0;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const auto edge = [&](double p, double q) {
        if (p == 0.0) return q >= 0.0;
        const double t = q / p;
        if (p < 0.0) {
            if (t > t1) return false;
            t0 = std::max(t0, t);
        } else {
            if (t < t0) return false;
            t1 = std::min(t1, t);
        }
        return true;
    };
    if (!edge(-dx, a.x - x0) || !edge(dx, x1 - a.x) || !edge(-dy, a.y - y0) || !edge(dy, y1 - a.y)) return false;
    const sat::Vec2 na{a.x + t0 * dx, a.y + t0 * dy};
    const sat::Vec2 nb{a.x + t1 * dx, a.y + t1 * dy};
    a = na;
    b = nb;
    return true;
}

/// A line of the given width, clipped to the field so nothing reaches into the panels or chips (the raw MTV can be huge).
void segment(SDL_Renderer* r, sat::Vec2 a, sat::Vec2 b, double width, gd::Color c) {
    if (!clipLine(a, b, 0.0, kFieldTop, static_cast<double>(kWidth), kFieldBottom)) return;
    const sat::Vec2 d = b - a;
    const double len = sat::length(d);
    if (len < 1e-6) return;
    const sat::Vec2 n{-d.y / len * width / 2.0, d.x / len * width / 2.0};
    const SDL_FColor col = gd::fcolor(c);
    const SDL_Vertex v[4] = {{fp(a + n), col, {0, 0}}, {fp(b + n), col, {0, 0}}, {fp(b - n), col, {0, 0}}, {fp(a - n), col, {0, 0}}};
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
}

void dashed(SDL_Renderer* r, sat::Vec2 a, sat::Vec2 b, double width, gd::Color c, double dash = 8.0, double gap = 6.0) {
    if (!clipLine(a, b, 0.0, kFieldTop, static_cast<double>(kWidth), kFieldBottom)) return;
    const sat::Vec2 d = b - a;
    const double len = sat::length(d);
    if (len < 1e-6) return;
    const sat::Vec2 u = d * (1.0 / len);
    for (double s = 0.0; s < len; s += dash + gap) segment(r, a + u * s, a + u * std::min(s + dash, len), width, c);
}

void outline(SDL_Renderer* r, const sat::Polygon& p, double width, gd::Color c) {
    for (std::size_t i = 0; i < p.size(); ++i) segment(r, p[i], p[(i + 1) % p.size()], width, c);
}

void dashedOutline(SDL_Renderer* r, const sat::Polygon& p, double width, gd::Color c) {
    for (std::size_t i = 0; i < p.size(); ++i) dashed(r, p[i], p[(i + 1) % p.size()], width, c);
}

void fillConvex(SDL_Renderer* r, const sat::Polygon& p, gd::Color c) {
    std::vector<SDL_Vertex> v;
    std::vector<int> idx;
    const SDL_FColor col = gd::fcolor(c);
    v.push_back({fp(sat::center(p)), col, {0, 0}});
    for (std::size_t i = 0; i < p.size(); ++i) {
        v.push_back({fp(p[i]), col, {0, 0}});
        const auto a = static_cast<int>(i) + 1;
        const int b = static_cast<int>((i + 1) % p.size()) + 1;
        idx.insert(idx.end(), {0, a, b});
    }
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

void arrow(SDL_Renderer* r, sat::Vec2 from, sat::Vec2 to, gd::Color c) {
    segment(r, from, to, 4.0, c);
    const bool inField = to.x >= 0.0 && to.x <= kWidth && to.y >= kFieldTop && to.y <= kFieldBottom;
    const sat::Vec2 d = to - from;
    const double len = sat::length(d);
    if (!inField || len < 14.0) return;
    const sat::Vec2 u = d * (1.0 / len);
    const sat::Vec2 n{-u.y, u.x};
    const SDL_FColor col = gd::fcolor(c);
    const SDL_Vertex v[3] = {{fp(to), col, {0, 0}}, {fp(to - u * 14.0 + n * 7.0), col, {0, 0}}, {fp(to - u * 14.0 - n * 7.0), col, {0, 0}}};
    SDL_RenderGeometry(r, nullptr, v, 3, nullptr, 0);
}

/// Numbers can be absurd with raw axes (that is the point), keep them inside the panel.
std::string meters(double v) {
    char buf[32];
    if (std::fabs(v) > 9999.0) return ">9999";
    SDL_snprintf(buf, sizeof buf, "%.2f", v);
    return buf;
}

std::string axisName(const sat::AxisResult& a) {
    char buf[16];
    SDL_snprintf(buf, sizeof buf, "%c%d", a.owner == 0 ? 'A' : 'B', static_cast<int>(a.edge) + 1);
    return buf;
}

SDL_FRect axisRowRect(std::size_t i) { return SDL_FRect{kAxisPanelX + 6.0F, kAxisPanelY + kAxisRowsTop + kRowH * static_cast<float>(i), kAxisPanelW - 12.0F, kRowH - 2.0F}; }

/// The selected axis as a line through the scene with both shadows on it.
void drawAxis(App& app, const sat::Result& res, std::size_t idx, const sat::Polygon& pa, const sat::Polygon& pb) {
    SDL_Renderer* r = app.renderer;
    const sat::AxisResult& ax = res.axes[idx];
    const double axisLen = sat::length(ax.axis);
    if (axisLen < 1e-12) return;
    const sat::Vec2 u = ax.axis * (1.0 / axisLen);  // display direction, always unit length
    const sat::Vec2 n{-u.y, u.x};
    sat::Vec2 anchor = (sat::center(pa) + sat::center(pb)) * 0.5;
    anchor = {std::clamp(anchor.x, 60.0, 900.0), std::clamp(anchor.y, 90.0, 400.0)};
    const double base = sat::dot(anchor, u);
    const sat::Interval ia = sat::project(pa, u);
    const sat::Interval ib = sat::project(pb, u);
    const auto at = [&](double s, double side) { return anchor + u * (s - base) + n * side; };

    segment(r, anchor - u * 3000.0, anchor + u * 3000.0, 2.0, kInkSoft);

    // Thin guides from the corners that cast the ends of each shadow.
    const std::array<std::pair<const sat::Polygon*, const sat::Interval*>, 2> shadows{{{&pa, &ia}, {&pb, &ib}}};
    for (std::size_t k = 0; k < 2; ++k) {
        const double side = k == 0 ? -9.0 : 9.0;
        for (const std::size_t corner : {shadows[k].second->iMin, shadows[k].second->iMax}) {
            const sat::Vec2 v = (*shadows[k].first)[corner];
            dashed(r, v, at(sat::dot(v, u), side), 1.0, kInkSoft, 4.0, 4.0);
        }
    }
    segment(r, at(ia.min, -9.0), at(ia.max, -9.0), 8.0, gd::kTeal);
    segment(r, at(ib.min, 9.0), at(ib.max, 9.0), 8.0, gd::kOrange);

    // Overlap (or gap) between the two shadows, on the line itself.
    const double lo = std::max(ia.min, ib.min);
    const double hi = std::min(ia.max, ib.max);
    const bool overlap = hi > lo;
    const double s0 = overlap ? lo : hi;
    const double s1 = overlap ? hi : lo;
    segment(r, at(s0, 0.0), at(s1, 0.0), 6.0, gd::kInk);
    for (const double s : {s0, s1}) segment(r, at(s, -14.0), at(s, 14.0), 2.0, gd::kInk);

    const char* word = overlap ? (app.options.orient ? "depth" : "overlap") : "gap";
    const std::string label = axisName(ax) + " " + word + " " + meters(std::fabs(ax.amount));
    const sat::Vec2 mid = at((s0 + s1) / 2.0, 0.0);
    const float lw = app.hud.width(label, gd::Hud::Style::Bold);
    const float lx = static_cast<float>(std::clamp(mid.x - lw / 2.0, 6.0, kWidth - lw - 6.0));
    const float ly = static_cast<float>(std::clamp(mid.y + (n.y > 0.0 ? 70.0 : -90.0), kFieldTop + 4.0, kFieldBottom - 24.0));
    gd::fillRect(r, lx - 4.0F, ly - 2.0F, lw + 8.0F, 24.0F, gd::Color{250, 250, 249, 240});
    app.hud.text(r, lx, ly, label, gd::kInk, gd::Hud::Style::Bold);
}

void drawResultPanel(App& app, const sat::Result& res, const sat::Result& right, const sat::Polygon& a, const sat::Polygon& b) {
    SDL_Renderer* r = app.renderer;
    const gd::Hud::Style bold = gd::Hud::Style::Bold;
    gd::card(r, 24.0F, 44.0F, 300.0F, 196.0F, gd::kWhite, gd::kInk);
    float y = 52.0F;
    char line[64];
    if (res.colliding) {
        SDL_snprintf(line, sizeof line, "overlap on all %d axes", static_cast<int>(res.axes.size()));
        app.hud.text(r, 38.0F, y, line, gd::kMuted);
        app.hud.text(r, 38.0F, y + 22.0F, "COLLISION", gd::kOrange, gd::Hud::Style::Large);
        SDL_snprintf(line, sizeof line, "push B by %s m", meters(sat::length(res.mtv)).c_str());
        app.hud.text(r, 38.0F, y + 70.0F, line, gd::kInk, bold);
        SDL_snprintf(line, sizeof line, "needed       %s m", meters(sat::length(right.mtv)).c_str());
        app.hud.text(r, 38.0F, y + 92.0F, line, gd::kMuted, bold);
        const double deep = sat::penetration(a, sat::translated(b, res.mtv));
        if (deep > 0.0) {
            SDL_snprintf(line, sizeof line, "after push: %s m deep", meters(deep).c_str());
            app.hud.text(r, 38.0F, y + 114.0F, line, gd::kOrange, bold);
        } else {
            app.hud.text(r, 38.0F, y + 114.0F, "after push: apart", gd::kTeal, bold);
        }
    } else if (!res.axes.empty()) {
        SDL_snprintf(line, sizeof line, "axis %s separates", axisName(res.axes[res.best]).c_str());
        app.hud.text(r, 38.0F, y, line, gd::kMuted);
        app.hud.text(r, 38.0F, y + 22.0F, "NO COLLISION", gd::kTeal, gd::Hud::Style::Large);
        SDL_snprintf(line, sizeof line, "gap %s m", meters(-right.axes[right.best].amount).c_str());
        app.hud.text(r, 38.0F, y + 70.0F, line, gd::kInk, bold);
    }
    app.hud.text(r, 38.0F, y + 140.0F, app.options.normalize ? "axes: unit length" : "axes: as long as the edge", app.options.normalize ? gd::kTeal : gd::kOrange, bold);
    app.hud.text(r, 38.0F, y + 162.0F, app.options.orient ? "MTV: B moves away from A" : "MTV: axis direction as is", app.options.orient ? gd::kTeal : gd::kOrange, bold);
}

void drawAxisPanel(App& app, const sat::Result& res, std::size_t shown) {
    SDL_Renderer* r = app.renderer;
    const float h = kAxisRowsTop + kRowH * static_cast<float>(res.axes.size()) + 8.0F;
    gd::card(r, kAxisPanelX, kAxisPanelY, kAxisPanelW, h, gd::kWhite, gd::kInk);
    app.hud.text(r, kAxisPanelX + 14.0F, kAxisPanelY + 8.0F, "AXES", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kAxisPanelX + 14.0F, kAxisPanelY + 30.0F, "click row to show", gd::kMuted);
    for (std::size_t i = 0; i < res.axes.size(); ++i) {
        const sat::AxisResult& a = res.axes[i];
        const SDL_FRect row = axisRowRect(i);
        if (i == shown) {
            gd::fillRect(r, row.x, row.y, row.w, row.h, gd::kTealSoft);
            gd::strokeRect(r, row.x, row.y, row.w, row.h, gd::kInk);
        }
        const bool overlap = a.amount > sat::kTouch;
        const std::string text = axisName(a) + (overlap ? " overlap " : " gap ") + meters(std::fabs(a.amount));
        app.hud.text(r, row.x + 8.0F, row.y + 1.0F, text, overlap ? gd::kInk : gd::kTeal, i == res.best ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        if (i == res.best) gd::disc(r, row.x + row.w - 10.0F, row.y + row.h / 2.0F, 4.0F, gd::kOrange);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const sat::Polygon wa = world(app, 0);
    const sat::Polygon wb = world(app, 1);
    const sat::Result res = sat::collide(wa, wb, app.options);
    const sat::Result right = sat::collide(wa, wb, sat::Options{true, true});
    app.axisCount = res.axes.size();
    const sat::Polygon pa = pxPolygon(wa);
    const sat::Polygon pb = pxPolygon(wb);

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ SAT COLLISION ]", res.colliding ? "colliding" : "apart");

    fillConvex(r, pa, kTealFill);
    fillConvex(r, pb, kOrangeFill);
    if (res.colliding) dashedOutline(r, pxPolygon(sat::translated(wb, res.mtv)), 2.0, gd::kOrange);  // B after the push
    outline(r, pa, app.selected == 0 ? 4.0 : 2.0, gd::kTealDark);
    outline(r, pb, app.selected == 1 ? 4.0 : 2.0, gd::kOrange);
    const sat::Vec2 ca = sat::center(pa);
    const sat::Vec2 cb = sat::center(pb);
    app.hud.text(r, static_cast<float>(ca.x) - 5.0F, static_cast<float>(ca.y) - 11.0F, "A", gd::kTealDark, gd::Hud::Style::Bold);
    app.hud.text(r, static_cast<float>(cb.x) - 5.0F, static_cast<float>(cb.y) - 11.0F, "B", gd::kOrange, gd::Hud::Style::Bold);

    if (!res.axes.empty()) drawAxis(app, res, shownAxis(app, res), pa, pb);
    if (res.colliding) arrow(r, cb, cb + res.mtv * kScale, gd::kInk);

    drawResultPanel(app, res, right, wa, wb);
    drawAxisPanel(app, res, shownAxis(app, res));

    app.hud.text(r, 40.0F, 436.0F, "drag: move   wheel / Q E: turn the shape with the thick outline   < > : axis", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const gd::Color fill = on ? gd::kTeal : (isMistakeSwitch(c.action) ? gd::kOrange : gd::kWhite);
        const bool lit = on || isMistakeSwitch(c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, lit ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("sat-collision", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    loadPreset(*app, 0);
    layoutChips(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            if (app.dragging >= 0) {
                sat::Shape& s = app.shape[static_cast<std::size_t>(app.dragging)];
                s.pos = clampToWorld(app.mouse * (1.0 / kScale) - app.grabOffset);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = {static_cast<double>(p.x), static_cast<double>(p.y)};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            for (std::size_t i = 0; i < app.axisCount; ++i) {
                const SDL_FRect row = axisRowRect(i);
                if (SDL_PointInRectFloat(&p, &row)) {
                    app.axisPick = static_cast<int>(i);
                    return SDL_APP_CONTINUE;
                }
            }
            const sat::Vec2 m = app.mouse * (1.0 / kScale);
            for (const int i : {1, 0}) {  // B is drawn on top
                if (sat::contains(world(app, i), m)) {
                    app.dragging = i;
                    app.selected = i;
                    app.grabOffset = m - app.shape[static_cast<std::size_t>(i)].pos;
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = -1;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging = -1; break;
        case SDL_EVENT_MOUSE_WHEEL: {
            const sat::Vec2 m = app.mouse * (1.0 / kScale);
            int target = app.selected;
            if (!sat::contains(world(app, target), m) && sat::contains(world(app, 1 - target), m)) target = 1 - target;
            app.selected = target;
            app.shape[static_cast<std::size_t>(target)].angle += kWheelStep * std::clamp(static_cast<double>(event->wheel.y), -3.0, 3.0);
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode key = event->key.key;
            // Turning may repeat while the key is held, everything else is a one-shot toggle.
            if (key == SDLK_Q || key == SDLK_E) {
                app.shape[static_cast<std::size_t>(app.selected)].angle += key == SDLK_E ? kTurnStep : -kTurnStep;
                break;
            }
            if (event->key.repeat) break;
            switch (key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Preset0); break;
                case SDLK_2: apply(app, Action::Preset1); break;
                case SDLK_3: apply(app, Action::Preset2); break;
                case SDLK_R: apply(app, Action::Reset); break;
                case SDLK_N: apply(app, Action::Normalize); break;
                case SDLK_O: apply(app, Action::Orient); break;
                case SDLK_A: apply(app, Action::AutoAxis); break;
                case SDLK_RIGHT: stepAxis(app, 1); break;
                case SDLK_LEFT: stepAxis(app, -1); break;
                case SDLK_SPACE: apply(app, Action::Push); break;
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
