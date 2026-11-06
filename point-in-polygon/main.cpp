// Point in polygon with SDL3 main callbacks.
//
// The mouse cursor is the test point. A ray runs from it to the right and every edge it crosses is marked. Two ways
// to read the same hits: ray casting (even-odd, an odd count means inside) and the winding number (each hit counts
// +1 or -1 by edge direction, a sum other than 0 means inside). They agree on ordinary polygons and disagree in the
// middle of the pentagram, which is why that is the first thing you see. The corners are draggable, the geometry
// and its edge cases (a ray through a vertex) live in logic.hpp and are covered by test.cpp.

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
constexpr double kVertexGrab = 12.0;  // px around a corner that grabs it
constexpr double kEdgeGrab = 8.0;     // px around an edge that inserts a corner
constexpr double kSnapRange = 14.0;   // px within which the ray snaps to a corner's row
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr gd::Color kInkSoft{17, 24, 39, 190};

enum class Action { RuleEvenOdd, RuleNonZero, PresetConcave, PresetStar, Snap };

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
    pip::Polygon poly = pip::starPreset();
    int preset = 1;  // 0 concave, 1 star, -1 edited by hand
    pip::Rule rule = pip::Rule::EvenOdd;
    bool snap = false;
    pip::Vec2 testPoint = pip::starHome();
    pip::Vec2 mouse{-100.0, -100.0};
    std::optional<std::size_t> dragging;
    long maxFrames = -1;
    long frames = 0;
};

pip::Vec2 toVec(SDL_FPoint p) { return {static_cast<double>(p.x), static_cast<double>(p.y)}; }

bool inField(pip::Vec2 p) { return p.y >= static_cast<double>(kFieldTop) && p.y <= static_cast<double>(kFieldBottom); }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 5> defs{{{Action::RuleEvenOdd, "1 Ray casting", 0.0F},
                                   {Action::RuleNonZero, "2 Winding", 8.0F},
                                   {Action::PresetConcave, "3 Concave", 24.0F},
                                   {Action::PresetStar, "4 Star", 8.0F},
                                   {Action::Snap, "S Snap ray to corner", 24.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void setPreset(App& app, int preset) {
    app.preset = preset;
    app.poly = preset == 0 ? pip::concavePreset() : pip::starPreset();
    app.testPoint = preset == 0 ? pip::concaveHome() : pip::starHome();
    app.dragging.reset();
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::RuleEvenOdd: app.rule = pip::Rule::EvenOdd; break;
        case Action::RuleNonZero: app.rule = pip::Rule::NonZero; break;
        case Action::PresetConcave: setPreset(app, 0); break;
        case Action::PresetStar: setPreset(app, 1); break;
        case Action::Snap: app.snap = !app.snap; break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::RuleEvenOdd: return app.rule == pip::Rule::EvenOdd;
        case Action::RuleNonZero: return app.rule == pip::Rule::NonZero;
        case Action::PresetConcave: return app.preset == 0;
        case Action::PresetStar: return app.preset == 1;
        case Action::Snap: return app.snap;
    }
    return false;
}

/// The corner under the cursor (or the one being dragged), the target of Del and right click.
std::optional<std::size_t> pickedVertex(const App& app) {
    return app.dragging ? app.dragging : pip::nearestVertex(app.poly, app.mouse, kVertexGrab);
}

void removePicked(App& app) {
    const auto v = pickedVertex(app);
    if (v && pip::removeVertex(app.poly, *v)) {
        app.preset = -1;
        app.dragging.reset();
    }
}

pip::Vec2 rayPoint(const App& app) {
    pip::Vec2 p = app.testPoint;
    if (app.snap) p.y = pip::snapToVertexRow(app.poly, p.y, kSnapRange);
    return p;
}

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, pip::Vec2 a, pip::Vec2 b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F),
                       static_cast<float>(b.x) + (flat ? 0.0F : off), static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

void drawFill(App& app) {
    float top = 1e9F;
    float bottom = -1e9F;
    for (const pip::Vec2& v : app.poly) {
        top = std::min(top, static_cast<float>(v.y));
        bottom = std::max(bottom, static_cast<float>(v.y));
    }
    for (float y = std::floor(top); y < bottom; y += 2.0F) {
        for (const pip::Span& s : pip::scanSpans(app.poly, static_cast<double>(y) + 1.0, app.rule)) {
            gd::fillRect(app.renderer, static_cast<float>(s.x0), y, static_cast<float>(s.x1 - s.x0), 2.0F, gd::Color{13, 148, 136, 70});
        }
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const pip::Vec2 rp = rayPoint(app);
    const auto crossings = pip::rayCrossings(app.poly, rp);
    const bool evenOdd = app.rule == pip::Rule::EvenOdd;
    int winding = 0;
    for (const pip::Crossing& c : crossings) winding += c.dir;
    const bool isInside = pip::inside(app.poly, rp, app.rule);
    const gd::Color verdict = isInside ? gd::kTeal : gd::kOrange;

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ POINT IN POLYGON ]", evenOdd ? "ray casting (even-odd)" : "winding number (nonzero)");

    drawFill(app);

    // Edges, the ones the ray crosses in orange.
    const std::size_t n = app.poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const bool hit = std::any_of(crossings.begin(), crossings.end(), [i](const pip::Crossing& c) { return c.edge == i; });
        thickLine(r, app.poly[i], app.poly[(i + 1) % n], hit ? gd::kOrange : gd::kInk, hit ? 2 : 1);
    }

    // The ray: dashed, with an arrow head.
    const auto y = static_cast<float>(rp.y);
    const float x0 = static_cast<float>(rp.x);
    constexpr float kRayEnd = 948.0F;
    for (float x = x0; x < kRayEnd; x += 16.0F) thickLine(r, {static_cast<double>(x), static_cast<double>(y)}, {static_cast<double>(std::min(x + 10.0F, kRayEnd)), static_cast<double>(y)}, kInkSoft, 1);
    thickLine(r, {static_cast<double>(kRayEnd), static_cast<double>(y)}, {static_cast<double>(kRayEnd) - 12.0, static_cast<double>(y) - 6.0}, kInkSoft, 0);
    thickLine(r, {static_cast<double>(kRayEnd), static_cast<double>(y)}, {static_cast<double>(kRayEnd) - 12.0, static_cast<double>(y) + 6.0}, kInkSoft, 0);

    // Corners that sit exactly on the ray's row are the interesting case: ring them.
    for (const pip::Vec2& v : app.poly) {
        if (v.y == rp.y) gd::ring(r, static_cast<float>(v.x), static_cast<float>(v.y), 12.0F, gd::kInk);
    }

    // Crossing markers with a running count (even-odd) or the direction (winding). Marks at the same spot stack up.
    int stack = 0;
    int count = 0;
    double lastX = -1e9;
    for (const pip::Crossing& c : crossings) {
        stack = std::fabs(c.x - lastX) < 16.0 ? stack + 1 : 0;
        lastX = c.x;
        ++count;
        const auto cx = static_cast<float>(c.x);
        gd::disc(r, cx, y, 7.0F, gd::kOrange);
        gd::ring(r, cx, y, 7.0F, gd::kInk);
        char label[16];
        if (evenOdd) {
            SDL_snprintf(label, sizeof label, "%d", count);
        } else {
            SDL_snprintf(label, sizeof label, "%s", c.dir > 0 ? "+1" : "-1");
        }
        float ly = y - 32.0F - 22.0F * static_cast<float>(stack);
        if (ly < kFieldTop + 4.0F) ly = y + 12.0F + 22.0F * static_cast<float>(stack);
        app.hud.text(r, cx - app.hud.width(label, gd::Hud::Style::Bold) / 2.0F, ly, label, gd::kOrange, gd::Hud::Style::Bold);
    }

    // Corners.
    const auto picked = pickedVertex(app);
    for (std::size_t i = 0; i < n; ++i) {
        const bool hot = picked && *picked == i;
        const auto vx = static_cast<float>(app.poly[i].x);
        const auto vy = static_cast<float>(app.poly[i].y);
        gd::fillRect(r, vx - 6.0F, vy - 6.0F, 12.0F, 12.0F, hot ? gd::kTeal : gd::kWhite);
        gd::strokeRect(r, vx - 6.0F, vy - 6.0F, 12.0F, 12.0F, gd::kInk);
    }

    // The test point.
    gd::disc(r, x0, y, 9.0F, verdict);
    gd::ring(r, x0, y, 9.0F, gd::kInk);
    gd::ring(r, x0, y, 10.0F, gd::kInk);

    // Result panel.
    gd::card(r, 24.0F, 44.0F, 320.0F, 106.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, 52.0F, evenOdd ? "odd crossings = inside" : "winding not 0 = inside", gd::kMuted);
    app.hud.text(r, 38.0F, 76.0F, isInside ? "INSIDE" : "OUTSIDE", verdict, gd::Hud::Style::Large);
    char line[64];
    if (evenOdd) {
        SDL_snprintf(line, sizeof line, "%d crossings = %s", static_cast<int>(crossings.size()), crossings.size() % 2 == 1 ? "odd" : "even");
    } else {
        SDL_snprintf(line, sizeof line, "winding %d = %s", winding, winding != 0 ? "nonzero" : "zero");
    }
    app.hud.text(r, 38.0F, 122.0F, line, gd::kInk, gd::Hud::Style::Bold);

    app.hud.text(r, 40.0F, 446.0F, "drag corner: move   click edge: add corner   right click / Del: remove corner", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("point-in-polygon", kWidth, kHeight, 0, &app->window, &app->renderer)) {
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
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = toVec({event->motion.x, event->motion.y});
            if (app.dragging) {
                // The test point stays where it was, so you see the result change while a corner moves.
                app.poly[*app.dragging] = {std::clamp(app.mouse.x, 8.0, 952.0), std::clamp(app.mouse.y, static_cast<double>(kFieldTop) + 6.0, static_cast<double>(kFieldBottom) - 4.0)};
                app.preset = -1;
            } else if (inField(app.mouse)) {
                app.testPoint = app.mouse;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = toVec(p);
            if (event->button.button == SDL_BUTTON_RIGHT) {
                removePicked(app);
                break;
            }
            if (event->button.button != SDL_BUTTON_LEFT) break;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                }
            }
            if (onChip || !inField(app.mouse)) break;
            if (const auto v = pip::nearestVertex(app.poly, app.mouse, kVertexGrab)) {
                app.dragging = v;
            } else if (const auto e = pip::nearestEdge(app.poly, app.mouse, kEdgeGrab)) {
                app.dragging = pip::insertVertex(app.poly, e->index, e->point);
                app.preset = -1;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging.reset();
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE: app.dragging.reset(); break;
        case SDL_EVENT_KEY_DOWN:
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::RuleEvenOdd); break;
                case SDLK_2: apply(app, Action::RuleNonZero); break;
                case SDLK_3: apply(app, Action::PresetConcave); break;
                case SDLK_4: apply(app, Action::PresetStar); break;
                case SDLK_S: apply(app, Action::Snap); break;
                case SDLK_DELETE:
                case SDLK_BACKSPACE: removePicked(app); break;
                default: break;
            }
            break;
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
