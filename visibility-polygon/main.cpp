// 2D visibility with SDL3 main callbacks.
//
// The light follows the mouse, the walls are line segments. The lit area is a polygon: one ray per wall end, plus
// one just before and one just after it, hits sorted by angle and joined. Switch to "exact corners" to see the
// classic mistake (the point behind a wall end is missing, the shadow edge turns into a chord) and to "uniform
// rays" to see why 360 evenly spaced rays are neither cheap nor right. The geometry lives in logic.hpp and is
// covered by test.cpp, this file is input and drawing.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldLeft = 8.0F;
constexpr float kFieldRight = 952.0F;
constexpr float kFieldTop = 40.0F;
constexpr float kFieldBottom = 410.0F;
constexpr double kCornerGrab = 12.0;  // px around a wall end that grabs it
constexpr double kConeHalf = 0.5;     // radians, half the opening of the flashlight
constexpr float kFadeDistance = 650.0F;
constexpr float kChipH = 30.0F;
constexpr std::array<int, 4> kUniformCounts{24, 48, 90, 360};

constexpr gd::Color kLightCore{253, 224, 71};
constexpr gd::Color kLitNear{255, 253, 236};
constexpr gd::Color kLitFar{196, 246, 236};
constexpr gd::Color kNaiveNear{255, 249, 236};
constexpr gd::Color kNaiveFar{254, 214, 172};

enum class Action { MethodCorners, MethodExact, MethodUniform, CycleCount, ToggleRays, ToggleFlash, NextScene, ResetWalls };

struct Chip {
    Action action;
    SDL_FRect rect;
};

struct Scene {
    std::vector<vis::Segment> walls;  // movable shapes first, the frame of the field last
    std::size_t movable = 0;
    vis::Vec2 home;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    int sceneIndex = 0;
    Scene scene;
    vis::Method method = vis::Method::CornerRays;
    int countIndex = 2;
    bool showRays = true;
    bool flashlight = false;
    double aim = 0.0;
    vis::Vec2 light;
    vis::Vec2 mouse{-100.0, -100.0};
    std::optional<vis::Vec2> dragging;  // the wall end being dragged
    vis::Result result;
    std::vector<SDL_Vertex> verts;
    std::vector<int> indices;
    long maxFrames = -1;
    long frames = 0;
};

vis::Vec2 toVec(SDL_FPoint p) { return {static_cast<double>(p.x), static_cast<double>(p.y)}; }

bool inField(vis::Vec2 p) {
    return p.x >= static_cast<double>(kFieldLeft) && p.x <= static_cast<double>(kFieldRight) && p.y >= static_cast<double>(kFieldTop) &&
           p.y <= static_cast<double>(kFieldBottom);
}

vis::Vec2 clampToField(vis::Vec2 p, double margin) {
    return {std::clamp(p.x, static_cast<double>(kFieldLeft) + margin, static_cast<double>(kFieldRight) - margin),
            std::clamp(p.y, static_cast<double>(kFieldTop) + margin, static_cast<double>(kFieldBottom) - margin)};
}

Scene buildScene(int index) {
    Scene s;
    if (index == 0) {
        vis::addBox(s.walls, 130, 100, 90, 70);
        vis::addBox(s.walls, 300, 250, 60, 110);
        vis::addBox(s.walls, 520, 80, 120, 48);
        vis::addBox(s.walls, 720, 230, 80, 90);
        vis::addLoop(s.walls, {{470, 230}, {540, 310}, {430, 330}});
        vis::addLoop(s.walls, {{150, 250}, {230, 250}, {230, 272}, {172, 272}, {172, 336}, {150, 336}});
        s.walls.push_back({{610, 340}, {700, 390}});
        s.walls.push_back({{820, 80}, {910, 160}});
        s.home = {420.0, 170.0};
    } else {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 6; ++col) {
                const double x = 120.0 + 135.0 * col + (row % 2 == 1 ? 60.0 : 0.0);
                vis::addBox(s.walls, x, 84.0 + 105.0 * row, 34.0, 34.0);
            }
        }
        vis::addLoop(s.walls, {{40, 360}, {70, 330}, {100, 360}});
        s.home = {60.0, 200.0};
    }
    s.movable = s.walls.size();
    vis::addBox(s.walls, static_cast<double>(kFieldLeft), static_cast<double>(kFieldTop), static_cast<double>(kFieldRight - kFieldLeft),
                static_cast<double>(kFieldBottom - kFieldTop));
    return s;
}

void setScene(App& app, int index) {
    app.sceneIndex = index;
    app.scene = buildScene(index);
    app.light = app.scene.home;
    app.dragging.reset();
}

const char* methodName(vis::Method m) {
    switch (m) {
        case vis::Method::CornerRays: return "corner rays ±offset";
        case vis::Method::ExactCorners: return "exact corners only";
        case vis::Method::Uniform: return "uniform rays";
    }
    return "";
}

void labelFor(const App& app, Action action, char* out, std::size_t size) {
    switch (action) {
        case Action::MethodCorners: SDL_snprintf(out, size, "1 Corners ±offset"); break;
        case Action::MethodExact: SDL_snprintf(out, size, "2 Exact corners"); break;
        case Action::MethodUniform: SDL_snprintf(out, size, "3 Uniform rays"); break;
        case Action::CycleCount: SDL_snprintf(out, size, "N %d rays", kUniformCounts[static_cast<std::size_t>(app.countIndex)]); break;
        case Action::ToggleRays: SDL_snprintf(out, size, "R Show rays"); break;
        case Action::ToggleFlash: SDL_snprintf(out, size, "F Flashlight"); break;
        case Action::NextScene: SDL_snprintf(out, size, "S Scene: %s", app.sceneIndex == 0 ? "Rooms" : "Pillars"); break;
        case Action::ResetWalls: SDL_snprintf(out, size, "X Reset walls"); break;
    }
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* widest;
        float x;
        float y;
    };
    // Fixed widths from the longest label a chip can show, so a chip does not jump when its text changes.
    const std::array<Def, 8> defs{{{Action::MethodCorners, "1 Corners ±offset", 40.0F, 456.0F},
                                   {Action::MethodExact, "2 Exact corners", 0.0F, 456.0F},
                                   {Action::MethodUniform, "3 Uniform rays", 0.0F, 456.0F},
                                   {Action::CycleCount, "N 360 rays", 0.0F, 456.0F},
                                   {Action::ToggleRays, "R Show rays", 40.0F, 494.0F},
                                   {Action::ToggleFlash, "F Flashlight", 0.0F, 494.0F},
                                   {Action::NextScene, "S Scene: Pillars", 0.0F, 494.0F},
                                   {Action::ResetWalls, "X Reset walls", 0.0F, 494.0F}}};
    float x = 0.0F;
    for (const Def& d : defs) {
        x = d.x > 0.0F ? d.x : x + 8.0F;
        if (d.action == Action::CycleCount) x += 16.0F;
        const float w = app.hud.width(d.widest, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, SDL_FRect{x, d.y, w, kChipH}});
        x += w;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::MethodCorners: app.method = vis::Method::CornerRays; break;
        case Action::MethodExact: app.method = vis::Method::ExactCorners; break;
        case Action::MethodUniform: app.method = vis::Method::Uniform; break;
        case Action::CycleCount:
            app.countIndex = (app.countIndex + 1) % static_cast<int>(kUniformCounts.size());
            app.method = vis::Method::Uniform;
            break;
        case Action::ToggleRays: app.showRays = !app.showRays; break;
        case Action::ToggleFlash: app.flashlight = !app.flashlight; break;
        case Action::NextScene: setScene(app, 1 - app.sceneIndex); break;
        case Action::ResetWalls: {
            const vis::Vec2 keep = app.light;
            setScene(app, app.sceneIndex);
            app.light = keep;
            break;
        }
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::MethodCorners: return app.method == vis::Method::CornerRays;
        case Action::MethodExact: return app.method == vis::Method::ExactCorners;
        case Action::MethodUniform: return app.method == vis::Method::Uniform;
        case Action::ToggleRays: return app.showRays;
        case Action::ToggleFlash: return app.flashlight;
        default: return false;
    }
}

/// 1 px lines are too thin on a canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, vis::Vec2 a, vis::Vec2 b, gd::Color c, int half) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F),
                       static_cast<float>(b.x) + (flat ? 0.0F : off), static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

SDL_FColor mix(gd::Color a, gd::Color b, float t) {
    const float k = std::clamp(t, 0.0F, 1.0F);
    return SDL_FColor{(static_cast<float>(a.r) + (static_cast<float>(b.r) - static_cast<float>(a.r)) * k) / 255.0F,
                      (static_cast<float>(a.g) + (static_cast<float>(b.g) - static_cast<float>(a.g)) * k) / 255.0F,
                      (static_cast<float>(a.b) + (static_cast<float>(b.b) - static_cast<float>(a.b)) * k) / 255.0F, 1.0F};
}

void drawLight(App& app) {
    const vis::Result& res = app.result;
    const std::size_t rim = res.polygon.size() - res.rimBegin;
    if (rim < 2) return;
    const bool naive = app.method != vis::Method::CornerRays;
    const gd::Color nearColor = naive ? kNaiveNear : kLitNear;
    const gd::Color farColor = naive ? kNaiveFar : kLitFar;
    app.verts.clear();
    app.indices.clear();
    const auto lx = static_cast<float>(app.light.x);
    const auto ly = static_cast<float>(app.light.y);
    app.verts.push_back({{lx, ly}, mix(nearColor, farColor, 0.0F), {0, 0}});
    for (std::size_t i = res.rimBegin; i < res.polygon.size(); ++i) {
        const vis::Vec2 p = res.polygon[i];
        const auto dist = static_cast<float>(vis::length(p - app.light));
        app.verts.push_back({{static_cast<float>(p.x), static_cast<float>(p.y)}, mix(nearColor, farColor, dist / kFadeDistance), {0, 0}});
    }
    const int m = static_cast<int>(rim);
    const int last = res.full ? m : m - 1;
    for (int i = 0; i < last; ++i) {
        app.indices.push_back(0);
        app.indices.push_back(1 + i);
        app.indices.push_back(1 + (i + 1) % m);
    }
    SDL_RenderGeometry(app.renderer, nullptr, app.verts.data(), static_cast<int>(app.verts.size()), app.indices.data(),
                       static_cast<int>(app.indices.size()));
}

gd::Color rayColor(vis::RayKind k, bool naive) {
    switch (k) {
        case vis::RayKind::Corner: return naive ? gd::kOrange : gd::kTealDark;
        case vis::RayKind::Before:
        case vis::RayKind::After:
        case vis::RayKind::Uniform: return gd::kOrange;
        case vis::RayKind::ConeEdge: return gd::kInk;
    }
    return gd::kInk;
}

void drawRays(App& app) {
    SDL_Renderer* r = app.renderer;
    const bool naive = app.method != vis::Method::CornerRays;
    const auto lx = static_cast<float>(app.light.x);
    const auto ly = static_cast<float>(app.light.y);
    for (const vis::Ray& ray : app.result.rays) {
        // The rays just before and after a corner run almost on top of the one through it: dots only, no extra lines.
        if (ray.kind == vis::RayKind::Before || ray.kind == vis::RayKind::After) continue;
        gd::Color c = rayColor(ray.kind, naive);
        c.a = ray.kind == vis::RayKind::Corner ? 130 : 90;
        gd::setColor(r, c);
        SDL_RenderLine(r, lx, ly, static_cast<float>(ray.point.x), static_cast<float>(ray.point.y));
    }
    for (const vis::Ray& ray : app.result.rays) {
        const gd::Color c = rayColor(ray.kind, naive);
        const auto px = static_cast<float>(ray.point.x);
        const auto py = static_cast<float>(ray.point.y);
        gd::fillRect(r, px - 2.0F, py - 2.0F, 5.0F, 5.0F, c);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    vis::Options opt;
    opt.method = app.method;
    opt.uniformCount = kUniformCounts[static_cast<std::size_t>(app.countIndex)];
    opt.aim = app.aim;
    opt.halfAngle = app.flashlight ? kConeHalf : vis::kPi;
    vis::compute(app.light, app.scene.walls, opt, app.result);

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ VISIBILITY POLYGON ]", "the light follows the mouse, drag wall corners");

    // Everything outside the lit polygon stays under a light veil.
    gd::fillRect(r, kFieldLeft, kFieldTop, kFieldRight - kFieldLeft, kFieldBottom - kFieldTop, gd::Color{17, 24, 39, 34});
    drawLight(app);
    if (app.showRays) drawRays(app);

    // Walls, then the handles of the corners you can drag.
    for (std::size_t i = 0; i < app.scene.walls.size(); ++i) {
        const bool frame = i >= app.scene.movable;
        thickLine(r, app.scene.walls[i].a, app.scene.walls[i].b, gd::kInk, frame ? 2 : 1);
    }
    const auto hover = app.dragging ? app.dragging : vis::nearestCorner(app.scene.walls, app.scene.movable, app.mouse, kCornerGrab);
    for (std::size_t i = 0; i < app.scene.movable; ++i) {
        for (const vis::Vec2 c : {app.scene.walls[i].a, app.scene.walls[i].b}) {
            const bool hot = hover && *hover == c;
            const float s = hot ? 5.0F : 3.0F;
            gd::fillRect(r, static_cast<float>(c.x) - s, static_cast<float>(c.y) - s, 2.0F * s, 2.0F * s, hot ? gd::kTeal : gd::kWhite);
            gd::strokeRect(r, static_cast<float>(c.x) - s, static_cast<float>(c.y) - s, 2.0F * s, 2.0F * s, gd::kInk);
        }
    }

    // The light itself, and the flashlight's direction.
    const auto lx = static_cast<float>(app.light.x);
    const auto ly = static_cast<float>(app.light.y);
    if (app.flashlight) {
        thickLine(r, app.light, {app.light.x + std::cos(app.aim) * 26.0, app.light.y + std::sin(app.aim) * 26.0}, gd::kInk, 0);
    }
    gd::disc(r, lx, ly, 8.0F, kLightCore);
    gd::ring(r, lx, ly, 8.0F, gd::kInk);
    gd::ring(r, lx, ly, 9.0F, gd::kInk);

    // Numbers and a one line explanation of the active method.
    char line[160];
    const int rays = static_cast<int>(app.result.rays.size());
    const int points = static_cast<int>(app.result.polygon.size());
    if (app.flashlight) {
        SDL_snprintf(line, sizeof line, "%s   rays: %d   polygon points: %d   cone: %d°   aim: wheel or Q/E", methodName(app.method), rays, points,
                     static_cast<int>(std::lround(kConeHalf * 2.0 * 180.0 / vis::kPi)));
    } else {
        SDL_snprintf(line, sizeof line, "%s   rays: %d   polygon points: %d", methodName(app.method), rays, points);
    }
    app.hud.text(r, 40.0F, 416.0F, line, app.method == vis::Method::CornerRays ? gd::kTealDark : gd::kOrange, gd::Hud::Style::Bold);
    const char* note = "a ray at every wall end, one just before it and one just after it";
    if (app.method == vis::Method::ExactCorners) note = "only the ray through each end: the point behind it is missing, shadow edges turn into chords";
    if (app.method == vis::Method::Uniform) note = "evenly spaced rays ignore the walls: ends fall between two rays";
    app.hud.text(r, 40.0F, 434.0F, note, gd::kMuted);

    char label[48];
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool dim = c.action == Action::CycleCount && app.method != vis::Method::Uniform;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        labelFor(app, c.action, label, sizeof label);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, label,
                     on ? gd::kWhite : (dim ? gd::kMuted : gd::kInk), gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("visibility-polygon", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    setScene(*app, 0);
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
                const vis::Vec2 to = clampToField(app.mouse, 6.0);
                vis::moveCorner(app.scene.walls, *app.dragging, to);
                app.dragging = to;
            } else if (inField(app.mouse)) {
                app.light = clampToField(app.mouse, 2.0);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = toVec(p);
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inField(app.mouse)) app.dragging = vis::nearestCorner(app.scene.walls, app.scene.movable, app.mouse, kCornerGrab);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging.reset();
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            app.aim = vis::wrapAngle(app.aim + 0.12 * static_cast<double>(event->wheel.y));
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging.reset(); break;
        case SDL_EVENT_KEY_DOWN:
            // Q and E may repeat (turning the flashlight), everything else is a toggle and must not.
            if (event->key.key == SDLK_Q) app.aim = vis::wrapAngle(app.aim - 0.06);
            if (event->key.key == SDLK_E) app.aim = vis::wrapAngle(app.aim + 0.06);
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::MethodCorners); break;
                case SDLK_2: apply(app, Action::MethodExact); break;
                case SDLK_3: apply(app, Action::MethodUniform); break;
                case SDLK_N: apply(app, Action::CycleCount); break;
                case SDLK_R: apply(app, Action::ToggleRays); break;
                case SDLK_F: apply(app, Action::ToggleFlash); break;
                case SDLK_S: apply(app, Action::NextScene); break;
                case SDLK_X: apply(app, Action::ResetWalls); break;
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
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
