// Uniform sampling in a disc, a triangle and on a sphere with SDL3 main callbacks.
//
// Left the naive mapping, right the correct one, both fed the exact same random numbers, so the only difference is the
// formula. The naive disc takes the radius straight from the random number and piles up in the middle, the naive
// triangle piles up at the apex, the naive sphere at the poles. The bars under each panel count the points in ten bands
// of equal area: flat means uniform. The mappings, the binning and the camera for the sphere live in logic.hpp and are
// covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kBins = 10;
constexpr float kCenterY = 206.0F;
constexpr float kRadius = 124.0F;       // disc and sphere radius in px
constexpr float kBarBaseline = 420.0F;
constexpr float kBarMaxHeight = 56.0F;  // height of a bar at kBarMaxRatio
constexpr double kBarMaxRatio = 3.5;
constexpr float kChipY = 486.0F;
constexpr float kChipH = 34.0F;
constexpr std::array<float, 2> kPanelX{244.0F, 716.0F};
constexpr std::array<int, 3> kCounts{1000, 5000, 20000};
constexpr double kSpinSpeed = 0.4;  // rad/s
constexpr gd::Color kGuide{17, 24, 39, 45};

enum class Action { ShapeDisc, ShapeTriangle, ShapeSphere, Count1k, Count5k, Count20k, NewPoints, Fold };

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
    std::vector<SDL_FRect> rects;  // scratch for the point batches
    us::Shape shape = us::Shape::Disc;
    int countIndex = 1;
    bool fold = false;
    std::uint64_t seed = 2024;
    us::Cloud naive;
    us::Cloud good;
    double yaw = 0.6;
    double pitch = 0.5;
    bool dragging = false;
    SDL_FPoint lastMouse{0.0F, 0.0F};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

us::Method goodMethod(const App& app) { return app.shape == us::Shape::Triangle && app.fold ? us::Method::Fold : us::Method::Uniform; }

void regenerate(App& app) {
    const int n = kCounts[static_cast<std::size_t>(app.countIndex)];
    app.naive = us::generate(app.shape, us::Method::Naive, n, app.seed, kBins);
    app.good = us::generate(app.shape, goodMethod(app), n, app.seed, kBins);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Action::ShapeDisc, "1 Disc", 0.0F},
                                   {Action::ShapeTriangle, "2 Triangle", 8.0F},
                                   {Action::ShapeSphere, "3 Sphere", 8.0F},
                                   {Action::Count1k, "4 1k", 24.0F},
                                   {Action::Count5k, "5 5k", 8.0F},
                                   {Action::Count20k, "6 20k", 8.0F},
                                   {Action::NewPoints, "S New points", 24.0F},
                                   {Action::Fold, "F Fold", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

bool enabled(const App& app, Action action) { return action != Action::Fold || app.shape == us::Shape::Triangle; }

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ShapeDisc: return app.shape == us::Shape::Disc;
        case Action::ShapeTriangle: return app.shape == us::Shape::Triangle;
        case Action::ShapeSphere: return app.shape == us::Shape::Sphere;
        case Action::Count1k: return app.countIndex == 0;
        case Action::Count5k: return app.countIndex == 1;
        case Action::Count20k: return app.countIndex == 2;
        case Action::NewPoints: return false;
        case Action::Fold: return app.fold && app.shape == us::Shape::Triangle;
    }
    return false;
}

void setShape(App& app, us::Shape shape) {
    app.shape = shape;
    app.dragging = false;
    regenerate(app);
}

void apply(App& app, Action action) {
    if (!enabled(app, action)) return;
    switch (action) {
        case Action::ShapeDisc: setShape(app, us::Shape::Disc); break;
        case Action::ShapeTriangle: setShape(app, us::Shape::Triangle); break;
        case Action::ShapeSphere: setShape(app, us::Shape::Sphere); break;
        case Action::Count1k: app.countIndex = 0; regenerate(app); break;
        case Action::Count5k: app.countIndex = 1; regenerate(app); break;
        case Action::Count20k: app.countIndex = 2; regenerate(app); break;
        case Action::NewPoints: ++app.seed; regenerate(app); break;
        case Action::Fold: app.fold = !app.fold; regenerate(app); break;
    }
}

struct Corners {
    us::Vec2 a, b, c;
};

Corners triangleCorners(float cx) {
    constexpr float kHalf = 148.0F;
    constexpr float kHeightPx = 256.0F;
    const float top = kCenterY - kHeightPx / 2.0F - 2.0F;
    const float bottom = top + kHeightPx;
    return {{static_cast<double>(cx), static_cast<double>(top)}, {static_cast<double>(cx - kHalf), static_cast<double>(bottom)}, {static_cast<double>(cx + kHalf), static_cast<double>(bottom)}};
}

void line(SDL_Renderer* r, double x0, double y0, double x1, double y1) {
    SDL_RenderLine(r, static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(x1), static_cast<float>(y1));
}

void drawGuides(const App& app, float cx) {
    SDL_Renderer* r = app.renderer;
    gd::setColor(r, kGuide);
    switch (app.shape) {
        case us::Shape::Disc:
            for (int k = 1; k < kBins; ++k) gd::ring(r, cx, kCenterY, kRadius * static_cast<float>(std::sqrt(static_cast<double>(k) / kBins)), kGuide, 72);
            break;
        case us::Shape::Triangle: {
            const Corners t = triangleCorners(cx);
            for (int k = 1; k < kBins; ++k) {
                const double h = std::sqrt(static_cast<double>(k) / kBins);  // distance from the apex, so the stripe above it has area k/10
                const us::Vec2 p = us::triangleToPlane(t.a, t.b, t.c, {1.0 - h, h, 0.0});
                const us::Vec2 q = us::triangleToPlane(t.a, t.b, t.c, {1.0 - h, 0.0, h});
                line(r, p.x, p.y, q.x, q.y);
            }
            break;
        }
        case us::Shape::Sphere:
            for (int k = 1; k < kBins; ++k) {
                const double z = 2.0 * static_cast<double>(k) / kBins - 1.0;
                const double rho = std::sqrt(std::max(0.0, 1.0 - z * z));
                constexpr int kSteps = 72;
                us::View prev = us::project({rho, 0.0, z}, app.yaw, app.pitch);
                for (int i = 1; i <= kSteps; ++i) {
                    const double phi = us::kTwoPi * i / kSteps;
                    const us::View cur = us::project({rho * std::cos(phi), rho * std::sin(phi), z}, app.yaw, app.pitch);
                    if (prev.depth < 0.0 && cur.depth < 0.0) {
                        line(r, cx + prev.x * kRadius, kCenterY - prev.y * kRadius, cx + cur.x * kRadius, kCenterY - cur.y * kRadius);
                    }
                    prev = cur;
                }
            }
            break;
    }
}

void drawBackground(const App& app, float cx) {
    SDL_Renderer* r = app.renderer;
    if (app.shape == us::Shape::Triangle) {
        const Corners t = triangleCorners(cx);
        const SDL_FColor white = gd::fcolor(gd::kWhite);
        const SDL_Vertex v[3] = {{{static_cast<float>(t.a.x), static_cast<float>(t.a.y)}, white, {0, 0}},
                                 {{static_cast<float>(t.b.x), static_cast<float>(t.b.y)}, white, {0, 0}},
                                 {{static_cast<float>(t.c.x), static_cast<float>(t.c.y)}, white, {0, 0}}};
        SDL_RenderGeometry(r, nullptr, v, 3, nullptr, 0);
    } else {
        gd::disc(r, cx, kCenterY, kRadius, gd::kWhite, 72);
    }
}

void drawOutline(const App& app, float cx) {
    SDL_Renderer* r = app.renderer;
    if (app.shape == us::Shape::Triangle) {
        const Corners t = triangleCorners(cx);
        gd::setColor(r, gd::kInk);
        line(r, t.a.x, t.a.y, t.b.x, t.b.y);
        line(r, t.b.x, t.b.y, t.c.x, t.c.y);
        line(r, t.c.x, t.c.y, t.a.x, t.a.y);
    } else {
        gd::ring(r, cx, kCenterY, kRadius, gd::kInk, 96);
    }
}

void drawPoints(App& app, const us::Cloud& cloud, float cx, gd::Color color) {
    app.rects.clear();
    const float size = cloud.points.size() <= 1000 ? 3.0F : 2.0F;
    const Corners t = triangleCorners(cx);
    for (const us::Vec3& p : cloud.points) {
        float x = 0.0F;
        float y = 0.0F;
        switch (app.shape) {
            case us::Shape::Disc:
                x = cx + static_cast<float>(p.x) * kRadius;
                y = kCenterY - static_cast<float>(p.y) * kRadius;
                break;
            case us::Shape::Triangle: {
                const us::Vec2 q = us::triangleToPlane(t.a, t.b, t.c, p);
                x = static_cast<float>(q.x);
                y = static_cast<float>(q.y);
                break;
            }
            case us::Shape::Sphere: {
                const us::View v = us::project(p, app.yaw, app.pitch);
                if (v.depth >= 0.0) continue;  // far side
                x = cx + static_cast<float>(v.x) * kRadius;
                y = kCenterY - static_cast<float>(v.y) * kRadius;
                break;
            }
        }
        app.rects.push_back(SDL_FRect{x - size / 2.0F, y - size / 2.0F, size, size});
    }
    gd::setColor(app.renderer, color);
    if (!app.rects.empty()) SDL_RenderFillRects(app.renderer, app.rects.data(), static_cast<int>(app.rects.size()));
}

void drawBars(const App& app, const us::Cloud& cloud, float cx, gd::Color color) {
    SDL_Renderer* r = app.renderer;
    constexpr float kBarW = 28.0F;
    constexpr float kGap = 2.0F;
    const float left = cx - (kBins * kBarW + (kBins - 1) * kGap) / 2.0F;
    long long total = 0;
    for (int c : cloud.bins) total += c;
    const double expected = total > 0 ? static_cast<double>(total) / kBins : 0.0;
    for (int i = 0; i < kBins; ++i) {
        const double ratio = expected > 0.0 ? cloud.bins[static_cast<std::size_t>(i)] / expected : 0.0;
        const float h = static_cast<float>(std::min(ratio, kBarMaxRatio) / kBarMaxRatio) * kBarMaxHeight;
        gd::fillRect(r, left + static_cast<float>(i) * (kBarW + kGap), kBarBaseline - h, kBarW, h, color);
    }
    gd::fillRect(r, left - 4.0F, kBarBaseline, kBins * (kBarW + kGap) + 6.0F, 1.0F, gd::kInk);
    // The expected level (ratio 1.0), dashed, drawn over the bars.
    const float ey = kBarBaseline - static_cast<float>(1.0 / kBarMaxRatio) * kBarMaxHeight;
    gd::setColor(r, gd::kInk);
    const float right = left + kBins * (kBarW + kGap) - kGap;
    for (float x = left - 4.0F; x < right + 4.0F; x += 8.0F) SDL_RenderLine(r, x, ey, std::min(x + 4.0F, right + 4.0F), ey);
}

void drawPanel(App& app, int side) {
    const bool isGood = side == 1;
    const float cx = kPanelX[static_cast<std::size_t>(side)];
    const us::Cloud& cloud = isGood ? app.good : app.naive;
    const gd::Color color = isGood ? gd::kTeal : gd::kOrange;
    const gd::Color dot{color.r, color.g, color.b, 200};

    const char* formula = "";
    switch (app.shape) {
        case us::Shape::Disc: formula = isGood ? "r = R * sqrt(u)" : "r = R * u"; break;
        case us::Shape::Triangle: formula = !isGood ? "a = 1 - t" : app.fold ? "fold if s + t > 1" : "a = 1 - sqrt(t)"; break;
        case us::Shape::Sphere: formula = isGood ? "z = 1 - 2u" : "theta = pi * u"; break;
    }
    const std::string tag = isGood ? "CORRECT" : "NAIVE";
    const float labelX = cx - 200.0F;
    app.hud.text(app.renderer, labelX, 40.0F, tag, color, gd::Hud::Style::Bold);
    app.hud.text(app.renderer, labelX + app.hud.width(tag, gd::Hud::Style::Bold) + 16.0F, 40.0F, formula, gd::kInk);

    drawBackground(app, cx);
    drawGuides(app, cx);
    drawPoints(app, cloud, cx, dot);
    drawOutline(app, cx);
    drawBars(app, cloud, cx, color);

    const us::Spread s = us::spread(cloud.bins);
    char buf[64];
    SDL_snprintf(buf, sizeof buf, "bands %.2fx to %.2fx of flat", s.minRatio, s.maxRatio);
    app.hud.text(app.renderer, cx - app.hud.width(buf) / 2.0F, kBarBaseline + 8.0F, buf, isGood ? gd::kTealDark : gd::kOrange, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    const std::string points = us::withThousands(kCounts[static_cast<std::size_t>(app.countIndex)]);
    app.hud.titleBar(r, kWidth, "[ UNIFORM SAMPLING ]", points + " points per side, same random numbers");

    drawPanel(app, 0);
    drawPanel(app, 1);

    const char* what = "";
    switch (app.shape) {
        case us::Shape::Disc: what = "bars: 10 rings of equal area, center to rim"; break;
        case us::Shape::Triangle: what = "bars: 10 stripes of equal area, apex A to base"; break;
        case us::Shape::Sphere: what = "bars: 10 zones of equal area, pole to pole, far side counts. Drag to turn"; break;
    }
    app.hud.text(r, 40.0F, 458.0F, what, gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool usable = enabled(app, c.action);
        const gd::Color textColor = !usable ? gd::kMuted : on ? gd::kWhite : gd::kInk;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, usable ? gd::kInk : gd::kMuted);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, textColor, gd::Hud::Style::Bold);
    }
}

void update(App& app) {
    const Uint64 now = SDL_GetTicks();
    const double dt = std::min(static_cast<double>(now - app.lastTicks) / 1000.0, 0.25);
    app.lastTicks = now;
    if (!app.dragging) app.yaw = us::wrapAngle(app.yaw + kSpinSpeed * dt);
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
    if (!SDL_CreateWindowAndRenderer("uniform-sampling", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->rects.reserve(20000);
    layoutChips(*app);
    regenerate(*app);
    app->lastTicks = SDL_GetTicks();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.dragging) {
                app.yaw = us::wrapAngle(app.yaw + 0.01 * static_cast<double>(event->motion.x - app.lastMouse.x));
                app.pitch = std::clamp(app.pitch + 0.01 * static_cast<double>(event->motion.y - app.lastMouse.y), -us::kPi / 2.0, us::kPi / 2.0);
            }
            app.lastMouse = {event->motion.x, event->motion.y};
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.lastMouse = p;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                }
            }
            if (!onChip && app.shape == us::Shape::Sphere && p.y > 30.0F && p.y < kChipY - 8.0F) app.dragging = true;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging = false; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::ShapeDisc); break;
                case SDLK_2: apply(app, Action::ShapeTriangle); break;
                case SDLK_3: apply(app, Action::ShapeSphere); break;
                case SDLK_4: apply(app, Action::Count1k); break;
                case SDLK_5: apply(app, Action::Count5k); break;
                case SDLK_6: apply(app, Action::Count20k); break;
                case SDLK_S: apply(app, Action::NewPoints); break;
                case SDLK_F: apply(app, Action::Fold); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    update(app);
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
