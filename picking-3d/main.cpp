// Picking in 3D space: which object is under the mouse?
//
// The scene is drawn in software (own matrices, SDL_RenderGeometry for the triangles, objects sorted back to
// front), and picking never looks at the drawn pixels. The mouse pixel becomes a ray in world space (NDC, then the
// inverse of projection * view), and the ray is tested against the real spheres and boxes. The nearest hit wins.
// The small top view on the right shows that ray from above: picking is geometry in the world, not in the picture.
//
// Move the mouse over an object (orange outline), click to select it (the ray stays pinned in the 3D view), drag
// the background or use the right button to orbit, wheel to zoom. Keys: R toggles the ray view, C resets the camera.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

using pk::Mat4;
using pk::Object;
using pk::Ray;
using pk::Vec3;
using pk::Vec4;

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr pk::Viewport kView{0.0F, 30.0F, 690.0F, 510.0F};
constexpr float kPanelX = 700.0F;
constexpr float kPanelW = 250.0F;
constexpr float kFloorHalf = 8.0F;
constexpr float kDragThreshold = 4.0F;  // pixels the mouse has to move before a press counts as an orbit drag
constexpr float kOrbitSpeed = 0.008F;   // radians per pixel
constexpr float kInsetRange = 11.0F;    // the top view shows -11..11 on both axes

struct Hover {
    bool valid = false;
    int object = -1;  // -1: the floor
    float t = 0.0F;
    Vec3 point;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;

    pk::OrbitCamera cam;
    std::vector<Object> scene = pk::defaultScene();
    pk::Mesh boxMesh = pk::unitBox();
    pk::Mesh ballMesh = pk::icosphere(2);
    bool showRay = true;

    float mouseX = 0.0F, mouseY = 0.0F;
    bool hasMouse = true;
    bool leftDown = false, rightDown = false, dragging = false;
    float downX = 0.0F, downY = 0.0F, lastX = 0.0F, lastY = 0.0F;

    int selected = -1;
    bool pinned = false;
    Vec3 pinOrigin, pinHit;

    // derived once per frame from the camera and the mouse
    Mat4 viewProj, inverseViewProj;
    Ray ray{};
    bool hasRay = false;
    Hover hover;

    long maxFrames = -1;
    long frames = 0;
};

bool inView(float x, float y) { return x >= kView.x && x < kView.x + kView.w && y >= kView.y && y < kView.y + kView.h; }

SDL_FRect chipRay() { return SDL_FRect{kPanelX, 448.0F, 121.0F, 34.0F}; }
SDL_FRect chipReset() { return SDL_FRect{kPanelX + 129.0F, 448.0F, 121.0F, 34.0F}; }
SDL_FRect insetRect() { return SDL_FRect{kPanelX, 172.0F, kPanelW, 258.0F}; }

void updateMatrices(App& app) {
    app.viewProj = pk::perspective(pk::kFovY, kView.w / kView.h, pk::kNear, pk::kFar) * app.cam.view();
    app.inverseViewProj = pk::inverse(app.viewProj);
}

/// The whole chain: pixel -> ray -> nearest hit. A floor hit outside the drawn floor does not count.
void updateHover(App& app, float x, float y, Hover& out, Ray& ray, bool& hasRay) {
    out = Hover{};
    hasRay = app.hasMouse && inView(x, y);
    if (!hasRay) return;
    ray = pk::pixelToRay(app.inverseViewProj, kView, x, y, app.cam.eye());
    const auto hit = pk::pick(app.scene, ray);
    if (!hit) return;
    const Vec3 point = ray.origin + ray.dir * hit->t;
    if (hit->floor && (std::fabs(point.x) > kFloorHalf || std::fabs(point.z) > kFloorHalf)) return;
    out = Hover{true, hit->object, hit->t, point};
}

void refreshHover(App& app) {
    updateMatrices(app);
    if (app.dragging) {
        app.hover = Hover{};
        app.hasRay = false;
        return;
    }
    updateHover(app, app.mouseX, app.mouseY, app.hover, app.ray, app.hasRay);
}

void clickAt(App& app) {
    refreshHover(app);
    if (app.hover.valid) {
        app.selected = app.hover.object;
        app.pinned = true;
        app.pinOrigin = app.ray.origin;
        app.pinHit = app.hover.point;
    } else {
        app.selected = -1;
        app.pinned = false;
    }
}

void resetCamera(App& app) {
    app.cam = pk::OrbitCamera{};
    refreshHover(app);
}

// ------------------------------------------------------------------------------------------------ drawing helpers

SDL_FPoint toPixel(const Vec4& c) {
    const float nx = c.x / c.w;
    const float ny = c.y / c.w;
    return {kView.x + (nx * 0.5F + 0.5F) * kView.w, kView.y + (1.0F - (ny * 0.5F + 0.5F)) * kView.h};
}

Vec4 toClip(const App& app, Vec3 p) { return app.viewProj * Vec4{p.x, p.y, p.z, 1.0F}; }

gd::Color mix(gd::Color a, gd::Color b, float s) {
    const auto lerp = [s](Uint8 x, Uint8 y) { return static_cast<Uint8>(static_cast<float>(x) + (static_cast<float>(y) - static_cast<float>(x)) * s); };
    return gd::Color{lerp(a.r, b.r), lerp(a.g, b.g), lerp(a.b, b.b), 255};
}

/// Triangle fan for a convex polygon, appended to a vertex list (no index buffer needed).
void addFan(std::vector<SDL_Vertex>& out, const std::vector<SDL_FPoint>& pts, gd::Color color) {
    const SDL_FColor c = gd::fcolor(color);
    for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
        out.push_back({pts[0], c, {0, 0}});
        out.push_back({pts[i], c, {0, 0}});
        out.push_back({pts[i + 1], c, {0, 0}});
    }
}

void flush(SDL_Renderer* r, const std::vector<SDL_Vertex>& v) {
    if (!v.empty()) SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), nullptr, 0);
}

void fillClip(SDL_Renderer* r, const std::vector<Vec4>& clipped, gd::Color color) {
    std::vector<SDL_FPoint> pts;
    for (const Vec4& c : clipped) pts.push_back(toPixel(c));
    std::vector<SDL_Vertex> v;
    addFan(v, pts, color);
    flush(r, v);
}

/// A 3D line, cut at the near plane. `thick` repeats it a pixel to each side.
void line3(App& app, Vec3 a, Vec3 b, gd::Color color, bool thick = false) {
    const auto seg = pk::clipNear({toClip(app, a), toClip(app, b)});
    if (seg.size() != 2) return;
    const SDL_FPoint p = toPixel(seg[0]);
    const SDL_FPoint q = toPixel(seg[1]);
    gd::setColor(app.renderer, color);
    SDL_RenderLine(app.renderer, p.x, p.y, q.x, q.y);
    if (thick) {
        for (const SDL_FPoint d : {SDL_FPoint{1, 0}, SDL_FPoint{-1, 0}, SDL_FPoint{0, 1}, SDL_FPoint{0, -1}}) {
            SDL_RenderLine(app.renderer, p.x + d.x, p.y + d.y, q.x + d.x, q.y + d.y);
        }
    }
}

void drawFloor(App& app) {
    const Vec3 corners[4] = {{-kFloorHalf, 0, -kFloorHalf}, {kFloorHalf, 0, -kFloorHalf}, {kFloorHalf, 0, kFloorHalf}, {-kFloorHalf, 0, kFloorHalf}};
    std::vector<Vec4> quad;
    for (const Vec3 c : corners) quad.push_back(toClip(app, c));
    fillClip(app.renderer, pk::clipNear(quad), gd::Color{222, 244, 240, 255});
    const int n = static_cast<int>(kFloorHalf);
    for (int i = -n; i <= n; ++i) {
        const gd::Color c = i == 0 ? gd::Color{13, 148, 136, 170} : gd::Color{13, 148, 136, 60};
        const auto f = static_cast<float>(i);
        line3(app, {-kFloorHalf, 0, f}, {kFloorHalf, 0, f}, c);
        line3(app, {f, 0, -kFloorHalf}, {f, 0, kFloorHalf}, c);
    }
}

/// Contact shadow: the footprint of everything that stands on the floor, so the eye can read where it stands.
void drawShadows(App& app) {
    std::vector<SDL_Vertex> v;
    for (const Object& o : app.scene) {
        if (pk::boundsMin(o).y > 0.01F) continue;
        std::vector<Vec4> poly;
        if (o.kind == Object::Kind::Sphere) {
            for (int i = 0; i < 20; ++i) {
                const float a = 6.2831853F * static_cast<float>(i) / 20.0F;
                poly.push_back(toClip(app, {o.center.x + std::cos(a) * o.radius() * 1.1F, 0.0F, o.center.z + std::sin(a) * o.radius() * 1.1F}));
            }
        } else {
            const float e = 1.1F;
            for (const auto& s : {std::pair{-1, -1}, std::pair{1, -1}, std::pair{1, 1}, std::pair{-1, 1}}) {
                poly.push_back(toClip(app, {o.center.x + static_cast<float>(s.first) * o.half.x * e, 0.0F, o.center.z + static_cast<float>(s.second) * o.half.z * e}));
            }
        }
        std::vector<SDL_FPoint> pts;
        for (const Vec4& c : pk::clipNear(poly)) pts.push_back(toPixel(c));
        addFan(v, pts, gd::Color{17, 24, 39, 46});
    }
    flush(app.renderer, v);
}

struct Palette {
    gd::Color dark, light;
};
constexpr Palette kBoxTeal{{6, 66, 62, 255}, {122, 230, 214, 255}};
constexpr Palette kBallTeal{{14, 78, 96, 255}, {156, 238, 234, 255}};
constexpr Palette kOrangeSel{{128, 42, 6, 255}, {253, 192, 142, 255}};

pk::Mesh worldMesh(const App& app, const Object& o) {
    pk::Mesh m = o.kind == Object::Kind::Sphere ? app.ballMesh : app.boxMesh;
    for (Vec3& p : m.v) {
        p = o.kind == Object::Kind::Sphere ? o.center + p * o.radius() : o.center + Vec3{p.x * o.half.x, p.y * o.half.y, p.z * o.half.z};
    }
    return m;
}

void drawObject(App& app, int index, bool hovered) {
    const Object& o = app.scene[static_cast<std::size_t>(index)];
    const bool selected = index == app.selected;
    const Palette& pal = selected ? kOrangeSel : (o.kind == Object::Kind::Sphere ? kBallTeal : kBoxTeal);
    const pk::Mesh mesh = worldMesh(app, o);
    const Vec3 eye = app.cam.eye();
    const Vec3 light = pk::normalize({0.5F, 0.8F, 0.45F});

    std::vector<Vec4> clip;
    clip.reserve(mesh.v.size());
    for (const Vec3 p : mesh.v) clip.push_back(toClip(app, p));

    std::vector<SDL_Vertex> tris;
    for (const pk::Tri& tri : mesh.t) {
        if (!pk::frontFacing(mesh, tri, eye)) continue;  // back-face culling: a convex shape never shows its far side
        const Vec3 a = mesh.v[static_cast<std::size_t>(tri[0])];
        const Vec3 n = pk::normalize(pk::cross(mesh.v[static_cast<std::size_t>(tri[1])] - a, mesh.v[static_cast<std::size_t>(tri[2])] - a));
        const float shade = std::clamp(0.15F + 0.85F * std::max(0.0F, pk::dot(n, light)), 0.0F, 1.0F);
        std::vector<Vec4> poly{clip[static_cast<std::size_t>(tri[0])], clip[static_cast<std::size_t>(tri[1])], clip[static_cast<std::size_t>(tri[2])]};
        poly = pk::clipNear(poly);
        std::vector<SDL_FPoint> pts;
        for (const Vec4& c : poly) pts.push_back(toPixel(c));
        addFan(tris, pts, mix(pal.dark, pal.light, shade));
    }
    flush(app.renderer, tris);

    // outline: thin ink for everything, thick orange for the hovered object (ink when it is selected, which is orange)
    const gd::Color outline = hovered ? (selected ? gd::kInk : gd::kOrange) : gd::Color{17, 24, 39, 120};
    for (const auto& [i, j] : pk::silhouette(mesh, eye)) {
        line3(app, mesh.v[static_cast<std::size_t>(i)], mesh.v[static_cast<std::size_t>(j)], outline, hovered);
    }
}

void drawScene(App& app) {
    drawFloor(app);
    drawShadows(app);

    // Painter's algorithm per object: far to near. Convex shapes that do not overlap need nothing finer.
    const Vec3 eye = app.cam.eye();
    std::vector<int> order(app.scene.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return pk::length(app.scene[static_cast<std::size_t>(a)].center - eye) > pk::length(app.scene[static_cast<std::size_t>(b)].center - eye);
    });
    for (const int i : order) drawObject(app, i, app.hover.valid && app.hover.object == i);

    // the ray of the last click, seen from the current camera (it starts where the camera was)
    if (app.showRay && app.pinned) {
        line3(app, app.pinOrigin, app.pinHit, gd::kOrange, true);
        if (const auto o = pk::project(app.viewProj, kView, app.pinOrigin)) gd::fillRect(app.renderer, o->x - 4.0F, o->y - 4.0F, 8.0F, 8.0F, gd::kInk);
        if (const auto h = pk::project(app.viewProj, kView, app.pinHit)) {
            gd::disc(app.renderer, h->x, h->y, 4.0F, gd::kInk);
        }
    }

    // where the hovered ray lands
    if (app.hover.valid) {
        if (const auto p = pk::project(app.viewProj, kView, app.hover.point)) {
            gd::disc(app.renderer, p->x, p->y, 5.0F, gd::kOrange);
            gd::ring(app.renderer, p->x, p->y, 6.5F, gd::kInk);
            if (app.hover.object >= 0) {
                const char* name = app.scene[static_cast<std::size_t>(app.hover.object)].name;
                const float w = app.hud.width(name, gd::Hud::Style::Bold) + 16.0F;
                const float x = std::min(p->x + 14.0F, kView.x + kView.w - w - 6.0F);
                const float y = std::min(p->y + 14.0F, kView.y + kView.h - 30.0F);
                gd::card(app.renderer, x, y, w, 24.0F, gd::kWhite, gd::kInk);
                app.hud.text(app.renderer, x + 8.0F, y + 1.0F, name, gd::kInk, gd::Hud::Style::Bold);
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------ side panel

void drawChip(App& app, const SDL_FRect& c, const char* label, bool active) {
    gd::card(app.renderer, c.x, c.y, c.w, c.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
    app.hud.text(app.renderer, c.x + (c.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.y + 6.0F, label, active ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
}

void infoRow(App& app, float y, const char* label, const char* value, gd::Color color = gd::kInk) {
    app.hud.text(app.renderer, kPanelX + 8.0F, y, label, gd::kMuted);
    app.hud.text(app.renderer, kPanelX + 68.0F, y, value, color, gd::Hud::Style::Bold);
}

void drawInfo(App& app) {
    gd::card(app.renderer, kPanelX, 42.0F, kPanelW, 118.0F, gd::kWhite, gd::kInk);
    char text[64];
    const Hover& h = app.hover;
    if (app.hasRay) {
        SDL_snprintf(text, sizeof text, "%5.2f %5.2f %5.2f", static_cast<double>(app.ray.dir.x), static_cast<double>(app.ray.dir.y), static_cast<double>(app.ray.dir.z));
    } else {
        SDL_snprintf(text, sizeof text, "-");
    }
    infoRow(app, 50.0F, "dir", text);
    const char* name = !h.valid ? "-" : h.object < 0 ? "floor" : app.scene[static_cast<std::size_t>(h.object)].name;
    infoRow(app, 72.0F, "hit", name, h.valid ? gd::kOrange : gd::kMuted);
    if (h.valid) {
        SDL_snprintf(text, sizeof text, "%.2f", static_cast<double>(h.t));
    } else {
        SDL_snprintf(text, sizeof text, "-");
    }
    infoRow(app, 94.0F, "t", text);
    if (h.valid) {
        SDL_snprintf(text, sizeof text, "%.1f %.1f %.1f", static_cast<double>(h.point.x), static_cast<double>(h.point.y), static_cast<double>(h.point.z));
    } else {
        SDL_snprintf(text, sizeof text, "-");
    }
    infoRow(app, 116.0F, "point", text);
    infoRow(app, 138.0F, "sel", app.selected < 0 ? "none" : app.scene[static_cast<std::size_t>(app.selected)].name, app.selected < 0 ? gd::kMuted : gd::kInk);
}

/// Top view (x right, z down) of the same scene: footprints, the camera with its view wedge, and the pick ray.
void drawInset(App& app) {
    const SDL_FRect box = insetRect();
    SDL_Renderer* r = app.renderer;
    gd::card(r, box.x, box.y, box.w, box.h, gd::kWhite, gd::kInk);
    if (!app.showRay) {
        app.hud.text(r, box.x + 36.0F, box.y + box.h / 2.0F - 8.0F, "ray view off (R)", gd::kMuted);
        return;
    }
    const float scale = box.w / (2.0F * kInsetRange);
    const float cx = box.x + box.w / 2.0F;
    const float cy = box.y + box.h / 2.0F;
    const auto mapX = [&](float x) { return cx + x * scale; };
    const auto mapY = [&](float z) { return cy + z * scale; };

    const SDL_Rect clip{static_cast<int>(box.x) + 1, static_cast<int>(box.y) + 1, static_cast<int>(box.w) - 1, static_cast<int>(box.h) - 1};
    SDL_SetRenderClipRect(r, &clip);

    for (int i = -10; i <= 10; i += 2) {
        gd::setColor(r, i == 0 ? gd::Color{13, 148, 136, 120} : gd::kGrid);
        SDL_RenderLine(r, mapX(static_cast<float>(i)), box.y, mapX(static_cast<float>(i)), box.y + box.h);
        SDL_RenderLine(r, box.x, mapY(static_cast<float>(i)), box.x + box.w, mapY(static_cast<float>(i)));
    }
    gd::strokeRect(r, mapX(-kFloorHalf), mapY(-kFloorHalf), 2.0F * kFloorHalf * scale, 2.0F * kFloorHalf * scale, gd::Color{13, 148, 136, 150});

    for (std::size_t i = 0; i < app.scene.size(); ++i) {
        const Object& o = app.scene[i];
        const bool sel = static_cast<int>(i) == app.selected;
        const bool hov = app.hover.valid && app.hover.object == static_cast<int>(i);
        const gd::Color fill = sel ? gd::Color{253, 192, 142, 255} : gd::kTealSoft;
        const gd::Color edge = hov ? gd::kOrange : (sel ? gd::kOrange : gd::kTealDark);
        if (o.kind == Object::Kind::Sphere) {
            gd::disc(r, mapX(o.center.x), mapY(o.center.z), o.radius() * scale, fill);
            gd::ring(r, mapX(o.center.x), mapY(o.center.z), o.radius() * scale, edge);
            if (hov) gd::ring(r, mapX(o.center.x), mapY(o.center.z), o.radius() * scale + 1.0F, edge);
        } else {
            gd::fillRect(r, mapX(o.center.x - o.half.x), mapY(o.center.z - o.half.z), o.half.x * 2.0F * scale, o.half.z * 2.0F * scale, fill);
            gd::strokeRect(r, mapX(o.center.x - o.half.x), mapY(o.center.z - o.half.z), o.half.x * 2.0F * scale, o.half.z * 2.0F * scale, edge);
            if (hov) {
                gd::strokeRect(r, mapX(o.center.x - o.half.x) - 1.0F, mapY(o.center.z - o.half.z) - 1.0F, o.half.x * 2.0F * scale + 2.0F, o.half.z * 2.0F * scale + 2.0F, edge);
            }
        }
    }

    // camera and the edges of its view (rays through the left and right end of the middle pixel row)
    const Vec3 eye = app.cam.eye();
    const float midY = kView.y + kView.h / 2.0F;
    gd::setColor(r, gd::Color{107, 114, 128, 150});
    for (const float px : {kView.x, kView.x + kView.w}) {
        const Ray edge = pk::pixelToRay(app.inverseViewProj, kView, px, midY, eye);
        const Vec3 horizontal = pk::normalize({edge.dir.x, 0.0F, edge.dir.z});
        SDL_RenderLine(r, mapX(eye.x), mapY(eye.z), mapX(eye.x + horizontal.x * 40.0F), mapY(eye.z + horizontal.z * 40.0F));
    }

    // the pick ray from above, up to the hit (or far away when it hits nothing)
    if (app.hasRay) {
        const Vec3 end = app.hover.valid ? app.hover.point : app.ray.origin + app.ray.dir * 60.0F;
        for (const SDL_FPoint d : {SDL_FPoint{0, 0}, SDL_FPoint{0.7F, 0.7F}, SDL_FPoint{-0.7F, -0.7F}}) {
            gd::setColor(r, gd::kOrange);
            SDL_RenderLine(r, mapX(eye.x) + d.x, mapY(eye.z) + d.y, mapX(end.x) + d.x, mapY(end.z) + d.y);
        }
        if (app.hover.valid) {
            // the ray does not stop at the hit, the nearest hit just wins
            const Vec3 beyond = app.ray.origin + app.ray.dir * 60.0F;
            gd::setColor(r, gd::Color{234, 88, 12, 90});
            SDL_RenderLine(r, mapX(end.x), mapY(end.z), mapX(beyond.x), mapY(beyond.z));
            gd::disc(r, mapX(end.x), mapY(end.z), 4.0F, gd::kOrange);
            gd::ring(r, mapX(end.x), mapY(end.z), 5.5F, gd::kInk);
        }
    }
    gd::disc(r, mapX(eye.x), mapY(eye.z), 5.0F, gd::kInk);
    SDL_SetRenderClipRect(r, nullptr);
    app.hud.text(r, box.x + 6.0F, box.y + 4.0F, "top view", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("picking-3d", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);

    // The first frame should already show a hit: start with the mouse on the big ball.
    updateMatrices(*app);
    if (const auto p = pk::project(app->viewProj, kView, app->scene[3].center)) {
        app->mouseX = p->x;
        app->mouseY = p->y;
    }
    refreshHover(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT:
            return SDL_APP_SUCCESS;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // R toggles, C resets: neither is meant to repeat while held
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            if (event->key.key == SDLK_R) app.showRay = !app.showRay;
            if (event->key.key == SDLK_C) resetCamera(app);
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.hasMouse = false;
            app.leftDown = app.rightDown = app.dragging = false;  // a release outside the canvas never arrives as BUTTON_UP
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.leftDown = app.rightDown = app.dragging = false;
            break;
        case SDL_EVENT_MOUSE_MOTION: {
            app.hasMouse = true;
            app.mouseX = event->motion.x;
            app.mouseY = event->motion.y;
            if ((app.leftDown || app.rightDown) && !app.dragging) {
                const float moved = std::hypot(app.mouseX - app.downX, app.mouseY - app.downY);
                if (app.rightDown || moved > kDragThreshold) app.dragging = true;
            }
            if (app.dragging) {
                app.cam.orbit(-(app.mouseX - app.lastX) * kOrbitSpeed, (app.mouseY - app.lastY) * kOrbitSpeed);
            }
            app.lastX = app.mouseX;
            app.lastY = app.mouseY;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouseX = p.x;
            app.mouseY = p.y;
            app.lastX = p.x;
            app.lastY = p.y;
            const SDL_FRect ray = chipRay();
            const SDL_FRect reset = chipReset();
            if (SDL_PointInRectFloat(&p, &ray)) {
                app.showRay = !app.showRay;
            } else if (SDL_PointInRectFloat(&p, &reset)) {
                resetCamera(app);
            } else if (inView(p.x, p.y)) {
                app.downX = p.x;
                app.downY = p.y;
                if (event->button.button == SDL_BUTTON_LEFT) app.leftDown = true;
                if (event->button.button == SDL_BUTTON_RIGHT) app.rightDown = true;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT && app.leftDown) {
                app.leftDown = false;
                if (!app.dragging) {
                    app.mouseX = event->button.x;
                    app.mouseY = event->button.y;
                    clickAt(app);
                }
            }
            if (event->button.button == SDL_BUTTON_RIGHT) app.rightDown = false;
            if (!app.leftDown && !app.rightDown) app.dragging = false;
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            app.cam.zoom(std::clamp(event->wheel.y, -3.0F, 3.0F));
            break;
        default:
            break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    refreshHover(app);

    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);

    const SDL_Rect view{static_cast<int>(kView.x), static_cast<int>(kView.y), static_cast<int>(kView.w), static_cast<int>(kView.h)};
    SDL_SetRenderClipRect(r, &view);
    drawScene(app);
    SDL_SetRenderClipRect(r, nullptr);

    drawInfo(app);
    drawInset(app);
    drawChip(app, chipRay(), "ray (R)", app.showRay);
    drawChip(app, chipReset(), "reset (C)", false);
    app.hud.text(r, kPanelX + 4.0F, 494.0F, "drag: orbit  wheel: zoom", gd::kMuted);
    app.hud.text(r, kPanelX + 4.0F, 514.0F, "click: pick", gd::kMuted);
    app.hud.titleBar(r, kWidth, "[ PICKING IN 3D ]", "pixel > ray > nearest hit");
    SDL_RenderPresent(r);

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
