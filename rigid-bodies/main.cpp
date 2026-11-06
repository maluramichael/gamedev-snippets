// Rigid bodies: circles and boxes that collide, bounce, slide and stack.
//
// The physics lives in logic.hpp (no SDL, covered by test.cpp). This file only owns the loop and the picture.
// The loop is the fixed timestep from the earlier demos: frame time goes into an accumulator and the world is
// stepped in slices of exactly 1/120 s, so the simulation behaves the same at 30, 60 or 144 frames per second.
// Frame time is clamped, otherwise a stalled tab would trigger hundreds of catch-up steps.
//
// Orange marks what the solver sees: contact points with their normals and the mouse spring. Click empty space
// to spawn a body, drag a body to pull it around, R clears the scene, G switches gravity.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kScale = 30.0F;                 // pixels per meter, one grid cell of the backdrop is one meter
constexpr float kStep = 1.0F / 120.0F;          // fixed physics step
constexpr double kMaxFrameTime = 0.25;          // clamp: no spiral of death after a stall
constexpr float kFloorY = 14.0F;                // top of the floor in meters (420 px)
constexpr float kWorldWidth = 32.0F;            // 960 px
constexpr std::size_t kMaxBodies = 90;          // statics included
constexpr std::array<float, 4> kRestitutions{0.0F, 0.5F, 0.9F, 1.0F};
constexpr std::array<const char*, 4> kRestitutionLabels{"0.0", "0.5", "0.9", "1.0"};

enum class Act { SpawnMix, SpawnCircle, SpawnBox, Restitution, Gravity, Contacts, Clear };
enum class Spawn { Mix, Circle, Box };

struct Chip {
    SDL_FRect rect;
    const char* label;
    Act act;
    int value;
    bool active;
    bool accent;  // orange edge instead of ink
};

struct Label {
    float x;
    float y;
    const char* text;
};

struct Layout {
    std::vector<Chip> chips;
    std::vector<Label> labels;
};

struct Drag {
    int body = -1;
    rb::Vec2 local;   // grabbed point in body coordinates
    rb::Vec2 target;  // mouse in world coordinates
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    rb::World world;
    Drag drag;
    Spawn spawn = Spawn::Mix;
    int restitutionIndex = 1;
    bool gravityOn = true;
    bool showContacts = true;
    int spawned = 0;
    Uint64 last = 0;
    double accumulator = 0.0;
    int stepsThisFrame = 0;
    long maxFrames = -1;
    long frames = 0;
};

rb::Vec2 toWorld(float px, float py) { return {px / kScale, py / kScale}; }

void addScene(rb::World& w) {
    using rb::Body;
    // Statics first: floor, walls (outside the picture), a ramp. Their indices never change.
    w.bodies.push_back(Body::box({kWorldWidth / 2.0F, kFloorY + 1.0F}, {kWorldWidth, 1.0F}, 0.0F, true));
    w.bodies.push_back(Body::box({-1.0F, 7.0F}, {1.0F, 12.0F}, 0.0F, true));
    w.bodies.push_back(Body::box({kWorldWidth + 1.0F, 7.0F}, {1.0F, 12.0F}, 0.0F, true));
    w.bodies.push_back(Body::box({24.0F, 8.6F}, {4.5F, 0.25F}, 0.28F, true));

    // A pyramid of 4-3-2-1 bricks, 1 x 0.6 m, standing exactly on the floor.
    constexpr float kBrickW = 1.0F;
    constexpr float kBrickH = 0.6F;
    for (int row = 0; row < 4; ++row) {
        const int n = 4 - row;
        for (int i = 0; i < n; ++i) {
            const float x = 10.0F + (static_cast<float>(i) - static_cast<float>(n - 1) / 2.0F) * kBrickW;
            w.bodies.push_back(Body::box({x, kFloorY - kBrickH / 2.0F - static_cast<float>(row) * kBrickH}, {kBrickW / 2.0F, kBrickH / 2.0F}));
        }
    }
    // A tower next to it, and a few things that are about to fall: onto the floor, onto the apex, down the ramp.
    for (int i = 0; i < 5; ++i) w.bodies.push_back(Body::box({14.5F, kFloorY - 0.5F - static_cast<float>(i)}, {0.5F, 0.5F}));
    w.bodies.push_back(Body::circle({6.5F, 5.0F}, 0.6F));
    w.bodies.push_back(Body::circle({10.0F, 6.0F}, 0.4F));
    w.bodies.push_back(Body::box({18.0F, 3.0F}, {0.7F, 0.4F}, 0.6F));
    w.bodies.push_back(Body::circle({21.5F, 4.5F}, 0.6F));
    w.bodies.push_back(Body::circle({22.8F, 2.5F}, 0.45F));
    w.bodies.push_back(Body::box({25.0F, 5.0F}, {0.5F, 0.5F}, 0.3F));
}

void spawnBody(App& app, rb::Vec2 at) {
    if (app.world.bodies.size() >= kMaxBodies) return;
    const int n = app.spawned++;
    const bool circle = app.spawn == Spawn::Circle || (app.spawn == Spawn::Mix && n % 2 == 0);
    if (circle) {
        constexpr std::array<float, 3> radii{0.45F, 0.6F, 0.75F};
        app.world.bodies.push_back(rb::Body::circle(at, radii[static_cast<std::size_t>(n / 2 % 3)]));
    } else {
        constexpr std::array<rb::Vec2, 3> sizes{rb::Vec2{0.5F, 0.5F}, rb::Vec2{0.8F, 0.4F}, rb::Vec2{0.4F, 0.7F}};
        app.world.bodies.push_back(rb::Body::box(at, sizes[static_cast<std::size_t>(n / 2 % 3)], 0.5F * static_cast<float>(n % 5)));
    }
}

Layout makeLayout(const App& app) {
    Layout out;
    constexpr float kChipH = 30.0F;
    constexpr float kGap = 8.0F;
    float x = 40.0F;
    float y = 446.0F;

    auto label = [&](const char* text) {
        out.labels.push_back({x, y, text});
        x += app.hud.width(text) + 12.0F;
    };
    auto chip = [&](const char* text, Act act, int value, bool active, bool accent = false) {
        const float w = app.hud.width(text, gd::Hud::Style::Bold) + 24.0F;
        out.chips.push_back({SDL_FRect{x, y, w, kChipH}, text, act, value, active, accent});
        x += w + kGap;
    };

    label("spawn");
    chip("mix", Act::SpawnMix, 0, app.spawn == Spawn::Mix);
    chip("circle", Act::SpawnCircle, 0, app.spawn == Spawn::Circle);
    chip("box", Act::SpawnBox, 0, app.spawn == Spawn::Box);
    x += 20.0F;
    label("restitution");
    for (int i = 0; i < 4; ++i) chip(kRestitutionLabels[static_cast<std::size_t>(i)], Act::Restitution, i, app.restitutionIndex == i);

    x = 40.0F;
    y = 488.0F;
    chip("gravity (G)", Act::Gravity, 0, app.gravityOn);
    chip("contacts", Act::Contacts, 0, app.showContacts);
    chip("clear (R)", Act::Clear, 0, false, true);
    return out;
}

void apply(App& app, const Chip& c) {
    switch (c.act) {
        case Act::SpawnMix: app.spawn = Spawn::Mix; break;
        case Act::SpawnCircle: app.spawn = Spawn::Circle; break;
        case Act::SpawnBox: app.spawn = Spawn::Box; break;
        case Act::Restitution:
            app.restitutionIndex = c.value;
            app.world.restitution = kRestitutions[static_cast<std::size_t>(c.value)];
            break;
        case Act::Gravity:
            app.gravityOn = !app.gravityOn;
            app.world.gravity = app.gravityOn ? rb::Vec2{0.0F, rb::kGravity} : rb::Vec2{0.0F, 0.0F};
            break;
        case Act::Contacts: app.showContacts = !app.showContacts; break;
        case Act::Clear:
            app.world.clearDynamic();
            app.drag.body = -1;
            break;
    }
}

// ---- drawing ----

SDL_FPoint px(rb::Vec2 p) { return SDL_FPoint{p.x * kScale, p.y * kScale}; }

std::array<rb::Vec2, 4> corners(const rb::Body& b) {
    const rb::Rot r = rb::rot(b.angle);
    return {b.pos + rb::mul(r, {-b.half.x, -b.half.y}), b.pos + rb::mul(r, {b.half.x, -b.half.y}), b.pos + rb::mul(r, {b.half.x, b.half.y}),
            b.pos + rb::mul(r, {-b.half.x, b.half.y})};
}

void line(SDL_Renderer* r, rb::Vec2 a, rb::Vec2 b, gd::Color c) {
    gd::setColor(r, c);
    const SDL_FPoint p = px(a);
    const SDL_FPoint q = px(b);
    SDL_RenderLine(r, p.x, p.y, q.x, q.y);
}

void drawBox(SDL_Renderer* r, const rb::Body& b, gd::Color fill, gd::Color edge, bool thick) {
    const auto c = corners(b);
    const SDL_FColor f = gd::fcolor(fill);
    const SDL_Vertex v[4] = {{px(c[0]), f, {0, 0}}, {px(c[1]), f, {0, 0}}, {px(c[2]), f, {0, 0}}, {px(c[3]), f, {0, 0}}};
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
    for (int i = 0; i < 4; ++i) {
        line(r, c[static_cast<std::size_t>(i)], c[static_cast<std::size_t>((i + 1) % 4)], edge);
        if (thick) {
            // second line shifted by one pixel towards the center makes the outline two pixels wide
            const rb::Vec2 in0 = c[static_cast<std::size_t>(i)] + (b.pos - c[static_cast<std::size_t>(i)]) * (1.5F / (kScale * rb::length(b.half)));
            const rb::Vec2 in1 = c[static_cast<std::size_t>((i + 1) % 4)] + (b.pos - c[static_cast<std::size_t>((i + 1) % 4)]) * (1.5F / (kScale * rb::length(b.half)));
            line(r, in0, in1, edge);
        }
    }
}

void drawCircle(SDL_Renderer* r, const rb::Body& b, gd::Color fill, gd::Color edge, bool thick) {
    const SDL_FPoint c = px(b.pos);
    gd::disc(r, c.x, c.y, b.radius * kScale, fill);
    gd::ring(r, c.x, c.y, b.radius * kScale, edge);
    if (thick) gd::ring(r, c.x, c.y, b.radius * kScale - 1.5F, edge);
    line(r, b.pos, b.pos + rb::mul(rb::rot(b.angle), {b.radius, 0.0F}), edge);
}

void drawWorld(App& app) {
    SDL_Renderer* r = app.renderer;
    // Statics are clipped so the floor does not paint over the chip rows.
    const SDL_Rect clip{0, 30, kWidth, 402};
    SDL_SetRenderClipRect(r, &clip);
    for (const rb::Body& b : app.world.bodies) {
        if (b.isStatic()) drawBox(r, b, gd::kInk, gd::kInk, false);
    }
    SDL_SetRenderClipRect(r, nullptr);

    for (std::size_t i = 0; i < app.world.bodies.size(); ++i) {
        const rb::Body& b = app.world.bodies[i];
        if (b.isStatic()) continue;
        const bool grabbed = app.drag.body == static_cast<int>(i);
        const gd::Color edge = grabbed ? gd::kOrange : gd::kInk;
        if (b.shape == rb::Shape::Circle) {
            drawCircle(r, b, gd::kTeal, edge, grabbed);
        } else {
            drawBox(r, b, gd::kTealSoft, edge, grabbed);
        }
    }

    if (app.showContacts) {
        for (const rb::Arbiter& arb : app.world.arbiters) {
            for (int i = 0; i < arb.manifold.count; ++i) {
                const rb::Contact& c = arb.manifold.contacts[i];
                const SDL_FPoint p = px(c.position);
                line(r, c.position, c.position + arb.manifold.normal * 0.45F, gd::kOrange);
                gd::fillRect(r, p.x - 2.5F, p.y - 2.5F, 5.0F, 5.0F, gd::kOrange);
            }
        }
    }

    if (app.drag.body >= 0) {
        const rb::Body& b = app.world.bodies[static_cast<std::size_t>(app.drag.body)];
        const rb::Vec2 anchor = b.pos + rb::mul(rb::rot(b.angle), app.drag.local);
        line(r, anchor, app.drag.target, gd::kOrange);
        const SDL_FPoint t = px(app.drag.target);
        gd::ring(r, t.x, t.y, 6.0F, gd::kOrange);
        const SDL_FPoint a = px(anchor);
        gd::fillRect(r, a.x - 3.0F, a.y - 3.0F, 6.0F, 6.0F, gd::kOrange);
    }

    // The side walls sit exactly at the picture edges: draw them as thin bars.
    gd::fillRect(r, 0.0F, 30.0F, 3.0F, 402.0F, gd::kInk);
    gd::fillRect(r, kWidth - 3.0F, 30.0F, 3.0F, 402.0F, gd::kInk);
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
    if (!SDL_CreateWindowAndRenderer("rigid-bodies", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    app->world.restitution = kRestitutions[static_cast<std::size_t>(app->restitutionIndex)];
    addScene(app->world);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) {
        return SDL_APP_SUCCESS;
    }

    if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN && event->button.button == SDL_BUTTON_LEFT) {
        const SDL_FPoint p{event->button.x, event->button.y};
        const Layout layout = makeLayout(app);
        for (const Chip& c : layout.chips) {
            if (SDL_PointInRectFloat(&p, &c.rect)) {
                apply(app, c);
                return SDL_APP_CONTINUE;
            }
        }
        if (p.y >= kFloorY * kScale) return SDL_APP_CONTINUE;  // below the floor: nothing to grab or spawn
        const rb::Vec2 w = toWorld(p.x, p.y);
        // Topmost (last added) dynamic body under the cursor gets grabbed.
        for (int i = static_cast<int>(app.world.bodies.size()) - 1; i >= 0; --i) {
            const rb::Body& b = app.world.bodies[static_cast<std::size_t>(i)];
            if (!b.isStatic() && rb::contains(b, w)) {
                app.drag = Drag{i, rb::mulT(rb::rot(b.angle), w - b.pos), w};
                return SDL_APP_CONTINUE;
            }
        }
        if (p.y > 30.0F) spawnBody(app, w);
    }
    if (event->type == SDL_EVENT_MOUSE_MOTION && app.drag.body >= 0) {
        app.drag.target = toWorld(event->motion.x, event->motion.y);
    }
    if (event->type == SDL_EVENT_MOUSE_BUTTON_UP && event->button.button == SDL_BUTTON_LEFT) {
        app.drag.body = -1;
    }
    // A release outside the canvas or the window never arrives here, so let go when the pointer or the focus leaves.
    if (event->type == SDL_EVENT_WINDOW_MOUSE_LEAVE || event->type == SDL_EVENT_WINDOW_FOCUS_LOST) {
        app.drag.body = -1;
    }
    if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat) {
        if (event->key.key == SDLK_R) {
            apply(app, Chip{{}, "", Act::Clear, 0, false, false});
        } else if (event->key.key == SDLK_G) {
            apply(app, Chip{{}, "", Act::Gravity, 0, false, false});
        }
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    double frameTime = static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency());
    app.last = now;
    frameTime = std::min(frameTime, kMaxFrameTime);

    app.accumulator += frameTime;
    app.stepsThisFrame = 0;
    while (app.accumulator >= static_cast<double>(kStep)) {
        if (app.drag.body >= 0) {
            rb::applySpring(app.world.bodies[static_cast<std::size_t>(app.drag.body)], app.drag.local, app.drag.target, kStep);
        }
        app.world.step(kStep);
        app.accumulator -= static_cast<double>(kStep);
        ++app.stepsThisFrame;
    }

    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    drawWorld(app);

    std::size_t dynamicCount = 0;
    for (const rb::Body& b : app.world.bodies) dynamicCount += b.isStatic() ? 0 : 1;
    char text[128];
    SDL_snprintf(text, sizeof text, "%d bodies  %d steps/frame  E %.0f J", static_cast<int>(dynamicCount), app.stepsThisFrame,
                 static_cast<double>(rb::totalEnergy(app.world, kFloorY)));
    app.hud.titleBar(r, kWidth, "[ RIGID BODIES ]", text);

    const Layout layout = makeLayout(app);
    for (const Label& l : layout.labels) app.hud.text(r, l.x, l.y + 4.0F, l.text, gd::kMuted);
    for (const Chip& c : layout.chips) {
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, c.active ? gd::kTeal : gd::kWhite, c.accent ? gd::kOrange : gd::kInk);
        const gd::Color ink = c.active ? gd::kWhite : (c.accent ? gd::kOrange : gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, ink, gd::Hud::Style::Bold);
    }
    const char* hint = dynamicCount + 4 >= kMaxBodies ? "limit reached" : "click: spawn   drag: pull";
    app.hud.text(r, kWidth - 40.0F - app.hud.width(hint), 496.0F, hint, dynamicCount + 4 >= kMaxBodies ? gd::kOrange : gd::kMuted);
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
