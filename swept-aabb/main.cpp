// Swept AABB: the same box, the same speed and the same wall in two lanes.
//
// Top lane: the naive way. Move the box, then ask "do they overlap now?". That question is only asked at the end of each
// frame, so a box that covers more than wall + box in one frame lands on the far side and nothing is ever wrong.
// Bottom lane: the sweep from logic.hpp. It does not look at where the box ends up but at the whole path of the frame and
// returns t, the fraction of the motion that is free, plus the side that was hit.
//
// The simulation runs in slow motion so the jumps are visible: one frame of the box every 200 ms, whatever the display
// does. Frame time is clamped and the accumulator is reset on every restart.

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
constexpr double kTick = 0.2;                 // seconds per simulated frame
constexpr double kMaxFrameTime = 0.25;        // clamp: no burst of catch-up frames after a stall
constexpr int kHoldTicks = 8;                 // pause after both boxes are done, then the scene restarts
constexpr std::array<float, 2> kLaneTop{38.0F, 238.0F};
constexpr std::array<float, 5> kSpeeds{20.0F, 45.0F, 80.0F, 150.0F, 400.0F};
constexpr std::array<const char*, 5> kSpeedLabels{"20", "45", "80", "150", "400"};
constexpr std::array<float, 3> kAngles{0.0F, 5.0F, 10.0F};
constexpr std::array<const char*, 3> kAngleLabels{"0°", "5°", "10°"};
constexpr std::array<float, 2> kWalls{10.0F, 60.0F};
constexpr std::array<const char*, 2> kWallLabels{"10", "60"};
constexpr float kMinStartY = 50.0F;           // keeps the box clear of the lane label and the status line
constexpr float kMaxStartY = 140.0F;

enum class Act { Speed, Angle, Wall, Stop, Slide, Restart };

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

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::array<sw::Run, 2> runs;  // 0 = naive, 1 = sweep
    int speedIndex = 3;
    int angleIndex = 1;
    int wallIndex = 0;
    bool slide = false;
    float startY = 60.0F;
    double accumulator = 0.0;
    int holdTicks = 0;
    Uint64 last = 0;
    long maxFrames = -1;
    long frames = 0;
};

sw::Vec2 currentVelocity(const App& app) {
    return sw::velocity(kSpeeds[static_cast<std::size_t>(app.speedIndex)], kAngles[static_cast<std::size_t>(app.angleIndex)]);
}

sw::Aabb currentWall(const App& app) { return sw::wallBox(kWalls[static_cast<std::size_t>(app.wallIndex)]); }

void tick(App& app) {
    if (app.runs[0].done && app.runs[1].done) {
        ++app.holdTicks;
        return;
    }
    const sw::Vec2 v = currentVelocity(app);
    const sw::Aabb wall = currentWall(app);
    for (sw::Run& run : app.runs) sw::step(run, v, wall, app.slide);
}

// Starts both lanes from the same box. Option changes call this too, so no old state survives a switch.
void restart(App& app, int prerollTicks = 0) {
    app.runs[0] = sw::startRun(sw::Mode::Naive, sw::startBox(app.startY));
    app.runs[1] = sw::startRun(sw::Mode::Sweep, sw::startBox(app.startY));
    app.accumulator = 0.0;
    app.holdTicks = 0;
    for (int i = 0; i < prerollTicks; ++i) tick(app);
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

    label("speed");
    for (int i = 0; i < 5; ++i) chip(kSpeedLabels[static_cast<std::size_t>(i)], Act::Speed, i, app.speedIndex == i);
    x += 12.0F;
    label("angle");
    for (int i = 0; i < 3; ++i) chip(kAngleLabels[static_cast<std::size_t>(i)], Act::Angle, i, app.angleIndex == i);
    x += 12.0F;
    label("wall");
    for (int i = 0; i < 2; ++i) chip(kWallLabels[static_cast<std::size_t>(i)], Act::Wall, i, app.wallIndex == i);

    x = 40.0F;
    y = 488.0F;
    label("response");
    chip("stop", Act::Stop, 0, !app.slide);
    chip("slide (S)", Act::Slide, 0, app.slide);
    x += 12.0F;
    chip("restart (Space)", Act::Restart, 0, false, true);
    return out;
}

void apply(App& app, const Chip& c) {
    switch (c.act) {
        case Act::Speed: app.speedIndex = c.value; break;
        case Act::Angle: app.angleIndex = c.value; break;
        case Act::Wall: app.wallIndex = c.value; break;
        case Act::Stop: app.slide = false; break;
        case Act::Slide: app.slide = true; break;
        case Act::Restart: break;
    }
    restart(app);
}

// ---- drawing ----

SDL_FPoint at(int lane, sw::Vec2 p) { return SDL_FPoint{p.x, p.y + kLaneTop[static_cast<std::size_t>(lane)]}; }

void line(SDL_Renderer* r, int lane, sw::Vec2 a, sw::Vec2 b, gd::Color c, bool bold = false) {
    gd::setColor(r, c);
    const SDL_FPoint p = at(lane, a);
    const SDL_FPoint q = at(lane, b);
    SDL_RenderLine(r, p.x, p.y, q.x, q.y);
    if (bold) {
        SDL_RenderLine(r, p.x, p.y - 1.0F, q.x, q.y - 1.0F);
        SDL_RenderLine(r, p.x, p.y + 1.0F, q.x, q.y + 1.0F);
        SDL_RenderLine(r, p.x - 1.0F, p.y, q.x - 1.0F, q.y);
        SDL_RenderLine(r, p.x + 1.0F, p.y, q.x + 1.0F, q.y);
    }
}

void outline(SDL_Renderer* r, int lane, const sw::Aabb& b, gd::Color c, bool thick = false) {
    const SDL_FPoint p = at(lane, b.min);
    const float w = b.max.x - b.min.x;
    const float h = b.max.y - b.min.y;
    gd::strokeRect(r, p.x, p.y, w, h, c);
    if (thick) gd::strokeRect(r, p.x + 1.0F, p.y + 1.0F, w - 2.0F, h - 2.0F, c);
}

void solid(SDL_Renderer* r, int lane, const sw::Aabb& b, gd::Color c) {
    const SDL_FPoint p = at(lane, b.min);
    gd::fillRect(r, p.x, p.y, b.max.x - b.min.x, b.max.y - b.min.y, c);
}

void marker(SDL_Renderer* r, int lane, sw::Vec2 p, gd::Color c) {
    const SDL_FPoint q = at(lane, p);
    gd::fillRect(r, q.x - 3.0F, q.y - 3.0F, 6.0F, 6.0F, c);
}

void arrow(SDL_Renderer* r, int lane, sw::Vec2 from, sw::Vec2 dir, float len, gd::Color c) {
    const sw::Vec2 tip = from + dir * len;
    line(r, lane, from, tip, c, true);
    const sw::Vec2 side{-dir.y, dir.x};
    line(r, lane, tip, tip - dir * 8.0F + side * 5.0F, c, true);
    line(r, lane, tip, tip - dir * 8.0F - side * 5.0F, c, true);
}

void drawLane(App& app, int lane) {
    SDL_Renderer* r = app.renderer;
    const sw::Run& run = app.runs[static_cast<std::size_t>(lane)];
    const bool naive = lane == 0;
    const float top = kLaneTop[static_cast<std::size_t>(lane)];
    const gd::Color mark = naive ? gd::kOrange : gd::kTealDark;
    const gd::Color faint{107, 114, 128, 140};

    gd::fillRect(r, 0.0F, top, sw::kFieldW, sw::kLaneH, gd::Color{255, 255, 255, 190});
    gd::strokeRect(r, 0.0F, top, sw::kFieldW, sw::kLaneH, gd::kInk);

    const SDL_Rect clip{0, static_cast<int>(top), kWidth, static_cast<int>(sw::kLaneH)};
    SDL_SetRenderClipRect(r, &clip);

    solid(r, lane, currentWall(app), gd::kInk);

    // Where the box was after each frame, and the straight line between those spots.
    for (std::size_t i = 0; i < run.trail.size(); ++i) {
        outline(r, lane, run.trail[i], faint);
        if (i > 0) line(r, lane, sw::center(run.trail[i - 1]), sw::center(run.trail[i]), faint);
    }

    // The frame in which the path meets the wall.
    if (run.key.valid) {
        const sw::Aabb end = sw::translated(run.key.from, run.key.v);
        const sw::Vec2 from = sw::center(run.key.from);
        if (naive) {
            line(r, lane, from, sw::center(end), gd::kOrange, true);
            outline(r, lane, end, gd::kOrange);
        } else {
            line(r, lane, from, sw::center(end), faint);
            outline(r, lane, end, faint);
            const sw::Aabb impact = sw::impactBox(run.key);
            solid(r, lane, impact, gd::Color{204, 251, 241, 150});
            line(r, lane, from, sw::center(impact), gd::kTeal, true);
            outline(r, lane, impact, gd::kTealDark, true);
        }
    }

    // The box itself.
    const sw::Aabb& b = run.box;
    solid(r, lane, b, naive ? gd::kWhite : gd::kTealSoft);
    outline(r, lane, b, naive ? gd::kOrange : gd::kTealDark, true);

    // The normal of the hit, drawn last so the box does not cover it: from the contact point on the wall back at the box.
    if (run.key.valid && !naive) {
        const sw::Vec2 n = run.key.hit.normal;
        const sw::Vec2 contact = sw::center(sw::impactBox(run.key)) - n * (sw::kBoxSize * 0.5F);
        arrow(r, lane, contact, n, 44.0F, gd::kOrange);
        marker(r, lane, contact, gd::kOrange);
    }
    SDL_SetRenderClipRect(r, nullptr);

    if (run.key.valid && !naive) {
        const sw::Aabb impact = sw::impactBox(run.key);
        char t[32];
        SDL_snprintf(t, sizeof t, "t = %.2f", static_cast<double>(run.key.hit.t));
        const float tx = std::clamp(impact.min.x - app.hud.width(t, gd::Hud::Style::Bold) - 6.0F, 4.0F, 870.0F);
        const float ty = impact.min.y - 26.0F >= 4.0F ? impact.min.y - 26.0F : impact.max.y + 6.0F;
        app.hud.text(r, tx, top + ty, t, gd::kTealDark, gd::Hud::Style::Bold);
    }

    app.hud.text(r, 10.0F, top + 5.0F, naive ? "NAIVE  move, then test overlap" : "SWEEP  time of impact t in 0..1", mark, gd::Hud::Style::Bold);

    char status[96];
    gd::Color statusColor = gd::kMuted;
    if (naive) {
        if (run.missed) {
            SDL_snprintf(status, sizeof status, "MISSED: jumped over the wall");
            statusColor = gd::kOrange;
        } else if (run.blocked && run.shortBy > 0.5F) {
            SDL_snprintf(status, sizeof status, "stopped %.0f px short of the wall", static_cast<double>(run.shortBy));
            statusColor = gd::kOrange;
        } else if (run.blocked) {
            SDL_snprintf(status, sizeof status, "stopped at the wall");
        } else {
            SDL_snprintf(status, sizeof status, "frame %d: no overlap yet", run.frame);
        }
    } else if (run.key.valid) {
        SDL_snprintf(status, sizeof status, "hit at t = %.2f, normal (%d, %d)", static_cast<double>(run.key.hit.t),
                     static_cast<int>(run.key.hit.normal.x), static_cast<int>(run.key.hit.normal.y));
        statusColor = gd::kTealDark;
    } else {
        SDL_snprintf(status, sizeof status, "frame %d: path is free", run.frame);
    }
    app.hud.text(r, 10.0F, top + sw::kLaneH - 27.0F, status, statusColor);

    if (naive) {
        const float thickness = kWalls[static_cast<std::size_t>(app.wallIndex)];
        char jump[64];
        SDL_snprintf(jump, sizeof jump, "wall %.0f + box %.0f = %.0f px per frame to jump", static_cast<double>(thickness),
                     static_cast<double>(sw::kBoxSize), static_cast<double>(thickness + sw::kBoxSize));
        app.hud.text(r, sw::kFieldW - 10.0F - app.hud.width(jump), top + sw::kLaneH - 27.0F, jump, gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("swept-aabb", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    restart(*app, 4);  // the first picture already shows the boxes on their way
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
        for (const float top : kLaneTop) {
            if (p.y >= top && p.y < top + sw::kLaneH) {
                app.startY = std::clamp(p.y - top, kMinStartY, kMaxStartY);
                restart(app);
            }
        }
    }
    if (event->type == SDL_EVENT_KEY_DOWN) {
        // Up and down repeat on purpose, they nudge the start height; the toggles must not flip on key repeat.
        if (event->key.key == SDLK_UP || event->key.key == SDLK_DOWN) {
            app.startY = std::clamp(app.startY + (event->key.key == SDLK_UP ? -10.0F : 10.0F), kMinStartY, kMaxStartY);
            restart(app);
        } else if (!event->key.repeat && event->key.key == SDLK_SPACE) {
            restart(app);
        } else if (!event->key.repeat && event->key.key == SDLK_S) {
            app.slide = !app.slide;
            restart(app);
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
    while (app.accumulator >= kTick) {
        tick(app);
        app.accumulator -= kTick;
        if (app.holdTicks >= kHoldTicks) restart(app);
    }

    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    drawLane(app, 0);
    drawLane(app, 1);

    const sw::Vec2 v = currentVelocity(app);
    char text[96];
    SDL_snprintf(text, sizeof text, "frame %d  step %.0f px  (1 frame = 200 ms)", std::max(app.runs[0].frame, app.runs[1].frame), static_cast<double>(sw::length(v)));
    app.hud.titleBar(r, kWidth, "[ SWEPT AABB ]", text);

    const Layout layout = makeLayout(app);
    for (const Label& l : layout.labels) app.hud.text(r, l.x, l.y + 4.0F, l.text, gd::kMuted);
    for (const Chip& c : layout.chips) {
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, c.active ? gd::kTeal : gd::kWhite, c.accent ? gd::kOrange : gd::kInk);
        const gd::Color ink = c.active ? gd::kWhite : (c.accent ? gd::kOrange : gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, ink, gd::Hud::Style::Bold);
    }
    const char* hint = "click a lane or up/down: start height";
    app.hud.text(r, sw::kFieldW - 40.0F - app.hud.width(hint), 496.0F, hint, gd::kMuted);
    SDL_RenderPresent(r);

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
