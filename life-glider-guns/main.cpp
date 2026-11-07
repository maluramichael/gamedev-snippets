// Gliders and guns in Conway's Game of Life with SDL3 main callbacks.
//
// A palette of the small things that make Life interesting: the glider (four directions, four phases), the block, the
// blinker and the Gosper gun, which shoots a glider every 30 generations. Click to place, right click erases, pause and
// single-step to watch what a collision does. The first thing you see is the gun at work and two gliders about to meet.
// The graph shows the population: the gun adds 5 cells per glider, so it climbs by 5 every 30 generations.
// The rules, the shapes and everything that can be wrong live in logic.hpp and are covered by test.cpp.

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

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCellPx = 6;
constexpr float kFieldTop = 32.0F;
constexpr float kFieldBottom = kFieldTop + static_cast<float>(life::kRows * kCellPx);  // 416
constexpr int kStableAge = 8;  // cells alive this long count as stable (blocks, the body of the gun)
constexpr float kChipH = 30.0F;
constexpr float kChipRows[3] = {426.0F, 462.0F, 498.0F};
constexpr int kPlotWindow = 224;  // generations on the graph, one pixel each
constexpr int kDefaultSpeed = 2;
constexpr int kStartCollision = 2;  // offset (1, 1), leaves a block
constexpr gd::Color kGhost{234, 88, 12, 140};

enum class Action { Glider, Block, Blinker, Gun, Draw, Rotate, Phase, Edge, Clear, Pause, Step, Speed, Collide, Reset };
enum class Tool { Glider, Block, Blinker, Gun, Draw };
enum class Paint { None, Draw, Erase };

struct Chip {
    Action action;
    int row;
    int chars;  // widest label, so the chip does not change size when its text does
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    life::World world{life::kCols, life::kRows, true};
    life::PopulationHistory history;
    life::Pacer pacer;
    Tool tool = Tool::Glider;
    int rotation = 0;
    int phase = 0;
    int speed = kDefaultSpeed;
    int collision = kStartCollision;
    bool paused = false;
    Paint paint = Paint::None;
    life::Cell lastPainted{};
    bool hover = false;
    life::Cell hoverCell{};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

// ---------------------------------------------------------------- scene

void restartHistory(App& app) {
    app.history.clear();
    app.history.push(app.world.population());
    app.pacer.reset();
}

void resetScene(App& app) {
    app.world.clear();
    app.world.stamp(life::shapeOf(life::Kind::Gun), 4, 4);
    life::loadCollision(app.world, kStartCollision, 80, 8);
    app.collision = kStartCollision;
    app.paused = false;
    restartHistory(app);
}

void loadCollision(App& app) {
    app.collision = (app.collision + 1) % static_cast<int>(life::kCollisions.size());
    app.world.clear();
    life::loadCollision(app.world, app.collision, 64, 14);
    app.paused = false;
    restartHistory(app);
}

void afterEdit(App& app) { app.history.replaceLast(app.world.population()); }

life::Kind kindOf(Tool tool) {
    switch (tool) {
        case Tool::Glider: return life::Kind::Glider;
        case Tool::Block: return life::Kind::Block;
        case Tool::Blinker: return life::Kind::Blinker;
        case Tool::Gun: return life::Kind::Gun;
        case Tool::Draw: break;
    }
    return life::Kind::Block;
}

bool inField(float x, float y) { return x >= 0.0F && x < static_cast<float>(kWidth) && y >= kFieldTop && y < kFieldBottom; }

life::Cell cellAt(float x, float y) { return {static_cast<int>(std::floor(x / static_cast<float>(kCellPx))), static_cast<int>(std::floor((y - kFieldTop) / static_cast<float>(kCellPx)))}; }

void placeStamp(App& app, life::Cell at) {
    const life::Shape shape = life::shapeOf(kindOf(app.tool), app.rotation, app.phase);
    const life::Cell tl = life::topLeftFor(shape, at.x, at.y);
    app.world.stamp(shape, tl.x, tl.y);
    afterEdit(app);
}

void paintCell(App& app, life::Cell at) {
    // A drag is drawn as a line from the last cell, so a fast mouse leaves no gaps.
    const bool on = app.paint == Paint::Draw;
    for (const life::Cell& c : life::lineCells(app.lastPainted, at)) app.world.set(c.x, c.y, on);
    app.lastPainted = at;
    afterEdit(app);
}

// ---------------------------------------------------------------- chips

std::string label(const App& app, Action action) {
    switch (action) {
        case Action::Glider: return "1 Glider";
        case Action::Block: return "2 Block";
        case Action::Blinker: return "3 Blinker";
        case Action::Gun: return "4 Gun";
        case Action::Draw: return "5 Draw";
        case Action::Rotate: {
            if (app.tool == Tool::Draw) return "R Turn --";
            if (app.tool == Tool::Glider) return std::string("R Turn ") + life::headingName(app.rotation);
            return "R Turn " + std::to_string(90 * life::mod(app.rotation, 4)) + "°";
        }
        case Action::Phase: return "F Phase " + std::to_string(app.phase);
        case Action::Edge: return app.world.torus() ? "T Torus" : "T Walls";
        case Action::Clear: return "C Clear";
        case Action::Pause: return app.paused ? "Space Run" : "Space Pause";
        case Action::Step: return "N Step";
        case Action::Speed: return "S Speed " + std::to_string(life::kSpeeds[static_cast<std::size_t>(app.speed)]) + "/s";
        case Action::Collide: return "V Collision " + std::to_string(app.collision + 1) + "/" + std::to_string(life::kCollisions.size());
        case Action::Reset: return "0 Reset";
    }
    return {};
}

bool isOn(const App& app, Action action) {
    switch (action) {
        case Action::Glider: return app.tool == Tool::Glider;
        case Action::Block: return app.tool == Tool::Block;
        case Action::Blinker: return app.tool == Tool::Blinker;
        case Action::Gun: return app.tool == Tool::Gun;
        case Action::Draw: return app.tool == Tool::Draw;
        case Action::Edge: return app.world.torus();
        case Action::Pause: return app.paused;
        default: return false;
    }
}

/// Turn and phase do nothing for some tools, their chips are drawn dim.
bool isUsable(const App& app, Action action) {
    if (action == Action::Rotate) return app.tool != Tool::Draw;
    if (action == Action::Phase) return app.tool == Tool::Glider;
    return true;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        int row;
        int chars;
    };
    const std::array<Def, 14> defs{{{Action::Glider, 0, 8},
                                    {Action::Block, 0, 7},
                                    {Action::Blinker, 0, 9},
                                    {Action::Gun, 0, 5},
                                    {Action::Draw, 0, 6},
                                    {Action::Rotate, 1, 11},
                                    {Action::Phase, 1, 9},
                                    {Action::Edge, 1, 7},
                                    {Action::Clear, 1, 7},
                                    {Action::Pause, 2, 11},
                                    {Action::Step, 2, 6},
                                    {Action::Speed, 2, 13},
                                    {Action::Collide, 2, 15},
                                    {Action::Reset, 2, 7}}};
    float x[3] = {16.0F, 16.0F, 16.0F};
    for (const Def& d : defs) {
        const float w = static_cast<float>(d.chars) * 10.0F + 24.0F;
        app.chips.push_back({d.action, d.row, d.chars, SDL_FRect{x[d.row], kChipRows[d.row], w, kChipH}});
        x[d.row] += w + 8.0F;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Glider: app.tool = Tool::Glider; break;
        case Action::Block: app.tool = Tool::Block; break;
        case Action::Blinker: app.tool = Tool::Blinker; break;
        case Action::Gun: app.tool = Tool::Gun; break;
        case Action::Draw: app.tool = Tool::Draw; break;
        case Action::Rotate: app.rotation = (app.rotation + 1) % 4; break;
        case Action::Phase: app.phase = (app.phase + 1) % 4; break;
        case Action::Edge: app.world.setTorus(!app.world.torus()); break;
        case Action::Clear:
            app.world.clear();
            restartHistory(app);
            break;
        case Action::Pause:
            app.paused = !app.paused;
            app.pacer.reset();
            break;
        case Action::Step:
            app.paused = true;
            app.pacer.reset();
            app.world.step();
            app.history.push(app.world.population());
            break;
        case Action::Speed:
            app.speed = (app.speed + 1) % static_cast<int>(life::kSpeeds.size());
            app.pacer.reset();
            break;
        case Action::Collide: loadCollision(app); break;
        case Action::Reset: resetScene(app); break;
    }
}

// ---------------------------------------------------------------- drawing

void drawField(App& app) {
    SDL_Renderer* r = app.renderer;
    const float fieldH = kFieldBottom - kFieldTop;
    gd::fillRect(r, 0.0F, kFieldTop, static_cast<float>(kWidth), fieldH, gd::kWhite);
    gd::setColor(r, gd::kGrid);
    for (int x = 0; x <= life::kCols; ++x) {
        const auto fx = static_cast<float>(x * kCellPx);
        SDL_RenderLine(r, fx, kFieldTop, fx, kFieldBottom);
    }
    for (int y = 0; y <= life::kRows; ++y) {
        const float fy = kFieldTop + static_cast<float>(y * kCellPx);
        SDL_RenderLine(r, 0.0F, fy, static_cast<float>(kWidth), fy);
    }

    // Live cells in two batches: moving things in teal, cells that have been alive for a while (still lifes, the gun's body) in ink.
    std::vector<SDL_FRect> young;
    std::vector<SDL_FRect> stable;
    for (int y = 0; y < life::kRows; ++y) {
        for (int x = 0; x < life::kCols; ++x) {
            const int age = app.world.age(x, y);
            if (age == 0) continue;
            const SDL_FRect rect{static_cast<float>(x * kCellPx + 1), kFieldTop + static_cast<float>(y * kCellPx + 1), 5.0F, 5.0F};
            (age >= kStableAge ? stable : young).push_back(rect);
        }
    }
    gd::setColor(r, gd::kTeal);
    if (!young.empty()) SDL_RenderFillRects(r, young.data(), static_cast<int>(young.size()));
    gd::setColor(r, gd::kInk);
    if (!stable.empty()) SDL_RenderFillRects(r, stable.data(), static_cast<int>(stable.size()));

    // What a click would place, in orange. Cells beyond a wall are not drawn, on a torus they wrap.
    if (app.hover) {
        std::vector<SDL_FRect> ghost;
        auto add = [&](int x, int y) {
            const int i = app.world.index(x, y);
            if (i < 0) return;
            ghost.push_back(SDL_FRect{static_cast<float>((i % life::kCols) * kCellPx + 1), kFieldTop + static_cast<float>((i / life::kCols) * kCellPx + 1), 5.0F, 5.0F});
        };
        if (app.tool == Tool::Draw) {
            add(app.hoverCell.x, app.hoverCell.y);
        } else {
            const life::Shape shape = life::shapeOf(kindOf(app.tool), app.rotation, app.phase);
            const life::Cell tl = life::topLeftFor(shape, app.hoverCell.x, app.hoverCell.y);
            for (const life::Cell& c : shape) add(tl.x + c.x, tl.y + c.y);
        }
        gd::setColor(r, kGhost);
        if (!ghost.empty()) SDL_RenderFillRects(r, ghost.data(), static_cast<int>(ghost.size()));
    }
    gd::fillRect(r, 0.0F, kFieldBottom, static_cast<float>(kWidth), 2.0F, gd::kInk);
}

void drawGraph(App& app) {
    SDL_Renderer* r = app.renderer;
    constexpr float kX = 704.0F;
    constexpr float kY = 426.0F;
    constexpr float kW = 240.0F;
    constexpr float kH = 102.0F;
    constexpr float kPlotX1 = kX + kW - 8.0F;
    constexpr float kPlotBottom = kY + kH - 8.0F;
    constexpr float kPlotTop = kY + 32.0F;
    gd::card(r, kX, kY, kW, kH, gd::kWhite, gd::kInk);

    const std::size_t n = app.history.size();
    char left[24];
    SDL_snprintf(left, sizeof left, "pop %d", app.world.population());
    app.hud.text(r, kX + 10.0F, kY + 6.0F, left, gd::kInk, gd::Hud::Style::Bold);
    const auto delta = app.history.delta(30);
    char right[24];
    gd::Color rightColor = gd::kMuted;
    if (delta) {
        SDL_snprintf(right, sizeof right, "30 gens %+d", *delta);
        rightColor = *delta > 0 ? gd::kTeal : *delta < 0 ? gd::kOrange : gd::kMuted;
    } else {
        SDL_snprintf(right, sizeof right, "30 gens --");
    }
    app.hud.text(r, kX + kW - 10.0F - app.hud.width(right, gd::Hud::Style::Bold), kY + 6.0F, right, rightColor, gd::Hud::Style::Bold);

    // One column per generation, a faint line at every multiple of 30.
    const std::size_t shown = std::min<std::size_t>(n, kPlotWindow);
    const int top = std::max(10, app.history.peak(shown));
    const long newestGen = app.world.generation();
    std::vector<SDL_FPoint> pts;
    pts.reserve(shown);
    for (std::size_t k = 0; k < shown; ++k) {
        const std::size_t i = n - shown + k;
        const float x = kPlotX1 - static_cast<float>(shown - 1 - k);
        const long gen = newestGen - static_cast<long>(n - 1 - i);
        if (gen >= 0 && gen % 30 == 0) {
            gd::setColor(r, gd::kGrid);
            SDL_RenderLine(r, x, kPlotTop, x, kPlotBottom);
        }
        const float v = static_cast<float>(app.history.at(i)) / static_cast<float>(top);
        pts.push_back({x, kPlotBottom - v * (kPlotBottom - kPlotTop)});
    }
    gd::setColor(r, gd::kMuted);
    SDL_RenderLine(r, kX + 8.0F, kPlotBottom, kPlotX1, kPlotBottom);
    if (pts.size() > 1) {
        gd::setColor(r, gd::kTeal);
        SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
        for (SDL_FPoint& p : pts) p.y += 1.0F;
        SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
    }
    // The gun shows the same picture every 30 generations, so the dots there climb in clean steps of 5.
    for (std::size_t k = 0; k < shown; ++k) {
        const long gen = newestGen - static_cast<long>(shown - 1 - k);
        if (gen >= 0 && gen % 30 == 0) gd::fillRect(r, pts[k].x - 1.5F, pts[k].y - 1.5F, 4.0F, 4.0F, gd::kOrange);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "gen %ld   %dx%d %s", app.world.generation(), life::kCols, life::kRows, app.world.torus() ? "torus" : "walls");
    app.hud.titleBar(r, kWidth, "[ GAME OF LIFE ]", right);
    drawField(app);
    drawGraph(app);

    app.hud.text(r, 540.0F, 432.0F, "click: place", gd::kMuted);
    app.hud.text(r, 490.0F, 468.0F, "right click: erase", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = isOn(app, c.action);
        const bool usable = isUsable(app, c.action);
        const std::string text = label(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        const gd::Color ink = on ? gd::kWhite : (usable ? gd::kInk : gd::kMuted);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(text, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, text, ink, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("life-glider-guns", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation)) {
        SDL_Log("logical presentation failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    resetScene(*app);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            const float mx = event->motion.x;
            const float my = event->motion.y;
            app.hover = inField(mx, my);
            if (app.hover) {
                app.hoverCell = cellAt(mx, my);
                if (app.paint != Paint::None) paintCell(app, app.hoverCell);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            const bool left = event->button.button == SDL_BUTTON_LEFT;
            if (!left && event->button.button != SDL_BUTTON_RIGHT) break;
            if (left) {
                bool onChip = false;
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) {
                        apply(app, c.action);
                        onChip = true;
                    }
                }
                if (onChip) break;
            }
            if (!inField(p.x, p.y)) break;
            app.hoverCell = cellAt(p.x, p.y);
            app.hover = true;
            if (!left) {
                app.paint = Paint::Erase;
                app.lastPainted = app.hoverCell;
                paintCell(app, app.hoverCell);
            } else if (app.tool == Tool::Draw) {
                app.paint = Paint::Draw;
                app.lastPainted = app.hoverCell;
                paintCell(app, app.hoverCell);
            } else {
                placeStamp(app, app.hoverCell);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP: app.paint = Paint::None; break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.paint = Paint::None;
            app.hover = false;
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.paint = Paint::None; break;
        case SDL_EVENT_KEY_DOWN:
            // Stepping is the only key that makes sense while held, it scrubs through the generations.
            if (event->key.repeat && event->key.key != SDLK_N) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Glider); break;
                case SDLK_2: apply(app, Action::Block); break;
                case SDLK_3: apply(app, Action::Blinker); break;
                case SDLK_4: apply(app, Action::Gun); break;
                case SDLK_5: apply(app, Action::Draw); break;
                case SDLK_R: apply(app, Action::Rotate); break;
                case SDLK_F: apply(app, Action::Phase); break;
                case SDLK_T: apply(app, Action::Edge); break;
                case SDLK_C: apply(app, Action::Clear); break;
                case SDLK_SPACE: apply(app, Action::Pause); break;
                case SDLK_N: apply(app, Action::Step); break;
                case SDLK_S: apply(app, Action::Speed); break;
                case SDLK_V: apply(app, Action::Collide); break;
                case SDLK_0: apply(app, Action::Reset); break;
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
    const double dt = static_cast<double>(now - app.lastTicks) / 1.0e9;
    app.lastTicks = now;
    if (app.paused) {
        app.pacer.reset();
    } else {
        const int steps = app.pacer.due(dt, static_cast<double>(life::kSpeeds[static_cast<std::size_t>(app.speed)]));
        for (int i = 0; i < steps; ++i) {
            app.world.step();
            app.history.push(app.world.population());
        }
    }
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
