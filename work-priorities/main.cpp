// Work priorities as in RimWorld with SDL3 main callbacks.
//
// Three colonists, three kinds of work (walls, stones, meals) and a grid of priorities 1 to 4 or empty. The rule from
// the game: an empty cell never, the smallest number first, equal numbers go to the column further left, and a level
// is finished before the next one starts. So "Haul 1" means every stone is carried to the stock before anyone builds.
// The second mode scores every task by walking distance plus a penalty per priority step, which keeps the numbers
// soft: a wall next to you can beat a stone across the map. Both modes are simulated to the end for the current grid
// so the walked distance and time can be compared at once. The decisions and the simulation live in logic.hpp and are
// covered by test.cpp.

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
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kMapX = 24.0F;
constexpr float kMapY = 44.0F;
constexpr float kMapW = 616.0F;
constexpr float kMapH = 360.0F;
constexpr float kGridX = 656.0F;
constexpr float kGridY = 44.0F;
constexpr float kCellX0 = 724.0F;
constexpr float kCellY0 = 102.0F;
constexpr float kCellW = 56.0F;
constexpr float kCellH = 34.0F;
constexpr float kCellStrideX = 60.0F;
constexpr float kCellStrideY = 40.0F;
constexpr double kSpeedNormal = 3.0;
constexpr double kSpeedFast = 10.0;
constexpr int kMaxStepsPerFrame = 120;
constexpr double kHoldSeconds = 3.0;  // a finished run stays on screen this long before it restarts

constexpr gd::Color kCookColor{202, 138, 4};
constexpr std::array<const char*, wp::kColonists> kNames{"Ann", "Bob", "Cy"};
constexpr std::array<const char*, wp::kKinds> kKindNames{"Build", "Haul", "Cook"};

gd::Color kindColor(wp::Kind k) {
    switch (k) {
        case wp::Kind::Build: return gd::kTealDark;
        case wp::Kind::Haul: return gd::kOrange;
        case wp::Kind::Cook: return kCookColor;
    }
    return gd::kInk;
}

enum class Action { RulePriority, RuleScore, PresetHaul, PresetBuild, PresetEqual, Fast, Restart };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

wp::Grid presetGrid(Action a) {
    wp::Grid g{};
    for (wp::Row& row : g) {
        switch (a) {
            case Action::PresetBuild: row = {1, 2, 3}; break;
            case Action::PresetEqual: row = {1, 1, 1}; break;
            default: row = {2, 1, 3}; break;
        }
    }
    return g;
}

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    wp::Grid grid = presetGrid(Action::PresetHaul);
    wp::Rule rule = wp::Rule::Priority;
    wp::World world;
    std::array<wp::Result, 2> results;  // full run per rule for the current grid, index = static_cast<int>(rule)
    bool fast = false;
    double acc = 0.0;
    double hold = 0.0;
    Uint64 lastNs = 0;
    float mouseX = -100.0F;
    float mouseY = -100.0F;
    long maxFrames = -1;
    long frames = 0;
};

int ruleIndex(wp::Rule r) { return r == wp::Rule::Priority ? 0 : 1; }

void restart(App& app) {
    app.world = wp::makeWorld(app.grid, app.rule);
    app.acc = 0.0;
    app.hold = 0.0;
}

void refreshResults(App& app) {
    app.results[0] = wp::run(app.grid, wp::Rule::Priority);
    app.results[1] = wp::run(app.grid, wp::Rule::Score);
}

void setGrid(App& app, const wp::Grid& g) {
    app.grid = g;
    refreshResults(app);
    restart(app);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::RulePriority, "1 Rule", 0.0F},
                                   {Action::RuleScore, "2 Score", 8.0F},
                                   {Action::PresetHaul, "3 Haul first", 20.0F},
                                   {Action::PresetBuild, "4 Build first", 8.0F},
                                   {Action::PresetEqual, "5 Equal", 8.0F},
                                   {Action::Fast, "F Fast", 20.0F},
                                   {Action::Restart, "R Restart", 8.0F}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 20.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::RulePriority: return app.rule == wp::Rule::Priority;
        case Action::RuleScore: return app.rule == wp::Rule::Score;
        case Action::PresetHaul:
        case Action::PresetBuild:
        case Action::PresetEqual: return app.grid == presetGrid(a);
        case Action::Fast: return app.fast;
        case Action::Restart: return false;
    }
    return false;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::RulePriority:
        case Action::RuleScore:
            app.rule = a == Action::RulePriority ? wp::Rule::Priority : wp::Rule::Score;
            restart(app);
            break;
        case Action::PresetHaul:
        case Action::PresetBuild:
        case Action::PresetEqual: setGrid(app, presetGrid(a)); break;
        case Action::Fast: app.fast = !app.fast; break;
        case Action::Restart: restart(app); break;
    }
}

SDL_FRect cellRect(int row, int col) {
    return SDL_FRect{kCellX0 + kCellStrideX * static_cast<float>(col), kCellY0 + kCellStrideY * static_cast<float>(row), kCellW, kCellH};
}

void changeCell(App& app, int row, int col, int dir) {
    wp::Grid g = app.grid;
    int& p = g[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
    p = wp::nextPriority(p, dir);
    setGrid(app, g);
}

void dashedLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c) {
    const float len = std::hypot(x1 - x0, y1 - y0);
    if (len < 1.0F) return;
    const float dx = (x1 - x0) / len;
    const float dy = (y1 - y0) / len;
    gd::setColor(r, c);
    for (float t = 0.0F; t < len; t += 12.0F) {
        const float e = std::min(t + 6.0F, len);
        SDL_RenderLine(r, x0 + dx * t, y0 + dy * t, x0 + dx * e, y0 + dy * e);
        SDL_RenderLine(r, x0 + dx * t + dy, y0 + dy * t - dx, x0 + dx * e + dy, y0 + dy * e - dx);
    }
}

float f(double v) { return static_cast<float>(v); }

void centered(App& app, float cx, float y, const char* s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, cx - app.hud.width(s, style) / 2.0F, y, s, c, style);
}

/// Seconds as "12.3" without printf's floating point, which follows the locale.
std::string tenths(double seconds) {
    const int t = static_cast<int>(std::lround(std::clamp(seconds, 0.0, 99999.0) * 10.0));
    return std::to_string(t / 10) + "." + std::to_string(t % 10);
}

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    const wp::World& w = app.world;
    gd::card(r, kMapX, kMapY, kMapW, kMapH, gd::kWhite, gd::kInk);

    // Stock: the zone every stone has to reach.
    gd::fillRect(r, f(w.stock.x) - 44.0F, f(w.stock.y) - 44.0F, 88.0F, 88.0F, gd::kTealSoft);
    gd::strokeRect(r, f(w.stock.x) - 44.0F, f(w.stock.y) - 44.0F, 88.0F, 88.0F, gd::kMuted);
    centered(app, f(w.stock.x), f(w.stock.y) - 64.0F, "STOCK", gd::kMuted);

    // Labels for the work places, then the tasks.
    bool labelBuild = false;
    bool labelCook = false;
    int cookRow = 0;
    for (std::size_t i = 0; i < w.tasks.size(); ++i) {
        const wp::Task& t = w.tasks[i];
        const bool nobody = std::none_of(app.grid.begin(), app.grid.end(), [&](const wp::Row& row) { return row[static_cast<std::size_t>(t.kind)] > 0; });
        const gd::Color col = kindColor(t.kind);
        const float progress = t.work > 0.0 ? f(1.0 - t.left / t.work) : 0.0F;
        if (t.kind == wp::Kind::Build) {
            if (!labelBuild) {
                app.hud.text(r, f(t.pos.x) - 17.0F, f(t.pos.y) - 34.0F, "WALL", gd::kMuted);
                labelBuild = true;
            }
            const float x = f(t.pos.x) - 17.0F;
            const float y = f(t.pos.y) - 7.0F;
            gd::fillRect(r, x, y, 34.0F, 14.0F, t.done ? col : gd::kWhite);
            if (!t.done && progress > 0.0F) gd::fillRect(r, x, y, 34.0F * progress, 14.0F, gd::kTeal);
            gd::strokeRect(r, x, y, 34.0F, 14.0F, nobody ? gd::kOrange : col);
            if (t.claimed && !t.done) gd::strokeRect(r, x - 2.0F, y - 2.0F, 38.0F, 18.0F, gd::kInk);
        } else if (t.kind == wp::Kind::Cook) {
            if (!labelCook) {
                app.hud.text(r, f(t.pos.x) - 20.0F, f(t.pos.y) - 40.0F, "STOVE", gd::kMuted);
                gd::fillRect(r, f(t.pos.x) - 24.0F, f(t.pos.y) - 14.0F, 48.0F, 28.0F, gd::kWhite);
                gd::strokeRect(r, f(t.pos.x) - 24.0F, f(t.pos.y) - 14.0F, 48.0F, 28.0F, nobody ? gd::kOrange : gd::kInk);
                labelCook = true;
            }
            const float px = f(t.pos.x) - 14.0F + 14.0F * static_cast<float>(cookRow);
            ++cookRow;
            if (t.done) {
                gd::disc(r, px, f(t.pos.y), 5.0F, col);
            } else {
                gd::ring(r, px, f(t.pos.y), 5.0F, t.claimed ? gd::kInk : gd::kMuted);
                if (progress > 0.0F) gd::disc(r, px, f(t.pos.y), 5.0F * progress, col);
            }
        } else {
            const bool carried = std::any_of(w.colonists.begin(), w.colonists.end(), [&](const wp::Colonist& c) {
                return c.phase == wp::Phase::Carrying && c.task == static_cast<int>(i);
            });
            if (carried) continue;
            gd::disc(r, f(t.pos.x), f(t.pos.y), 7.0F, col);
            gd::ring(r, f(t.pos.x), f(t.pos.y), 7.0F, gd::kInk);
            if (t.claimed && !t.done) gd::ring(r, f(t.pos.x), f(t.pos.y), 11.0F, gd::kInk);
        }
    }

    // Colonists, their paths and what they carry.
    for (std::size_t i = 0; i < w.colonists.size(); ++i) {
        const wp::Colonist& c = w.colonists[i];
        if (c.task >= 0 && (c.phase == wp::Phase::ToTask || c.phase == wp::Phase::Carrying)) {
            const wp::Task& t = w.tasks[static_cast<std::size_t>(c.task)];
            const wp::Vec2 target = c.phase == wp::Phase::Carrying ? w.stock : t.pos;
            dashedLine(r, f(c.pos.x), f(c.pos.y), f(target.x), f(target.y), gd::Color{17, 24, 39, 120});
        }
    }
    for (std::size_t i = 0; i < w.colonists.size(); ++i) {
        const wp::Colonist& c = w.colonists[i];
        gd::disc(r, f(c.pos.x), f(c.pos.y), 9.0F, gd::kTeal);
        gd::ring(r, f(c.pos.x), f(c.pos.y), 9.0F, gd::kInk);
        gd::ring(r, f(c.pos.x), f(c.pos.y), 10.0F, gd::kInk);
        const float labelW = app.hud.width(kNames[i], gd::Hud::Style::Bold);
        gd::fillRect(r, f(c.pos.x) - labelW / 2.0F - 2.0F, f(c.pos.y) - 33.0F, labelW + 4.0F, 24.0F, gd::Color{255, 255, 255, 210});
        centered(app, f(c.pos.x), f(c.pos.y) - 32.0F, kNames[i], gd::kInk, gd::Hud::Style::Bold);
        if (c.phase == wp::Phase::Carrying) {
            gd::disc(r, f(c.pos.x) + 9.0F, f(c.pos.y) - 11.0F, 7.0F, gd::kOrange);
            gd::ring(r, f(c.pos.x) + 9.0F, f(c.pos.y) - 11.0F, 7.0F, gd::kInk);
        }
    }
}

void drawGrid(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kGridX, kGridY, 280.0F, 184.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, kGridX + 12.0F, kGridY + 8.0F, "WORK PRIORITIES", gd::kMuted);
    for (int col = 0; col < wp::kKinds; ++col) {
        const SDL_FRect cell = cellRect(0, col);
        const gd::Color kc = kindColor(static_cast<wp::Kind>(col));
        gd::fillRect(r, cell.x, kGridY + 28.0F, cell.w, 4.0F, kc);
        app.hud.text(r, cell.x + 3.0F, kGridY + 33.0F, kKindNames[static_cast<std::size_t>(col)], gd::kInk, gd::Hud::Style::Bold);
    }
    const SDL_FPoint mouse{app.mouseX, app.mouseY};
    for (int row = 0; row < wp::kColonists; ++row) {
        const SDL_FRect first = cellRect(row, 0);
        app.hud.text(r, kGridX + 12.0F, first.y + 6.0F, kNames[static_cast<std::size_t>(row)], gd::kInk, gd::Hud::Style::Bold);
        for (int col = 0; col < wp::kKinds; ++col) {
            const SDL_FRect cell = cellRect(row, col);
            const int p = app.grid[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
            gd::Color fill = gd::kPaper;
            gd::Color ink = gd::kMuted;
            switch (p) {
                case 1: fill = gd::kTeal; ink = gd::kWhite; break;
                case 2: fill = gd::Color{94, 190, 180}; ink = gd::kInk; break;
                case 3: fill = gd::kTealSoft; ink = gd::kInk; break;
                case 4: fill = gd::kWhite; ink = gd::kInk; break;
                default: break;
            }
            gd::fillRect(r, cell.x, cell.y, cell.w, cell.h, fill);
            gd::strokeRect(r, cell.x, cell.y, cell.w, cell.h, p == 0 ? gd::kMuted : gd::kInk);
            if (SDL_PointInRectFloat(&mouse, &cell)) gd::strokeRect(r, cell.x - 2.0F, cell.y - 2.0F, cell.w + 4.0F, cell.h + 4.0F, gd::kOrange);
            centered(app, cell.x + cell.w / 2.0F, cell.y + 6.0F, p == 0 ? "-" : std::to_string(p).c_str(), ink, gd::Hud::Style::Bold);
        }
    }
}

void drawResults(App& app) {
    SDL_Renderer* r = app.renderer;
    const wp::World& w = app.world;
    gd::card(r, kGridX, 244.0F, 280.0F, 160.0F, gd::kWhite, gd::kInk);
    double walked = 0.0;
    for (const wp::Colonist& c : w.colonists) walked += c.walked;
    app.hud.text(r, kGridX + 12.0F, 252.0F, w.finished ? "this run, done" : "this run", gd::kMuted);
    app.hud.text(r, kGridX + 12.0F, 270.0F, (std::to_string(static_cast<long>(std::lround(walked))) + " px").c_str(), gd::kInk, gd::Hud::Style::Large);
    int undone = 0;
    for (const wp::Task& t : w.tasks) undone += t.done ? 0 : 1;
    const std::string line = "t " + tenths(w.time) + " s   open " + std::to_string(undone);
    app.hud.text(r, kGridX + 12.0F, 316.0F, line.c_str(), gd::kInk, gd::Hud::Style::Bold);

    gd::fillRect(r, kGridX + 12.0F, 342.0F, 256.0F, 1.0F, gd::kGrid);
    const std::array<const char*, 2> names{"Rule ", "Score"};
    for (int i = 0; i < 2; ++i) {
        const wp::Result& res = app.results[static_cast<std::size_t>(i)];
        const bool on = i == ruleIndex(app.rule);
        std::string s = std::string(names[static_cast<std::size_t>(i)]) + " " + std::to_string(static_cast<long>(std::lround(res.walked))) + " px  " + tenths(res.seconds) + " s";
        if (res.undone > 0) s += "  " + std::to_string(res.undone) + "!";
        app.hud.text(r, kGridX + 12.0F, 352.0F + 24.0F * static_cast<float>(i), s.c_str(), on ? gd::kTealDark : gd::kInk, on ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
    }
}

void drawStrip(App& app) {
    SDL_Renderer* r = app.renderer;
    const wp::World& w = app.world;
    app.hud.text(r, 24.0F, 418.0F, "done order", gd::kMuted);
    const float x0 = 130.0F;
    for (std::size_t i = 0; i < w.tasks.size(); ++i) {
        const float x = x0 + 24.0F * static_cast<float>(i);
        if (i < w.doneOrder.size()) {
            gd::fillRect(r, x, 414.0F, 20.0F, 22.0F, kindColor(w.doneOrder[i]));
            gd::strokeRect(r, x, 414.0F, 20.0F, 22.0F, gd::kInk);
        } else {
            gd::strokeRect(r, x, 414.0F, 20.0F, 22.0F, gd::kGrid);
        }
    }
    const std::string count = std::to_string(w.doneOrder.size()) + "/" + std::to_string(w.tasks.size());
    app.hud.text(r, x0 + 24.0F * static_cast<float>(w.tasks.size()) + 6.0F, 418.0F, count.c_str(), gd::kInk, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ WORK PRIORITIES ]",
                     app.rule == wp::Rule::Priority ? "lowest number first, tie: left column, empty never" : "distance + 150 per priority step, empty never");
    drawMap(app);
    drawGrid(app);
    drawResults(app);
    drawStrip(app);
    app.hud.text(r, 24.0F, 446.0F, "click cell: priority 1 to 4, then empty   right click: back", gd::kMuted);
    if (app.results[0].undone > 0 || app.results[1].undone > 0) {
        app.hud.text(r, 650.0F, 446.0F, "! = tasks nobody may do", gd::kOrange);
    }
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void advance(App& app, double dt) {
    if (app.world.finished) {
        app.hold += dt;
        if (app.hold >= kHoldSeconds) restart(app);
        return;
    }
    app.acc += dt * (app.fast ? kSpeedFast : kSpeedNormal);
    int steps = 0;
    while (app.acc >= wp::kStep && steps < kMaxStepsPerFrame && !app.world.finished) {
        wp::step(app.world, wp::kStep);
        app.acc -= wp::kStep;
        ++steps;
    }
    if (steps >= kMaxStepsPerFrame) app.acc = 0.0;  // drop the backlog instead of racing to catch up
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
    if (!SDL_CreateWindowAndRenderer("work-priorities", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    setGrid(*app, app->grid);
    app->lastNs = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouseX = event->motion.x;
            app.mouseY = event->motion.y;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.mouseX = -100.0F;
            app.mouseY = -100.0F;
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouseX = p.x;
            app.mouseY = p.y;
            const bool left = event->button.button == SDL_BUTTON_LEFT;
            const bool right = event->button.button == SDL_BUTTON_RIGHT;
            if (!left && !right) break;
            for (int row = 0; row < wp::kColonists; ++row) {
                for (int col = 0; col < wp::kKinds; ++col) {
                    const SDL_FRect cell = cellRect(row, col);
                    if (SDL_PointInRectFloat(&p, &cell)) changeCell(app, row, col, left ? 1 : -1);
                }
            }
            if (left) {
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c.action);
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here toggles or restarts, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::RulePriority); break;
                case SDLK_2: apply(app, Action::RuleScore); break;
                case SDLK_3: apply(app, Action::PresetHaul); break;
                case SDLK_4: apply(app, Action::PresetBuild); break;
                case SDLK_5: apply(app, Action::PresetEqual); break;
                case SDLK_F: apply(app, Action::Fast); break;
                case SDLK_R: apply(app, Action::Restart); break;
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
    const double dt = std::min(static_cast<double>(now - app.lastNs) / 1e9, 0.25);
    app.lastNs = now;
    advance(app, dt);
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
