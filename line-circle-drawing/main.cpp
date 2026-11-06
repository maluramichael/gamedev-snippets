// Lines and circles on a grid with SDL3 main callbacks.
//
// Two handles, A and B. In line mode they are the ends of a line and three algorithms say which cells it covers:
// lerp, Bresenham and supercover. They give different answers, and there is no single right one: it depends on what
// the cells are for. Drawing wants one cell per step (Bresenham, lerp), a line of sight wants every cell the line
// touches (supercover). In circle mode A is the center and B sets the radius. The geometry lives in logic.hpp and is
// covered by test.cpp, this file is the drawing and the input. Walls (right click or W) make the difference between
// the line algorithms matter: the sight is blocked or clear per algorithm.

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
constexpr int kCols = 28;
constexpr int kRows = 16;
constexpr float kCell = 24.0F;
constexpr float kGridX = 24.0F;
constexpr float kGridY = 40.0F;
constexpr float kPanelX = 712.0F;
constexpr float kPanelW = 224.0F;
constexpr float kChipH = 34.0F;
constexpr float kChipRow1 = 440.0F;
constexpr float kChipRow2 = 478.0F;
constexpr gd::Color kInkSoft{17, 24, 39, 150};
constexpr gd::Color kStone{120, 113, 108};

enum class Mode { Line, Circle };
enum class Action { ModeLine, ModeCircle, Algo0, Algo1, Algo2, Compare, Swap, Corners, WallEdit, ClearWalls, Fill, Octants };

struct Chip {
    Action action;
    std::string label;
    bool on;
    SDL_FRect rect;
};

/// Everything that is derived from the state, recomputed when the state changes and not every frame.
struct Result {
    std::array<lcd::Cells, 3> cells;
    std::array<lcd::Cells, 3> frames;  // what the compare frames mark: all cells, or for filled discs only the ones that differ
    std::array<bool, 3> sight{};      // line mode
    std::array<bool, 3> connected{};  // circle mode, ring only
    int differ = 0;                   // line mode: cells that not all three lines share
    int radius = 0;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    Mode mode = Mode::Line;
    std::array<lcd::Cell, 2> line{lcd::linePresetA(), lcd::linePresetB()};
    std::array<lcd::Cell, 2> circle{lcd::Cell{14, 8}, lcd::Cell{21, 8}};
    std::array<int, 2> algo{2, 1};  // selected algorithm per mode: supercover, midpoint
    bool compare = true;
    bool fill = false;
    bool octants = false;
    bool wallEdit = false;
    lcd::Corner corner = lcd::Corner::Both;
    lcd::Walls walls = lcd::lineWalls();
    int dragHandle = -1;
    std::optional<bool> paint;  // wall value being painted while a button is held
    Uint8 paintButton = 0;
    Result result;
    std::vector<Chip> chips;
    bool dirty = true;
    long maxFrames = -1;
    long frames = 0;
};

int modeIndex(const App& app) { return app.mode == Mode::Line ? 0 : 1; }
std::array<lcd::Cell, 2>& handles(App& app) { return app.mode == Mode::Line ? app.line : app.circle; }

gd::Color algoColor(const App& app, int i) {
    static constexpr std::array<gd::Color, 3> line{gd::kOrange, gd::kInk, gd::kTeal};
    static constexpr std::array<gd::Color, 3> circle{gd::kInk, gd::kTeal, gd::kOrange};
    return app.mode == Mode::Line ? line[static_cast<std::size_t>(i)] : circle[static_cast<std::size_t>(i)];
}

const char* algoName(const App& app, int i) {
    static constexpr std::array<const char*, 3> line{"Lerp", "Bresenham", "Supercover"};
    static constexpr std::array<const char*, 3> circle{"Distance test", "Midpoint", "Rows only"};
    return app.mode == Mode::Line ? line[static_cast<std::size_t>(i)] : circle[static_cast<std::size_t>(i)];
}

lcd::Cell cellAt(float x, float y) {
    return {static_cast<int>(std::floor((x - kGridX) / kCell)), static_cast<int>(std::floor((y - kGridY) / kCell))};
}

bool inGrid(lcd::Cell c) { return c.x >= 0 && c.y >= 0 && c.x < kCols && c.y < kRows; }
lcd::Cell clampToGrid(lcd::Cell c) { return {std::clamp(c.x, 0, kCols - 1), std::clamp(c.y, 0, kRows - 1)}; }

void recompute(App& app) {
    Result& res = app.result;
    const auto& h = handles(app);
    if (app.mode == Mode::Line) {
        res.cells[0] = lcd::lerpLine(h[0], h[1]);
        res.cells[1] = lcd::bresenhamLine(h[0], h[1]);
        res.cells[2] = lcd::supercoverLine(h[0], h[1], app.corner);
        for (std::size_t i = 0; i < 3; ++i) res.sight[i] = lcd::lineOfSight(res.cells[i], h[0], h[1], app.walls);
        // cells that are in the union but not in the intersection of the three lines
        lcd::Cells all;
        for (const lcd::Cells& c : res.cells) all.insert(all.end(), c.begin(), c.end());
        lcd::normalize(all);
        res.differ = 0;
        for (const lcd::Cell& c : all) {
            const auto in = [&](const lcd::Cells& s) { return std::find(s.begin(), s.end(), c) != s.end(); };
            if (!(in(res.cells[0]) && in(res.cells[1]) && in(res.cells[2]))) ++res.differ;
        }
    } else {
        res.radius = lcd::radiusOf(h[0], h[1]);
        if (app.fill) {
            res.cells[0] = lcd::discDistance(h[0], res.radius);
            res.cells[1] = lcd::discMidpoint(h[0], res.radius);
            res.cells[2] = lcd::discByRows(h[0], res.radius);
        } else {
            res.cells[0] = lcd::ringDistance(h[0], res.radius);
            res.cells[1] = lcd::ringMidpoint(h[0], res.radius);
            res.cells[2] = lcd::ringRowsOnly(h[0], res.radius);
        }
        for (std::size_t i = 0; i < 3; ++i) res.connected[i] = lcd::connected8(res.cells[i]);
    }
    for (std::size_t i = 0; i < 3; ++i) {
        res.frames[i].clear();
        for (const lcd::Cell& c : res.cells[i]) {
            const auto in = [&](const lcd::Cells& set) { return std::find(set.begin(), set.end(), c) != set.end(); };
            // inside a filled disc the three sets agree almost everywhere, framing all of it only hides the difference
            const bool shared = in(res.cells[0]) && in(res.cells[1]) && in(res.cells[2]);
            if (!(app.mode == Mode::Circle && app.fill && shared)) res.frames[i].push_back(c);
        }
    }
}

float labelWidth(const App& app, const std::string& s) { return app.hud.width(s, gd::Hud::Style::Bold) + 24.0F; }

void layoutChips(App& app) {
    app.chips.clear();
    struct Def {
        Action action;
        std::string label;
        bool on;
        float gapBefore;
        int row;
    };
    const bool lineMode = app.mode == Mode::Line;
    const int sel = app.algo[static_cast<std::size_t>(modeIndex(app))];
    std::vector<Def> defs{{Action::ModeLine, "1 Line", lineMode, 0.0F, 0}, {Action::ModeCircle, "2 Circle", !lineMode, 8.0F, 0}};
    for (int i = 0; i < 3; ++i) {
        defs.push_back({static_cast<Action>(static_cast<int>(Action::Algo0) + i), std::string(1, static_cast<char>('3' + i)) + " " + algoName(app, i), sel == i, i == 0 ? 24.0F : 8.0F, 0});
    }
    defs.push_back({Action::Compare, "C Compare", app.compare, 0.0F, 1});
    if (lineMode) {
        defs.push_back({Action::Swap, "X Swap ends", false, 8.0F, 1});
        defs.push_back({Action::Corners, app.corner == lcd::Corner::Both ? "K Corners: both" : "K Corners: diagonal", false, 8.0F, 1});
        defs.push_back({Action::WallEdit, "W Draw walls", app.wallEdit, 24.0F, 1});
        defs.push_back({Action::ClearWalls, "Del Clear walls", false, 8.0F, 1});
    } else {
        defs.push_back({Action::Fill, "F Fill", app.fill, 8.0F, 1});
        defs.push_back({Action::Octants, "O Octants", app.octants, 8.0F, 1});
    }
    float x = kGridX;
    int row = 0;
    for (const Def& d : defs) {
        if (d.row != row) {
            row = d.row;
            x = kGridX;
        }
        x += d.gapBefore;
        const float w = labelWidth(app, d.label);
        app.chips.push_back({d.action, d.label, d.on, SDL_FRect{x, row == 0 ? kChipRow1 : kChipRow2, w, kChipH}});
        x += w;
    }
}

void refresh(App& app) {
    recompute(app);
    layoutChips(app);
    app.dirty = false;
}

void resetPointer(App& app) {
    app.dragHandle = -1;
    app.paint.reset();
    app.paintButton = 0;
}

void setMode(App& app, Mode mode) {
    if (app.mode == mode) return;
    app.mode = mode;
    resetPointer(app);
    app.dirty = true;
}

void apply(App& app, Action action) {
    const bool lineMode = app.mode == Mode::Line;
    switch (action) {
        case Action::ModeLine: setMode(app, Mode::Line); break;
        case Action::ModeCircle: setMode(app, Mode::Circle); break;
        case Action::Algo0:
        case Action::Algo1:
        case Action::Algo2: app.algo[static_cast<std::size_t>(modeIndex(app))] = static_cast<int>(action) - static_cast<int>(Action::Algo0); break;
        case Action::Compare: app.compare = !app.compare; break;
        case Action::Swap:
            if (lineMode) std::swap(app.line[0], app.line[1]);
            break;
        case Action::Corners:
            if (lineMode) app.corner = app.corner == lcd::Corner::Both ? lcd::Corner::Diagonal : lcd::Corner::Both;
            break;
        case Action::WallEdit:
            if (lineMode) app.wallEdit = !app.wallEdit;
            break;
        case Action::ClearWalls:
            if (lineMode) app.walls.clear();
            break;
        case Action::Fill:
            if (!lineMode) app.fill = !app.fill;
            break;
        case Action::Octants:
            if (!lineMode) app.octants = !app.octants;
            break;
    }
    app.dirty = true;
}

void startPaint(App& app, lcd::Cell c, Uint8 button) {
    app.paint = !app.walls.at(c);
    app.paintButton = button;
    app.walls.set(c, *app.paint);
    app.dirty = true;
}

void moveHandle(App& app, int which, lcd::Cell c) {
    c = clampToGrid(c);
    auto& h = handles(app)[static_cast<std::size_t>(which)];
    if (!(h == c)) {
        h = c;
        app.dirty = true;
    }
}

// ----------------------------------------------------------------------------------------------------------- drawing

SDL_FRect cellRect(lcd::Cell c, float inset = 0.0F) {
    return {kGridX + static_cast<float>(c.x) * kCell + inset, kGridY + static_cast<float>(c.y) * kCell + inset, kCell - 2.0F * inset, kCell - 2.0F * inset};
}

gd::Color withAlpha(gd::Color c, Uint8 a) { return gd::Color{c.r, c.g, c.b, a}; }

float centerX(lcd::Cell c) { return kGridX + (static_cast<float>(c.x) + 0.5F) * kCell; }
float centerY(lcd::Cell c) { return kGridY + (static_cast<float>(c.y) + 0.5F) * kCell; }

void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -1; o <= 1; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

void drawCells(App& app, const lcd::Cells& cells, gd::Color color, float inset, bool filled, Uint8 alpha) {
    for (const lcd::Cell& c : cells) {
        if (!inGrid(c)) continue;
        const SDL_FRect rect = cellRect(c, inset);
        if (filled) {
            gd::fillRect(app.renderer, rect.x, rect.y, rect.w, rect.h, withAlpha(color, alpha));
        } else {
            gd::strokeRect(app.renderer, rect.x, rect.y, rect.w, rect.h, color);
        }
    }
}

void drawGrid(App& app) {
    SDL_Renderer* r = app.renderer;
    const float w = kCell * kCols;
    const float h = kCell * kRows;
    gd::fillRect(r, kGridX, kGridY, w, h, gd::kWhite);
    gd::setColor(r, gd::kGrid);
    for (int x = 0; x <= kCols; ++x) SDL_RenderLine(r, kGridX + static_cast<float>(x) * kCell, kGridY, kGridX + static_cast<float>(x) * kCell, kGridY + h);
    for (int y = 0; y <= kRows; ++y) SDL_RenderLine(r, kGridX, kGridY + static_cast<float>(y) * kCell, kGridX + w, kGridY + static_cast<float>(y) * kCell);
}

void drawWalls(App& app) {
    for (int y = 0; y < kRows; ++y) {
        for (int x = 0; x < kCols; ++x) {
            if (!app.walls.at({x, y})) continue;
            const SDL_FRect rc = cellRect({x, y}, 1.0F);
            gd::fillRect(app.renderer, rc.x, rc.y, rc.w, rc.h, kStone);
            gd::setColor(app.renderer, gd::kWhite);
            SDL_RenderLine(app.renderer, rc.x + 3.0F, rc.y + 3.0F, rc.x + rc.w - 3.0F, rc.y + rc.h - 3.0F);
            SDL_RenderLine(app.renderer, rc.x + rc.w - 3.0F, rc.y + 3.0F, rc.x + 3.0F, rc.y + rc.h - 3.0F);
        }
    }
}

void drawFills(App& app) {
    const int sel = app.algo[static_cast<std::size_t>(modeIndex(app))];
    const auto& h = handles(app);
    const lcd::Cells& cells = app.result.cells[static_cast<std::size_t>(sel)];
    const Uint8 alpha = app.compare ? 120 : 230;
    if (app.mode == Mode::Circle && app.octants && !app.fill) {
        // the computed octant orange, its seven mirror images teal
        lcd::Cells first, mirrored;
        for (const lcd::Cell& c : cells) (lcd::inFirstOctant(c.x - h[0].x, c.y - h[0].y) ? first : mirrored).push_back(c);
        drawCells(app, mirrored, gd::kTeal, 1.0F, true, alpha);
        drawCells(app, first, gd::kOrange, 1.0F, true, 255);
    } else {
        drawCells(app, cells, algoColor(app, sel), 1.0F, true, alpha);
    }
}

void drawFrames(App& app) {
    if (!app.compare) return;
    // a frame per algorithm, nested: a cell with three frames is shared by all three
    for (int i = 0; i < 3; ++i) {
        drawCells(app, app.result.frames[static_cast<std::size_t>(i)], algoColor(app, i), 2.0F + 3.0F * static_cast<float>(i), false, 255);
    }
}

void drawGuides(App& app) {
    SDL_Renderer* r = app.renderer;
    const auto& h = handles(app);
    if (app.mode == Mode::Line) {
        thickLine(r, centerX(h[0]), centerY(h[0]), centerX(h[1]), centerY(h[1]), kInkSoft);
        return;
    }
    const float cx = centerX(h[0]);
    const float cy = centerY(h[0]);
    const float rad = static_cast<float>(app.result.radius) * kCell;
    gd::ring(r, cx, cy, rad, kInkSoft, 120);
    gd::ring(r, cx, cy, rad + 1.0F, kInkSoft, 120);
    if (app.octants) {
        // the mirror axes: the algorithm only computes the orange part, the rest is reflection
        gd::setColor(r, gd::kOrange);
        constexpr float kFar = 600.0F;
        SDL_RenderLine(r, cx - kFar, cy, cx + kFar, cy);
        SDL_RenderLine(r, cx, cy - kFar, cx, cy + kFar);
        SDL_RenderLine(r, cx - kFar, cy - kFar, cx + kFar, cy + kFar);
        SDL_RenderLine(r, cx - kFar, cy + kFar, cx + kFar, cy - kFar);
    }
}

void drawHandles(App& app) {
    SDL_Renderer* r = app.renderer;
    const auto& h = handles(app);
    const char* names = app.mode == Mode::Line ? "AB" : "CR";
    for (std::size_t i = 0; i < 2; ++i) {
        const float x = centerX(h[i]);
        const float y = centerY(h[i]);
        const bool hot = app.dragHandle == static_cast<int>(i);
        gd::disc(r, x, y, 8.0F, hot ? gd::kOrange : gd::kWhite);
        gd::ring(r, x, y, 8.0F, gd::kInk);
        gd::ring(r, x, y, 9.0F, gd::kInk);
        const char label[2] = {names[i], 0};
        const float ty = y - 34.0F < kGridY ? y + 14.0F : y - 34.0F;
        app.hud.text(r, x - 5.0F, ty, label, gd::kInk, gd::Hud::Style::Bold);
    }
}

void line(App& app, float x, float y, const std::string& s, gd::Color c = gd::kInk, gd::Hud::Style st = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, x, y, s, c, st);
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    const Result& res = app.result;
    const auto& h = handles(app);
    const bool lineMode = app.mode == Mode::Line;
    const int sel = app.algo[static_cast<std::size_t>(modeIndex(app))];
    gd::card(r, kPanelX, kGridY, kPanelW, kCell * kRows, gd::kWhite, gd::kInk);
    const float x = kPanelX + 14.0F;
    char buf[64];
    if (lineMode) {
        line(app, x, 48.0F, "LINE", gd::kMuted);
        SDL_snprintf(buf, sizeof buf, "A %d,%d   B %d,%d", h[0].x, h[0].y, h[1].x, h[1].y);
    } else {
        line(app, x, 48.0F, app.fill ? "FILLED CIRCLE" : "CIRCLE OUTLINE", gd::kMuted);
        SDL_snprintf(buf, sizeof buf, "C %d,%d   r %d", h[0].x, h[0].y, res.radius);
    }
    line(app, x, 72.0F, buf, gd::kInk, gd::Hud::Style::Bold);
    for (int i = 0; i < 3; ++i) {
        const float y = 108.0F + 76.0F * static_cast<float>(i);
        const gd::Color color = algoColor(app, i);
        const bool shown = i == sel || app.compare;
        const bool primary = i == sel;
        gd::fillRect(r, x, y + 3.0F, 14.0F, 14.0F, primary ? color : gd::kWhite);
        gd::strokeRect(r, x, y + 3.0F, 14.0F, 14.0F, shown ? color : gd::kMuted);
        if (shown && !primary) gd::strokeRect(r, x + 3.0F, y + 6.0F, 8.0F, 8.0F, color);
        line(app, x + 24.0F, y, algoName(app, i), gd::kInk, gd::Hud::Style::Bold);
        SDL_snprintf(buf, sizeof buf, "%d cells", static_cast<int>(res.cells[static_cast<std::size_t>(i)].size()));
        line(app, x + 24.0F, y + 22.0F, buf, gd::kMuted);
        if (lineMode) {
            const bool clear = res.sight[static_cast<std::size_t>(i)];
            line(app, x + 24.0F, y + 44.0F, clear ? "sight clear" : "sight BLOCKED", clear ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
        } else if (!app.fill) {
            const bool conn = res.connected[static_cast<std::size_t>(i)];
            line(app, x + 24.0F, y + 44.0F, conn ? "closed ring" : "GAPS", conn ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
        }
    }
    if (lineMode) {
        SDL_snprintf(buf, sizeof buf, "%d cells differ", res.differ);
        line(app, x, 340.0F, buf, res.differ == 0 ? gd::kMuted : gd::kOrange, gd::Hud::Style::Bold);
        SDL_snprintf(buf, sizeof buf, "%d walls", app.walls.count());
        line(app, x, 362.0F, buf, gd::kMuted);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const bool lineMode = app.mode == Mode::Line;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ LINES AND CIRCLES ON A GRID ]", lineMode ? "lerp / bresenham / supercover" : "distance test / midpoint");

    drawGrid(app);
    const SDL_Rect clip{static_cast<int>(kGridX), static_cast<int>(kGridY), static_cast<int>(kCell) * kCols + 1, static_cast<int>(kCell) * kRows + 1};
    SDL_SetRenderClipRect(r, &clip);
    drawFills(app);
    if (lineMode) drawWalls(app);
    drawFrames(app);
    drawGuides(app);
    drawHandles(app);
    SDL_SetRenderClipRect(r, nullptr);
    gd::strokeRect(r, kGridX, kGridY, kCell * kCols, kCell * kRows, gd::kInk);

    drawPanel(app);

    for (const Chip& c : app.chips) {
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, c.on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, c.on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, kGridX, 516.0F,
                 lineMode ? "drag A and B   right click or W: draw walls   sight: no wall between A and B"
                          : "drag C (center) and R (rim)   O: the part the algorithm computes, mirrored 8 times",
                 gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("line-circle-drawing", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    refresh(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            const lcd::Cell c = cellAt(event->motion.x, event->motion.y);
            if (app.dragHandle >= 0) {
                moveHandle(app, app.dragHandle, c);
            } else if (app.paint && inGrid(c) && app.walls.at(c) != *app.paint) {
                app.walls.set(c, *app.paint);
                app.dirty = true;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            const lcd::Cell c = cellAt(p.x, p.y);
            if (event->button.button == SDL_BUTTON_LEFT) {
                bool onChip = false;
                for (const Chip& chip : app.chips) {
                    if (SDL_PointInRectFloat(&p, &chip.rect)) {
                        apply(app, chip.action);
                        onChip = true;
                        break;
                    }
                }
                if (onChip || !inGrid(c)) break;
                if (app.mode == Mode::Line && app.wallEdit) {
                    startPaint(app, c, SDL_BUTTON_LEFT);
                } else {
                    // grab the handle that is closer to the click
                    const auto& h = handles(app);
                    const auto dist2 = [&](const lcd::Cell& a) { return (a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y); };
                    app.dragHandle = dist2(h[0]) < dist2(h[1]) ? 0 : 1;
                    moveHandle(app, app.dragHandle, c);
                }
            } else if (event->button.button == SDL_BUTTON_RIGHT && app.mode == Mode::Line && inGrid(c)) {
                startPaint(app, c, SDL_BUTTON_RIGHT);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragHandle = -1;
            if (event->button.button == app.paintButton) {
                app.paint.reset();
                app.paintButton = 0;
            }
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: resetPointer(app); break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::ModeLine); break;
                case SDLK_2: apply(app, Action::ModeCircle); break;
                case SDLK_3: apply(app, Action::Algo0); break;
                case SDLK_4: apply(app, Action::Algo1); break;
                case SDLK_5: apply(app, Action::Algo2); break;
                case SDLK_C: apply(app, Action::Compare); break;
                case SDLK_X: apply(app, Action::Swap); break;
                case SDLK_K: apply(app, Action::Corners); break;
                case SDLK_W: apply(app, Action::WallEdit); break;
                case SDLK_F: apply(app, Action::Fill); break;
                case SDLK_O: apply(app, Action::Octants); break;
                case SDLK_DELETE:
                case SDLK_BACKSPACE: apply(app, Action::ClearWalls); break;
                default: break;
            }
            break;
        default: break;
    }
    if (app.dirty) refresh(app);
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
