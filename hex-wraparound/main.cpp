// Hex map wraparound with SDL3 main callbacks.
//
// A hexagon-shaped map of radius N whose edges join up. The trick: copies of the map, moved by the six mirror centers
// (2N+1, -N, -N-1) turned in 60 degree steps, tile the whole plane. "Plane" draws the map and its six copies, the
// figure exists once per copy, and when it leaves one it enters the next. A step that leaves the map is fixed by
// subtracting that copy's center, the 6(N+1) tiles just outside are worked out once in a lookup table. "Naive" jumps
// to the tile opposite the one you left instead, and the walk never leaves its row. The math is in logic.hpp and
// covered by test.cpp, this file draws and handles input.

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

using hexwrap::Cube;

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr SDL_FRect kField{16.0F, 38.0F, 620.0F, 398.0F};
constexpr float kPanelX = 650.0F;
constexpr float kPanelW = 294.0F;
constexpr float kMaxSize = 46.0F;
constexpr float kWalkInterval = 0.45F;  // seconds between two automatic steps
constexpr float kSlide = 0.18F;         // seconds a step takes on screen
constexpr float kFlash = 0.7F;          // seconds the naive jump stays marked
constexpr float kLabelSize = 30.0F;     // coordinates are printed in the tiles from this hex size on
constexpr float kMaxDt = 0.25F;

enum class View { Plane, Fringe };
enum class Rule { Wrap, Naive };
enum class Action { Radius1, Radius2, Radius3, Plane, Fringe, Wrap, Naive, Walk, Reset, NW, NE, W, E, SW, SE };

struct Chip {
    Action action;
    SDL_FRect rect;
};

/// One tile that is drawn: its place in the plane, which of the seven maps it belongs to and the tile of the real map it stands for.
struct Tile {
    Cube h;
    int copy;  // 0 = the map, 1..6 = copy around mirrorCenter(copy - 1)
    Cube canon;
    SDL_FPoint px;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    int radius = 2;
    View view = View::Plane;
    Rule rule = Rule::Wrap;
    hexwrap::WrapTable table{2};

    std::vector<Tile> tiles;
    std::vector<int> tileAt;  // (q + gridR) * (2 gridR + 1) + (r + gridR) -> index into tiles or -1
    int gridR = 0;
    float size = 20.0F;

    Cube figure{0, 0, 0};
    int heading = 0;
    bool walking = true;
    float walkTimer = 0.0F;
    std::vector<char> visited;
    int visitedCount = 0;

    bool sliding = false;
    float slideT = 0.0F;
    Cube slideFrom{0, 0, 0};
    int slideDir = 0;
    bool flashing = false;
    float flashT = 0.0F;
    Cube flashFrom{0, 0, 0};

    int hover = -1;
    SDL_FPoint mouse{-1000.0F, -1000.0F};
    bool mouseSeen = false;

    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

const char* const kDirNames[hexwrap::kDirections] = {"E", "NE", "NW", "W", "SW", "SE"};

// ---- tiles on the screen ----------------------------------------------------------------------------------------

int indexOf(const App& app, Cube h) {
    if (std::abs(h.q) > app.gridR || std::abs(h.r) > app.gridR) return -1;
    return app.tileAt[static_cast<std::size_t>((h.q + app.gridR) * (2 * app.gridR + 1) + (h.r + app.gridR))];
}

std::size_t visitedSlot(const App& app, Cube c) { return static_cast<std::size_t>((c.q + app.radius) * (2 * app.radius + 1) + (c.r + app.radius)); }

bool shown(const App& app, int copy, Cube h) {
    if (copy < 0) return false;
    if (app.rule == Rule::Naive) return copy == 0;
    if (app.view == View::Fringe) return hexwrap::length(h) <= app.radius + 1;
    return true;
}

SDL_FPoint unitToScreen(const App& app, SDL_FPoint origin, Cube h) {
    const hexwrap::Point c = hexwrap::center(h);
    return {origin.x + static_cast<float>(c.x) * app.size, origin.y + static_cast<float>(c.y) * app.size};
}

/// Collects the tiles to draw and fits them into the field. Call whenever the radius, view or rule changes.
void rebuild(App& app) {
    app.gridR = 3 * app.radius + 1;
    const int side = 2 * app.gridR + 1;
    app.tileAt.assign(static_cast<std::size_t>(side * side), -1);
    app.tiles.clear();

    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (int q = -app.gridR; q <= app.gridR; ++q) {
        for (int r = -app.gridR; r <= app.gridR; ++r) {
            const Cube h = hexwrap::make(q, r);
            if (hexwrap::length(h) > app.gridR) continue;
            const int copy = hexwrap::copyIndex(h, app.radius);
            if (!shown(app, copy, h)) continue;
            app.tileAt[static_cast<std::size_t>((q + app.gridR) * side + (r + app.gridR))] = static_cast<int>(app.tiles.size());
            app.tiles.push_back({h, copy, hexwrap::wrap(h, app.radius), {0.0F, 0.0F}});
            const hexwrap::Point c = hexwrap::center(h);
            minX = std::min(minX, c.x - hexwrap::kSqrt3 / 2.0);
            maxX = std::max(maxX, c.x + hexwrap::kSqrt3 / 2.0);
            minY = std::min(minY, c.y - 1.0);
            maxY = std::max(maxY, c.y + 1.0);
        }
    }
    // The map itself is always among the tiles, so the extents are positive.
    app.size = std::min({kMaxSize, kField.w / static_cast<float>(maxX - minX), kField.h / static_cast<float>(maxY - minY)});
    const SDL_FPoint origin{kField.x + kField.w / 2.0F - static_cast<float>((minX + maxX) / 2.0) * app.size,
                            kField.y + kField.h / 2.0F - static_cast<float>((minY + maxY) / 2.0) * app.size};
    for (Tile& t : app.tiles) t.px = unitToScreen(app, origin, t.h);
    app.hover = -1;
}

int pickTile(const App& app, SDL_FPoint p) {
    int best = -1;
    float bestDistance = 1e30F;
    for (std::size_t i = 0; i < app.tiles.size(); ++i) {
        const float dx = p.x - app.tiles[i].px.x;
        const float dy = p.y - app.tiles[i].px.y;
        const float d = dx * dx + dy * dy;
        if (d < bestDistance) {
            bestDistance = d;
            best = static_cast<int>(i);
        }
    }
    if (best < 0) return -1;
    const SDL_FPoint c = app.tiles[static_cast<std::size_t>(best)].px;
    return hexwrap::insideHex({static_cast<double>((p.x - c.x) / app.size), static_cast<double>((p.y - c.y) / app.size)}) ? best : -1;
}

// ---- the walk ---------------------------------------------------------------------------------------------------

void visit(App& app, Cube c) {
    char& seen = app.visited[visitedSlot(app, c)];
    if (seen == 0) {
        seen = 1;
        ++app.visitedCount;
    }
}

void resetWalk(App& app) {
    app.figure = {0, 0, 0};
    app.heading = 0;
    app.walkTimer = 0.0F;
    app.sliding = false;
    app.flashing = false;
    const int side = 2 * app.radius + 1;
    app.visited.assign(static_cast<std::size_t>(side * side), 0);
    app.visitedCount = 0;
    visit(app, app.figure);
}

void step(App& app, int dir) {
    const Cube from = app.figure;
    const bool crossed = hexwrap::length(from + hexwrap::direction(dir)) > app.radius;
    app.heading = dir;
    app.figure = app.rule == Rule::Wrap ? app.table.step(from, dir) : hexwrap::naiveStep(from, dir, app.radius);
    visit(app, app.figure);
    app.walkTimer = 0.0F;
    if (app.rule == Rule::Naive && crossed) {
        app.sliding = false;
        app.flashing = true;
        app.flashT = 0.0F;
        app.flashFrom = from;
    } else {
        app.sliding = true;
        app.slideT = 0.0F;
        app.slideFrom = from;
        app.slideDir = dir;
    }
}

void setRadius(App& app, int radius) {
    if (radius == app.radius) return;
    app.radius = radius;
    app.table = hexwrap::WrapTable(radius);
    rebuild(app);
    resetWalk(app);
}

void setRule(App& app, Rule rule) {
    if (rule == app.rule) return;
    app.rule = rule;
    rebuild(app);
    resetWalk(app);
}

void setView(App& app, View view) {
    app.view = view;
    if (app.rule != Rule::Wrap) {
        setRule(app, Rule::Wrap);  // the copies only exist for the real wraparound, this rebuilds and restarts the walk
    } else {
        rebuild(app);
    }
}

void teleport(App& app, Cube tile) {
    app.figure = tile;
    app.sliding = false;
    app.flashing = false;
    app.walkTimer = 0.0F;
    visit(app, tile);
}

// ---- chips ------------------------------------------------------------------------------------------------------

void layoutChips(App& app) {
    const std::vector<std::vector<Action>> rows{
        {Action::Radius1, Action::Radius2, Action::Radius3}, {Action::Plane, Action::Fringe}, {Action::Wrap, Action::Naive},
        {Action::Walk, Action::Reset},                       {Action::NW, Action::NE},         {Action::W, Action::E},
        {Action::SW, Action::SE}};
    constexpr float kChipH = 27.0F;
    constexpr float kGap = 12.0F;
    float y = 288.0F;
    for (const auto& row : rows) {
        const float w = (kPanelW - kGap * static_cast<float>(row.size() - 1)) / static_cast<float>(row.size());
        for (std::size_t i = 0; i < row.size(); ++i) app.chips.push_back({row[i], SDL_FRect{kPanelX + static_cast<float>(i) * (w + kGap), y, w, kChipH}});
        y += 33.0F;
    }
}

std::string chipLabel(Action a) {
    switch (a) {
        case Action::Radius1: return "1 N=1";
        case Action::Radius2: return "2 N=2";
        case Action::Radius3: return "3 N=3";
        case Action::Plane: return "4 Plane";
        case Action::Fringe: return "5 Fringe";
        case Action::Wrap: return "6 Wrap";
        case Action::Naive: return "7 Naive";
        case Action::Walk: return "Space Walk";
        case Action::Reset: return "R Reset";
        case Action::NW: return "Q NW";
        case Action::NE: return "E NE";
        case Action::W: return "A W";
        case Action::E: return "D E";
        case Action::SW: return "Z SW";
        case Action::SE: return "C SE";
    }
    return {};
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::Radius1: return app.radius == 1;
        case Action::Radius2: return app.radius == 2;
        case Action::Radius3: return app.radius == 3;
        case Action::Plane: return app.rule == Rule::Wrap && app.view == View::Plane;
        case Action::Fringe: return app.rule == Rule::Wrap && app.view == View::Fringe;
        case Action::Wrap: return app.rule == Rule::Wrap;
        case Action::Naive: return app.rule == Rule::Naive;
        case Action::Walk: return app.walking;
        default: return false;
    }
}

/// Direction index (0..5) of a step chip, or -1.
int stepOf(Action a) {
    switch (a) {
        case Action::E: return 0;
        case Action::NE: return 1;
        case Action::NW: return 2;
        case Action::W: return 3;
        case Action::SW: return 4;
        case Action::SE: return 5;
        default: return -1;
    }
}

void apply(App& app, Action a) {
    if (const int d = stepOf(a); d >= 0) {
        step(app, d);
        return;
    }
    switch (a) {
        case Action::Radius1: setRadius(app, 1); break;
        case Action::Radius2: setRadius(app, 2); break;
        case Action::Radius3: setRadius(app, 3); break;
        case Action::Plane: setView(app, View::Plane); break;
        case Action::Fringe: setView(app, View::Fringe); break;
        case Action::Wrap: setRule(app, Rule::Wrap); break;
        case Action::Naive: setRule(app, Rule::Naive); break;
        case Action::Walk:
            app.walking = !app.walking;
            app.walkTimer = 0.0F;
            break;
        case Action::Reset: resetWalk(app); break;
        default: break;
    }
}

// ---- drawing ----------------------------------------------------------------------------------------------------

SDL_FPoint cornerAt(SDL_FPoint c, int i, float size) {
    const hexwrap::Point p = hexwrap::corner(i);
    return {c.x + static_cast<float>(p.x) * size, c.y + static_cast<float>(p.y) * size};
}

void fillHex(SDL_Renderer* r, SDL_FPoint c, float size, gd::Color color) {
    const SDL_FColor fc = gd::fcolor(color);
    std::array<SDL_Vertex, 7> v{};
    v[0] = {c, fc, {0, 0}};
    for (int i = 0; i < 6; ++i) v[static_cast<std::size_t>(i) + 1] = {cornerAt(c, i, size), fc, {0, 0}};
    constexpr std::array<int, 18> idx{0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 1};
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

void outlineHex(SDL_Renderer* r, SDL_FPoint c, float size, gd::Color color) {
    gd::setColor(r, color);
    for (int i = 0; i < 6; ++i) {
        const SDL_FPoint a = cornerAt(c, i, size);
        const SDL_FPoint b = cornerAt(c, (i + 1) % 6, size);
        SDL_RenderLine(r, a.x, a.y, b.x, b.y);
    }
}

/// The side of a tile that faces direction d, two pixels wide.
void thickSide(SDL_Renderer* r, const App& app, SDL_FPoint c, int d, gd::Color color) {
    const int a = hexwrap::sideCorner(d);
    const SDL_FPoint p = cornerAt(c, a, app.size);
    const SDL_FPoint q = cornerAt(c, (a + 1) % 6, app.size);
    gd::setColor(r, color);
    SDL_RenderLine(r, p.x, p.y, q.x, q.y);
    SDL_RenderLine(r, p.x + 1.0F, p.y, q.x + 1.0F, q.y);
    SDL_RenderLine(r, p.x, p.y + 1.0F, q.x, q.y + 1.0F);
}

void centeredText(App& app, SDL_FPoint c, const std::string& s, gd::Color color, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, c.x - app.hud.width(s, style) / 2.0F, c.y - app.hud.lineHeight(style) / 2.0F, s, color, style);
}

void drawTiles(App& app) {
    SDL_Renderer* r = app.renderer;
    const gd::Color rim{17, 24, 39, 90};
    for (const Tile& t : app.tiles) {
        fillHex(r, t.px, app.size, t.copy == 0 ? gd::kWhite : gd::kTealSoft);
        // A tile the figure has been on carries a smaller orange hex, in every copy. It moves down when there is a label.
        if (app.visited[visitedSlot(app, t.canon)] != 0) {
            const bool labels = app.size >= kLabelSize;
            fillHex(r, {t.px.x, t.px.y + (labels ? app.size * 0.42F : 0.0F)}, app.size * (labels ? 0.32F : 0.55F), gd::Color{253, 186, 140});
        }
        outlineHex(r, t.px, app.size, rim);
    }
    // Where one map ends and the next one begins (or the picture ends): a thick line.
    for (const Tile& t : app.tiles) {
        for (int d = 0; d < hexwrap::kDirections; ++d) {
            const int n = indexOf(app, t.h + hexwrap::direction(d));
            if (n < 0 || app.tiles[static_cast<std::size_t>(n)].copy != t.copy) thickSide(r, app, t.px, d, gd::kInk);
        }
    }
}

void drawLabels(App& app) {
    if (app.size < kLabelSize) return;
    for (const Tile& t : app.tiles) {
        char text[24];
        SDL_snprintf(text, sizeof text, "%d,%d", t.canon.q, t.canon.r);
        centeredText(app, {t.px.x, t.px.y - app.size * 0.3F}, text, t.copy == 0 ? gd::kInk : gd::kMuted);
    }
}

void drawMirrors(App& app) {
    if (app.rule != Rule::Wrap || app.view != View::Plane) return;
    SDL_Renderer* r = app.renderer;
    const int origin = indexOf(app, Cube{0, 0, 0});
    if (origin < 0) return;
    const SDL_FPoint o = app.tiles[static_cast<std::size_t>(origin)].px;
    for (int k = 0; k < 6; ++k) {
        const int i = indexOf(app, hexwrap::mirrorCenter(app.radius, k));
        if (i < 0) continue;
        const SDL_FPoint c = app.tiles[static_cast<std::size_t>(i)].px;
        gd::setColor(r, gd::Color{17, 24, 39, 110});
        SDL_RenderLine(r, o.x, o.y, c.x, c.y);
    }
    outlineHex(r, o, app.size * 0.72F, gd::kInk);
    for (int k = 0; k < 6; ++k) {
        const int i = indexOf(app, hexwrap::mirrorCenter(app.radius, k));
        if (i < 0) continue;
        const SDL_FPoint c = app.tiles[static_cast<std::size_t>(i)].px;
        outlineHex(app.renderer, c, app.size * 0.72F, gd::kInk);
        char text[8];
        SDL_snprintf(text, sizeof text, "m%d", k);
        centeredText(app, c, text, gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawHover(App& app) {
    if (app.hover < 0) return;
    const Cube canon = app.tiles[static_cast<std::size_t>(app.hover)].canon;
    for (const Tile& t : app.tiles) {
        if (!(t.canon == canon)) continue;
        outlineHex(app.renderer, t.px, app.size - 1.0F, gd::kInk);
        outlineHex(app.renderer, t.px, app.size - 2.5F, gd::kInk);
    }
}

void drawPiece(App& app, SDL_FPoint p, bool strong) {
    const gd::Color body = app.rule == Rule::Wrap ? gd::kTealDark : gd::kOrange;
    const float radius = app.size * 0.42F;
    if (strong) {
        gd::disc(app.renderer, p.x, p.y, radius + 1.5F, gd::kWhite);
        gd::disc(app.renderer, p.x, p.y, radius, body);
    } else {
        gd::disc(app.renderer, p.x, p.y, radius, gd::Color{body.r, body.g, body.b, 150});
    }
}

float smooth(float t) {
    const float c = std::clamp(t, 0.0F, 1.0F);
    return c * c * (3.0F - 2.0F * c);
}

/// The figure in every copy of the map that is on screen. While a step is on its way each copy slides one step, so a
/// copy that leaves the picture on one side comes in on the other.
void drawFigures(App& app) {
    const float t = app.sliding ? smooth(app.slideT / kSlide) : 1.0F;
    const hexwrap::Point dirUnit = hexwrap::center(hexwrap::direction(app.slideDir));
    const SDL_FPoint d{static_cast<float>(dirUnit.x) * app.size, static_cast<float>(dirUnit.y) * app.size};
    const float back = app.sliding ? 1.0F - t : 0.0F;
    for (const Tile& tile : app.tiles) {
        if (tile.canon == app.figure) drawPiece(app, {tile.px.x - d.x * back, tile.px.y - d.y * back}, tile.copy == 0);
    }
    if (!app.sliding) return;
    for (const Tile& tile : app.tiles) {
        // A copy whose next tile is not on screen has no tile to arrive on, it slides off the picture.
        if (tile.canon == app.slideFrom && indexOf(app, tile.h + hexwrap::direction(app.slideDir)) < 0) {
            drawPiece(app, {tile.px.x + d.x * t, tile.px.y + d.y * t}, tile.copy == 0);
        }
    }
}

void drawFlash(App& app) {
    if (!app.flashing) return;
    const int a = indexOf(app, app.flashFrom);
    const int b = indexOf(app, app.figure);
    if (a < 0 || b < 0) return;
    const SDL_FPoint p = app.tiles[static_cast<std::size_t>(a)].px;
    const SDL_FPoint q = app.tiles[static_cast<std::size_t>(b)].px;
    const auto alpha = static_cast<Uint8>(255.0F * (1.0F - app.flashT / kFlash));
    gd::setColor(app.renderer, gd::Color{234, 88, 12, alpha});
    for (int k = -1; k <= 1; ++k) SDL_RenderLine(app.renderer, p.x + static_cast<float>(k), p.y, q.x + static_cast<float>(k), q.y);
    gd::disc(app.renderer, p.x, p.y, app.size * 0.3F, gd::Color{234, 88, 12, alpha});
}

void drawMirrorCard(App& app) {
    SDL_Renderer* r = app.renderer;
    const float y = 44.0F;
    const float tx = kPanelX + 12.0F;
    gd::card(r, kPanelX, y, kPanelW, 120.0F, gd::kWhite, gd::kInk);
    char line[64];
    if (app.rule == Rule::Naive) {
        app.hud.text(r, tx, y + 8.0F, "NAIVE", gd::kOrange, gd::Hud::Style::Bold);
        app.hud.text(r, tx, y + 36.0F, "leave the map: appear on", gd::kInk);
        app.hud.text(r, tx, y + 58.0F, "the tile opposite the one", gd::kInk);
        app.hud.text(r, tx, y + 80.0F, "you left, (-q, -r, -s)", gd::kInk);
        return;
    }
    app.hud.text(r, tx, y + 8.0F, "MIRROR CENTERS", gd::kTealDark, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "%d tiles", hexwrap::tileCount(app.radius));
    app.hud.text(r, kPanelX + kPanelW - 12.0F - app.hud.width(line), y + 8.0F, line, gd::kMuted);
    app.hud.text(r, tx, y + 30.0F, "m0=(2N+1,-N,-N-1), turn 60°", gd::kMuted);
    for (int k = 0; k < 6; ++k) {
        const Cube m = hexwrap::mirrorCenter(app.radius, k);
        SDL_snprintf(line, sizeof line, "m%d %d,%d,%d", k, m.q, m.r, m.s);
        app.hud.text(r, tx + static_cast<float>(k / 3) * 140.0F, y + 54.0F + static_cast<float>(k % 3) * 22.0F, line, gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawStateCard(App& app) {
    SDL_Renderer* r = app.renderer;
    const float y = 174.0F;
    const float tx = kPanelX + 12.0F;
    gd::card(r, kPanelX, y, kPanelW, 104.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, tx, y + 8.0F, "FIGURE", gd::kTealDark, gd::Hud::Style::Bold);
    char line[64];
    SDL_snprintf(line, sizeof line, "heading %s", kDirNames[app.heading]);
    app.hud.text(r, kPanelX + kPanelW - 12.0F - app.hud.width(line), y + 8.0F, line, gd::kMuted);
    SDL_snprintf(line, sizeof line, "tile %d,%d,%d", app.figure.q, app.figure.r, app.figure.s);
    app.hud.text(r, tx, y + 32.0F, line, gd::kInk, gd::Hud::Style::Bold);
    const int total = hexwrap::tileCount(app.radius);
    SDL_snprintf(line, sizeof line, "visited %d of %d", app.visitedCount, total);
    app.hud.text(r, tx, y + 56.0F, line, app.visitedCount == total ? gd::kTealDark : gd::kInk, gd::Hud::Style::Bold);
    const int different = app.rule == Rule::Wrap ? hexwrap::distinctNeighbors(app.figure, [&](Cube c, int dir) { return app.table.step(c, dir); })
                                                 : hexwrap::distinctNeighbors(app.figure, [&](Cube c, int dir) { return hexwrap::naiveStep(c, dir, app.radius); });
    SDL_snprintf(line, sizeof line, "%d of 6 neighbors differ", different);
    app.hud.text(r, tx, y + 78.0F, line, different == hexwrap::kDirections ? gd::kTealDark : gd::kOrange, gd::Hud::Style::Bold);
}

std::string hoverLine(const App& app) {
    if (app.hover < 0) return app.rule == Rule::Naive ? "walk east: the row never changes" : "hover a tile: it lights up in every copy";
    const Tile& t = app.tiles[static_cast<std::size_t>(app.hover)];
    char line[96];
    if (t.copy <= 0) {
        SDL_snprintf(line, sizeof line, "tile %d,%d,%d is on the map", t.h.q, t.h.r, t.h.s);
    } else {
        const Cube m = hexwrap::mirrorCenter(app.radius, t.copy - 1);
        SDL_snprintf(line, sizeof line, "%d,%d,%d minus m%d (%d,%d,%d) = %d,%d,%d", t.h.q, t.h.r, t.h.s, t.copy - 1, m.q, m.r, m.s, t.canon.q, t.canon.r, t.canon.s);
    }
    return line;
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "radius %d, %s", app.radius, app.rule == Rule::Naive ? "naive jump" : app.view == View::Plane ? "plane" : "fringe");
    app.hud.titleBar(r, kWidth, "[ HEX WRAPAROUND ]", right);

    drawTiles(app);
    drawLabels(app);
    drawMirrors(app);
    drawHover(app);
    drawFigures(app);
    drawFlash(app);
    drawMirrorCard(app);
    drawStateCard(app);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const std::string label = chipLabel(c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 2.0F, label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }

    app.hud.text(r, 24.0F, 446.0F, "step: Q E / A D / Z C, Space walks the last heading", gd::kMuted);
    app.hud.text(r, 24.0F, 470.0F, "click a tile: put the figure there", gd::kMuted);
    app.hud.text(r, 24.0F, 494.0F, hoverLine(app), app.hover < 0 ? gd::kMuted : gd::kInk, gd::Hud::Style::Bold);
}

// ---- input ------------------------------------------------------------------------------------------------------

void handleKey(App& app, const SDL_KeyboardEvent& key) {
    // Steps may repeat when the key is held, everything else is a toggle and must not.
    switch (key.key) {
        case SDLK_Q: step(app, 2); return;
        case SDLK_E: step(app, 1); return;
        case SDLK_A: step(app, 3); return;
        case SDLK_D: step(app, 0); return;
        case SDLK_Z: step(app, 4); return;
        case SDLK_C: step(app, 5); return;
        default: break;
    }
    if (key.repeat) return;
    switch (key.key) {
        case SDLK_1: apply(app, Action::Radius1); break;
        case SDLK_2: apply(app, Action::Radius2); break;
        case SDLK_3: apply(app, Action::Radius3); break;
        case SDLK_4: apply(app, Action::Plane); break;
        case SDLK_5: apply(app, Action::Fringe); break;
        case SDLK_6: apply(app, Action::Wrap); break;
        case SDLK_7: apply(app, Action::Naive); break;
        case SDLK_SPACE: apply(app, Action::Walk); break;
        case SDLK_R: apply(app, Action::Reset); break;
        default: break;
    }
}

void updateHover(App& app) { app.hover = app.mouseSeen ? pickTile(app, app.mouse) : -1; }

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
    if (!SDL_CreateWindowAndRenderer("hex-wraparound", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    rebuild(*app);
    resetWalk(*app);
    layoutChips(*app);
    app->lastTicks = SDL_GetTicks();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {event->motion.x, event->motion.y};
            app.mouseSeen = true;
            updateHover(app);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            app.mouseSeen = true;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                    break;
                }
            }
            if (!onChip) {
                const int i = pickTile(app, p);
                if (i >= 0) teleport(app, app.tiles[static_cast<std::size_t>(i)].canon);
            }
            updateHover(app);
            break;
        }
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.mouseSeen = false;
            app.hover = -1;
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            handleKey(app, event->key);
            updateHover(app);  // a key can rebuild the picture under a mouse that stands still
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicks();
    const float dt = std::min(static_cast<float>(now - app.lastTicks) / 1000.0F, kMaxDt);
    app.lastTicks = now;

    if (app.sliding) {
        app.slideT += dt;
        if (app.slideT >= kSlide) app.sliding = false;
    }
    if (app.flashing) {
        app.flashT += dt;
        if (app.flashT >= kFlash) app.flashing = false;
    }
    if (app.walking) {
        app.walkTimer += dt;
        if (app.walkTimer >= kWalkInterval) step(app, app.heading);  // step() zeroes the timer, at most one step per frame
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
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
