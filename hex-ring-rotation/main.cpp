// Hex rings, spirals, rotation and mirroring with SDL3 main callbacks.
//
// Ring and spiral modes walk a ring (or a spiral of rings) cell by cell around the origin, with the walking direction,
// the cell count and its formula 1 + 3N(N+1) next to the board. The Rotate mode turns a small shape around a pivot
// in 60 degree steps and mirrors it, and prints the coordinate swap that does it. "Naive" turns it the way you would
// in a square grid (q and r as x and y, sine and cosine, round): the shape tears apart. The geometry lives in
// logic.hpp and is covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldTop = 34.0F;
constexpr float kFieldBottom = 440.0F;
constexpr float kBoardLimitX = 436.0F;  // the board lives left of the info panels
constexpr double kSize = 19.0;          // corner radius of a hex in pixels
constexpr double kOriginX = 232.0;
constexpr double kOriginY = 238.0;
constexpr int kBoardRadius = 5;
constexpr double kCellsPerSecond = 14.0;
constexpr double kHoldCells = 20.0;  // pause after the last cell, counted in cells
constexpr float kPanelX = 444.0F;
constexpr float kPanelW = 492.0F;
constexpr float kChipH = 30.0F;

enum class Mode { Ring, Spiral, Rotate };
enum class Action { ModeRing, ModeSpiral, ModeRotate, RadiusDown, RadiusUp, Replay, RotLeft, RotRight, MirQ, MirR, MirS, Naive, Reset };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct LastOp {
    enum class Kind { None, Rotate, Mirror } kind = Kind::None;
    int steps = 0;  // +1 left, -1 right
    hrr::Axis axis = hrr::Axis::Q;
    bool naive = false;
    hrr::Hex before{0, 0};  // tip relative to the pivot
    hrr::Hex after{0, 0};
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    Mode mode = Mode::Spiral;
    int radius = 3;
    double shown = 14.0;  // cells revealed so far, so the first frame is not empty
    Uint64 lastTicks = 0;
    std::vector<hrr::Hex> cells;
    std::vector<hrr::Hex> shape = hrr::kShape;
    hrr::Hex pivot{0, 0};
    bool naive = false;
    LastOp last;
    hrr::Point mouse{-100.0, -100.0};
    bool mouseInside = false;
    long maxFrames = -1;
    long frames = 0;
};

// --- Helpers ------------------------------------------------------------------------------------------------------

struct Px {
    float x, y;
};

Px pixelOf(hrr::Hex h) {
    const hrr::Point p = hrr::toPixel(h, kSize);
    return {static_cast<float>(kOriginX + p.x), static_cast<float>(kOriginY + p.y)};
}

bool onScreen(Px p) { return p.x > 20.0F && p.x < kBoardLimitX - 14.0F && p.y > kFieldTop + 20.0F && p.y < kFieldBottom - 6.0F; }

void fillHex(SDL_Renderer* r, float cx, float cy, float s, gd::Color c) {
    const SDL_FColor col = gd::fcolor(c);
    SDL_Vertex v[7];
    v[0] = {{cx, cy}, col, {0, 0}};
    for (int i = 0; i < 6; ++i) {
        const float a = 1.04719755F * static_cast<float>(i);
        v[i + 1] = {{cx + std::cos(a) * s, cy + std::sin(a) * s}, col, {0, 0}};
    }
    constexpr int idx[18] = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 1};
    SDL_RenderGeometry(r, nullptr, v, 7, idx, 18);
}

void outlineHex(SDL_Renderer* r, float cx, float cy, float s, gd::Color c) {
    gd::setColor(r, c);
    for (int i = 0; i < 6; ++i) {
        const float a = 1.04719755F * static_cast<float>(i);
        const float b = 1.04719755F * static_cast<float>(i + 1);
        SDL_RenderLine(r, cx + std::cos(a) * s, cy + std::sin(a) * s, cx + std::cos(b) * s, cy + std::sin(b) * s);
    }
}

std::string cubeText(hrr::Hex h) {
    char buf[48];
    SDL_snprintf(buf, sizeof buf, "(%d, %d, %d)", h.q, h.r, h.s());
    return buf;
}

void refresh(App& app) {
    app.cells = app.mode == Mode::Ring ? hrr::ring({0, 0}, app.radius) : hrr::spiral({0, 0}, app.radius);
    app.shown = 0.0;
}

bool enabled(const App& app, Action a) {
    switch (a) {
        case Action::ModeRing:
        case Action::ModeSpiral:
        case Action::ModeRotate: return true;
        case Action::RadiusDown:
        case Action::RadiusUp:
        case Action::Replay: return app.mode != Mode::Rotate;
        default: return app.mode == Mode::Rotate;
    }
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::ModeRing: return app.mode == Mode::Ring;
        case Action::ModeSpiral: return app.mode == Mode::Spiral;
        case Action::ModeRotate: return app.mode == Mode::Rotate;
        case Action::Naive: return app.naive;
        default: return false;
    }
}

void resetShape(App& app) {
    app.shape = hrr::kShape;
    app.pivot = {0, 0};
    app.last = LastOp{};
}

void rotateShape(App& app, int steps) {
    LastOp op;
    op.kind = LastOp::Kind::Rotate;
    op.steps = steps;
    op.naive = app.naive;
    op.before = app.shape.back() - app.pivot;
    for (hrr::Hex& h : app.shape) {
        h = app.naive ? app.pivot + hrr::naiveRotate(h - app.pivot, steps) : hrr::rotateAround(h, app.pivot, steps);
    }
    op.after = app.shape.back() - app.pivot;
    app.last = op;
}

void mirrorShape(App& app, hrr::Axis axis) {
    LastOp op;
    op.kind = LastOp::Kind::Mirror;
    op.axis = axis;
    op.before = app.shape.back() - app.pivot;
    for (hrr::Hex& h : app.shape) h = hrr::mirrorAround(h, app.pivot, axis);
    op.after = app.shape.back() - app.pivot;
    app.last = op;
}

void setMode(App& app, Mode m) {
    app.mode = m;
    if (m != Mode::Rotate) refresh(app);
}

void setRadius(App& app, int radius) {
    app.radius = std::clamp(radius, 0, kBoardRadius);
    refresh(app);
}

void apply(App& app, Action a) {
    if (!enabled(app, a)) return;
    switch (a) {
        case Action::ModeRing: setMode(app, Mode::Ring); break;
        case Action::ModeSpiral: setMode(app, Mode::Spiral); break;
        case Action::ModeRotate: setMode(app, Mode::Rotate); break;
        case Action::RadiusDown: setRadius(app, app.radius - 1); break;
        case Action::RadiusUp: setRadius(app, app.radius + 1); break;
        case Action::Replay: refresh(app); break;
        case Action::RotLeft: rotateShape(app, 1); break;
        case Action::RotRight: rotateShape(app, -1); break;
        case Action::MirQ: mirrorShape(app, hrr::Axis::Q); break;
        case Action::MirR: mirrorShape(app, hrr::Axis::R); break;
        case Action::MirS: mirrorShape(app, hrr::Axis::S); break;
        case Action::Naive:
            app.naive = !app.naive;
            resetShape(app);  // both ways start from the same shape, so the same keys can be compared
            break;
        case Action::Reset: resetShape(app); break;
    }
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float y;
    };
    constexpr float rowA = 468.0F;
    constexpr float rowB = 504.0F;
    const std::array<Def, 13> defs{{{Action::ModeRing, "1 Ring", 0.0F, rowA},
                                    {Action::ModeSpiral, "2 Spiral", 8.0F, rowA},
                                    {Action::ModeRotate, "3 Rotate", 8.0F, rowA},
                                    {Action::RadiusDown, "- Radius", 24.0F, rowA},
                                    {Action::RadiusUp, "+ Radius", 8.0F, rowA},
                                    {Action::Replay, "R Replay", 24.0F, rowA},
                                    {Action::RotLeft, "Q Left 60°", 0.0F, rowB},
                                    {Action::RotRight, "E Right 60°", 8.0F, rowB},
                                    {Action::MirQ, "4 Mirror q", 24.0F, rowB},
                                    {Action::MirR, "5 Mirror r", 8.0F, rowB},
                                    {Action::MirS, "6 Mirror s", 8.0F, rowB},
                                    {Action::Naive, "N Naive", 24.0F, rowB},
                                    {Action::Reset, "0 Reset", 8.0F, rowB}}};
    float x = 40.0F;
    float lastY = 0.0F;
    for (const Def& d : defs) {
        if (d.y != lastY) x = 40.0F;
        lastY = d.y;
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 20.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, kChipH}});
        x += w;
    }
}

hrr::Hex hexAt(hrr::Point p) { return hrr::fromPixel(p.x - kOriginX, p.y - kOriginY, kSize); }

bool inBoard(const hrr::Point& p) {
    return p.x < static_cast<double>(kBoardLimitX) && p.y >= static_cast<double>(kFieldTop) && p.y <= static_cast<double>(kFieldBottom);
}

// --- Drawing ------------------------------------------------------------------------------------------------------

void drawLabel(App& app, Px p, const std::string& text, gd::Color c) {
    app.hud.text(app.renderer, p.x - app.hud.width(text) / 2.0F, p.y - 11.0F, text, c);
}

int visibleCount(const App& app) { return std::min(static_cast<int>(app.cells.size()), static_cast<int>(std::floor(app.shown)) + 1); }

/// Which of the six walking directions the newest revealed cell is on.
int headSide(const App& app) {
    const int count = visibleCount(app);
    if (count <= 0 || app.mode == Mode::Rotate) return -1;
    const int index = count - 1;
    if (app.mode == Mode::Ring) return app.radius > 0 ? hrr::ringSide(index, app.radius) : -1;
    const int k = hrr::distance(app.cells[static_cast<std::size_t>(index)], {0, 0});
    if (k == 0) return -1;
    return hrr::ringSide(index - hrr::spiralCount(k - 1), k);
}

void drawBoard(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const hrr::Hex& h : hrr::spiral({0, 0}, kBoardRadius)) {
        const Px p = pixelOf(h);
        fillHex(r, p.x, p.y, static_cast<float>(kSize) - 1.5F, gd::Color{255, 255, 255, 190});
        outlineHex(r, p.x, p.y, static_cast<float>(kSize) - 1.5F, gd::Color{156, 163, 175});
    }
    const Px o = pixelOf({0, 0});
    gd::fillRect(r, o.x - 2.0F, o.y - 2.0F, 4.0F, 4.0F, gd::kMuted);
}

void drawWalk(App& app) {
    SDL_Renderer* r = app.renderer;
    const int count = visibleCount(app);
    for (int i = 0; i < count; ++i) {
        const hrr::Hex h = app.cells[static_cast<std::size_t>(i)];
        const Px p = pixelOf(h);
        const bool head = i == count - 1;
        bool dark = false;
        if (app.mode == Mode::Ring) {
            dark = app.radius > 0 && hrr::ringSide(i, app.radius) % 2 == 0;
        } else {
            dark = hrr::distance(h, {0, 0}) % 2 == 1;
        }
        const gd::Color fill = head ? gd::kOrange : dark ? gd::kTeal : gd::kTealSoft;
        fillHex(r, p.x, p.y, static_cast<float>(kSize) - 1.5F, fill);
        outlineHex(r, p.x, p.y, static_cast<float>(kSize) - 1.5F, gd::kInk);
        char num[8];
        SDL_snprintf(num, sizeof num, "%d", i + 1);
        drawLabel(app, p, num, head || dark ? gd::kWhite : gd::kInk);
    }
}

void drawTurnScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const hrr::Hex tip = app.shape.back();
    const int d = hrr::distance(tip, app.pivot);

    // The ring the tip stays on when it is turned correctly.
    for (const hrr::Hex& h : hrr::ring(app.pivot, d)) {
        const Px p = pixelOf(h);
        if (onScreen(p)) fillHex(r, p.x, p.y, static_cast<float>(kSize) - 1.5F, gd::Color{13, 148, 136, 45});
    }
    // The shape as it was.
    for (const hrr::Hex& h : hrr::kShape) {
        const Px p = pixelOf(h);
        if (!onScreen(p)) continue;
        fillHex(r, p.x, p.y, static_cast<float>(kSize) - 1.5F, gd::Color{17, 24, 39, 30});
        outlineHex(r, p.x, p.y, static_cast<float>(kSize) - 1.5F, gd::kMuted);
    }
    // The shape now.
    const bool intact = hrr::sameShape(hrr::kShape, app.shape);
    const gd::Color body = intact ? gd::kTeal : gd::kOrange;
    for (std::size_t i = 0; i < app.shape.size(); ++i) {
        const Px p = pixelOf(app.shape[i]);
        if (!onScreen(p)) continue;
        fillHex(r, p.x, p.y, static_cast<float>(kSize) - 3.0F, i + 1 == app.shape.size() ? (intact ? gd::kTealDark : gd::Color{194, 65, 12}) : body);
        outlineHex(r, p.x, p.y, static_cast<float>(kSize) - 3.0F, gd::kInk);
    }
    // The six places the tip visits when it is turned around the pivot.
    for (int k = 0; k < 6; ++k) {
        const Px p = pixelOf(hrr::rotateAround(tip, app.pivot, k));
        if (!onScreen(p)) continue;
        gd::disc(r, p.x, p.y, 3.5F, gd::kInk);
    }
    const Px tp = pixelOf(tip);
    if (onScreen(tp)) gd::ring(r, tp.x, tp.y, 7.0F, gd::kWhite);
    // The pivot.
    const Px pp = pixelOf(app.pivot);
    gd::disc(r, pp.x, pp.y, 6.0F, gd::kOrange);
    gd::ring(r, pp.x, pp.y, 6.0F, gd::kInk);
    gd::ring(r, pp.x, pp.y, 7.0F, gd::kInk);
}

void drawKeyValue(App& app, float x, float y, const std::string& label, const std::string& value, gd::Color c = gd::kInk) {
    app.hud.text(app.renderer, x, y, label, gd::kMuted);
    app.hud.text(app.renderer, x + app.hud.width(label) + 10.0F, y, value, c, gd::Hud::Style::Bold);
}

void drawCountPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 44.0F, kPanelW, 128.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, 52.0F, "RADIUS", gd::kMuted);
    char big[8];
    SDL_snprintf(big, sizeof big, "%d", app.radius);
    app.hud.text(r, kPanelX + 14.0F, 80.0F, big, gd::kOrange, gd::Hud::Style::Large);

    const int n = app.radius;
    const int total = static_cast<int>(app.cells.size());
    char line[96];
    const float x = kPanelX + 88.0F;
    if (app.mode == Mode::Ring) {
        if (n == 0) {
            SDL_snprintf(line, sizeof line, "ring 0 is the center: 1 cell");
        } else {
            SDL_snprintf(line, sizeof line, "6 × N = 6 × %d = %d cells", n, 6 * n);
        }
        app.hud.text(r, x, 54.0F, line, gd::kInk, gd::Hud::Style::Bold);
        SDL_snprintf(line, sizeof line, "6 sides × %d cells", n);
        app.hud.text(r, x, 82.0F, n > 0 ? line : "no sides, nothing to walk", gd::kMuted);
    } else {
        SDL_snprintf(line, sizeof line, "1 + 3 × N × (N+1) = 1 + 3 × %d × %d = %d", n, n + 1, hrr::spiralCount(n));
        app.hud.text(r, x, 54.0F, line, gd::kInk, gd::Hud::Style::Bold);
        std::string sum = "1";
        for (int k = 1; k <= n; ++k) sum += " + " + std::to_string(6 * k);
        sum += " = " + std::to_string(hrr::spiralCount(n));
        app.hud.text(r, x, 82.0F, sum, gd::kMuted);
    }
    SDL_snprintf(line, sizeof line, "%d / %d", visibleCount(app), total);
    drawKeyValue(app, x, 110.0F, "placed", line, gd::kOrange);
}

void drawDirectionsPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    static const char* const kNames[6] = {"lower right", "upper right", "up", "upper left", "lower left", "down"};
    gd::card(r, kPanelX, 184.0F, kPanelW, 256.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, 192.0F, "SIX DIRECTIONS (q, r, s), CURRENT SIDE MARKED", gd::kMuted);
    const int side = headSide(app);
    for (int i = 0; i < 6; ++i) {
        const float y = 222.0F + 26.0F * static_cast<float>(i);
        const hrr::Hex d = hrr::kDirections[static_cast<std::size_t>(i)];
        if (i == side) gd::fillRect(r, kPanelX + 6.0F, y - 2.0F, kPanelW - 12.0F, 26.0F, gd::Color{234, 88, 12, 45});
        if (app.mode == Mode::Ring) {
            gd::fillRect(r, kPanelX + 14.0F, y + 3.0F, 14.0F, 14.0F, i % 2 == 0 ? gd::kTeal : gd::kTealSoft);
            gd::strokeRect(r, kPanelX + 14.0F, y + 3.0F, 14.0F, 14.0F, gd::kInk);
        }
        char line[96];
        SDL_snprintf(line, sizeof line, "dir %d   (%2d, %2d, %2d)   %s", i, d.q, d.r, d.s(), kNames[i]);
        app.hud.text(r, kPanelX + 40.0F, y, line, i == side ? gd::kOrange : gd::kInk, i == side ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
    }
    const hrr::Hex start = hrr::kDirections[4] * app.radius;
    char line[96];
    SDL_snprintf(line, sizeof line, "ring start = center + dir 4 × %d = (%d, %d)", app.radius, start.q, start.r);
    app.hud.text(r, kPanelX + 14.0F, 384.0F, line, gd::kMuted);
    app.hud.text(r, kPanelX + 14.0F, 408.0F, "each side: step in its direction N times", gd::kMuted);
}

void drawOperationPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 44.0F, kPanelW, 128.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, 52.0F, "LAST OPERATION", gd::kMuted);
    const bool intact = hrr::sameShape(hrr::kShape, app.shape);
    const float x = kPanelX + 14.0F;
    if (app.last.kind == LastOp::Kind::None) {
        app.hud.text(r, x, 78.0F, "none yet", gd::kInk, gd::Hud::Style::Bold);
        app.hud.text(r, x, 104.0F, "Q / E turn, 4 5 6 mirror, click sets the pivot", gd::kMuted);
    } else {
        const LastOp& op = app.last;
        std::string title;
        std::string rule;
        if (op.kind == LastOp::Kind::Rotate) {
            title = op.steps > 0 ? "turn left 60°" : "turn right 60°";
            rule = op.naive ? "naive: q, r as x, y, sin / cos, round" : "(q, r, s) -> " + hrr::rotationFormula(op.steps);
        } else {
            const char* names[3] = {"q", "r", "s"};
            title = std::string("mirror ") + names[static_cast<int>(op.axis)];
            rule = "(q, r, s) -> " + hrr::mirrorFormula(op.axis);
        }
        app.hud.text(r, x, 76.0F, title, gd::kInk, gd::Hud::Style::Bold);
        app.hud.text(r, x, 100.0F, rule, op.naive ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);
        app.hud.text(r, x, 124.0F, "tip: " + cubeText(op.before) + " -> " + cubeText(op.after), gd::kInk);
    }
    const std::string verdict = intact ? "shape intact: yes" : "shape intact: NO";
    app.hud.text(r, kPanelX + kPanelW - app.hud.width(verdict, gd::Hud::Style::Bold) - 14.0F, 52.0F, verdict, intact ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
}

void drawTurnTable(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 184.0F, kPanelW, 256.0F, gd::kWhite, gd::kInk);
    const hrr::Hex rel = app.shape.back() - app.pivot;
    app.hud.text(r, kPanelX + 14.0F, 192.0F, "ALL TURNS OF THE TIP AROUND THE PIVOT", gd::kMuted);
    char line[96];
    SDL_snprintf(line, sizeof line, "tip - pivot = %s  distance %d", cubeText(rel).c_str(), hrr::distance(app.shape.back(), app.pivot));
    app.hud.text(r, kPanelX + 14.0F, 216.0F, line, gd::kInk, gd::Hud::Style::Bold);
    for (int k = 0; k < 6; ++k) {
        const float y = 242.0F + 20.0F * static_cast<float>(k);
        SDL_snprintf(line, sizeof line, "%3d°  %-14s -> %s", 60 * k, hrr::rotationFormula(k).c_str(), cubeText(hrr::rotateSteps(rel, k)).c_str());
        app.hud.text(r, kPanelX + 14.0F, y, line, gd::kInk);
    }
    const char* names[3] = {"q", "r", "s"};
    for (int a = 0; a < 3; ++a) {
        const hrr::Axis axis = static_cast<hrr::Axis>(a);
        const float y = 372.0F + 20.0F * static_cast<float>(a);
        SDL_snprintf(line, sizeof line, "mir %s %-14s -> %s", names[a], hrr::mirrorFormula(axis).c_str(), cubeText(hrr::mirror(rel, axis)).c_str());
        app.hud.text(r, kPanelX + 14.0F, y, line, gd::kTealDark);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    const char* right = app.mode == Mode::Ring ? "ring" : app.mode == Mode::Spiral ? "spiral" : "turn and mirror";
    app.hud.titleBar(r, kWidth, "[ HEX RINGS, SPIRALS, ROTATION ]", right);

    drawBoard(app);
    if (app.mode == Mode::Rotate) {
        drawTurnScene(app);
    } else {
        drawWalk(app);
    }

    // The hex under the cursor.
    hrr::Hex hovered{0, 0};
    const bool hoverOk = app.mouseInside && inBoard(app.mouse);
    if (hoverOk) {
        hovered = hexAt(app.mouse);
        const Px p = pixelOf(hovered);
        if (onScreen(p)) {
            outlineHex(r, p.x, p.y, static_cast<float>(kSize) - 1.5F, gd::kOrange);
            outlineHex(r, p.x, p.y, static_cast<float>(kSize) - 2.5F, gd::kOrange);
        }
    }

    if (app.mode == Mode::Rotate) {
        drawOperationPanel(app);
        drawTurnTable(app);
        app.hud.text(r, 40.0F, 442.0F, "click: pivot   ringed: tip   gray: start", gd::kMuted);
    } else {
        drawCountPanel(app);
        drawDirectionsPanel(app);
        app.hud.text(r, 40.0F, 442.0F, "cells are numbered in walking order", gd::kMuted);
    }
    if (hoverOk) {
        char line[64];
        SDL_snprintf(line, sizeof line, "hover %s  dist %d", cubeText(hovered).c_str(), hrr::distance(hovered, {0, 0}));
        app.hud.text(r, 640.0F, 442.0F, line, gd::kInk);
    }

    for (const Chip& c : app.chips) {
        const bool ok = enabled(app, c.action);
        const bool on = active(app, c.action);
        const gd::Color fill = !ok ? gd::kPaper : on ? (c.action == Action::Naive ? gd::kOrange : gd::kTeal) : gd::kWhite;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, ok ? gd::kInk : gd::kMuted);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, !ok ? gd::kMuted : on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void step(App& app) {
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(0.25, static_cast<double>(now - app.lastTicks) / 1.0e9);
    app.lastTicks = now;
    if (app.mode == Mode::Rotate) return;
    app.shown += dt * kCellsPerSecond;
    if (app.shown >= static_cast<double>(app.cells.size()) + kHoldCells) app.shown = 0.0;
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
    if (!SDL_CreateWindowAndRenderer("hex-ring-rotation", kWidth, kHeight, 0, &app->window, &app->renderer)) {
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
    app->cells = hrr::spiral({0, 0}, app->radius);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            app.mouseInside = true;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE: app.mouseInside = false; break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = {static_cast<double>(p.x), static_cast<double>(p.y)};
            app.mouseInside = true;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                    break;
                }
            }
            if (!onChip && app.mode == Mode::Rotate && inBoard(app.mouse)) {
                const hrr::Hex h = hexAt(app.mouse);
                if (hrr::distance(h, {0, 0}) <= kBoardRadius) app.pivot = h;
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::ModeRing); break;
                case SDLK_2: apply(app, Action::ModeSpiral); break;
                case SDLK_3: apply(app, Action::ModeRotate); break;
                case SDLK_MINUS:
                case SDLK_KP_MINUS:
                case SDLK_DOWN: apply(app, Action::RadiusDown); break;
                case SDLK_PLUS:
                case SDLK_EQUALS:
                case SDLK_KP_PLUS:
                case SDLK_UP: apply(app, Action::RadiusUp); break;
                case SDLK_R: apply(app, Action::Replay); break;
                case SDLK_Q: apply(app, Action::RotLeft); break;
                case SDLK_E: apply(app, Action::RotRight); break;
                case SDLK_4: apply(app, Action::MirQ); break;
                case SDLK_5: apply(app, Action::MirR); break;
                case SDLK_6: apply(app, Action::MirS); break;
                case SDLK_N: apply(app, Action::Naive); break;
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
    step(app);
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
