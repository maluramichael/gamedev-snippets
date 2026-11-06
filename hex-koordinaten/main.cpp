// Hex coordinates with SDL3 main callbacks.
//
// One grid, four ways to number the same hexes. The cursor cell is shown as offset (what you would store in an array),
// axial and cube (what you do the math in) and doubled. The six orange neighbors and the distance heatmap come from
// the cube form, while moving with the keys uses the offset neighbor tables, so the two have to agree. The
// "Naive" chip swaps the distance for |dcol| + |drow| on the offset grid and the rings fall apart. The hex-to-pixel
// formulas and the coordinate math are in logic.hpp and covered by test.cpp. Finding the hex under the mouse is a
// plain nearest-center scan here, the O(1) formula comes in a later post.

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
constexpr float kHexSize = 30.0F;  // px from center to corner
constexpr hex::Board kBoard{9, 7};
constexpr SDL_FRect kField{16.0F, 38.0F, 604.0F, 394.0F};  // the board is centered in here
constexpr hex::Offset kDemoHover{7, 1};                    // measured until the mouse moves for the first time
constexpr float kPanelX = 640.0F;
constexpr float kPanelW = 300.0F;
constexpr int kHeatMax = 8;  // distances from here on all get the darkest color

enum class Label { Offset, Axial, Distance };
enum class Action { Pointy, Flat, ParityOdd, ParityEven, LabelOffset, LabelAxial, LabelDistance, Naive };

struct Chip {
    Action action;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    hex::Layout layout{hex::Orientation::Pointy, hex::Parity::Odd};
    std::vector<hex::Axial> cells;  // row by row, so cell i has offset (i % cols, i / cols)
    SDL_FPoint origin{0.0F, 0.0F};  // pixel position of hex (0, 0)
    hex::Offset cursor{4, 3};
    Label label = Label::Distance;
    bool naive = false;
    SDL_FPoint mouse{-1000.0F, -1000.0F};
    bool mouseSeen = false;
    bool dragging = false;
    long maxFrames = -1;
    long frames = 0;
};

std::size_t indexOf(hex::Offset o) { return static_cast<std::size_t>(o.row * kBoard.cols + o.col); }

hex::Offset offsetOf(std::size_t index) { return {static_cast<int>(index) % kBoard.cols, static_cast<int>(index) / kBoard.cols}; }

bool onBoard(hex::Offset o) { return o.col >= 0 && o.col < kBoard.cols && o.row >= 0 && o.row < kBoard.rows; }

SDL_FPoint pixelOf(const App& app, hex::Axial a) {
    const hex::Point p = hex::center(app.layout.orientation, a);
    return {app.origin.x + static_cast<float>(p.x) * kHexSize, app.origin.y + static_cast<float>(p.y) * kHexSize};
}

/// New cell list and a new origin that centers the board in the field. Call whenever the layout changes.
void rebuild(App& app) {
    app.cells = hex::boardCells(app.layout, kBoard);
    const hex::Bounds b = hex::boardBounds(app.layout, app.cells);
    const float midX = static_cast<float>((b.minX + b.maxX) / 2.0) * kHexSize;
    const float midY = static_cast<float>((b.minY + b.maxY) / 2.0) * kHexSize;
    app.origin = {kField.x + kField.w / 2.0F - midX, kField.y + kField.h / 2.0F - midY};
    app.dragging = false;
}

void setLayout(App& app, hex::Layout layout) {
    if (layout.orientation == app.layout.orientation && layout.parity == app.layout.parity) return;
    app.layout = layout;
    rebuild(app);
}

std::optional<std::size_t> cellAt(const App& app, SDL_FPoint p) {
    const hex::Point unit{static_cast<double>(p.x - app.origin.x) / static_cast<double>(kHexSize), static_cast<double>(p.y - app.origin.y) / static_cast<double>(kHexSize)};
    return hex::pickCell(app.layout.orientation, app.cells, unit);
}

std::optional<std::size_t> hovered(const App& app) {
    if (!app.mouseSeen) return indexOf(kDemoHover);
    return cellAt(app, app.mouse);
}

void moveCursor(App& app, hex::Screen s) {
    const int d = hex::directionOnScreen(app.layout.orientation, s);
    if (d < 0) return;
    const hex::Offset next = hex::neighborOffset(app.layout, app.cursor, d);  // the two-table way
    if (onBoard(next)) app.cursor = next;
}

std::string layoutName(const App& app) {
    const bool pointy = app.layout.orientation == hex::Orientation::Pointy;
    const char* parity = app.layout.parity == hex::Parity::Odd ? "odd" : "even";
    return std::string(parity) + (pointy ? "-r" : "-q");
}

void layoutChips(App& app) {
    const std::array<Action, 8> order{Action::Pointy, Action::Flat, Action::ParityOdd, Action::ParityEven,
                                      Action::LabelOffset, Action::LabelAxial, Action::LabelDistance, Action::Naive};
    constexpr float kChipW = 144.0F;
    constexpr float kChipH = 29.0F;
    for (std::size_t i = 0; i < order.size(); ++i) {
        const float x = kPanelX + static_cast<float>(i % 2) * (kChipW + 12.0F);
        const float y = 378.0F + static_cast<float>(i / 2) * 35.0F;
        app.chips.push_back({order[i], SDL_FRect{x, y, kChipW, kChipH}});
    }
}

std::string chipLabel(const App& app, Action a) {
    const bool pointy = app.layout.orientation == hex::Orientation::Pointy;
    switch (a) {
        case Action::Pointy: return "1 Pointy";
        case Action::Flat: return "2 Flat";
        case Action::ParityOdd: return pointy ? "3 odd-r" : "3 odd-q";
        case Action::ParityEven: return pointy ? "4 even-r" : "4 even-q";
        case Action::LabelOffset: return "5 Offset";
        case Action::LabelAxial: return "6 Axial";
        case Action::LabelDistance: return "7 Distance";
        case Action::Naive: return "8 Naive";
    }
    return {};
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::Pointy: return app.layout.orientation == hex::Orientation::Pointy;
        case Action::Flat: return app.layout.orientation == hex::Orientation::Flat;
        case Action::ParityOdd: return app.layout.parity == hex::Parity::Odd;
        case Action::ParityEven: return app.layout.parity == hex::Parity::Even;
        case Action::LabelOffset: return app.label == Label::Offset;
        case Action::LabelAxial: return app.label == Label::Axial;
        case Action::LabelDistance: return app.label == Label::Distance;
        case Action::Naive: return app.naive;
    }
    return false;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::Pointy: setLayout(app, {hex::Orientation::Pointy, app.layout.parity}); break;
        case Action::Flat: setLayout(app, {hex::Orientation::Flat, app.layout.parity}); break;
        case Action::ParityOdd: setLayout(app, {app.layout.orientation, hex::Parity::Odd}); break;
        case Action::ParityEven: setLayout(app, {app.layout.orientation, hex::Parity::Even}); break;
        case Action::LabelOffset: app.label = Label::Offset; break;
        case Action::LabelAxial: app.label = Label::Axial; break;
        case Action::LabelDistance: app.label = Label::Distance; break;
        case Action::Naive:
            app.naive = !app.naive;
            app.label = Label::Distance;  // the naive numbers only exist there
            break;
    }
}

gd::Color lerp(gd::Color a, gd::Color b, float t) {
    const auto mix = [t](Uint8 x, Uint8 y) { return static_cast<Uint8>(std::lround(static_cast<float>(x) + (static_cast<float>(y) - static_cast<float>(x)) * t)); };
    return {mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b), 255};
}

std::array<SDL_FPoint, 6> hexCorners(const App& app, SDL_FPoint c, float size) {
    std::array<SDL_FPoint, 6> out{};
    for (int i = 0; i < 6; ++i) {
        const hex::Point p = hex::corner(app.layout.orientation, i);
        out[static_cast<std::size_t>(i)] = {c.x + static_cast<float>(p.x) * size, c.y + static_cast<float>(p.y) * size};
    }
    return out;
}

void fillHex(SDL_Renderer* r, const App& app, SDL_FPoint c, float size, gd::Color color) {
    const SDL_FColor fc = gd::fcolor(color);
    std::array<SDL_Vertex, 7> v{};
    v[0] = {c, fc, {0, 0}};
    const auto corners = hexCorners(app, c, size);
    for (std::size_t i = 0; i < 6; ++i) v[i + 1] = {corners[i], fc, {0, 0}};
    constexpr std::array<int, 18> idx{0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 1};
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

void outlineHex(SDL_Renderer* r, const App& app, SDL_FPoint c, float size, gd::Color color) {
    const auto corners = hexCorners(app, c, size);
    gd::setColor(r, color);
    for (std::size_t i = 0; i < 6; ++i) {
        const SDL_FPoint a = corners[i];
        const SDL_FPoint b = corners[(i + 1) % 6];
        SDL_RenderLine(r, a.x, a.y, b.x, b.y);
    }
}

void centeredText(App& app, SDL_FPoint c, const std::string& s, gd::Color color) {
    app.hud.text(app.renderer, c.x - app.hud.width(s, gd::Hud::Style::Bold) / 2.0F, c.y - 11.0F, s, color, gd::Hud::Style::Bold);
}

void drawBoard(App& app, std::size_t cursorIndex, std::optional<std::size_t> hover) {
    SDL_Renderer* r = app.renderer;
    const hex::Cube cursorCube = hex::toCube(app.cells[cursorIndex]);
    const gd::Color rim{17, 24, 39, 120};
    std::vector<gd::Color> inks(app.cells.size());
    std::vector<int> shownDistance(app.cells.size());

    for (std::size_t i = 0; i < app.cells.size(); ++i) {
        const hex::Axial a = app.cells[i];
        const hex::Offset off = offsetOf(i);
        const SDL_FPoint c = pixelOf(app, a);
        const int trueDistance = hex::cubeDistance(cursorCube, hex::toCube(a));
        const int shown = app.naive ? hex::naiveOffsetDistance(app.cursor, off) : trueDistance;

        gd::Color fill = gd::kWhite;
        gd::Color ink = gd::kInk;
        if (app.label == Label::Distance) {
            fill = lerp(gd::kTealSoft, gd::kTealDark, static_cast<float>(std::min(shown, kHeatMax)) / static_cast<float>(kHeatMax));
            ink = shown * 2 > kHeatMax ? gd::kWhite : gd::kInk;
        }
        if (i == cursorIndex) {
            fill = gd::kTealDark;
            ink = gd::kWhite;
        } else if (trueDistance == 1) {
            fill = gd::kOrange;
            ink = gd::kWhite;
        }
        fillHex(r, app, c, kHexSize - 1.0F, fill);
        outlineHex(r, app, c, kHexSize - 1.0F, rim);
        inks[i] = ink;
        shownDistance[i] = shown;
    }

    if (hover && *hover != cursorIndex) {
        // The line stops short of both centers so it does not run through the labels.
        const SDL_FPoint from = pixelOf(app, app.cells[cursorIndex]);
        const SDL_FPoint to = pixelOf(app, app.cells[*hover]);
        const float length = std::hypot(to.x - from.x, to.y - from.y);
        const float ux = (to.x - from.x) / length;
        const float uy = (to.y - from.y) / length;
        constexpr float kGap = 16.0F;
        if (length > 2.0F * kGap) {
            gd::setColor(r, gd::Color{17, 24, 39, 120});
            for (float d = 0.0F; d <= 1.0F; d += 1.0F) {
                SDL_RenderLine(r, from.x + ux * kGap + d, from.y + uy * kGap, to.x - ux * kGap + d, to.y - uy * kGap);
            }
        }
    }
    for (std::size_t i = 0; i < app.cells.size(); ++i) {
        const hex::Axial a = app.cells[i];
        const hex::Offset off = offsetOf(i);
        char text[24];
        switch (app.label) {
            case Label::Offset: SDL_snprintf(text, sizeof text, "%d,%d", off.col, off.row); break;
            case Label::Axial: SDL_snprintf(text, sizeof text, "%d,%d", a.q, a.r); break;
            case Label::Distance: SDL_snprintf(text, sizeof text, "%d", shownDistance[i]); break;
        }
        centeredText(app, pixelOf(app, a), text, inks[i]);
    }
    if (hover) {
        const SDL_FPoint c = pixelOf(app, app.cells[*hover]);
        outlineHex(r, app, c, kHexSize - 1.0F, gd::kInk);
        outlineHex(r, app, c, kHexSize - 2.5F, gd::kInk);
        outlineHex(r, app, c, kHexSize - 4.0F, gd::kInk);
    }
}

void drawCursorCard(App& app, std::size_t cursorIndex) {
    SDL_Renderer* r = app.renderer;
    const hex::Axial a = app.cells[cursorIndex];
    const hex::Cube cube = hex::toCube(a);
    const hex::Offset off = offsetOf(cursorIndex);
    const hex::Doubled dbl = hex::toDoubled(app.layout.orientation, a);
    const float x = kPanelX;
    const float y = 44.0F;
    const float tx = x + 12.0F;
    const float vx = tx + 90.0F;
    gd::card(r, x, y, kPanelW, 168.0F, gd::kWhite, gd::kInk);

    app.hud.text(r, tx, y + 8.0F, "CURSOR", gd::kTealDark, gd::Hud::Style::Bold);
    const std::string conv = layoutName(app);
    app.hud.text(r, x + kPanelW - 12.0F - app.hud.width(conv), y + 8.0F, conv, gd::kMuted);

    char line[64];
    const auto row = [&](float dy, const char* name, const char* value) {
        app.hud.text(r, tx, y + dy, name, gd::kMuted);
        app.hud.text(r, vx, y + dy, value, gd::kInk, gd::Hud::Style::Bold);
    };
    SDL_snprintf(line, sizeof line, "col %d, row %d", off.col, off.row);
    row(36.0F, "offset", line);
    SDL_snprintf(line, sizeof line, "q %d, r %d", a.q, a.r);
    row(60.0F, "axial", line);
    SDL_snprintf(line, sizeof line, "q %d, r %d, s ", cube.q, cube.r);
    row(84.0F, "cube", line);
    char sText[16];
    SDL_snprintf(sText, sizeof sText, "%d", cube.s);
    app.hud.text(r, vx + app.hud.width(line, gd::Hud::Style::Bold), y + 84.0F, sText, gd::kOrange, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "col %d, row %d", dbl.col, dbl.row);
    row(108.0F, "doubled", line);

    SDL_snprintf(line, sizeof line, "s = -(%d) - (%d) = %d", cube.q, cube.r, cube.s);
    app.hud.text(r, tx, y + 138.0F, line, gd::kOrange, gd::Hud::Style::Bold);
}

void drawHoverCard(App& app, std::size_t cursorIndex, std::optional<std::size_t> hover) {
    SDL_Renderer* r = app.renderer;
    const float x = kPanelX;
    const float y = 226.0F;
    const float tx = x + 12.0F;
    gd::card(r, x, y, kPanelW, 138.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, tx, y + 8.0F, "HOVER", gd::kTealDark, gd::Hud::Style::Bold);
    if (!hover) {
        app.hud.text(r, tx, y + 36.0F, "move the mouse over a cell", gd::kMuted);
        return;
    }
    const hex::Axial from = app.cells[cursorIndex];
    const hex::Axial to = app.cells[*hover];
    const hex::Cube cf = hex::toCube(from);
    const hex::Cube ct = hex::toCube(to);
    const int dq = ct.q - cf.q;
    const int dr = ct.r - cf.r;
    const int ds = ct.s - cf.s;
    const int distance = hex::cubeDistance(cf, ct);
    const int naiveCols = std::abs(offsetOf(*hover).col - app.cursor.col);
    const int naiveRows = std::abs(offsetOf(*hover).row - app.cursor.row);
    const int naive = naiveCols + naiveRows;

    char line[64];
    SDL_snprintf(line, sizeof line, "cell  q %d, r %d", to.q, to.r);
    app.hud.text(r, tx, y + 36.0F, line, gd::kInk, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "diff  dq %d  dr %d  ds %d", dq, dr, ds);
    app.hud.text(r, tx, y + 60.0F, line, gd::kMuted);
    SDL_snprintf(line, sizeof line, "distance max(%d,%d,%d) = %d", std::abs(dq), std::abs(dr), std::abs(ds), distance);
    app.hud.text(r, tx, y + 84.0F, line, gd::kTealDark, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "naive %d + %d = %d%s", naiveCols, naiveRows, naive, naive == distance ? " (same)" : " (wrong)");
    app.hud.text(r, tx, y + 108.0F, line, naive == distance ? gd::kMuted : gd::kOrange, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const std::size_t cursorIndex = indexOf(app.cursor);
    const auto hover = hovered(app);
    const bool pointy = app.layout.orientation == hex::Orientation::Pointy;

    gd::backdrop(r, kWidth, kHeight);
    const std::string right = std::string(pointy ? "pointy" : "flat") + "-top, " + layoutName(app);
    app.hud.titleBar(r, kWidth, "[ HEX COORDINATES ]", right);

    drawBoard(app, cursorIndex, hover);
    drawCursorCard(app, cursorIndex);
    drawHoverCard(app, cursorIndex, hover);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const std::string label = chipLabel(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 3.0F, label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }

    app.hud.text(r, 24.0F, 442.0F, pointy ? "move: Q E / A D / Z C or arrows (Shift: other side)" : "move: Q W E / Z X C or arrows (Shift: other side)", gd::kMuted);
    app.hud.text(r, 24.0F, 466.0F, "click or drag: place the cursor, orange = its six neighbors", gd::kMuted);
    app.hud.text(r, 24.0F, 490.0F, "hover: measure the distance to the cursor", gd::kMuted);
}

void handleKey(App& app, const SDL_KeyboardEvent& key) {
    const bool shift = (key.mod & SDL_KMOD_SHIFT) != 0;
    const hex::Orientation o = app.layout.orientation;
    // Movement may repeat when the key is held, everything else is a toggle and must not.
    switch (key.key) {
        case SDLK_Q: moveCursor(app, hex::Screen::UpLeft); return;
        case SDLK_W: moveCursor(app, hex::Screen::Up); return;
        case SDLK_E: moveCursor(app, hex::Screen::UpRight); return;
        case SDLK_A: moveCursor(app, hex::Screen::Left); return;
        case SDLK_D: moveCursor(app, hex::Screen::Right); return;
        case SDLK_Z: moveCursor(app, hex::Screen::DownLeft); return;
        case SDLK_X: moveCursor(app, hex::Screen::Down); return;
        case SDLK_C: moveCursor(app, hex::Screen::DownRight); return;
        case SDLK_LEFT: moveCursor(app, hex::arrowToScreen(o, hex::Arrow::Left, shift)); return;
        case SDLK_RIGHT: moveCursor(app, hex::arrowToScreen(o, hex::Arrow::Right, shift)); return;
        case SDLK_UP: moveCursor(app, hex::arrowToScreen(o, hex::Arrow::Up, shift)); return;
        case SDLK_DOWN: moveCursor(app, hex::arrowToScreen(o, hex::Arrow::Down, shift)); return;
        default: break;
    }
    if (key.repeat) return;
    switch (key.key) {
        case SDLK_1: apply(app, Action::Pointy); break;
        case SDLK_2: apply(app, Action::Flat); break;
        case SDLK_3: apply(app, Action::ParityOdd); break;
        case SDLK_4: apply(app, Action::ParityEven); break;
        case SDLK_5: apply(app, Action::LabelOffset); break;
        case SDLK_6: apply(app, Action::LabelAxial); break;
        case SDLK_7: apply(app, Action::LabelDistance); break;
        case SDLK_8: apply(app, Action::Naive); break;
        default: break;
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
    if (!SDL_CreateWindowAndRenderer("hex-koordinaten", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    layoutChips(*app);
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
            if (app.dragging) {
                if (const auto cell = cellAt(app, app.mouse)) app.cursor = offsetOf(*cell);
            }
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
                }
            }
            if (onChip) break;
            if (const auto cell = cellAt(app, p)) {
                app.cursor = offsetOf(*cell);
                app.dragging = true;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.dragging = false;
            app.mouse = {-1000.0F, -1000.0F};
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging = false; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            handleKey(app, event->key);
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
