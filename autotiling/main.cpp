// Autotiling with SDL3 main callbacks.
//
// Paint walls and grass with the mouse: every cell looks at its eight neighbours and picks its own tile from a bit
// mask. Four modes show the same map three ways. 4-bit only looks at N E S W and cannot draw an inner corner. 8-bit
// raw looks at all eight neighbours and needs 256 tiles. 8-bit with the corner rule ignores a diagonal neighbour unless
// both of its edge neighbours are set, which makes the picture identical but needs only 47 tiles. The dual grid puts
// the tiles on the cell corners and gets by with 16. All tiles are drawn from rectangles (logic.hpp), there is no
// image file; the atlas on the right shows every tile of the mode and lights up the ones the map uses.

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
constexpr int kTile = 24;
constexpr int kCols = 24;
constexpr int kRows = 16;
constexpr float kFieldX = 24.0F;
constexpr float kFieldY = 44.0F;
constexpr float kPanelX = 624.0F;
constexpr float kPanelW = 312.0F;
constexpr float kChipH = 34.0F;
constexpr float kChipRow1 = 442.0F;
constexpr float kChipRow2 = 484.0F;

constexpr gd::Color kWallBody = gd::kTealSoft;
constexpr gd::Color kWallEdge = gd::kTealDark;
constexpr gd::Color kGrassBody{190, 242, 100};
constexpr gd::Color kGrassEdge{77, 124, 15};
constexpr gd::Color kUnusedBody{236, 235, 233};
constexpr gd::Color kUnusedEdge{168, 162, 158};

enum class Action { Mode4, ModeRaw, Mode47, ModeDual, ToolWall, ToolGrass, ToolErase, TileGrid, Clear, Reset };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Stroke {
    bool active = false;
    std::uint8_t material = 0;
    at::Cell last{0, 0};
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    at::Grid grid{kCols, kRows};
    at::Mode mode = at::Mode::EightBit47;
    std::uint8_t tool = 1;  // 1 wall, 2 grass, 0 erase
    bool tileGrid = true;
    Stroke stroke;
    float mouseX = 0.0F;
    float mouseY = 0.0F;
    bool mouseIn = false;
    long maxFrames = -1;
    long frames = 0;
};

at::Grid presetMap() {
    return at::parseMap({
        "........................",
        ".########...............",
        ".########.......#.......",
        ".##....##......###......",
        ".##.##.##.......#..gggg.",
        ".##.##.##.........ggggg.",
        ".##....##..#.#....gg.gg.",
        ".########...#.....ggggg.",
        ".########..#.#....gggg..",
        "..........#.#...........",
        "..###.....#.#..##.......",
        "..###..........##.......",
        "..#.#.....####..........",
        "..###.....####...g......",
        "........................",
        "........................",
    });
}

at::Cell cellAt(float x, float y) {
    return {static_cast<int>(std::floor((x - kFieldX) / static_cast<float>(kTile))), static_cast<int>(std::floor((y - kFieldY) / static_cast<float>(kTile)))};
}

at::Cell vertexAt(float x, float y) {
    const float half = static_cast<float>(kTile) / 2.0F;
    return {static_cast<int>(std::floor((x - kFieldX + half) / static_cast<float>(kTile))), static_cast<int>(std::floor((y - kFieldY + half) / static_cast<float>(kTile)))};
}

gd::Color bodyColor(int material) { return material == 2 ? kGrassBody : kWallBody; }
gd::Color edgeColor(int material) { return material == 2 ? kGrassEdge : kWallEdge; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float y;
    };
    const std::array<Def, 10> defs{{{Action::Mode4, "1 4-bit", 0.0F, kChipRow1},
                                    {Action::ModeRaw, "2 8-bit raw", 8.0F, kChipRow1},
                                    {Action::Mode47, "3 8-bit + corner rule", 8.0F, kChipRow1},
                                    {Action::ModeDual, "4 Dual grid", 8.0F, kChipRow1},
                                    {Action::ToolWall, "W Wall", 0.0F, kChipRow2},
                                    {Action::ToolGrass, "G Grass", 8.0F, kChipRow2},
                                    {Action::ToolErase, "E Erase", 8.0F, kChipRow2},
                                    {Action::TileGrid, "T Tile grid", 24.0F, kChipRow2},
                                    {Action::Clear, "C Clear", 24.0F, kChipRow2},
                                    {Action::Reset, "R Reset", 8.0F, kChipRow2}}};
    float x = kFieldX;
    float lastY = defs[0].y;
    for (const Def& d : defs) {
        if (d.y != lastY) {
            x = kFieldX;
            lastY = d.y;
        }
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, kChipH}});
        x += w;
    }
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::Mode4: return app.mode == at::Mode::FourBit;
        case Action::ModeRaw: return app.mode == at::Mode::EightBitRaw;
        case Action::Mode47: return app.mode == at::Mode::EightBit47;
        case Action::ModeDual: return app.mode == at::Mode::DualGrid;
        case Action::ToolWall: return app.tool == 1;
        case Action::ToolGrass: return app.tool == 2;
        case Action::ToolErase: return app.tool == 0;
        case Action::TileGrid: return app.tileGrid;
        case Action::Clear:
        case Action::Reset: return false;
    }
    return false;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::Mode4: app.mode = at::Mode::FourBit; break;
        case Action::ModeRaw: app.mode = at::Mode::EightBitRaw; break;
        case Action::Mode47: app.mode = at::Mode::EightBit47; break;
        case Action::ModeDual: app.mode = at::Mode::DualGrid; break;
        case Action::ToolWall: app.tool = 1; break;
        case Action::ToolGrass: app.tool = 2; break;
        case Action::ToolErase: app.tool = 0; break;
        case Action::TileGrid: app.tileGrid = !app.tileGrid; break;
        case Action::Clear: app.grid = at::Grid(kCols, kRows); break;
        case Action::Reset: app.grid = presetMap(); break;
    }
}

void paintLine(App& app, at::Cell to) {
    for (const at::Cell c : at::lineCells(app.stroke.last, to)) app.grid.set(c.x, c.y, app.stroke.material);
    app.stroke.last = to;
}

const char* modeTitle(at::Mode m) {
    switch (m) {
        case at::Mode::FourBit: return "4-bit: N E S W";
        case at::Mode::EightBitRaw: return "8-bit, raw mask";
        case at::Mode::EightBit47: return "8-bit + corner rule";
        case at::Mode::DualGrid: return "Dual grid, 4 corners";
    }
    return "";
}

const char* modeShort(at::Mode m) {
    switch (m) {
        case at::Mode::FourBit: return "4-bit";
        case at::Mode::EightBitRaw: return "8-bit raw";
        case at::Mode::EightBit47: return "8-bit, 47 tiles";
        case at::Mode::DualGrid: return "dual grid";
    }
    return "";
}

void rectPiece(SDL_Renderer* r, const at::Piece& p, gd::Color body, gd::Color edge) {
    gd::fillRect(r, static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.w), static_cast<float>(p.h), p.kind == at::Kind::Body ? body : edge);
}

void thickRect(SDL_Renderer* r, float x, float y, float w, float h, gd::Color c, float t) {
    for (float i = 0.0F; i < t; i += 1.0F) gd::strokeRect(r, x + i, y + i, w - 2.0F * i, h - 2.0F * i, c);
}

void drawField(App& app) {
    SDL_Renderer* r = app.renderer;
    const auto fw = static_cast<float>(kCols * kTile);
    const auto fh = static_cast<float>(kRows * kTile);
    const auto s = static_cast<float>(kTile);
    gd::fillRect(r, kFieldX, kFieldY, fw, fh, gd::kWhite);

    at::mapPieces(app.grid, app.mode, kTile, [&](const at::Piece& p, int material) {
        const at::Piece moved{p.x + static_cast<int>(kFieldX), p.y + static_cast<int>(kFieldY), p.w, p.h, p.kind};
        rectPiece(r, moved, bodyColor(material), edgeColor(material));
    });

    if (app.tileGrid) {
        const bool dual = app.mode == at::Mode::DualGrid;
        const gd::Color cellLine{17, 24, 39, dual ? std::uint8_t{18} : std::uint8_t{40}};
        for (int i = 1; i < kCols; ++i) gd::fillRect(r, kFieldX + static_cast<float>(i) * s, kFieldY, 1.0F, fh, cellLine);
        for (int i = 1; i < kRows; ++i) gd::fillRect(r, kFieldX, kFieldY + static_cast<float>(i) * s, fw, 1.0F, cellLine);
        if (dual) {
            // Display tiles are shifted by half a cell: their borders run through the cell centres.
            const gd::Color tileLine{234, 88, 12, 80};
            for (int i = 0; i < kCols; ++i) gd::fillRect(r, kFieldX + (static_cast<float>(i) + 0.5F) * s, kFieldY, 1.0F, fh, tileLine);
            for (int i = 0; i < kRows; ++i) gd::fillRect(r, kFieldX, kFieldY + (static_cast<float>(i) + 0.5F) * s, fw, 1.0F, tileLine);
        }
    }
    gd::strokeRect(r, kFieldX, kFieldY, fw, fh, gd::kInk);

    if (app.mouseIn) {
        const at::Cell c = cellAt(app.mouseX, app.mouseY);
        if (app.grid.inside(c.x, c.y)) {
            if (app.mode == at::Mode::DualGrid) {
                const at::Cell v = vertexAt(app.mouseX, app.mouseY);
                thickRect(r, kFieldX + static_cast<float>(v.x) * s - s / 2.0F, kFieldY + static_cast<float>(v.y) * s - s / 2.0F, s, s, gd::kOrange, 2.0F);
                gd::strokeRect(r, kFieldX + static_cast<float>(c.x) * s + 3.0F, kFieldY + static_cast<float>(c.y) * s + 3.0F, s - 6.0F, s - 6.0F, gd::kInk);
            } else {
                thickRect(r, kFieldX + static_cast<float>(c.x) * s, kFieldY + static_cast<float>(c.y) * s, s, s, gd::kOrange, 2.0F);
            }
        }
    }
}

struct AtlasLayout {
    int cols;
    int tile;
    int pitch;
};

AtlasLayout atlasLayout(at::Mode m) {
    switch (m) {
        case at::Mode::EightBitRaw: return {32, 8, 9};
        case at::Mode::EightBit47: return {10, 22, 27};
        case at::Mode::FourBit:
        case at::Mode::DualGrid: return {8, 28, 36};
    }
    return {8, 28, 36};
}

/// The tile number the hovered spot would get, or -1 when the mouse is not over the map.
int hoveredTile(const App& app, int& maskOut, std::uint8_t& materialOut) {
    maskOut = 0;
    materialOut = app.tool == 2 ? std::uint8_t{2} : std::uint8_t{1};
    if (!app.mouseIn) return -1;
    const at::Cell c = cellAt(app.mouseX, app.mouseY);
    if (!app.grid.inside(c.x, c.y)) return -1;
    if (app.mode == at::Mode::DualGrid) {
        const at::Cell v = vertexAt(app.mouseX, app.mouseY);
        maskOut = at::dualMask(app.grid, v.x, v.y, materialOut);
        return maskOut;
    }
    if (app.grid.at(c.x, c.y) != 0) materialOut = app.grid.at(c.x, c.y);
    maskOut = at::cellMask(app.grid, c.x, c.y, materialOut);
    return at::cellTileIndex(app.mode, maskOut);
}

void drawStatsCard(App& app, const std::vector<int>& use) {
    SDL_Renderer* r = app.renderer;
    const int total = at::tileCount(app.mode);
    int used = 0;
    for (int u : use) used += u > 0 ? 1 : 0;
    gd::card(r, kPanelX, kFieldY, kPanelW, 84.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, kFieldY + 8.0F, modeTitle(app.mode), gd::kInk, gd::Hud::Style::Bold);
    char line[64];
    SDL_snprintf(line, sizeof line, "atlas %d tiles · used %d", total, used);
    app.hud.text(r, kPanelX + 14.0F, kFieldY + 34.0F, line, gd::kTealDark, gd::Hud::Style::Bold);
    switch (app.mode) {
        case at::Mode::FourBit: {
            const int lost = at::mapInnerCorners(app.grid);
            SDL_snprintf(line, sizeof line, "inner corners lost: %d", lost);
            app.hud.text(r, kPanelX + 14.0F, kFieldY + 58.0F, line, lost > 0 ? gd::kOrange : gd::kMuted);
            break;
        }
        case at::Mode::EightBitRaw: app.hud.text(r, kPanelX + 14.0F, kFieldY + 58.0F, "only 47 look different", gd::kOrange); break;
        case at::Mode::EightBit47: app.hud.text(r, kPanelX + 14.0F, kFieldY + 58.0F, "256 masks fold to 47 tiles", gd::kMuted); break;
        case at::Mode::DualGrid: app.hud.text(r, kPanelX + 14.0F, kFieldY + 58.0F, "tiles sit on the cell corners", gd::kMuted); break;
    }
}

void drawAtlas(App& app, const std::vector<int>& use, int hovered) {
    SDL_Renderer* r = app.renderer;
    constexpr float kCardY = 140.0F;
    gd::card(r, kPanelX, kCardY, kPanelW, 150.0F, gd::kWhite, gd::kInk);
    const AtlasLayout lay = atlasLayout(app.mode);
    const float x0 = kPanelX + (kPanelW - static_cast<float>(lay.cols * lay.pitch)) / 2.0F;
    const float y0 = kCardY + 10.0F;
    const int count = at::tileCount(app.mode);
    const int pad = (lay.pitch - lay.tile) / 2;
    for (int i = 0; i < count; ++i) {
        const int col = i % lay.cols;
        const int row = i / lay.cols;
        const int sx = static_cast<int>(x0) + col * lay.pitch;
        const int sy = static_cast<int>(y0) + row * lay.pitch;
        const bool used = use[static_cast<std::size_t>(i)] > 0;
        if (app.mode == at::Mode::DualGrid && i == 0) {
            gd::strokeRect(r, static_cast<float>(sx + pad), static_cast<float>(sy + pad), static_cast<float>(lay.tile), static_cast<float>(lay.tile), kUnusedEdge);
        }
        at::tilePieces(app.mode, i, sx + pad, sy + pad, lay.tile, [&](const at::Piece& p) { rectPiece(r, p, used ? kWallBody : kUnusedBody, used ? kWallEdge : kUnusedEdge); });
        if (i == hovered) thickRect(r, static_cast<float>(sx), static_cast<float>(sy), static_cast<float>(lay.pitch), static_cast<float>(lay.pitch), gd::kOrange, lay.pitch > 12 ? 2.0F : 1.0F);
    }
}

void drawNeighbours(App& app, float x, float y, int mask, bool corners, bool ruleApplies) {
    SDL_Renderer* r = app.renderer;
    struct Spot {
        int bit;
        int col;
        int row;
    };
    static constexpr std::array<Spot, 8> kSpots{{{at::NW, 0, 0}, {at::N, 1, 0}, {at::NE, 2, 0}, {at::W, 0, 1}, {at::E, 2, 1}, {at::SW, 0, 2}, {at::S, 1, 2}, {at::SE, 2, 2}}};
    constexpr float kBox = 32.0F;
    const int kept = at::reduceCorners(mask);
    for (const Spot& sp : kSpots) {
        const bool isCorner = (sp.bit & (at::NE | at::SE | at::SW | at::NW)) != 0;
        const bool set = (mask & sp.bit) != 0;
        const bool dropped = isCorner && set && ((ruleApplies && (kept & sp.bit) == 0) || !corners);
        const float bx = x + static_cast<float>(sp.col) * kBox;
        const float by = y + static_cast<float>(sp.row) * kBox;
        gd::Color fill = gd::kWhite;
        gd::Color ink = gd::kMuted;
        if (dropped) {
            fill = gd::Color{254, 215, 170};
            ink = gd::kOrange;
        } else if (set) {
            fill = gd::kTeal;
            ink = gd::kWhite;
        }
        gd::fillRect(r, bx, by, kBox, kBox, fill);
        gd::strokeRect(r, bx, by, kBox, kBox, gd::kInk);
        char label[8];
        SDL_snprintf(label, sizeof label, "%d", sp.bit);
        app.hud.text(r, bx + (kBox - app.hud.width(label)) / 2.0F, by + 5.0F, label, ink);
    }
    gd::fillRect(r, x + kBox, y + kBox, kBox, kBox, gd::kInk);
}

void drawDualCorners(App& app, float x, float y, int mask) {
    SDL_Renderer* r = app.renderer;
    struct Spot {
        int bit;
        int col;
        int row;
    };
    static constexpr std::array<Spot, 4> kSpots{{{at::kDualNW, 0, 0}, {at::kDualNE, 1, 0}, {at::kDualSW, 0, 1}, {at::kDualSE, 1, 1}}};
    constexpr float kBox = 48.0F;
    for (const Spot& sp : kSpots) {
        const bool set = (mask & sp.bit) != 0;
        const float bx = x + static_cast<float>(sp.col) * kBox;
        const float by = y + static_cast<float>(sp.row) * kBox;
        gd::fillRect(r, bx, by, kBox, kBox, set ? gd::kTeal : gd::kWhite);
        gd::strokeRect(r, bx, by, kBox, kBox, gd::kInk);
        char label[8];
        SDL_snprintf(label, sizeof label, "%d", sp.bit);
        app.hud.text(r, bx + (kBox - app.hud.width(label)) / 2.0F, by + 13.0F, label, set ? gd::kWhite : gd::kMuted);
    }
    gd::disc(r, x + kBox, y + kBox, 5.0F, gd::kOrange);
    gd::ring(r, x + kBox, y + kBox, 5.0F, gd::kInk);
}

void drawHoverCard(App& app) {
    SDL_Renderer* r = app.renderer;
    constexpr float kCardY = 300.0F;
    constexpr float kCardH = 128.0F;
    gd::card(r, kPanelX, kCardY, kPanelW, kCardH, gd::kWhite, gd::kInk);
    int mask = 0;
    std::uint8_t material = 1;
    const int tile = hoveredTile(app, mask, material);
    if (tile < 0) {
        app.hud.text(r, kPanelX + 14.0F, kCardY + 12.0F, "move over the map", gd::kMuted);
        return;
    }
    const float dx = kPanelX + 14.0F;
    const float dy = kCardY + 14.0F;
    const float tx = kPanelX + 14.0F + 96.0F + 18.0F;
    const at::Cell c = cellAt(app.mouseX, app.mouseY);
    char line[64];
    float ty = kCardY + 14.0F;
    auto put = [&](const char* text, gd::Color col, gd::Hud::Style style) {
        app.hud.text(r, tx, ty, text, col, style);
        ty += 21.0F;
    };
    const int total = at::tileCount(app.mode);
    if (app.mode == at::Mode::DualGrid) {
        const at::Cell v = vertexAt(app.mouseX, app.mouseY);
        drawDualCorners(app, dx + 0.0F, dy, mask);
        SDL_snprintf(line, sizeof line, "vertex %d,%d", v.x, v.y);
        put(line, gd::kInk, gd::Hud::Style::Bold);
        SDL_snprintf(line, sizeof line, "corners %d", mask);
        put(line, gd::kInk, gd::Hud::Style::Regular);
        SDL_snprintf(line, sizeof line, "tile %d of %d", tile, total);
        put(line, gd::kTealDark, gd::Hud::Style::Bold);
        return;
    }
    const bool four = app.mode == at::Mode::FourBit;
    const bool rule = app.mode == at::Mode::EightBit47;
    drawNeighbours(app, dx, dy, mask, !four, rule);
    SDL_snprintf(line, sizeof line, "cell %d,%d", c.x, c.y);
    put(line, gd::kInk, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "mask %d", mask);
    put(line, gd::kInk, gd::Hud::Style::Regular);
    if (four) {
        SDL_snprintf(line, sizeof line, "N E S W = %d", at::fourBitIndex(mask));
        put(line, gd::kInk, gd::Hud::Style::Regular);
    } else if (rule) {
        SDL_snprintf(line, sizeof line, "reduced %d", at::reduceCorners(mask));
        put(line, gd::kInk, gd::Hud::Style::Regular);
    }
    SDL_snprintf(line, sizeof line, "tile %d of %d", tile, total);
    put(line, gd::kTealDark, gd::Hud::Style::Bold);
    if (four || rule) app.hud.text(r, tx, kCardY + kCardH - 26.0F, "orange = ignored", gd::kOrange);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ AUTOTILING ]", modeShort(app.mode));

    drawField(app);
    const std::vector<int> use = at::tileUsage(app.grid, app.mode);
    int mask = 0;
    std::uint8_t material = 1;
    const int hovered = hoveredTile(app, mask, material);
    drawStatsCard(app, use);
    drawAtlas(app, use, hovered);
    drawHoverCard(app);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    const Chip& modeLast = app.chips[3];
    app.hud.text(r, modeLast.rect.x + modeLast.rect.w + 20.0F, kChipRow1 + 6.0F, "drag: paint · right: erase", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("autotiling", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    app->grid = presetMap();
    layoutChips(*app);
    // Start over the top of the little pyramid: two of its diagonal neighbours are set and still get dropped by the corner rule.
    app->mouseX = kFieldX + 16.5F * static_cast<float>(kTile);
    app->mouseY = kFieldY + 2.5F * static_cast<float>(kTile);
    app->mouseIn = true;
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
            app.mouseIn = true;
            if (app.stroke.active) paintLine(app, cellAt(app.mouseX, app.mouseY));
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouseX = p.x;
            app.mouseY = p.y;
            app.mouseIn = true;
            if (event->button.button != SDL_BUTTON_LEFT && event->button.button != SDL_BUTTON_RIGHT) break;
            if (event->button.button == SDL_BUTTON_LEFT) {
                bool onChip = false;
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) {
                        apply(app, c.action);
                        onChip = true;
                    }
                }
                if (onChip) break;
            }
            const at::Cell c = cellAt(p.x, p.y);
            if (!app.grid.inside(c.x, c.y)) break;
            app.stroke.active = true;
            app.stroke.material = event->button.button == SDL_BUTTON_RIGHT ? std::uint8_t{0} : app.tool;
            app.stroke.last = c;
            paintLine(app, c);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP: app.stroke.active = false; break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.stroke.active = false;
            app.mouseIn = false;
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.stroke.active = false; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_1: apply(app, Action::Mode4); break;
                case SDLK_2: apply(app, Action::ModeRaw); break;
                case SDLK_3: apply(app, Action::Mode47); break;
                case SDLK_4: apply(app, Action::ModeDual); break;
                case SDLK_W: apply(app, Action::ToolWall); break;
                case SDLK_G: apply(app, Action::ToolGrass); break;
                case SDLK_E: apply(app, Action::ToolErase); break;
                case SDLK_T: apply(app, Action::TileGrid); break;
                case SDLK_C: apply(app, Action::Clear); break;
                case SDLK_R: apply(app, Action::Reset); break;
                default: break;
            }
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
