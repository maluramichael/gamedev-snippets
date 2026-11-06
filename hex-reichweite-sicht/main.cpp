// Hex range and sight with SDL3 main callbacks.
//
// A unit stands on a hex map with walls. Range: a loop over q and r claims every hex within N steps, the flood
// fill only reaches what can be walked to, the difference is the point of the first two modes. Sight: three ways to
// answer "can the unit see that hex", a line of hexes, a straight line from center to center, or any of seven lines
// (center and the six corners). Hover a hex to see the lines behind its answer. The orange dot marks a hex whose
// answer differs from the center line. Click or drag paints walls, dragging the unit (or a right click) moves it.
// The logic, including the rules for grazing a wall, lives in logic.hpp and is covered by test.cpp.

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
constexpr int kMinPoints = 1;
constexpr int kMaxPoints = 12;
constexpr int kDefaultPoints = 5;
constexpr double kHexSize = 20.0;
constexpr float kChipY = 484.0F;
constexpr float kChipH = 34.0F;
constexpr gd::Color kInkSoft{17, 24, 39, 140};

// Map rectangle starts at x = 262 (left of it the info panel) and y = 36 (below the title bar).
const hx::Layout kLayout{kHexSize, {262.0 + kHexSize * hx::kSqrt3 / 2.0, 36.0 + kHexSize}};

enum class Mode { RangeLoop, RangeFlood, SightHexLine, SightCenter, SightCorners };
enum class Action { ModeLoop, ModeFlood, ModeHexLine, ModeCenter, ModeCorners, Fewer, More, ClearWalls, Reset };
enum class Drag { None, Paint, Unit };

bool isSight(Mode m) { return m == Mode::SightHexLine || m == Mode::SightCenter || m == Mode::SightCorners; }

hx::Sight sightOf(Mode m) {
    switch (m) {
        case Mode::SightHexLine: return hx::Sight::HexLine;
        case Mode::SightCorners: return hx::Sight::Corners;
        default: return hx::Sight::Center;
    }
}

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
    const char* group;  // heading above the chip, nullptr when it continues the group before
};

/// Everything derived from the map, the unit, the mode and the size. Recomputed when one of them changes.
struct Analysis {
    hx::Reach reach;
    hx::SightField sight;
    std::vector<hx::Hex> walls;
    int inRange = 0;  // hexes within the size that are not walls (range modes)
    int reached = 0;
    int visible = 0;
    int hidden = 0;
    int differ = 0;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    hx::Map map{hx::kPresetCols, hx::kPresetRows};
    hx::Hex unit{};
    Mode mode = Mode::RangeFlood;
    int points = kDefaultPoints;
    Analysis analysis;
    bool dirty = true;
    std::optional<hx::Hex> hover;
    Drag drag = Drag::None;
    bool paintValue = true;
    long maxFrames = -1;
    long frames = 0;
};

hx::Vec2 toVec(float x, float y) { return {static_cast<double>(x), static_cast<double>(y)}; }

void recompute(App& app) {
    Analysis a;
    a.walls = app.map.walls();
    if (isSight(app.mode)) {
        a.sight = hx::computeSight(app.map, kLayout, app.unit, app.points, sightOf(app.mode));
        for (std::size_t i = 0; i < app.map.size(); ++i) {
            a.visible += a.sight.seen[i] == hx::Seen::Visible ? 1 : 0;
            a.hidden += a.sight.seen[i] == hx::Seen::Hidden ? 1 : 0;
            a.differ += a.sight.differs[i] != 0 ? 1 : 0;
        }
    } else {
        a.reach = hx::floodFill(app.map, app.unit, app.points);
        for (int c : a.reach.cost) a.reached += c >= 0 ? 1 : 0;
        for (const hx::Hex& h : hx::hexesInRange(app.map, app.unit, app.points)) a.inRange += app.map.wall(h) ? 0 : 1;
    }
    app.analysis = std::move(a);
    app.dirty = false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        const char* group;
    };
    const std::array<Def, 9> defs{{{Action::ModeLoop, "1 Loop", 0.0F, "RANGE"},
                                   {Action::ModeFlood, "2 Flood fill", 6.0F, nullptr},
                                   {Action::ModeHexLine, "3 Hex line", 20.0F, "SIGHT"},
                                   {Action::ModeCenter, "4 Center", 6.0F, nullptr},
                                   {Action::ModeCorners, "5 Corners", 6.0F, nullptr},
                                   {Action::Fewer, "-", 20.0F, "SIZE"},
                                   {Action::More, "+", 6.0F, nullptr},
                                   {Action::ClearWalls, "C Clear", 20.0F, "WALLS"},
                                   {Action::Reset, "R Reset", 6.0F, nullptr}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 20.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}, d.group});
        x += w;
    }
}

void resetMap(App& app) {
    if (const std::optional<hx::Hex> start = hx::loadLayout(app.map, hx::presetRows())) app.unit = *start;
    app.dirty = true;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ModeLoop: app.mode = Mode::RangeLoop; break;
        case Action::ModeFlood: app.mode = Mode::RangeFlood; break;
        case Action::ModeHexLine: app.mode = Mode::SightHexLine; break;
        case Action::ModeCenter: app.mode = Mode::SightCenter; break;
        case Action::ModeCorners: app.mode = Mode::SightCorners; break;
        case Action::Fewer: app.points = std::max(kMinPoints, app.points - 1); break;
        case Action::More: app.points = std::min(kMaxPoints, app.points + 1); break;
        case Action::ClearWalls: app.map.clearWalls(); break;
        case Action::Reset: resetMap(app); break;
    }
    app.drag = Drag::None;
    app.dirty = true;
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ModeLoop: return app.mode == Mode::RangeLoop;
        case Action::ModeFlood: return app.mode == Mode::RangeFlood;
        case Action::ModeHexLine: return app.mode == Mode::SightHexLine;
        case Action::ModeCenter: return app.mode == Mode::SightCenter;
        case Action::ModeCorners: return app.mode == Mode::SightCorners;
        default: return false;
    }
}

void paintAt(App& app, hx::Hex h) {
    if (h == app.unit || !app.map.contains(h) || app.map.wall(h) == app.paintValue) return;
    app.map.setWall(h, app.paintValue);
    app.dirty = true;
}

void moveUnit(App& app, hx::Hex h) {
    if (h == app.unit || !app.map.contains(h) || app.map.wall(h)) return;
    app.unit = h;
    app.dirty = true;
}

// ---- drawing ----------------------------------------------------------------------------------------------------

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, hx::Vec2 a, hx::Vec2 b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F),
                       static_cast<float>(b.x) + (flat ? 0.0F : off), static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

void fillHex(SDL_Renderer* r, hx::Vec2 c, double size, gd::Color color) {
    static constexpr std::array<int, 18> kIndices{0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 1};
    const SDL_FColor fc = gd::fcolor(color);
    std::array<SDL_Vertex, 7> v{};
    v[0] = SDL_Vertex{{static_cast<float>(c.x), static_cast<float>(c.y)}, fc, {0.0F, 0.0F}};
    for (std::size_t i = 0; i < 6; ++i) {
        v[i + 1] = SDL_Vertex{{static_cast<float>(c.x + size * hx::kCornerDirs[i].x), static_cast<float>(c.y + size * hx::kCornerDirs[i].y)}, fc, {0.0F, 0.0F}};
    }
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), kIndices.data(), static_cast<int>(kIndices.size()));
}

void outlineHex(SDL_Renderer* r, hx::Vec2 c, double size, gd::Color color, int thickness = 1) {
    gd::setColor(r, color);
    for (int t = 0; t < thickness; ++t) {
        const double s = size - t;
        for (std::size_t i = 0; i < 6; ++i) {
            const hx::Vec2 a = hx::kCornerDirs[i];
            const hx::Vec2 b = hx::kCornerDirs[(i + 1) % 6];
            SDL_RenderLine(r, static_cast<float>(c.x + s * a.x), static_cast<float>(c.y + s * a.y), static_cast<float>(c.x + s * b.x), static_cast<float>(c.y + s * b.y));
        }
    }
}

gd::Color withAlpha(gd::Color c, int alpha) { return gd::Color{c.r, c.g, c.b, static_cast<Uint8>(std::clamp(alpha, 0, 255))}; }

void centeredText(App& app, hx::Vec2 c, const char* s, gd::Color color) {
    app.hud.text(app.renderer, static_cast<float>(c.x) - app.hud.width(s, gd::Hud::Style::Bold) / 2.0F, static_cast<float>(c.y) - 11.0F, s, color, gd::Hud::Style::Bold);
}

void drawUnit(App& app) {
    const hx::Vec2 c = hx::toPixel(kLayout, app.unit);
    const auto x = static_cast<float>(c.x);
    const auto y = static_cast<float>(c.y);
    gd::disc(app.renderer, x, y, 9.0F, gd::kWhite);
    gd::ring(app.renderer, x, y, 9.0F, gd::kInk);
    gd::ring(app.renderer, x, y, 10.0F, gd::kInk);
    gd::disc(app.renderer, x, y, 4.0F, gd::kInk);
}

void drawCells(App& app) {
    SDL_Renderer* r = app.renderer;
    const Analysis& an = app.analysis;
    const bool sight = isSight(app.mode);
    const bool loop = app.mode == Mode::RangeLoop;
    for (std::size_t i = 0; i < app.map.size(); ++i) {
        const hx::Hex h = app.map.hexAtIndex(i);
        const hx::Vec2 c = hx::toPixel(kLayout, h);
        const bool wall = app.map.wall(h);
        fillHex(r, c, kHexSize - 1.0, withAlpha(gd::kWhite, 150));
        bool labelWhite = false;
        char label[8] = "";
        if (sight) {
            const hx::Seen seen = an.sight.seen[i];
            if (!wall && seen == hx::Seen::Visible) fillHex(r, c, kHexSize - 1.0, withAlpha(gd::kTeal, 150));
            if (!wall && seen == hx::Seen::Hidden) fillHex(r, c, kHexSize - 1.0, withAlpha(gd::kInk, 45));
        } else {
            const int cost = an.reach.layers.empty() ? -1 : an.reach.cost[i];
            const int dist = hx::distance(app.unit, h);
            if (!wall && dist <= app.points) {
                if (cost >= 0) {
                    const double t = static_cast<double>(cost) / static_cast<double>(app.points);
                    fillHex(r, c, kHexSize - 1.0, withAlpha(gd::kTeal, static_cast<int>(235.0 - 140.0 * t)));
                    labelWhite = t < 0.45;
                } else {
                    fillHex(r, c, kHexSize - 1.0, withAlpha(gd::kOrange, loop ? 130 : 28));
                }
                if (h != app.unit && (loop || cost >= 0)) SDL_snprintf(label, sizeof label, "%d", loop ? dist : cost);
            }
        }
        outlineHex(r, c, kHexSize, withAlpha(gd::kMuted, 90));
        if (wall) {
            const bool dim = sight && an.sight.seen[i] != hx::Seen::Visible;
            fillHex(r, c, kHexSize - 1.0, dim ? gd::kMuted : gd::kInk);
            fillHex(r, c, kHexSize * 0.55, dim ? withAlpha(gd::kWhite, 70) : withAlpha(gd::kWhite, 40));
        }
        if (label[0] != '\0') centeredText(app, c, label, labelWhite ? gd::kWhite : gd::kInk);
        if (sight && an.sight.differs[i] != 0) {
            gd::disc(r, static_cast<float>(c.x), static_cast<float>(c.y), 5.0F, gd::kOrange);
            gd::ring(r, static_cast<float>(c.x), static_cast<float>(c.y), 5.0F, gd::kInk);
        }
    }
    // Hexes the loop would claim but nobody can walk to: shown as an orange outline in the flood fill mode.
    if (!sight && !loop) {
        for (std::size_t i = 0; i < app.map.size(); ++i) {
            const hx::Hex h = app.map.hexAtIndex(i);
            if (!app.map.wall(h) && hx::distance(app.unit, h) <= app.points && !an.reach.layers.empty() && an.reach.cost[i] < 0) {
                outlineHex(r, hx::toPixel(kLayout, h), kHexSize - 4.0, gd::kOrange, 2);
            }
        }
    }
}

/// Lines and outlines for the hovered hex, returns the text for the "ray" row of the panel.
std::string drawHover(App& app) {
    SDL_Renderer* r = app.renderer;
    if (!app.hover) return "-";
    const hx::Hex t = *app.hover;
    const hx::Vec2 tc = hx::toPixel(kLayout, t);
    if (!isSight(app.mode)) {
        outlineHex(r, tc, kHexSize, gd::kInk, 2);
        if (app.analysis.reach.layers.empty()) return "-";
        const int cost = app.analysis.reach.cost[app.map.index(t)];
        if (app.map.wall(t)) return "wall";
        if (cost >= 0) return std::to_string(cost) + " points";
        return hx::distance(app.unit, t) <= app.points ? "cut off" : "too far";
    }
    outlineHex(r, tc, kHexSize, gd::kInk, 2);
    if (t == app.unit) return "-";
    const hx::Vec2 a = hx::toPixel(kLayout, app.unit);
    const std::vector<hx::Hex>& walls = app.analysis.walls;
    const std::optional<hx::Hex> centerBlock = hx::blocker(kLayout, walls, app.unit, t, a, tc);
    std::string verdict;
    if (app.mode == Mode::SightHexLine) {
        const std::vector<hx::Hex> line = hx::hexLine(app.unit, t);
        thickLine(r, a, tc, kInkSoft, 0);
        std::optional<hx::Hex> stop;
        for (std::size_t i = 1; i + 1 < line.size() && !stop; ++i) {
            if (app.map.wall(line[i])) stop = line[i];
        }
        for (std::size_t i = 1; i < line.size(); ++i) outlineHex(r, hx::toPixel(kLayout, line[i]), kHexSize - 3.0, gd::kInk, 2);
        if (stop) outlineHex(r, hx::toPixel(kLayout, *stop), kHexSize - 3.0, gd::kOrange, 3);
        verdict = stop ? "blocked" : "clear";
    } else {
        int clearCorner = -1;
        if (app.mode == Mode::SightCorners) {
            for (int i = 0; i < 6; ++i) {
                const hx::Vec2 p = hx::corner(kLayout, t, i);
                const bool blocked = hx::blocker(kLayout, walls, app.unit, t, a, p).has_value();
                thickLine(r, a, p, blocked ? withAlpha(gd::kOrange, 170) : withAlpha(gd::kTeal, 220), 0);
                if (!blocked && clearCorner < 0) clearCorner = i;
            }
        }
        thickLine(r, a, tc, centerBlock ? gd::kOrange : gd::kTeal, 1);
        if (centerBlock) outlineHex(r, hx::toPixel(kLayout, *centerBlock), kHexSize - 3.0, gd::kOrange, 3);
        if (!centerBlock) {
            verdict = "clear";
        } else if (clearCorner >= 0) {
            verdict = "via corner";
        } else {
            verdict = "blocked";
        }
    }
    gd::disc(r, static_cast<float>(tc.x), static_cast<float>(tc.y), 4.0F, verdict == "blocked" ? gd::kOrange : gd::kTeal);
    return verdict;
}

void statRow(App& app, float y, const char* name, const std::string& value, gd::Color color = gd::kInk) {
    app.hud.text(app.renderer, 38.0F, y, name, gd::kMuted);
    app.hud.text(app.renderer, 228.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, color, gd::Hud::Style::Bold);
}

void legendRow(App& app, float y, gd::Color swatch, bool outline, const char* text) {
    if (outline) {
        gd::strokeRect(app.renderer, 38.0F, y + 4.0F, 16.0F, 14.0F, swatch);
        gd::strokeRect(app.renderer, 39.0F, y + 5.0F, 14.0F, 12.0F, swatch);
    } else {
        gd::fillRect(app.renderer, 38.0F, y + 4.0F, 16.0F, 14.0F, swatch);
        gd::strokeRect(app.renderer, 38.0F, y + 4.0F, 16.0F, 14.0F, gd::kInk);
    }
    app.hud.text(app.renderer, 62.0F, y, text, gd::kInk);
}

void drawPanel(App& app, const std::string& rayText) {
    SDL_Renderer* r = app.renderer;
    const Analysis& an = app.analysis;
    const bool sight = isSight(app.mode);
    const char* title = "";
    const char* sub1 = "";
    const char* sub2 = "";
    switch (app.mode) {
        case Mode::RangeLoop: title = "LOOP OVER q, r"; sub1 = "every hex within"; sub2 = "N steps, no walls"; break;
        case Mode::RangeFlood: title = "FLOOD FILL"; sub1 = "one layer per"; sub2 = "point, walls stop"; break;
        case Mode::SightHexLine: title = "HEX LINE"; sub1 = "cube lerp and round"; sub2 = "walls on the line"; break;
        case Mode::SightCenter: title = "CENTER RAY"; sub1 = "exact line to the"; sub2 = "center of the hex"; break;
        case Mode::SightCorners: title = "CENTER + CORNERS"; sub1 = "any of 7 lines"; sub2 = "to the hex is clear"; break;
    }
    gd::card(r, 24.0F, 44.0F, 220.0F, 380.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, 52.0F, title, gd::kTealDark, gd::Hud::Style::Bold);
    app.hud.text(r, 38.0F, 74.0F, sub1, gd::kMuted);
    app.hud.text(r, 38.0F, 94.0F, sub2, gd::kMuted);

    const bool loop = app.mode == Mode::RangeLoop;
    const int headline = sight ? an.visible : (loop ? an.inRange : an.reached);
    char big[16];
    SDL_snprintf(big, sizeof big, "%d", headline);
    app.hud.text(r, 38.0F, 122.0F, big, loop ? gd::kOrange : gd::kTeal, gd::Hud::Style::Large);
    const char* bigLabel = sight ? "visible" : (loop ? "claimed" : "reachable");
    app.hud.text(r, 38.0F + app.hud.width(big, gd::Hud::Style::Large) + 10.0F, 140.0F, bigLabel, gd::kMuted, gd::Hud::Style::Bold);

    if (sight) {
        statRow(app, 172.0F, "radius", std::to_string(app.points));
        statRow(app, 194.0F, "in radius", std::to_string(an.visible + an.hidden));
        statRow(app, 216.0F, "visible", std::to_string(an.visible), gd::kTealDark);
        statRow(app, 238.0F, "hidden", std::to_string(an.hidden));
        statRow(app, 260.0F, "differs", std::to_string(an.differ), an.differ > 0 ? gd::kOrange : gd::kInk);
    } else {
        statRow(app, 172.0F, "points", std::to_string(app.points));
        statRow(app, 194.0F, loop ? "claimed" : "in distance", std::to_string(an.inRange));
        statRow(app, 216.0F, "reached", std::to_string(an.reached), gd::kTealDark);
        statRow(app, 238.0F, "cut off", std::to_string(std::max(0, an.inRange - an.reached)), an.inRange > an.reached ? gd::kOrange : gd::kInk);
    }
    gd::fillRect(r, 38.0F, 290.0F, 192.0F, 1.0F, gd::kGrid);
    if (app.hover) {
        char pos[24];
        SDL_snprintf(pos, sizeof pos, "%d, %d", app.hover->q, app.hover->r);
        statRow(app, 298.0F, "hover q, r", pos);
        statRow(app, 320.0F, sight ? "line" : "cost", rayText);
    } else {
        app.hud.text(r, 38.0F, 298.0F, "hover a hex", gd::kMuted);
    }
    gd::fillRect(r, 38.0F, 350.0F, 192.0F, 1.0F, gd::kGrid);
    if (sight) {
        legendRow(app, 356.0F, withAlpha(gd::kTeal, 150), false, "visible");
        legendRow(app, 376.0F, withAlpha(gd::kInk, 45), false, "hidden");
        legendRow(app, 396.0F, gd::kOrange, false, app.mode == Mode::SightCenter ? "reference: none" : "differs: Center");
    } else {
        legendRow(app, 356.0F, gd::kTeal, false, "reachable (cost)");
        if (app.mode == Mode::RangeLoop) {
            legendRow(app, 376.0F, withAlpha(gd::kOrange, 130), false, "claimed, cut off");
            app.hud.text(r, 62.0F, 396.0F, "number = distance", gd::kMuted);
        } else {
            legendRow(app, 376.0F, gd::kOrange, true, "cut off");
            app.hud.text(r, 62.0F, 396.0F, "number = cost", gd::kMuted);
        }
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[48];
    const char* names[] = {"loop", "flood fill", "hex line", "center ray", "corners"};
    SDL_snprintf(right, sizeof right, "%s, %s %d", names[static_cast<int>(app.mode)], isSight(app.mode) ? "radius" : "points", app.points);
    app.hud.titleBar(r, kWidth, "[ HEX RANGE AND SIGHT ]", right);

    drawCells(app);
    const std::string rayText = drawHover(app);
    drawUnit(app);
    drawPanel(app, rayText);

    app.hud.text(r, 24.0F, 440.0F, "drag: paint walls   drag the unit (or right click): move it   + -: size", gd::kMuted);
    for (const Chip& c : app.chips) {
        if (c.group != nullptr) app.hud.text(r, c.rect.x, kChipY - 20.0F, c.group, gd::kMuted);
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void updateHover(App& app, float x, float y) {
    const hx::Hex h = hx::fromPixel(kLayout, toVec(x, y));
    app.hover = app.map.contains(h) ? std::optional<hx::Hex>(h) : std::nullopt;
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
    if (!SDL_CreateWindowAndRenderer("hex-reichweite-sicht", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    const std::optional<hx::Hex> start = hx::loadLayout(app->map, hx::presetRows());
    if (!start) {
        SDL_Log("start layout is malformed");
        return SDL_APP_FAILURE;
    }
    app->unit = *start;
    layoutChips(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            updateHover(app, event->motion.x, event->motion.y);
            // A button released outside the window never reaches us as an UP event: trust the button state instead.
            if ((event->motion.state & SDL_BUTTON_LMASK) == 0) app.drag = Drag::None;
            if (app.hover && app.drag == Drag::Paint) paintAt(app, *app.hover);
            if (app.hover && app.drag == Drag::Unit) moveUnit(app, *app.hover);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            updateHover(app, p.x, p.y);
            if (event->button.button == SDL_BUTTON_RIGHT) {
                if (app.hover) moveUnit(app, *app.hover);
                break;
            }
            if (event->button.button != SDL_BUTTON_LEFT) break;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                    break;
                }
            }
            if (onChip || !app.hover) break;
            if (*app.hover == app.unit) {
                app.drag = Drag::Unit;
            } else {
                app.paintValue = !app.map.wall(*app.hover);
                app.drag = Drag::Paint;
                paintAt(app, *app.hover);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.hover.reset();
            app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.drag = Drag::None; break;
        case SDL_EVENT_KEY_DOWN:
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_PLUS:
                case SDLK_KP_PLUS:
                case SDLK_EQUALS: apply(app, Action::More); break;
                case SDLK_MINUS:
                case SDLK_KP_MINUS: apply(app, Action::Fewer); break;
                default: break;
            }
            if (!event->key.repeat) {
                switch (event->key.key) {
                    case SDLK_1: apply(app, Action::ModeLoop); break;
                    case SDLK_2: apply(app, Action::ModeFlood); break;
                    case SDLK_3: apply(app, Action::ModeHexLine); break;
                    case SDLK_4: apply(app, Action::ModeCenter); break;
                    case SDLK_5: apply(app, Action::ModeCorners); break;
                    case SDLK_C: apply(app, Action::ClearWalls); break;
                    case SDLK_R: apply(app, Action::Reset); break;
                    default: break;
                }
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    if (app.dirty) recompute(app);
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
