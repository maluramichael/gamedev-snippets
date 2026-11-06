// Terrain shaping with SDL3 main callbacks.
//
// The same noise, reshaped. A fractal sum of gradient noise gives a smooth height field (and, from the same octaves, a
// ridged one). Between the noise and the picture sit three small functions: a mix that adds the ridge detail on the
// summits only, a monotone height curve whose five nodes can be dragged, and optional terraces. The map shows the result as
// a shaded relief, the histogram shows what the curve does to the distribution of heights, and the cross-section lays the
// raw noise over the shaped line along one row. "Naive" switches ridges to everywhere and terraces to vertical cliffs,
// the two usual mistakes. The math lives in logic.hpp and is covered by test.cpp.

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

constexpr int kGridW = 210;  // noise cells, drawn 2x2 px each
constexpr int kGridH = 158;
constexpr int kCellPx = 2;
constexpr float kMapX = 24.0F;
constexpr float kMapY = 44.0F;
constexpr float kMapW = static_cast<float>(kGridW * kCellPx);
constexpr float kMapH = static_cast<float>(kGridH * kCellPx);

constexpr float kCurveX = 474.0F;
constexpr float kCurveY = 62.0F;
constexpr float kCurveS = 164.0F;
constexpr float kHistX = 686.0F;
constexpr float kHistY = 62.0F;
constexpr float kHistW = 250.0F;
constexpr float kHistH = 164.0F;
constexpr float kProfX = 474.0F;
constexpr float kProfY = 258.0F;
constexpr float kProfW = 462.0F;
constexpr float kProfH = 140.0F;
constexpr float kNodeGrab = 14.0F;

constexpr float kChipH = 32.0F;
constexpr float kChipRow1 = 432.0F;
constexpr float kChipRow2 = 474.0F;
constexpr double kShadeScale = 50.0;
constexpr gd::Color kInkSoft{17, 24, 39, 120};
constexpr gd::Color kWater{14, 116, 144};

enum class Action { Linear, Valleys, SCurve, NewSeed, Octaves, Terraces, Steps, Ridged, Naive };

struct Chip {
    Action action;
    SDL_FRect rect;
};

enum class Drag { None, Pan, Node };

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* map = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    ts::Params params;
    ts::Fields fields;
    ts::CurveY curveY = ts::valleysCurve();
    int preset = 1;  // 0 linear, 1 valleys, 2 s-curve, -1 dragged by hand
    ts::Shape shape;
    std::vector<float> shaped;
    ts::Histogram before;
    ts::Histogram after;
    std::vector<std::uint8_t> pixels;
    bool dirtyNoise = true;
    bool dirtyShape = true;
    double landPct = 0.0;
    double peak = 0.0;

    int profileRow = kGridH / 2;
    Drag drag = Drag::None;
    int dragNode = -1;
    SDL_FPoint mouse{-100.0F, -100.0F};
    SDL_FPoint lastMouse{-100.0F, -100.0F};
    int seedCounter = 0;

    long maxFrames = -1;
    long frames = 0;
};

constexpr std::array<std::uint32_t, 6> kSeeds{3, 11, 27, 42, 58, 77};

// ---- colors -------------------------------------------------------------------------------------------------------

struct Rgb {
    float r;
    float g;
    float b;
};

Rgb mix(Rgb a, Rgb b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }

/// Land color by height above the sea, 0..1: sand, grass, forest, rock, snow.
Rgb landColor(float t) {
    struct Stop {
        float at;
        Rgb c;
    };
    static constexpr std::array<Stop, 6> stops{{{0.00F, {226, 214, 160}}, {0.08F, {150, 190, 110}}, {0.35F, {90, 150, 95}}, {0.60F, {150, 140, 128}}, {0.80F, {190, 184, 176}}, {1.00F, {250, 250, 250}}}};
    t = std::clamp(t, 0.0F, 1.0F);
    for (std::size_t i = 1; i < stops.size(); ++i) {
        if (t <= stops[i].at) return mix(stops[i - 1].c, stops[i].c, (t - stops[i - 1].at) / (stops[i].at - stops[i - 1].at));
    }
    return stops.back().c;
}

std::uint8_t toByte(float v) { return static_cast<std::uint8_t>(std::clamp(v, 0.0F, 255.0F)); }

// ---- state changes ------------------------------------------------------------------------------------------------

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float row;
    };
    const std::array<Def, 9> defs{{{Action::Linear, "1 Linear", 0.0F, kChipRow1},
                                   {Action::Valleys, "2 Valleys", 8.0F, kChipRow1},
                                   {Action::SCurve, "3 S-curve", 8.0F, kChipRow1},
                                   {Action::NewSeed, "Space New seed", 24.0F, kChipRow1},
                                   {Action::Octaves, "O Octaves: 5", 8.0F, kChipRow1},
                                   {Action::Terraces, "T Terraces", 0.0F, kChipRow2},
                                   {Action::Steps, "S Steps: 6", 8.0F, kChipRow2},
                                   {Action::Ridged, "R Ridged", 24.0F, kChipRow2},
                                   {Action::Naive, "N Naive", 24.0F, kChipRow2}}};
    float x = 24.0F;
    float row = kChipRow1;
    for (const Def& d : defs) {
        if (d.row != row) {
            row = d.row;
            x = 24.0F;
        }
        x += d.gapBefore;
        // Widths are fixed for the longest label of a chip, so the row does not shift when "5" becomes "10".
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F + (d.action == Action::Steps ? 10.0F : 0.0F);
        app.chips.push_back({d.action, SDL_FRect{x, d.row, w, kChipH}});
        x += w;
    }
}

std::string chipLabel(const App& app, Action a) {
    switch (a) {
        case Action::Linear: return "1 Linear";
        case Action::Valleys: return "2 Valleys";
        case Action::SCurve: return "3 S-curve";
        case Action::NewSeed: return "Space New seed";
        case Action::Octaves: return "O Octaves: " + std::to_string(app.params.octaves);
        case Action::Terraces: return "T Terraces";
        case Action::Steps: return "S Steps: " + std::to_string(app.shape.steps);
        case Action::Ridged: return "R Ridged";
        case Action::Naive: return "N Naive";
    }
    return {};
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::Linear: return app.preset == 0;
        case Action::Valleys: return app.preset == 1;
        case Action::SCurve: return app.preset == 2;
        case Action::Terraces: return app.shape.terraces;
        case Action::Ridged: return app.shape.ridged;
        case Action::Naive: return app.shape.naive;
        default: return false;
    }
}

void setPreset(App& app, int preset) {
    app.preset = preset;
    app.curveY = preset == 0 ? ts::linearCurve() : (preset == 1 ? ts::valleysCurve() : ts::sCurve());
    app.drag = Drag::None;
    app.dirtyShape = true;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::Linear: setPreset(app, 0); break;
        case Action::Valleys: setPreset(app, 1); break;
        case Action::SCurve: setPreset(app, 2); break;
        case Action::NewSeed:
            app.seedCounter = (app.seedCounter + 1) % static_cast<int>(kSeeds.size());
            app.params.seed = kSeeds[static_cast<std::size_t>(app.seedCounter)];
            app.dirtyNoise = true;
            break;
        case Action::Octaves:
            app.params.octaves = app.params.octaves >= 7 ? 3 : app.params.octaves + 1;
            app.dirtyNoise = true;
            break;
        case Action::Terraces: app.shape.terraces = !app.shape.terraces; app.dirtyShape = true; break;
        case Action::Steps:
            app.shape.steps = app.shape.steps >= 10 ? 3 : app.shape.steps + 1;
            app.dirtyShape = true;
            break;
        case Action::Ridged: app.shape.ridged = !app.shape.ridged; app.dirtyShape = true; break;
        case Action::Naive: app.shape.naive = !app.shape.naive; app.dirtyShape = true; break;
    }
}

bool inRect(SDL_FPoint p, float x, float y, float w, float h) { return p.x >= x && p.x < x + w && p.y >= y && p.y < y + h; }

float nodeX(int i) { return kCurveX + static_cast<float>(i) / static_cast<float>(ts::kCurvePoints - 1) * kCurveS; }
float curveToY(double v) { return kCurveY + (1.0F - static_cast<float>(v)) * kCurveS; }

int nodeUnderMouse(const App& app, SDL_FPoint p) {
    int best = -1;
    float bestD = kNodeGrab;
    for (int i = 0; i < ts::kCurvePoints; ++i) {
        const float d = std::hypot(p.x - nodeX(i), p.y - curveToY(app.curveY[static_cast<std::size_t>(i)]));
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    }
    return best;
}

void dragNodeTo(App& app, float mouseY) {
    ts::setCurvePoint(app.curveY, app.dragNode, 1.0 - static_cast<double>((mouseY - kCurveY) / kCurveS));
    app.preset = -1;
    app.dirtyShape = true;
}

// ---- rebuild ------------------------------------------------------------------------------------------------------

void paint(App& app) {
    const auto at = [&](int x, int y) {
        x = std::clamp(x, 0, kGridW - 1);
        y = std::clamp(y, 0, kGridH - 1);
        return static_cast<double>(app.shaped[static_cast<std::size_t>(y * kGridW + x)]);
    };
    for (int y = 0; y < kGridH; ++y) {
        for (int x = 0; x < kGridW; ++x) {
            const double h = at(x, y);
            Rgb c;
            if (h < ts::kSeaLevel) {
                c = mix({kWater.r * 0.55F, kWater.g * 0.55F, kWater.b * 0.55F}, {110, 205, 200}, static_cast<float>(h / ts::kSeaLevel));
            } else {
                c = landColor(static_cast<float>((h - ts::kSeaLevel) / (1.0 - ts::kSeaLevel)));
                const auto light = static_cast<float>(ts::shade(at(x - 1, y), at(x + 1, y), at(x, y - 1), at(x, y + 1), kShadeScale));
                const float k = 0.45F + 0.65F * light;
                c = {c.r * k, c.g * k, c.b * k};
            }
            std::uint8_t* px = &app.pixels[static_cast<std::size_t>(y * kGridW + x) * 4];
            px[0] = toByte(c.r);
            px[1] = toByte(c.g);
            px[2] = toByte(c.b);
            px[3] = 255;
        }
    }
    SDL_UpdateTexture(app.map, nullptr, app.pixels.data(), kGridW * 4);
}

void rebuild(App& app) {
    if (app.dirtyNoise) {
        ts::computeFields(app.fields, app.params, kGridW, kGridH);
        app.dirtyShape = true;
        app.dirtyNoise = false;
    }
    if (!app.dirtyShape) return;
    app.shape.curve = ts::makeCurve(app.curveY);
    ts::shapeFields(app.fields, app.shape, app.shaped, app.before, app.after);
    // The statistics belong to the current picture: recomputed whenever the shape or the noise changes.
    std::size_t land = 0;
    double peak = 0.0;
    for (float v : app.shaped) {
        if (static_cast<double>(v) >= ts::kSeaLevel) ++land;
        peak = std::max(peak, static_cast<double>(v));
    }
    app.landPct = app.shaped.empty() ? 0.0 : 100.0 * static_cast<double>(land) / static_cast<double>(app.shaped.size());
    app.peak = peak;
    paint(app);
    app.dirtyShape = false;
}

// ---- drawing ------------------------------------------------------------------------------------------------------

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

void dashedH(SDL_Renderer* r, float x0, float x1, float y, gd::Color c) {
    for (float x = x0; x < x1; x += 8.0F) thickLine(r, x, y, std::min(x + 4.0F, x1), y, c, 0);
}

void drawCurve(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kCurveX - 4.0F, kCurveY - 4.0F, kCurveS + 8.0F, kCurveS + 8.0F, gd::kWhite, gd::kInk);
    for (int i = 1; i < 4; ++i) {
        const float o = kCurveS * static_cast<float>(i) / 4.0F;
        gd::fillRect(r, kCurveX + o, kCurveY, 1.0F, kCurveS, gd::kGrid);
        gd::fillRect(r, kCurveX, kCurveY + o, kCurveS, 1.0F, gd::kGrid);
    }
    // Identity (no change) and the sea level.
    thickLine(r, kCurveX, kCurveY + kCurveS, kCurveX + kCurveS, kCurveY, kInkSoft, 0);
    dashedH(r, kCurveX, kCurveX + kCurveS, curveToY(ts::kSeaLevel), kWater);

    const auto toPoint = [&](double x, double v) { return SDL_FPoint{kCurveX + static_cast<float>(x) * kCurveS, curveToY(v)}; };
    constexpr int kSegments = 64;
    SDL_FPoint prev = toPoint(0.0, app.shape.curve(0.0));
    SDL_FPoint prevT = toPoint(0.0, app.shape.terraces ? ts::terrace(app.shape.curve(0.0), app.shape.steps, app.shape.naive ? 0.0 : ts::kRiser) : 0.0);
    for (int i = 1; i <= kSegments; ++i) {
        const double x = static_cast<double>(i) / kSegments;
        const double v = app.shape.curve(x);
        const SDL_FPoint cur = toPoint(x, v);
        thickLine(r, prev.x, prev.y, cur.x, cur.y, gd::kTeal, 1);
        prev = cur;
        if (app.shape.terraces) {
            const SDL_FPoint t = toPoint(x, ts::terrace(v, app.shape.steps, app.shape.naive ? 0.0 : ts::kRiser));
            thickLine(r, prevT.x, prevT.y, t.x, t.y, gd::kOrange, 0);
            prevT = t;
        }
    }
    for (int i = 0; i < ts::kCurvePoints; ++i) {
        const float px = nodeX(i);
        const float py = curveToY(app.curveY[static_cast<std::size_t>(i)]);
        const bool hot = app.drag == Drag::Node ? app.dragNode == i : nodeUnderMouse(app, app.mouse) == i;
        gd::fillRect(r, px - 6.0F, py - 6.0F, 12.0F, 12.0F, hot ? gd::kTeal : gd::kWhite);
        gd::strokeRect(r, px - 6.0F, py - 6.0F, 12.0F, 12.0F, gd::kInk);
    }
    const char* name = app.preset == 0 ? "linear" : (app.preset == 1 ? "valleys" : (app.preset == 2 ? "s-curve" : "custom"));
    app.hud.text(r, kCurveX - 4.0F, 36.0F, "CURVE", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kCurveX - 4.0F + 60.0F, 36.0F, name, gd::kMuted);
    if (app.shape.terraces) app.hud.text(r, kCurveX + 4.0F, kCurveY + 4.0F, "orange: steps", gd::kOrange);
}

void drawHistogram(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kHistX - 4.0F, kHistY - 4.0F, kHistW + 8.0F, kHistH + 8.0F, gd::kWhite, gd::kInk);
    int top = 1;
    for (int v : app.before) top = std::max(top, v);
    for (int v : app.after) top = std::max(top, v);
    const float bw = kHistW / static_cast<float>(ts::kHistBins);
    for (int i = 0; i < ts::kHistBins; ++i) {
        const float x = kHistX + static_cast<float>(i) * bw;
        const float ha = kHistH * static_cast<float>(app.after[static_cast<std::size_t>(i)]) / static_cast<float>(top);
        const float hb = kHistH * static_cast<float>(app.before[static_cast<std::size_t>(i)]) / static_cast<float>(top);
        const bool water = (static_cast<double>(i) + 0.5) / ts::kHistBins < ts::kSeaLevel;
        if (ha >= 1.0F) gd::fillRect(r, x + 1.0F, kHistY + kHistH - ha, bw - 2.0F, ha, water ? gd::Color{14, 116, 144, 150} : gd::Color{13, 148, 136, 200});
        if (hb >= 1.0F) gd::strokeRect(r, x + 1.0F, kHistY + kHistH - hb, bw - 2.0F, hb, gd::kInk);
    }
    const float seaX = kHistX + static_cast<float>(ts::kSeaLevel) * kHistW;
    thickLine(r, seaX, kHistY, seaX, kHistY + kHistH, kWater, 0);
    app.hud.text(r, kHistX - 4.0F, 36.0F, "HEIGHTS", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kHistX - 4.0F + 80.0F, 36.0F, "outline: before", gd::kMuted);
    gd::fillRect(r, seaX + 2.0F, kHistY + 2.0F, 40.0F, 22.0F, gd::kWhite);  // keeps the label readable over the bars
    app.hud.text(r, seaX + 4.0F, kHistY + 2.0F, "sea", kWater);
}

void drawProfile(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kProfX - 4.0F, kProfY - 4.0F, kProfW + 8.0F, kProfH + 8.0F, gd::kWhite, gd::kInk);
    constexpr float kPad = 6.0F;
    const float base = kProfY + kProfH - kPad;
    const float span = kProfH - 2.0F * kPad;
    const float step = kProfW / static_cast<float>(kGridW);
    const std::size_t row = static_cast<std::size_t>(app.profileRow) * kGridW;
    const float seaY = base - static_cast<float>(ts::kSeaLevel) * span;
    for (int i = 0; i < kGridW; ++i) {
        const float x = kProfX + static_cast<float>(i) * step;
        const float h = base - app.shaped[row + static_cast<std::size_t>(i)] * span;
        if (h > seaY) {
            gd::fillRect(r, x, seaY, step + 0.5F, h - seaY, gd::Color{14, 116, 144, 60});  // water above the sea floor
            gd::fillRect(r, x, h, step + 0.5F, base - h, gd::Color{14, 116, 144, 90});
        } else {
            gd::fillRect(r, x, h, step + 0.5F, base - h, gd::kTealSoft);
        }
    }
    dashedH(r, kProfX, kProfX + kProfW, seaY, kWater);
    float px = kProfX;
    float pyRaw = base - app.fields.base[row] * span;
    float pyShaped = base - app.shaped[row] * span;
    for (int i = 1; i < kGridW; ++i) {
        const float x = kProfX + (static_cast<float>(i) + 0.5F) * step;
        const float yr = base - app.fields.base[row + static_cast<std::size_t>(i)] * span;
        const float ys = base - app.shaped[row + static_cast<std::size_t>(i)] * span;
        thickLine(r, px, pyRaw, x, yr, gd::kMuted, 0);
        thickLine(r, px, pyShaped, x, ys, gd::kInk, 1);
        px = x;
        pyRaw = yr;
        pyShaped = ys;
    }
    app.hud.text(r, kProfX - 4.0F, 232.0F, "CROSS-SECTION", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kProfX - 4.0F + 140.0F, 232.0F, "gray: noise  black: shaped", gd::kMuted);
}

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    const SDL_FRect dst{kMapX, kMapY, kMapW, kMapH};
    SDL_RenderTexture(r, app.map, nullptr, &dst);
    gd::strokeRect(r, kMapX - 1.0F, kMapY - 1.0F, kMapW + 2.0F, kMapH + 2.0F, gd::kInk);
    gd::strokeRect(r, kMapX - 2.0F, kMapY - 2.0F, kMapW + 4.0F, kMapH + 4.0F, gd::kInk);
    // The row of the cross-section.
    const float y = kMapY + (static_cast<float>(app.profileRow) + 0.5F) * static_cast<float>(kCellPx);
    for (float x = kMapX; x < kMapX + kMapW; x += 10.0F) thickLine(r, x, y, std::min(x + 6.0F, kMapX + kMapW), y, gd::Color{255, 255, 255, 230}, 0);
    for (float x = kMapX + 3.0F; x < kMapX + kMapW; x += 10.0F) thickLine(r, x, y + 1.0F, std::min(x + 6.0F, kMapX + kMapW), y + 1.0F, kInkSoft, 0);
}

void drawStatus(App& app) {
    SDL_Renderer* r = app.renderer;
    char line[96];
    app.hud.text(r, kMapX, 366.0F, "drag: pan   hover: cross-section row", gd::kMuted);
    SDL_snprintf(line, sizeof line, "land %.0f%%   highest point %.2f", app.landPct, app.peak);
    app.hud.text(r, kMapX, 388.0F, line, gd::kInk, gd::Hud::Style::Bold);
    if (app.shape.naive && (app.shape.ridged || app.shape.terraces)) {
        std::string text = app.shape.ridged ? "naive: ridges in the valleys too" : "";
        if (app.shape.terraces) text += app.shape.ridged ? ", vertical cliffs" : "naive: vertical cliffs, no slope";
        app.hud.text(r, kMapX, 410.0F, text, gd::kOrange);
    } else if (app.shape.ridged || app.shape.terraces) {
        app.hud.text(r, kMapX, 410.0F, app.shape.ridged ? "ridges only on the summits" : "sloped risers between terraces", gd::kTeal);
    }
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool naiveChip = c.action == Action::Naive;
        const gd::Color fill = on ? (naiveChip ? gd::kOrange : gd::kTeal) : gd::kWhite;
        const std::string label = chipLabel(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ HEIGHT CURVES AND TERRACES ]", "same noise, different shape");
    drawMap(app);
    drawStatus(app);
    drawCurve(app);
    drawHistogram(app);
    drawProfile(app);
    drawChips(app);
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
    if (!SDL_CreateWindowAndRenderer("terrain-shaping", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    app->map = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kGridW, kGridH);
    if (app->map == nullptr) {
        SDL_Log("map texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->map, SDL_SCALEMODE_NEAREST);
    app->pixels.assign(static_cast<std::size_t>(kGridW) * kGridH * 4, 255);
    app->params.seed = kSeeds[0];
    app->params.octaves = 5;
    app->shape.ridged = true;
    layoutChips(*app);
    rebuild(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = {event->motion.x, event->motion.y};
            if (app.drag == Drag::Pan) {
                const double perPx = app.params.span / static_cast<double>(kMapW);
                app.params.offX = std::clamp(app.params.offX - static_cast<double>(app.mouse.x - app.lastMouse.x) * perPx, -1e5, 1e5);
                app.params.offY = std::clamp(app.params.offY - static_cast<double>(app.mouse.y - app.lastMouse.y) * perPx, -1e5, 1e5);
                app.dirtyNoise = true;
            } else if (app.drag == Drag::Node) {
                dragNodeTo(app, app.mouse.y);
            }
            if (app.drag != Drag::Node && inRect(app.mouse, kMapX, kMapY, kMapW, kMapH)) {
                app.profileRow = std::clamp(static_cast<int>((app.mouse.y - kMapY) / static_cast<float>(kCellPx)), 0, kGridH - 1);
            }
            app.lastMouse = app.mouse;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            app.lastMouse = p;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inRect(p, kMapX, kMapY, kMapW, kMapH)) {
                app.drag = Drag::Pan;
                app.profileRow = std::clamp(static_cast<int>((p.y - kMapY) / static_cast<float>(kCellPx)), 0, kGridH - 1);
            } else if (const int node = nodeUnderMouse(app, p); node >= 0) {
                app.drag = Drag::Node;
                app.dragNode = node;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.drag = Drag::None; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Linear); break;
                case SDLK_2: apply(app, Action::Valleys); break;
                case SDLK_3: apply(app, Action::SCurve); break;
                case SDLK_SPACE: apply(app, Action::NewSeed); break;
                case SDLK_O: apply(app, Action::Octaves); break;
                case SDLK_T: apply(app, Action::Terraces); break;
                case SDLK_S: apply(app, Action::Steps); break;
                case SDLK_R: apply(app, Action::Ridged); break;
                case SDLK_N: apply(app, Action::Naive); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    rebuild(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->map != nullptr) SDL_DestroyTexture(app->map);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
