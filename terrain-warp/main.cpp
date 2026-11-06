// Domain warping with SDL3 main callbacks.
//
// A terrain height is f(p), noise sampled at the point p. Domain warping samples it somewhere else: f(p + s * q(p)),
// where q is a second noise field that says how far to look to the side. One more round (stage 2) feeds the first
// result into the second field. Left of the divider is the plain map, right of it the warped one, from the same seed.
// Hover the map to see where a pixel really reads its height: the line leads from the pixel to the point of the
// plain noise it looks at. The noise, the warp and the fold measure are in logic.hpp and covered by test.cpp.

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
constexpr float kMapX = 24.0F;
constexpr float kMapY = 44.0F;
constexpr int kCellPx = 2;  // one noise sample per 2x2 pixels, the texture is scaled up smoothly
constexpr int kCols = 312;
constexpr int kRows = 195;
constexpr float kMapW = static_cast<float>(kCols * kCellPx);
constexpr float kMapH = static_cast<float>(kRows * kCellPx);
static_assert(kCols * kCellPx == 624 && kRows * kCellPx == 390, "the map is 624x390 px, the panel and chips are laid out around it");
constexpr double kCellsPerMapHeight = 2.4;  // base lattice cells across the map height
constexpr double kUnitsPerPixel = kCellsPerMapHeight / static_cast<double>(kMapH);
constexpr warp::Vec2 kOrigin{0.6, 0.4};  // domain position of the map's top left corner
constexpr double kMaxStrength = 4.0;
constexpr double kSweepSeconds = 8.0;
constexpr double kPi = 3.14159265358979323846;

constexpr float kPanelX = 668.0F;
constexpr float kPanelY = 44.0F;
constexpr float kPanelW = 268.0F;
constexpr float kPanelH = 390.0F;
constexpr float kSliderX = 682.0F;
constexpr float kSliderW = 240.0F;
constexpr float kSliderY = 130.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kSeaLevel = 0.42F;
constexpr gd::Color kInkSoft{17, 24, 39, 200};

enum class Action { Stage0, Stage1, Stage2, Compare, Arrows, Animate, NewSeed };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

/// One rendered map: heights per sample and the shaded RGBA bytes made from them.
struct Layer {
    std::vector<float> heights = std::vector<float>(static_cast<std::size_t>(kCols * kRows), 0.0F);
    std::vector<Uint8> rgba = std::vector<Uint8>(static_cast<std::size_t>(kCols * kRows * 4), 0);
    SDL_Texture* texture = nullptr;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    warp::Params params;
    bool compare = true;
    bool arrows = false;
    bool animate = false;
    double phase = 0.0;
    float split = kMapX + kMapW / 2.0F;
    bool dragSplit = false;
    bool dragSlider = false;
    Layer plain;
    Layer warped;
    bool dirtyPlain = true;
    bool dirtyWarped = true;
    double folded = 0.0;
    SDL_FPoint mouse{-100.0F, -100.0F};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

warp::Vec2 toDomain(float px, float py) {
    return {kOrigin.x + static_cast<double>(px - kMapX) * kUnitsPerPixel, kOrigin.y + static_cast<double>(py - kMapY) * kUnitsPerPixel};
}

SDL_FPoint toPixel(warp::Vec2 p) {
    return {kMapX + static_cast<float>((p.x - kOrigin.x) / kUnitsPerPixel), kMapY + static_cast<float>((p.y - kOrigin.y) / kUnitsPerPixel)};
}

bool inMap(SDL_FPoint p) { return p.x >= kMapX && p.x < kMapX + kMapW && p.y >= kMapY && p.y < kMapY + kMapH; }

struct Rgb {
    float r, g, b;
};

Rgb mix(Rgb a, Rgb b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }

/// Height to a flat map color: shades of blue under the sea level, then sand, green, rock and snow.
Rgb terrainColor(float h) {
    if (h < kSeaLevel) return mix({26, 88, 138}, {120, 196, 206}, std::clamp(h / kSeaLevel, 0.0F, 1.0F));
    const float t = std::clamp((h - kSeaLevel) / (1.0F - kSeaLevel), 0.0F, 1.0F);
    constexpr std::array<Rgb, 6> kStops{{{226, 214, 168}, {150, 190, 106}, {96, 150, 84}, {150, 126, 92}, {170, 165, 160}, {246, 246, 244}}};
    constexpr std::array<float, 6> kAt{{0.0F, 0.06F, 0.30F, 0.55F, 0.75F, 1.0F}};
    for (std::size_t i = 1; i < kStops.size(); ++i) {
        if (t <= kAt[i]) return mix(kStops[i - 1], kStops[i], (t - kAt[i - 1]) / (kAt[i] - kAt[i - 1]));
    }
    return kStops.back();
}

float heightAt(const Layer& layer, int x, int y) {
    return layer.heights[static_cast<std::size_t>(std::clamp(y, 0, kRows - 1) * kCols + std::clamp(x, 0, kCols - 1))];
}

/// Fills the heights from the parameters, then shades with a light from the top left (slope between the neighbors).
void render(Layer& layer, const warp::Params& params) {
    for (int j = 0; j < kRows; ++j) {
        for (int i = 0; i < kCols; ++i) {
            const warp::Vec2 p{kOrigin.x + (i + 0.5) * kCellPx * kUnitsPerPixel, kOrigin.y + (j + 0.5) * kCellPx * kUnitsPerPixel};
            layer.heights[static_cast<std::size_t>(j * kCols + i)] = static_cast<float>(warp::height(p, params));
        }
    }
    for (int j = 0; j < kRows; ++j) {
        for (int i = 0; i < kCols; ++i) {
            const float h = heightAt(layer, i, j);
            const float slope = (heightAt(layer, i + 1, j) - heightAt(layer, i - 1, j)) + (heightAt(layer, i, j + 1) - heightAt(layer, i, j - 1));
            const float light = h < kSeaLevel ? 1.0F + std::clamp(slope * 6.0F, -0.1F, 0.1F) : 1.0F + std::clamp(slope * 20.0F, -0.45F, 0.45F);
            const Rgb c = terrainColor(h);
            Uint8* px = &layer.rgba[static_cast<std::size_t>((j * kCols + i) * 4)];
            px[0] = static_cast<Uint8>(std::clamp(c.r * light, 0.0F, 255.0F));
            px[1] = static_cast<Uint8>(std::clamp(c.g * light, 0.0F, 255.0F));
            px[2] = static_cast<Uint8>(std::clamp(c.b * light, 0.0F, 255.0F));
            px[3] = 255;
        }
    }
}

bool upload(Layer& layer) { return SDL_UpdateTexture(layer.texture, nullptr, layer.rgba.data(), kCols * 4); }

bool refresh(App& app) {
    if (app.dirtyPlain) {
        warp::Params plainParams = app.params;
        plainParams.stages = 0;
        render(app.plain, plainParams);
        if (!upload(app.plain)) return false;
        app.dirtyPlain = false;
    }
    if (app.dirtyWarped) {
        render(app.warped, app.params);
        if (!upload(app.warped)) return false;
        const warp::Params p = app.params;
        app.folded = warp::foldedFraction([&p](warp::Vec2 q) { return warp::warpPoint(q, p); }, kOrigin.x, kOrigin.y,
                                          kOrigin.x + static_cast<double>(kMapW) * kUnitsPerPixel, kOrigin.y + kCellsPerMapHeight, 48, 30);
        app.dirtyWarped = false;
    }
    return true;
}

void setStrength(App& app, double value) {
    app.params.strength = std::clamp(value, 0.0, kMaxStrength);
    app.dirtyWarped = true;
}

/// The user takes over from the sweep.
void setStrengthByHand(App& app, double value) {
    app.animate = false;
    setStrength(app, value);
}

void setAnimate(App& app, bool on) {
    app.animate = on;
    if (on) {
        // Start the sweep where the slider is, so the map does not jump: strength = max * (1 - cos(phase)) / 2.
        app.phase = std::acos(std::clamp(1.0 - 2.0 * app.params.strength / kMaxStrength, -1.0, 1.0));
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Stage0:
        case Action::Stage1:
        case Action::Stage2:
            app.params.stages = action == Action::Stage0 ? 0 : (action == Action::Stage1 ? 1 : 2);
            app.dirtyWarped = true;
            break;
        case Action::Compare: app.compare = !app.compare; break;
        case Action::Arrows: app.arrows = !app.arrows; break;
        case Action::Animate: setAnimate(app, !app.animate); break;
        case Action::NewSeed:
            app.params.seed = app.params.seed * 31U + 11U;
            app.dirtyPlain = true;
            app.dirtyWarped = true;
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Stage0: return app.params.stages == 0;
        case Action::Stage1: return app.params.stages == 1;
        case Action::Stage2: return app.params.stages == 2;
        case Action::Compare: return app.compare;
        case Action::Arrows: return app.arrows;
        case Action::Animate: return app.animate;
        case Action::NewSeed: return false;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::Stage0, "1 Plain", 0.0F},
                                   {Action::Stage1, "2 Warp ×1", 8.0F},
                                   {Action::Stage2, "3 Warp ×2", 8.0F},
                                   {Action::Compare, "C Compare", 24.0F},
                                   {Action::Arrows, "V Arrows", 8.0F},
                                   {Action::Animate, "A Animate", 8.0F},
                                   {Action::NewSeed, "N New seed", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void thickLine(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, a.x + (flat ? 0.0F : off), a.y + (flat ? off : 0.0F), b.x + (flat ? 0.0F : off), b.y + (flat ? off : 0.0F));
    }
}

void tag(App& app, float x, float y, const char* label, gd::Color fill, bool alignRight) {
    const float w = app.hud.width(label, gd::Hud::Style::Bold) + 16.0F;
    const float left = alignRight ? x - w : x;
    gd::card(app.renderer, left, y, w, 26.0F, fill, gd::kInk);
    app.hud.text(app.renderer, left + 8.0F, y + 2.0F, label, gd::kWhite, gd::Hud::Style::Bold);
}

std::string hundredths(double value) {
    const int v = static_cast<int>(std::lround(value * 100.0));
    char buf[16];
    SDL_snprintf(buf, sizeof buf, "%d.%02d", v / 100, v % 100);
    return buf;
}

int splitCell(const App& app) {
    return app.compare ? std::clamp(static_cast<int>(std::lround((app.split - kMapX) / static_cast<float>(kCellPx))), 0, kCols) : 0;
}

/// A 72x72 px cut-out of the plain map around the point the pixel reads from, so the source is visible even when it lies
/// on the warped side of the divider. Its center is the plain height the hovered pixel shows.
void drawLens(App& app, SDL_FPoint source) {
    constexpr float kLens = 72.0F;
    constexpr float kHalfTexels = kLens / 2.0F / static_cast<float>(kCellPx);
    const float sx = std::clamp((source.x - kMapX) / static_cast<float>(kCellPx) - kHalfTexels, 0.0F, static_cast<float>(kCols) - 2.0F * kHalfTexels);
    const float sy = std::clamp((source.y - kMapY) / static_cast<float>(kCellPx) - kHalfTexels, 0.0F, static_cast<float>(kRows) - 2.0F * kHalfTexels);
    const SDL_FRect src{sx, sy, 2.0F * kHalfTexels, 2.0F * kHalfTexels};
    float lx = app.mouse.x + 26.0F;
    if (lx + kLens > kMapX + kMapW - 4.0F) lx = app.mouse.x - 26.0F - kLens;
    const float ly = std::clamp(app.mouse.y - kLens - 18.0F, kMapY + 4.0F, kMapY + kMapH - kLens - 4.0F);
    const SDL_FRect dst{lx, ly, kLens, kLens};
    SDL_RenderTexture(app.renderer, app.plain.texture, &src, &dst);
    gd::strokeRect(app.renderer, lx - 1.0F, ly - 1.0F, kLens + 2.0F, kLens + 2.0F, gd::kOrange);
    gd::strokeRect(app.renderer, lx - 2.0F, ly - 2.0F, kLens + 4.0F, kLens + 4.0F, gd::kInk);
    // The crosshair marks the source point inside the cut-out (it moves off-center at the map edge).
    const float cx = lx + (source.x - kMapX) - sx * static_cast<float>(kCellPx);
    const float cy = ly + (source.y - kMapY) - sy * static_cast<float>(kCellPx);
    thickLine(app.renderer, {cx - 8.0F, cy}, {cx + 8.0F, cy}, gd::kWhite, 2);
    thickLine(app.renderer, {cx, cy - 8.0F}, {cx, cy + 8.0F}, gd::kWhite, 2);
    thickLine(app.renderer, {cx - 8.0F, cy}, {cx + 8.0F, cy}, gd::kOrange, 1);
    thickLine(app.renderer, {cx, cy - 8.0F}, {cx, cy + 8.0F}, gd::kOrange, 1);
}

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    const int cut = splitCell(app);
    if (cut > 0) {
        const SDL_FRect src{0.0F, 0.0F, static_cast<float>(cut), static_cast<float>(kRows)};
        const SDL_FRect dst{kMapX, kMapY, static_cast<float>(cut * kCellPx), kMapH};
        SDL_RenderTexture(r, app.plain.texture, &src, &dst);
    }
    if (cut < kCols) {
        const SDL_FRect src{static_cast<float>(cut), 0.0F, static_cast<float>(kCols - cut), static_cast<float>(kRows)};
        const SDL_FRect dst{kMapX + static_cast<float>(cut * kCellPx), kMapY, static_cast<float>((kCols - cut) * kCellPx), kMapH};
        SDL_RenderTexture(r, app.warped.texture, &src, &dst);
    }
    const float edgeX = kMapX + static_cast<float>(cut * kCellPx);
    const SDL_Rect clip{static_cast<int>(kMapX), static_cast<int>(kMapY), static_cast<int>(kMapW), static_cast<int>(kMapH)};
    SDL_SetRenderClipRect(r, &clip);

    // Arrows: the offset of the lookup, drawn at a third of its length so they do not run into each other.
    if (app.arrows && cut < kCols) {
        constexpr float kStep = 39.0F;
        constexpr float kScale = 0.33F;
        for (float y = kMapY + kStep / 2.0F; y < kMapY + kMapH; y += kStep) {
            for (float x = kMapX + kStep / 2.0F; x < kMapX + kMapW; x += kStep) {
                if (x < edgeX + 4.0F) continue;
                const warp::Vec2 p = toDomain(x, y);
                const SDL_FPoint to = toPixel(warp::warpPoint(p, app.params));
                const SDL_FPoint tip{x + (to.x - x) * kScale, y + (to.y - y) * kScale};
                const float len = std::hypot(tip.x - x, tip.y - y);
                thickLine(r, {x, y}, tip, kInkSoft, 0);
                gd::disc(r, x, y, 2.5F, gd::kInk, 12);
                if (len > 5.0F) {
                    const float ux = (tip.x - x) / len;
                    const float uy = (tip.y - y) / len;
                    thickLine(r, tip, {tip.x - ux * 6.0F - uy * 3.5F, tip.y - uy * 6.0F + ux * 3.5F}, kInkSoft, 0);
                    thickLine(r, tip, {tip.x - ux * 6.0F + uy * 3.5F, tip.y - uy * 6.0F - ux * 3.5F}, kInkSoft, 0);
                }
            }
        }
    }

    // Hover: the pixel under the mouse and the point of the plain noise it reads its height from.
    if (inMap(app.mouse) && app.mouse.x >= edgeX) {
        const SDL_FPoint to = toPixel(warp::warpPoint(toDomain(app.mouse.x, app.mouse.y), app.params));
        thickLine(r, app.mouse, to, gd::kWhite, 2);
        thickLine(r, app.mouse, to, gd::kOrange, 1);
        gd::disc(r, to.x, to.y, 6.0F, gd::kOrange);
        gd::ring(r, to.x, to.y, 6.0F, gd::kInk);
        drawLens(app, to);
        gd::disc(r, app.mouse.x, app.mouse.y, 6.0F, gd::kTeal);
        gd::ring(r, app.mouse.x, app.mouse.y, 6.0F, gd::kInk);
    }

    if (app.compare) {
        gd::fillRect(r, edgeX - 1.0F, kMapY, 2.0F, kMapH, gd::kInk);
        gd::card(r, edgeX - 14.0F, kMapY + kMapH / 2.0F - 17.0F, 28.0F, 34.0F, gd::kWhite, gd::kInk);
        app.hud.text(r, edgeX - 10.0F, kMapY + kMapH / 2.0F - 11.0F, "<>", gd::kInk, gd::Hud::Style::Bold);
    }
    SDL_SetRenderClipRect(r, nullptr);
    gd::strokeRect(r, kMapX, kMapY, kMapW, kMapH, gd::kInk);

    if (!app.compare) {
        tag(app, kMapX + 8.0F, kMapY + 8.0F, app.params.stages == 0 ? "PLAIN" : "WARPED", app.params.stages == 0 ? gd::kOrange : gd::kTeal, false);
    } else {
        if (cut > 14) tag(app, kMapX + 8.0F, kMapY + 8.0F, "PLAIN", gd::kOrange, false);
        if (cut < kCols - 20) tag(app, kMapX + kMapW - 8.0F, kMapY + 8.0F, "WARPED", gd::kTeal, true);
    }
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, kPanelY, kPanelW, kPanelH, gd::kWhite, gd::kInk);
    constexpr float x = kPanelX + 14.0F;
    app.hud.text(r, x, 54.0F, "STRENGTH s", gd::kMuted);
    app.hud.text(r, x, 74.0F, hundredths(app.params.strength), gd::kInk, gd::Hud::Style::Large);

    // Slider.
    gd::fillRect(r, kSliderX, kSliderY, kSliderW, 8.0F, gd::kGrid);
    gd::strokeRect(r, kSliderX, kSliderY, kSliderW, 8.0F, gd::kInk);
    const float fillW = kSliderW * static_cast<float>(app.params.strength / kMaxStrength);
    gd::fillRect(r, kSliderX + 1.0F, kSliderY + 1.0F, std::max(0.0F, fillW - 2.0F), 6.0F, gd::kTeal);
    gd::card(r, kSliderX + fillW - 7.0F, kSliderY - 8.0F, 14.0F, 24.0F, gd::kWhite, gd::kInk);

    app.hud.text(r, x, 170.0F, "LOOKUP", gd::kMuted);
    const int stages = app.params.stages;
    app.hud.text(r, x, 192.0F, stages == 0 ? "f(p)" : (stages == 1 ? "f(p + s×q)" : "f(p + s×r)"), gd::kTealDark, gd::Hud::Style::Bold);
    app.hud.text(r, x, 216.0F, stages == 0 ? "no warp, plain noise" : "q = noise2(p)", gd::kInk);
    if (stages == 2) app.hud.text(r, x, 238.0F, "r = noise2(p + s×q)", gd::kInk);

    app.hud.text(r, x, 272.0F, "FOLDED", gd::kMuted);
    const bool tearing = app.folded >= 0.3;
    char pct[16];
    SDL_snprintf(pct, sizeof pct, "%d%%", static_cast<int>(std::lround(app.folded * 100.0)));
    app.hud.text(r, x, 292.0F, pct, tearing ? gd::kOrange : gd::kTeal, gd::Hud::Style::Large);
    gd::fillRect(r, kSliderX, 342.0F, kSliderW, 12.0F, gd::kGrid);
    gd::fillRect(r, kSliderX, 342.0F, kSliderW * static_cast<float>(std::clamp(app.folded, 0.0, 1.0)), 12.0F, tearing ? gd::kOrange : gd::kTeal);
    gd::strokeRect(r, kSliderX, 342.0F, kSliderW, 12.0F, gd::kInk);
    app.hud.text(r, x, 366.0F, "where the lookup runs", gd::kMuted);
    app.hud.text(r, x, 388.0F, "backwards (tears)", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    const int stages = app.params.stages;
    app.hud.titleBar(r, kWidth, "[ DOMAIN WARPING ]", stages == 0 ? "f(p)" : (stages == 1 ? "f(p + s×q(p))" : "f(p + s×r(p))"));
    drawMap(app);
    drawPanel(app);
    app.hud.text(r, 40.0F, 446.0F, "hover: where a pixel reads its height   drag map: divider   Left/Right: strength", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void sliderToStrength(App& app, float x) {
    setStrengthByHand(app, static_cast<double>((x - kSliderX) / kSliderW) * kMaxStrength);
}

bool onSlider(SDL_FPoint p) { return p.x >= kSliderX - 10.0F && p.x <= kSliderX + kSliderW + 10.0F && p.y >= kSliderY - 14.0F && p.y <= kSliderY + 22.0F; }

void releaseDrags(App& app) {
    app.dragSplit = false;
    app.dragSlider = false;
}

void setSplit(App& app, float x) { app.split = std::clamp(x, kMapX, kMapX + kMapW); }

void destroyTextures(App& app) {
    for (Layer* layer : {&app.plain, &app.warped}) {
        if (layer->texture != nullptr) SDL_DestroyTexture(layer->texture);
        layer->texture = nullptr;
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
    if (!SDL_CreateWindowAndRenderer("terrain-warp", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    for (Layer* layer : {&app->plain, &app->warped}) {
        layer->texture = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kCols, kRows);
        if (layer->texture == nullptr) {
            SDL_Log("map texture failed: %s", SDL_GetError());
            return SDL_APP_FAILURE;
        }
        SDL_SetTextureScaleMode(layer->texture, SDL_SCALEMODE_LINEAR);
    }
    layoutChips(*app);
    app->lastTicks = SDL_GetTicks();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = {event->motion.x, event->motion.y};
            if ((event->motion.state & SDL_BUTTON_LMASK) == 0) {
                releaseDrags(app);  // the button went up outside the window or the page
            } else if (app.dragSlider) {
                sliderToStrength(app, app.mouse.x);
            } else if (app.dragSplit) {
                setSplit(app, app.mouse.x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                }
            }
            if (onChip) break;
            if (onSlider(p)) {
                app.dragSlider = true;
                sliderToStrength(app, p.x);
            } else if (app.compare && inMap(p)) {
                app.dragSplit = true;
                setSplit(app, p.x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) releaseDrags(app);
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            releaseDrags(app);
            app.mouse = {-100.0F, -100.0F};
            break;
        case SDL_EVENT_KEY_DOWN: {
            const bool fresh = !event->key.repeat;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: if (fresh) apply(app, Action::Stage0); break;
                case SDLK_2: if (fresh) apply(app, Action::Stage1); break;
                case SDLK_3: if (fresh) apply(app, Action::Stage2); break;
                case SDLK_C: if (fresh) apply(app, Action::Compare); break;
                case SDLK_V: if (fresh) apply(app, Action::Arrows); break;
                case SDLK_A: if (fresh) apply(app, Action::Animate); break;
                case SDLK_N: if (fresh) apply(app, Action::NewSeed); break;
                case SDLK_LEFT: setStrengthByHand(app, std::round(app.params.strength * 10.0 - 1.0) / 10.0); break;
                case SDLK_RIGHT: setStrengthByHand(app, std::round(app.params.strength * 10.0 + 1.0) / 10.0); break;
                default: break;
            }
            break;
        }
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicks();
    const double dt = std::min(0.25, static_cast<double>(now - app.lastTicks) / 1000.0);
    app.lastTicks = now;
    if (app.animate) {
        app.phase = std::fmod(app.phase + dt * 2.0 * kPi / kSweepSeconds, 2.0 * kPi);
        setStrength(app, kMaxStrength * (1.0 - std::cos(app.phase)) / 2.0);
    }
    if (!refresh(app)) {
        SDL_Log("map upload failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        destroyTextures(*app);
        app->hud.destroy();
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
