// Terrain from three noise layers with SDL3 main callbacks.
//
// One layer of noise gives clouds. A believable landscape needs layers of different size: a slow continent layer
// (land or sea), a ridged mountain layer and a fast detail layer. How they are put together decides how it looks.
// Plain adding gives every place the same relief, so mountain ranges stand in the sea. A mask takes the continent
// value and lets the mountains in only where the land is high. The demo blends the same three layers four ways and
// measures the relief on the lowlands against the highlands, so the difference is a number as well as a picture.
// All the math (noise, blend modes, mask, statistics, shading) lives in logic.hpp and is covered by test.cpp.

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
constexpr int kGridW = 288;
constexpr int kGridH = 192;
constexpr int kThumbFactor = 3;  // layer thumbnails are the grid divided by this: 96 x 64

constexpr float kMapX = 24.0F;
constexpr float kMapY = 44.0F;
constexpr float kMapW = 576.0F;
constexpr float kMapH = 384.0F;

constexpr float kPanelX = 624.0F;
constexpr float kCardW = 312.0F;
constexpr float kCardH = 70.0F;
constexpr float kCardPitch = 76.0F;
constexpr float kSliderY0 = 352.0F;
constexpr float kSliderPitch = 40.0F;
constexpr float kTrackX0 = 636.0F;
constexpr float kTrackX1 = 924.0F;

constexpr float kStatsY = 436.0F;
constexpr float kChipH = 30.0F;
constexpr float kChipRow1 = 464.0F;
constexpr float kChipRow2 = 500.0F;

constexpr float kShadeExaggeration = 18.0F;
constexpr float kArrowStep = 0.1F;  // world units per arrow key press

enum class View { Terrain, Height, Relief };
enum class Action { ToggleContinent, ToggleMountains, ToggleDetail, ModeAdd, ModeMultiply, ModeMask, ModeLerp, ViewTerrain, ViewHeight, ViewRelief, NewSeed, Overlay };
enum class Drag { None, Pan, Threshold, Softness };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    SDL_Texture* mapTex = nullptr;
    std::array<SDL_Texture*, 4> thumbTex{};  // continent, mountains, detail, mask
    std::vector<Chip> chips;
    std::vector<Uint8> pixels;
    std::vector<Uint8> thumbPixels;  // scratch for the gray thumbnails, reused

    tl::Layers layers;
    tl::Field field;
    tl::Relief relief;
    tl::Enabled enabled;
    tl::Mode mode = tl::Mode::Mask;
    View view = View::Terrain;
    bool overlay = false;
    float threshold = tl::kDefaultThreshold;
    float softness = tl::kDefaultSoftness;
    std::uint32_t seed = tl::kDefaultSeed;
    float ox = 0.0F;
    float oy = 0.0F;
    bool layersDirty = true;
    bool blendDirty = true;

    Drag drag = Drag::None;
    SDL_FPoint last{0.0F, 0.0F};
    long maxFrames = -1;
    long frames = 0;
};

bool inRect(SDL_FPoint p, float x, float y, float w, float h) { return p.x >= x && p.x < x + w && p.y >= y && p.y < y + h; }

bool usesMask(const App& app) { return app.mode == tl::Mode::Mask || app.mode == tl::Mode::Lerp; }

// The overlay is orange, so it would fight with the orange of the relief view; and it only means something when a mask is used.
bool overlayVisible(const App& app) { return app.overlay && usesMask(app) && app.view != View::Relief; }

bool layerOn(const App& app, int i) { return i == 0 ? app.enabled.continent : i == 1 ? app.enabled.mountains : app.enabled.detail; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float row;
    };
    const std::array<Def, 9> defs{{{Action::ModeAdd, "4 Add", 0.0F, kChipRow1},
                                    {Action::ModeMultiply, "5 Multiply", 8.0F, kChipRow1},
                                    {Action::ModeMask, "6 Mask", 8.0F, kChipRow1},
                                    {Action::ModeLerp, "7 Lerp", 8.0F, kChipRow1},
                                    {Action::NewSeed, "N New seed", 36.0F, kChipRow1},
                                    {Action::ViewTerrain, "T Terrain", 0.0F, kChipRow2},
                                    {Action::ViewHeight, "H Height", 8.0F, kChipRow2},
                                    {Action::ViewRelief, "R Relief", 8.0F, kChipRow2},
                                    {Action::Overlay, "O Mask overlay", 24.0F, kChipRow2}}};
    float x = 0.0F;
    float rowY = -1.0F;
    for (const Def& d : defs) {
        if (d.row != rowY) {
            rowY = d.row;
            x = 98.0F;  // after the row label
        } else {
            x += d.gapBefore;
        }
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.row, w, kChipH}});
        x += w;
    }
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::ToggleContinent: return app.enabled.continent;
        case Action::ToggleMountains: return app.enabled.mountains;
        case Action::ToggleDetail: return app.enabled.detail;
        case Action::ModeAdd: return app.mode == tl::Mode::Add;
        case Action::ModeMultiply: return app.mode == tl::Mode::Multiply;
        case Action::ModeMask: return app.mode == tl::Mode::Mask;
        case Action::ModeLerp: return app.mode == tl::Mode::Lerp;
        case Action::ViewTerrain: return app.view == View::Terrain;
        case Action::ViewHeight: return app.view == View::Height;
        case Action::ViewRelief: return app.view == View::Relief;
        case Action::Overlay: return app.overlay;
        case Action::NewSeed: return false;
    }
    return false;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::ToggleContinent: app.enabled.continent = !app.enabled.continent; app.blendDirty = true; break;
        case Action::ToggleMountains: app.enabled.mountains = !app.enabled.mountains; app.blendDirty = true; break;
        case Action::ToggleDetail: app.enabled.detail = !app.enabled.detail; app.blendDirty = true; break;
        case Action::ModeAdd: app.mode = tl::Mode::Add; app.blendDirty = true; break;
        case Action::ModeMultiply: app.mode = tl::Mode::Multiply; app.blendDirty = true; break;
        case Action::ModeMask: app.mode = tl::Mode::Mask; app.blendDirty = true; break;
        case Action::ModeLerp: app.mode = tl::Mode::Lerp; app.blendDirty = true; break;
        case Action::ViewTerrain: app.view = View::Terrain; app.blendDirty = true; break;
        case Action::ViewHeight: app.view = View::Height; app.blendDirty = true; break;
        case Action::ViewRelief: app.view = View::Relief; app.blendDirty = true; break;
        case Action::Overlay: app.overlay = !app.overlay; app.blendDirty = true; break;
        case Action::NewSeed: ++app.seed; app.layersDirty = true; break;
    }
}

float& sliderValueRef(App& app, int i) { return i == 0 ? app.threshold : app.softness; }
float sliderLo(int i) { return i == 0 ? 0.10F : 0.0F; }
float sliderHi(int i) { return i == 0 ? 0.90F : 0.40F; }
float sliderTop(int i) { return kSliderY0 + kSliderPitch * static_cast<float>(i); }

void setSliderFromMouse(App& app, int i, float mouseX) {
    sliderValueRef(app, i) = tl::sliderValue(mouseX, kTrackX0, kTrackX1, sliderLo(i), sliderHi(i));
    app.blendDirty = true;
}

void uploadGray(App& app, SDL_Texture* tex, const tl::Grid& g) {
    if (tex == nullptr || g.v.empty()) return;
    app.thumbPixels.resize(g.v.size() * 4);
    for (std::size_t i = 0; i < g.v.size(); ++i) {
        const auto v = static_cast<Uint8>(std::lround(tl::clamp01(g.v[i]) * 255.0F));
        app.thumbPixels[i * 4 + 0] = v;
        app.thumbPixels[i * 4 + 1] = v;
        app.thumbPixels[i * 4 + 2] = v;
        app.thumbPixels[i * 4 + 3] = 255;
    }
    SDL_UpdateTexture(tex, nullptr, app.thumbPixels.data(), g.w * 4);
}

void paintMap(App& app) {
    const tl::Field& f = app.field;
    if (f.w <= 0 || f.h <= 0) return;
    app.pixels.resize(static_cast<std::size_t>(f.w) * static_cast<std::size_t>(f.h) * 4);
    const bool tint = overlayVisible(app);
    for (int y = 0; y < f.h; ++y) {
        for (int x = 0; x < f.w; ++x) {
            const auto i = static_cast<std::size_t>(y) * static_cast<std::size_t>(f.w) + static_cast<std::size_t>(x);
            const float h = f.height[i];
            tl::Rgb c{};
            switch (app.view) {
                case View::Terrain: {
                    c = tl::terrainColor(h);
                    if (h >= tl::kSea) {
                        const float s = std::clamp(tl::shadeAt(f, x, y, kShadeExaggeration), 0.35F, 1.5F);
                        const auto ch = [s](std::uint8_t v) { return static_cast<std::uint8_t>(std::clamp(static_cast<float>(v) * s, 0.0F, 255.0F)); };
                        c = {ch(c.r), ch(c.g), ch(c.b)};
                    }
                    break;
                }
                case View::Height: c = tl::heightColor(h); break;
                case View::Relief: c = h >= tl::kSea ? tl::reliefColor(tl::slopeAt(f, x, y)) : tl::Rgb{196, 210, 220}; break;
            }
            if (tint && h >= tl::kSea) c = tl::mix(c, tl::Rgb{234, 88, 12}, 0.55F * (1.0F - f.mask[i]));
            const std::size_t o = i * 4;
            app.pixels[o + 0] = c.r;
            app.pixels[o + 1] = c.g;
            app.pixels[o + 2] = c.b;
            app.pixels[o + 3] = 255;
        }
    }
    SDL_UpdateTexture(app.mapTex, nullptr, app.pixels.data(), f.w * 4);
}

void refresh(App& app) {
    if (app.layersDirty) {
        tl::fillLayers(app.layers, kGridW, kGridH, app.seed, app.ox, app.oy, tl::kViewSpan);
        uploadGray(app, app.thumbTex[0], tl::downsample(app.layers.c, kGridW, kGridH, kThumbFactor));
        uploadGray(app, app.thumbTex[1], tl::downsample(app.layers.m, kGridW, kGridH, kThumbFactor));
        uploadGray(app, app.thumbTex[2], tl::downsample(app.layers.d, kGridW, kGridH, kThumbFactor));
        app.layersDirty = false;
        app.blendDirty = true;
    }
    if (app.blendDirty) {
        tl::blendField(app.layers, app.enabled, app.mode, app.threshold, app.softness, app.field);
        uploadGray(app, app.thumbTex[3], tl::downsample(app.field.mask, kGridW, kGridH, kThumbFactor));
        app.relief = tl::reliefStats(app.field);
        paintMap(app);
        app.blendDirty = false;
    }
}

void formulaLines(const App& app, std::string& line1, std::string& line2) {
    char a[96];
    switch (app.mode) {
        case tl::Mode::Add: SDL_snprintf(a, sizeof a, "ADD  h = %.1f + %.1f c + %.1f m + detail", tl::kBase, tl::kContinentWeight, tl::kMountainWeight); break;
        case tl::Mode::Multiply:
            SDL_snprintf(a, sizeof a, "MULTIPLY  h = (%.1f + %.1f c) * (%.1f + %.1f m) + detail", tl::kBase, tl::kContinentWeight, tl::kMultBase, tl::kMultGain);
            break;
        case tl::Mode::Mask: SDL_snprintf(a, sizeof a, "MASK  h = %.1f + %.1f c + mask * %.1f m + detail", tl::kBase, tl::kContinentWeight, tl::kMountainWeight); break;
        case tl::Mode::Lerp: SDL_snprintf(a, sizeof a, "LERP  h = lerp(plains, add, mask) + detail"); break;
    }
    line1 = a;
    line2.clear();
    if (usesMask(app)) {
        char b[96];
        SDL_snprintf(b, sizeof b, "mask = smoothstep(%.2f, %.2f, c)", app.threshold - app.softness, app.threshold + app.softness);
        line2 = b;
    }
}

void drawLayerCard(App& app, int i) {
    SDL_Renderer* r = app.renderer;
    static const char* const kNames[4] = {"1 Continent", "2 Mountains", "3 Detail", "Mask"};
    static const char* const kInfo[4] = {"slow fBm, land/sea", "ridged fBm, crests", "fast fBm, rough", "smoothstep(c)"};
    const float y = kMapY + kCardPitch * static_cast<float>(i);
    const bool toggle = i < 3;
    const bool on = toggle ? layerOn(app, i) : true;
    gd::card(r, kPanelX, y, kCardW, kCardH, on ? gd::kWhite : gd::kGrid, gd::kInk);
    SDL_Texture* tex = app.thumbTex[static_cast<std::size_t>(i)];
    if (tex != nullptr) {
        SDL_SetTextureAlphaMod(tex, on ? 255 : 70);
        const SDL_FRect dst{kPanelX + 4.0F, y + 3.0F, 96.0F, 64.0F};
        SDL_RenderTexture(r, tex, nullptr, &dst);
    }
    gd::strokeRect(r, kPanelX + 4.0F, y + 3.0F, 96.0F, 64.0F, gd::kInk);
    const gd::Color text = on ? gd::kInk : gd::kMuted;
    app.hud.text(r, kPanelX + 110.0F, y + 6.0F, kNames[i], text, gd::Hud::Style::Bold);
    app.hud.text(r, kPanelX + 110.0F, y + 30.0F, kInfo[i], gd::kMuted);
    if (toggle) {
        const char* label = on ? "ON" : "OFF";
        const float bw = app.hud.width(label, gd::Hud::Style::Bold) + 12.0F;
        gd::fillRect(r, kPanelX + kCardW - bw - 8.0F, y + 6.0F, bw, 18.0F, on ? gd::kTeal : gd::kMuted);
        app.hud.text(r, kPanelX + kCardW - bw - 2.0F, y + 6.0F, label, gd::kWhite, gd::Hud::Style::Bold);
    } else {
        app.hud.text(r, kPanelX + 110.0F, y + 48.0F, usesMask(app) ? "in use" : "unused here", usesMask(app) ? gd::kTeal : gd::kMuted);
    }
}

void drawSlider(App& app, int i) {
    SDL_Renderer* r = app.renderer;
    const float y = sliderTop(i);
    const bool used = usesMask(app) || i == 0;  // the threshold also splits the relief statistics
    const gd::Color text = used ? gd::kInk : gd::kMuted;
    app.hud.text(r, kPanelX, y, i == 0 ? "Mask threshold" : "Mask softness", text);
    char value[16];
    SDL_snprintf(value, sizeof value, "%.2f", static_cast<double>(sliderValueRef(app, i)));
    app.hud.text(r, kPanelX + kCardW - app.hud.width(value, gd::Hud::Style::Bold), y, value, text, gd::Hud::Style::Bold);
    const float ty = y + 28.0F;
    gd::fillRect(r, kTrackX0, ty, kTrackX1 - kTrackX0, 6.0F, gd::kWhite);
    const float kx = tl::sliderPos(sliderValueRef(app, i), kTrackX0, kTrackX1, sliderLo(i), sliderHi(i));
    gd::fillRect(r, kTrackX0, ty, kx - kTrackX0, 6.0F, used ? gd::kTeal : gd::kMuted);
    gd::strokeRect(r, kTrackX0, ty, kTrackX1 - kTrackX0, 6.0F, gd::kInk);
    gd::card(r, kx - 6.0F, ty - 8.0F, 12.0F, 22.0F, gd::kWhite, gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    static const char* const kViews[3] = {"terrain", "height", "relief (slope)"};
    app.hud.titleBar(r, kWidth, "[ BLENDING NOISE LAYERS ]", kViews[static_cast<int>(app.view)]);

    // The map.
    const SDL_FRect mapDst{kMapX, kMapY, kMapW, kMapH};
    SDL_RenderTexture(r, app.mapTex, nullptr, &mapDst);
    gd::strokeRect(r, kMapX, kMapY, kMapW, kMapH, gd::kInk);

    std::string line1, line2;
    formulaLines(app, line1, line2);
    const float strip = line2.empty() ? 26.0F : 48.0F;
    gd::fillRect(r, kMapX + 1.0F, kMapY + 1.0F, kMapW - 2.0F, strip, gd::Color{250, 250, 249, 215});
    app.hud.text(r, kMapX + 10.0F, kMapY + 3.0F, line1, gd::kInk, gd::Hud::Style::Bold);
    if (!line2.empty()) app.hud.text(r, kMapX + 10.0F, kMapY + 25.0F, line2, gd::kMuted);
    if (overlayVisible(app)) {
        gd::fillRect(r, kMapX + 1.0F, kMapY + kMapH - 27.0F, kMapW - 2.0F, 26.0F, gd::Color{250, 250, 249, 215});
        app.hud.text(r, kMapX + 10.0F, kMapY + kMapH - 25.0F, "orange = mask off: no mountains here", gd::kOrange, gd::Hud::Style::Bold);
    }

    // Layer cards and sliders.
    for (int i = 0; i < 4; ++i) drawLayerCard(app, i);
    drawSlider(app, 0);
    drawSlider(app, 1);

    // The number behind the picture.
    char low[24], high[24];
    SDL_snprintf(low, sizeof low, "%.3f", app.relief.lowland);
    SDL_snprintf(high, sizeof high, "%.3f", app.relief.highland);
    std::string stats = "Relief on land (slope per cell)   lowland ";
    stats += app.relief.nLow > 0 ? low : "-";
    stats += "   highland ";
    stats += app.relief.nHigh > 0 ? high : "-";
    stats += "   ratio ";
    app.hud.text(r, kMapX, kStatsY, stats, gd::kInk);
    const float rx = kMapX + app.hud.width(stats);
    char ratio[24];
    if (app.relief.ratio() > 0.0) {
        SDL_snprintf(ratio, sizeof ratio, "%.1fx", app.relief.ratio());
    } else {
        SDL_snprintf(ratio, sizeof ratio, "-");
    }
    const gd::Color rc = app.relief.ratio() >= 2.0 ? gd::kTeal : app.relief.ratio() > 0.0 ? gd::kOrange : gd::kMuted;
    app.hud.text(r, rx, kStatsY, ratio, rc, gd::Hud::Style::Bold);

    // Chips.
    app.hud.text(r, kMapX, kChipRow1 + 4.0F, "Blend", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, kMapX, kChipRow2 + 4.0F, "View", gd::kMuted, gd::Hud::Style::Bold);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool dim = c.action == Action::Overlay && !(usesMask(app) && app.view != View::Relief);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        const gd::Color tc = on ? gd::kWhite : dim ? gd::kMuted : gd::kInk;
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, tc, gd::Hud::Style::Bold);
    }
    app.hud.text(r, 672.0F, kChipRow1 + 4.0F, "click a card: layer on/off", gd::kMuted);
    app.hud.text(r, 672.0F, kChipRow2 + 4.0F, "drag map / arrows: pan", gd::kMuted);
}

void endDrag(App& app) { app.drag = Drag::None; }

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
    if (!SDL_CreateWindowAndRenderer("terrain-layers", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->mapTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kGridW, kGridH);
    if (app->mapTex == nullptr) {
        SDL_Log("map texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->mapTex, SDL_SCALEMODE_LINEAR);
    for (SDL_Texture*& t : app->thumbTex) {
        t = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kGridW / kThumbFactor, kGridH / kThumbFactor);
        if (t == nullptr) {
            SDL_Log("thumbnail texture failed: %s", SDL_GetError());
            return SDL_APP_FAILURE;
        }
        SDL_SetTextureScaleMode(t, SDL_SCALEMODE_NEAREST);
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            const SDL_FPoint p{event->motion.x, event->motion.y};
            if (app.drag == Drag::Pan) {
                const float perPixel = tl::kViewSpan / kMapW;
                app.ox = tl::clampPan(app.ox - (p.x - app.last.x) * perPixel);
                app.oy = tl::clampPan(app.oy - (p.y - app.last.y) * perPixel);
                app.layersDirty = true;
            } else if (app.drag == Drag::Threshold) {
                setSliderFromMouse(app, 0, p.x);
            } else if (app.drag == Drag::Softness) {
                setSliderFromMouse(app, 1, p.x);
            }
            app.last = p;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.last = p;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int i = 0; i < 3; ++i) {
                if (inRect(p, kPanelX, kMapY + kCardPitch * static_cast<float>(i), kCardW, kCardH)) {
                    apply(app, i == 0 ? Action::ToggleContinent : i == 1 ? Action::ToggleMountains : Action::ToggleDetail);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int i = 0; i < 2; ++i) {
                if (inRect(p, kPanelX, sliderTop(i) - 4.0F, kCardW, kSliderPitch)) {
                    app.drag = i == 0 ? Drag::Threshold : Drag::Softness;
                    setSliderFromMouse(app, i, p.x);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inRect(p, kMapX, kMapY, kMapW, kMapH)) app.drag = Drag::Pan;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) endDrag(app);
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: endDrag(app); break;
        case SDL_EVENT_KEY_DOWN: {
            // Arrows may repeat (holding them keeps panning), every toggle must not.
            const bool repeat = event->key.repeat;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_LEFT: app.ox = tl::clampPan(app.ox - kArrowStep); app.layersDirty = true; break;
                case SDLK_RIGHT: app.ox = tl::clampPan(app.ox + kArrowStep); app.layersDirty = true; break;
                case SDLK_UP: app.oy = tl::clampPan(app.oy - kArrowStep); app.layersDirty = true; break;
                case SDLK_DOWN: app.oy = tl::clampPan(app.oy + kArrowStep); app.layersDirty = true; break;
                default: break;
            }
            if (repeat) break;
            switch (event->key.key) {
                case SDLK_1: apply(app, Action::ToggleContinent); break;
                case SDLK_2: apply(app, Action::ToggleMountains); break;
                case SDLK_3: apply(app, Action::ToggleDetail); break;
                case SDLK_4: apply(app, Action::ModeAdd); break;
                case SDLK_5: apply(app, Action::ModeMultiply); break;
                case SDLK_6: apply(app, Action::ModeMask); break;
                case SDLK_7: apply(app, Action::ModeLerp); break;
                case SDLK_T: apply(app, Action::ViewTerrain); break;
                case SDLK_H: apply(app, Action::ViewHeight); break;
                case SDLK_R: apply(app, Action::ViewRelief); break;
                case SDLK_O: apply(app, Action::Overlay); break;
                case SDLK_N: apply(app, Action::NewSeed); break;
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
    refresh(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        for (SDL_Texture* t : app->thumbTex) {
            if (t != nullptr) SDL_DestroyTexture(t);
        }
        if (app->mapTex != nullptr) SDL_DestroyTexture(app->mapTex);
        app->hud.destroy();
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
