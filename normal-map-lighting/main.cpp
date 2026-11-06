// Lighting a flat sprite with a normal map, per pixel on the CPU, with SDL3 main callbacks.
//
// The sprite is a stone tile painted in flat colours (the albedo, top right). Its depth lives only in the normal map
// next to it: for every texel the direction the surface faces. Each frame the big picture is shaded pixel by pixel:
// direction from the pixel to the light, dot product with the normal, plus a Blinn-Phong highlight. 208 x 208 pixels is
// cheap enough for the CPU, so there is no shader and the demo runs the same natively and in the browser.
// The switches are the typical mistakes: no normal map at all, a green channel that points the wrong way, and
// interpolated normals that are not renormalized (visible with the small maps). All maths and the sprite itself
// are in logic.hpp and covered by test.cpp.

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

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;

constexpr int kTex = 208;                  // pixels of the lit picture
constexpr float kZoom = 2.0F;              // screen pixels per picture pixel
constexpr float kViewX = 24.0F;
constexpr float kViewY = 38.0F;
constexpr float kViewSize = static_cast<float>(kTex) * kZoom;
constexpr std::array<int, 3> kMapSizes{208, 52, 26};

constexpr float kColX = 496.0F;            // right column
constexpr float kColW = 440.0F;
constexpr float kThumb = 140.0F;
constexpr float kThumbY = 66.0F;
constexpr float kCardY = 216.0F;
constexpr float kCardH = 148.0F;
constexpr float kRowY = 380.0F;
constexpr float kSliderX = 630.0F;
constexpr float kSliderW = 250.0F;
constexpr float kSliderY = 435.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;

constexpr float kHeightMin = 0.08F;        // light height in picture widths
constexpr float kHeightMax = 1.2F;
constexpr float kAmbient = 0.2F;
constexpr float kSpecularStrength = 0.8F;
constexpr int kShininess = 24;
constexpr double kOrbitSeconds = 9.0;
constexpr double kTwoPi = 6.283185307179586;
constexpr gd::Color kRed{190, 50, 50};
constexpr gd::Color kGreen{30, 140, 60};
constexpr gd::Color kBlue{50, 80, 200};

enum class View { Lit, Diffuse, Specular };
enum class Action { ViewLit, ViewDiffuse, ViewSpecular, UseMap, FlipGreen, Renormalize, Map0, Map1, Map2, Orbit };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    SDL_Texture* litTex = nullptr;     // streaming, rewritten every frame
    SDL_Texture* albedoTex = nullptr;
    SDL_Texture* normalTex = nullptr;  // the map as it is stored, one texel per map texel
    std::vector<std::uint8_t> albedo;  // RGB, kTex x kTex
    std::vector<std::uint32_t> thumbPixels;
    nm::NormalMap map;
    int mapIndex = 0;

    View view = View::Lit;
    bool useMap = true;
    bool flipGreen = false;
    bool renormalize = true;

    bool orbit = true;
    double orbitPhase = 3.9269908;  // 5/4 pi: upper left, a light from the usual direction
    float lightX = 30.0F;           // in picture pixels, y down, may lie outside the picture
    float lightY = 30.0F;
    float heightFrac = 0.3F;
    bool draggingSlider = false;

    SDL_FPoint mouse{-100.0F, -100.0F};
    int inspectX = kTex / 2;
    int inspectY = kTex / 2;

    double shadeMs = -1.0;  // smoothed time of the shading loop, < 0 = no sample yet
    std::uint64_t lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

// SDL's performance counter only ticks in whole milliseconds in the browser build, far too coarse for a loop that takes
// a millisecond or two. The browser clock itself has 0.1 ms steps.
#ifdef __EMSCRIPTEN__
double nowMs() { return emscripten_get_now(); }
#else
double nowMs() { return static_cast<double>(SDL_GetPerformanceCounter()) * 1000.0 / static_cast<double>(SDL_GetPerformanceFrequency()); }
#endif

std::uint32_t pack(std::uint8_t r, std::uint8_t g, std::uint8_t b) { return 0xFF000000U | (static_cast<std::uint32_t>(r) << 16) | (static_cast<std::uint32_t>(g) << 8) | b; }

nm::Vec3 lightPosition(const App& app) { return {app.lightX, app.lightY, app.heightFrac * static_cast<float>(kTex)}; }

SDL_Texture* makeStaticTexture(SDL_Renderer* r, const std::vector<std::uint32_t>& pixels, int size, SDL_ScaleMode mode) {
    SDL_Texture* tex = SDL_CreateTexture(r, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, size, size);
    if (tex == nullptr) {
        SDL_Log("texture failed: %s", SDL_GetError());
        return nullptr;
    }
    if (!SDL_UpdateTexture(tex, nullptr, pixels.data(), size * 4)) {
        SDL_Log("texture upload failed: %s", SDL_GetError());
        SDL_DestroyTexture(tex);
        return nullptr;
    }
    SDL_SetTextureScaleMode(tex, mode);
    return tex;
}

/// Bakes the normal map in the given size and shows it as a thumbnail. The old texture goes after the new one exists.
bool setMapSize(App& app, int index) {
    const int size = kMapSizes[static_cast<std::size_t>(index)];
    nm::NormalMap map = nm::bakeNormalMap(size, [](float u, float v) { return nm::height(u, v); });
    app.thumbPixels.assign(static_cast<std::size_t>(size) * static_cast<std::size_t>(size), 0);
    for (std::size_t i = 0; i < app.thumbPixels.size(); ++i) app.thumbPixels[i] = pack(map.rgb[i * 3], map.rgb[i * 3 + 1], map.rgb[i * 3 + 2]);
    SDL_Texture* tex = makeStaticTexture(app.renderer, app.thumbPixels, size, SDL_SCALEMODE_NEAREST);
    if (tex == nullptr) return false;
    SDL_DestroyTexture(app.normalTex);
    app.normalTex = tex;
    app.map = std::move(map);
    app.mapIndex = index;
    app.shadeMs = -1.0;
    return true;
}

bool buildAlbedo(App& app) {
    app.albedo.assign(static_cast<std::size_t>(kTex) * static_cast<std::size_t>(kTex) * 3U, 0);
    std::vector<std::uint32_t> pixels(static_cast<std::size_t>(kTex) * static_cast<std::size_t>(kTex), 0);
    for (int y = 0; y < kTex; ++y) {
        for (int x = 0; x < kTex; ++x) {
            const nm::Rgb c = nm::albedoAt((static_cast<float>(x) + 0.5F) / kTex, (static_cast<float>(y) + 0.5F) / kTex);
            const std::size_t i = static_cast<std::size_t>(y) * kTex + static_cast<std::size_t>(x);
            app.albedo[i * 3] = nm::toByte(c.r);
            app.albedo[i * 3 + 1] = nm::toByte(c.g);
            app.albedo[i * 3 + 2] = nm::toByte(c.b);
            pixels[i] = pack(app.albedo[i * 3], app.albedo[i * 3 + 1], app.albedo[i * 3 + 2]);
        }
    }
    app.albedoTex = makeStaticTexture(app.renderer, pixels, kTex, SDL_SCALEMODE_LINEAR);
    return app.albedoTex != nullptr;
}

/// The normal that goes into the lighting at one pixel, according to the switches.
nm::Vec3 normalAt(const App& app, int x, int y) {
    if (!app.useMap) return {0.0F, 0.0F, 1.0F};
    return nm::sampleNormal(app.map, (static_cast<float>(x) + 0.5F) / kTex, (static_cast<float>(y) + 0.5F) / kTex, app.flipGreen, app.renormalize);
}

nm::Lit litAt(const App& app, int x, int y) {
    const nm::Vec3 l = nm::lightDirection(lightPosition(app), static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F);
    return nm::lightPixel(normalAt(app, x, y), l, kShininess);
}

bool shade(App& app) {
    void* locked = nullptr;
    int pitch = 0;
    if (!SDL_LockTexture(app.litTex, nullptr, &locked, &pitch)) {
        SDL_Log("lock failed: %s", SDL_GetError());
        return false;
    }
    const double start = nowMs();
    auto* base = static_cast<std::uint8_t*>(locked);
    for (int y = 0; y < kTex; ++y) {
        auto* row = reinterpret_cast<std::uint32_t*>(base + static_cast<std::ptrdiff_t>(y) * pitch);
        for (int x = 0; x < kTex; ++x) {
            const nm::Lit lit = litAt(app, x, y);
            std::uint8_t rgb[3];
            if (app.view == View::Lit) {
                const std::size_t i = (static_cast<std::size_t>(y) * kTex + static_cast<std::size_t>(x)) * 3U;
                for (std::size_t c = 0; c < 3; ++c) rgb[c] = nm::toByte(nm::compose(static_cast<float>(app.albedo[i + c]) / 255.0F, lit.diffuse, lit.specular, kAmbient, kSpecularStrength));
            } else {
                const std::uint8_t g = nm::toByte(app.view == View::Diffuse ? lit.diffuse : lit.specular);
                rgb[0] = rgb[1] = rgb[2] = g;
            }
            row[x] = pack(rgb[0], rgb[1], rgb[2]);
        }
    }
    const double ms = nowMs() - start;
    app.shadeMs = app.shadeMs < 0.0 ? ms : app.shadeMs * 0.9 + ms * 0.1;
    SDL_UnlockTexture(app.litTex);
    return true;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 6> bottom{{{Action::ViewLit, "1 Lit", 0.0F},
                                     {Action::ViewDiffuse, "2 Diffuse", 8.0F},
                                     {Action::ViewSpecular, "3 Specular", 8.0F},
                                     {Action::UseMap, "N Normal map", 24.0F},
                                     {Action::FlipGreen, "G Flip green", 8.0F},
                                     {Action::Renormalize, "R Renormalize", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : bottom) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
    const std::array<Def, 4> side{{{Action::Map0, "208", 0.0F}, {Action::Map1, "52", 6.0F}, {Action::Map2, "26", 6.0F}, {Action::Orbit, "Space Orbit", 20.0F}}};
    x = kColX + 80.0F;
    for (const Def& d : side) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kRowY, w, kChipH}});
        x += w;
    }
}

void setHeightFrac(App& app, float f) { app.heightFrac = std::clamp(f, kHeightMin, kHeightMax); }

void setFromSlider(App& app, float screenX) {
    const float t = std::clamp((screenX - kSliderX) / kSliderW, 0.0F, 1.0F);
    setHeightFrac(app, kHeightMin + t * (kHeightMax - kHeightMin));
}

void setLightFromMouse(App& app) {
    const float half = static_cast<float>(kTex) / 2.0F;
    app.lightX = std::clamp((app.mouse.x - kViewX) / kZoom, -half, static_cast<float>(kTex) + half);
    app.lightY = std::clamp((app.mouse.y - kViewY) / kZoom, -half, static_cast<float>(kTex) + half);
}

bool inView(SDL_FPoint p) { return p.x >= kViewX && p.y >= kViewY && p.x < kViewX + kViewSize && p.y < kViewY + kViewSize; }

/// The light follows the mouse over the picture and a thin border around it. Beyond that (chips, slider) it stays put,
/// so reaching for a button does not fling the light away.
bool inLightZone(SDL_FPoint p) {
    constexpr float kMargin = 16.0F;
    return p.x >= kViewX - kMargin && p.y >= kViewY - kMargin && p.x < kViewX + kViewSize + kMargin && p.y < kViewY + kViewSize + kMargin;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ViewLit: app.view = View::Lit; break;
        case Action::ViewDiffuse: app.view = View::Diffuse; break;
        case Action::ViewSpecular: app.view = View::Specular; break;
        case Action::UseMap: app.useMap = !app.useMap; break;
        case Action::FlipGreen: app.flipGreen = !app.flipGreen; break;
        case Action::Renormalize: app.renormalize = !app.renormalize; break;
        case Action::Map0: setMapSize(app, 0); break;
        case Action::Map1: setMapSize(app, 1); break;
        case Action::Map2: setMapSize(app, 2); break;
        case Action::Orbit: app.orbit = !app.orbit; break;
    }
    app.shadeMs = -1.0;  // the cost of a pixel changes with the switches, so the old average says nothing
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ViewLit: return app.view == View::Lit;
        case Action::ViewDiffuse: return app.view == View::Diffuse;
        case Action::ViewSpecular: return app.view == View::Specular;
        case Action::UseMap: return app.useMap;
        case Action::FlipGreen: return app.flipGreen;
        case Action::Renormalize: return app.renormalize;
        case Action::Map0: return app.mapIndex == 0;
        case Action::Map1: return app.mapIndex == 1;
        case Action::Map2: return app.mapIndex == 2;
        case Action::Orbit: return app.orbit;
    }
    return false;
}

std::string fixed(float v, int decimals) {
    if (std::fabs(v) < 0.5F * std::pow(10.0F, -static_cast<float>(decimals))) v = 0.0F;  // no "-0.00"
    char buf[32];
    SDL_snprintf(buf, sizeof buf, "%+.*f", decimals, static_cast<double>(v));
    return buf;
}

void drawCenteredText(App& app, SDL_FRect rect, const std::string& text, gd::Color c, gd::Hud::Style style) {
    app.hud.text(app.renderer, rect.x + (rect.w - app.hud.width(text, style)) / 2.0F, rect.y + 6.0F, text, c, style);
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        // Flip green is the mistake: when it is on the chip is orange. Everything else is teal when on.
        const gd::Color fill = on ? (c.action == Action::FlipGreen ? gd::kOrange : gd::kTeal) : gd::kWhite;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        drawCenteredText(app, c.rect, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, kColX, kRowY + 6.0F, "map (M)", gd::kMuted);
}

void drawInspector(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kColX, kCardY, kColW, kCardH, gd::kWhite, gd::kInk);
    const float x = kColX + 14.0F;
    float y = kCardY + 8.0F;
    const char* viewName = app.view == View::Lit ? "LIT" : app.view == View::Diffuse ? "DIFFUSE (N·L)" : "SPECULAR";
    app.hud.text(r, x, y, std::string(viewName) + (app.useMap ? "  with normal map" : "  flat normals only"), gd::kInk, gd::Hud::Style::Bold);
    y += 22.0F;
    if (!app.useMap) {
        app.hud.text(r, x, y, "N = (0, 0, 1) at every pixel", gd::kMuted);
    } else if (app.flipGreen && !app.renormalize) {
        app.hud.text(r, x, y, "green flipped, not renormalized", gd::kOrange);
    } else if (app.flipGreen) {
        app.hud.text(r, x, y, "green flipped", gd::kOrange);
    } else if (!app.renormalize) {
        app.hud.text(r, x, y, "not renormalized", gd::kOrange);
    } else {
        app.hud.text(r, x, y, "green up, renormalized", gd::kMuted);
    }
    y += 26.0F;
    const nm::Lit lit = litAt(app, app.inspectX, app.inspectY);
    app.hud.text(r, x, y, "pixel " + std::to_string(app.inspectX) + ", " + std::to_string(app.inspectY), gd::kInk, gd::Hud::Style::Bold);
    y += 22.0F;
    app.hud.text(r, x, y, "N " + fixed(lit.n.x, 2) + " " + fixed(lit.n.y, 2) + " " + fixed(lit.n.z, 2) + "  len " + fixed(nm::length(lit.n), 2).substr(1), gd::kInk);
    y += 22.0F;
    app.hud.text(r, x, y, "L " + fixed(lit.l.x, 2) + " " + fixed(lit.l.y, 2) + " " + fixed(lit.l.z, 2), gd::kInk);
    y += 22.0F;
    app.hud.text(r, x, y, "N·L " + fixed(lit.diffuse, 2).substr(1) + "   spec " + fixed(lit.specular, 2).substr(1), gd::kInk, gd::Hud::Style::Bold);
}

void drawSlider(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kColX, kSliderY - 9.0F, "light height", gd::kMuted);
    gd::fillRect(r, kSliderX, kSliderY, kSliderW, 8.0F, gd::kGrid);
    gd::strokeRect(r, kSliderX, kSliderY, kSliderW, 8.0F, gd::kInk);
    const float t = (app.heightFrac - kHeightMin) / (kHeightMax - kHeightMin);
    const float kx = kSliderX + t * kSliderW;
    gd::fillRect(r, kSliderX, kSliderY, kx - kSliderX, 8.0F, gd::kTeal);
    gd::card(r, kx - 6.0F, kSliderY - 6.0F, 12.0F, 20.0F, app.draggingSlider ? gd::kTeal : gd::kWhite, gd::kInk);
    char label[16];
    SDL_snprintf(label, sizeof label, "%.2f", static_cast<double>(app.heightFrac));
    app.hud.text(r, kSliderX + kSliderW + 12.0F, kSliderY - 9.0F, label, gd::kInk, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[48];
    if (app.shadeMs >= 0.0) {
        SDL_snprintf(right, sizeof right, "CPU shading %.1f ms", app.shadeMs);
    } else {
        SDL_snprintf(right, sizeof right, "CPU shading ...");
    }
    app.hud.titleBar(r, kWidth, "[ NORMAL MAP LIGHTING ]", right);

    // The lit picture.
    const SDL_FRect view{kViewX, kViewY, kViewSize, kViewSize};
    gd::fillRect(r, kViewX + 4.0F, kViewY + 4.0F, kViewSize, kViewSize, gd::Color{17, 24, 39, 40});
    SDL_RenderTexture(r, app.litTex, nullptr, &view);
    gd::strokeRect(r, kViewX, kViewY, kViewSize, kViewSize, gd::kInk);

    // The light, where the picture pixel (lightX, lightY) would be on screen.
    const float lx = kViewX + app.lightX * kZoom;
    const float ly = kViewY + app.lightY * kZoom;
    gd::disc(r, lx, ly, 16.0F, gd::Color{255, 236, 170, 200}, gd::Color{255, 236, 170, 0});
    gd::disc(r, lx, ly, 5.0F, gd::Color{255, 250, 225});
    gd::ring(r, lx, ly, 5.0F, gd::kInk);

    app.hud.text(r, kViewX, 455.0F, "mouse: light   wheel: height   Space: orbit", gd::kMuted);

    // Right column: the two textures the picture is made of.
    app.hud.text(r, kColX, 40.0F, "ALBEDO", gd::kMuted);
    const SDL_FRect albedoRect{kColX, kThumbY, kThumb, kThumb};
    SDL_RenderTexture(r, app.albedoTex, nullptr, &albedoRect);
    gd::strokeRect(r, albedoRect.x, albedoRect.y, albedoRect.w, albedoRect.h, gd::kInk);

    const float normalX = kColX + kThumb + 20.0F;
    const std::string mapName = "NORMAL " + std::to_string(app.map.size) + "x" + std::to_string(app.map.size);
    app.hud.text(r, normalX, 40.0F, mapName, gd::kMuted);
    const SDL_FRect normalRect{normalX, kThumbY, kThumb, kThumb};
    SDL_RenderTexture(r, app.normalTex, nullptr, &normalRect);
    gd::strokeRect(r, normalRect.x, normalRect.y, normalRect.w, normalRect.h, gd::kInk);

    const float legendX = normalX + kThumb + 20.0F;
    app.hud.text(r, legendX, kThumbY - 2.0F, "R = x right", kRed, gd::Hud::Style::Bold);
    app.hud.text(r, legendX, kThumbY + 20.0F, "G = y up", kGreen, gd::Hud::Style::Bold);
    app.hud.text(r, legendX, kThumbY + 42.0F, "B = z out", kBlue, gd::Hud::Style::Bold);
    app.hud.text(r, legendX, kThumbY + 76.0F, "flat surface", gd::kMuted);
    app.hud.text(r, legendX, kThumbY + 98.0F, "128 128 255", gd::kMuted);

    drawInspector(app);
    drawChips(app);
    drawSlider(app);

    // Mark the inspected pixel.
    const float px = kViewX + static_cast<float>(app.inspectX) * kZoom;
    const float py = kViewY + static_cast<float>(app.inspectY) * kZoom;
    gd::strokeRect(r, px - 1.0F, py - 1.0F, kZoom + 2.0F, kZoom + 2.0F, gd::kOrange);
}

void updateInspected(App& app) {
    if (!inView(app.mouse)) return;
    app.inspectX = std::clamp(static_cast<int>((app.mouse.x - kViewX) / kZoom), 0, kTex - 1);
    app.inspectY = std::clamp(static_cast<int>((app.mouse.y - kViewY) / kZoom), 0, kTex - 1);
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
    if (!SDL_CreateWindowAndRenderer("normal-map-lighting", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->litTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, kTex, kTex);
    if (app->litTex == nullptr) {
        SDL_Log("texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->litTex, SDL_SCALEMODE_LINEAR);
    if (!buildAlbedo(*app) || !setMapSize(*app, 0)) return SDL_APP_FAILURE;
    layoutChips(*app);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = SDL_FPoint{event->motion.x, event->motion.y};
            if (app.draggingSlider) {
                setFromSlider(app, app.mouse.x);
            } else if (inLightZone(app.mouse)) {
                app.orbit = false;  // taking the light with the mouse ends the demo orbit
                setLightFromMouse(app);
            }
            updateInspected(app);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            if (event->button.button != SDL_BUTTON_LEFT) break;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            const SDL_FRect grab{kSliderX - 10.0F, kSliderY - 10.0F, kSliderW + 20.0F, 28.0F};
            if (SDL_PointInRectFloat(&p, &grab)) {
                app.draggingSlider = true;
                setFromSlider(app, p.x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.draggingSlider = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.draggingSlider = false; break;
        case SDL_EVENT_MOUSE_WHEEL: setHeightFrac(app, app.heightFrac + event->wheel.y * 0.04F); break;
        case SDL_EVENT_KEY_DOWN: {
            const bool fresh = !event->key.repeat;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_UP: setHeightFrac(app, app.heightFrac + 0.02F); break;  // holding the key scrubs, so repeat is wanted
                case SDLK_DOWN: setHeightFrac(app, app.heightFrac - 0.02F); break;
                case SDLK_1: if (fresh) apply(app, Action::ViewLit); break;
                case SDLK_2: if (fresh) apply(app, Action::ViewDiffuse); break;
                case SDLK_3: if (fresh) apply(app, Action::ViewSpecular); break;
                case SDLK_N: if (fresh) apply(app, Action::UseMap); break;
                case SDLK_G: if (fresh) apply(app, Action::FlipGreen); break;
                case SDLK_R: if (fresh) apply(app, Action::Renormalize); break;
                case SDLK_SPACE: if (fresh) apply(app, Action::Orbit); break;
                case SDLK_M:
                    if (fresh) setMapSize(app, (app.mapIndex + 1) % static_cast<int>(kMapSizes.size()));
                    break;
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
    const std::uint64_t now = SDL_GetTicksNS();
    const double dt = std::min(static_cast<double>(now - app.lastTicks) * 1e-9, 0.25);
    app.lastTicks = now;

    if (app.orbit && !app.draggingSlider) {
        app.orbitPhase = std::fmod(app.orbitPhase + dt * kTwoPi / kOrbitSeconds, kTwoPi);  // reduced every frame, never grows
        const float half = static_cast<float>(kTex) / 2.0F;
        app.lightX = half + 0.38F * static_cast<float>(kTex) * static_cast<float>(std::cos(app.orbitPhase));
        app.lightY = half + 0.38F * static_cast<float>(kTex) * static_cast<float>(std::sin(app.orbitPhase));
    }

    if (!shade(app)) return SDL_APP_FAILURE;
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        SDL_DestroyTexture(app->litTex);
        SDL_DestroyTexture(app->albedoTex);
        SDL_DestroyTexture(app->normalTex);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
