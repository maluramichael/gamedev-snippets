// Biomes and hillshade with SDL3 main callbacks.
//
// A height map alone gives stripes: the temperature falls with height, so every biome is a band along a contour line.
// A second noise map for moisture adds the other axis of the Whittaker table and the bands break up into patches.
// The classic mistake is to pick one table cell per pixel: the borders between the cells show up as hard, straight-
// edged outlines. Mixing the colors of the surrounding cell centers (the "blend width" slider, 0 = hard) makes them
// fade. Hillshade then lights the height map from a sun, so the relief is readable. Probe a point by hovering the
// map, the table on the right shows where it sits. All math is in logic.hpp and covered by test.cpp.

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
constexpr int kGridW = 288;  // map cells, drawn 2x2 px each
constexpr int kGridH = 162;
constexpr float kMapX = 20.0F;
constexpr float kMapY = 40.0F;
constexpr float kMapW = 576.0F;
constexpr float kMapH = 324.0F;
constexpr int kTableRes = 80;  // preview texture of the Whittaker table
constexpr float kTableW = 200.0F;
constexpr float kTableH = 148.0F;
constexpr float kChipH = 28.0F;
constexpr std::uint32_t kStartSeed = 7;

enum class View { Biomes, Height, Moisture, Shade };
enum class Action { ViewBiomes, ViewHeight, ViewMoisture, ViewShade, Hillshade, NewSeed, ModeHeight, ModeBoth, Hard, Soft };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Slider {
    const char* label;
    double* value;
    double lo;
    double hi;
    int decimals;
    bool degrees;
    bool softOnly;
    SDL_FRect track;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* mapTex = nullptr;
    SDL_Texture* tableTex = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<Slider> sliders;
    bio::World world;
    bio::Params params;
    View view = View::Biomes;
    bool hillshade = true;
    double azimuth = 315.0;  // compass degrees, the cartographic default: light from the north west
    double altitude = 40.0;
    std::uint32_t seed = kStartSeed;
    bool dirty = true;
    int biomeCount = 0;
    std::vector<Uint8> pixels;
    std::vector<Uint8> tablePixels;
    SDL_FPoint mouse{-100.0F, -100.0F};
    std::size_t home = 0;  // probe cell while the mouse is off the map
    int dragSlider = -1;
    long maxFrames = -1;
    long frames = 0;
};

Uint8 toByte(float v) { return static_cast<Uint8>(std::clamp(v, 0.0F, 1.0F) * 255.0F + 0.5F); }

gd::Color toColor(bio::Rgb c) { return gd::Color{toByte(c.r), toByte(c.g), toByte(c.b)}; }

void put(Uint8* px, bio::Rgb c) {
    px[0] = toByte(c.r);
    px[1] = toByte(c.g);
    px[2] = toByte(c.b);
    px[3] = 255;
}

bool inMap(SDL_FPoint p) { return p.x >= kMapX && p.x < kMapX + kMapW && p.y >= kMapY && p.y < kMapY + kMapH; }

/// The map cell under the mouse, or a spot on land near the middle so the first frame already shows a probe.
std::size_t probeCell(const App& app) {
    if (!inMap(app.mouse)) return app.home;
    const int cx = std::clamp(static_cast<int>((app.mouse.x - kMapX) / 2.0F), 0, kGridW - 1);
    const int cy = std::clamp(static_cast<int>((app.mouse.y - kMapY) / 2.0F), 0, kGridH - 1);
    return static_cast<std::size_t>(cy * kGridW + cx);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float row;
        float gapBefore;
    };
    constexpr float kRowA = 370.0F;
    constexpr float kRowB = 404.0F;
    const std::array<Def, 10> defs{{{Action::ViewBiomes, "1 Biomes", kRowA, 0.0F},
                                    {Action::ViewHeight, "2 Height", kRowA, 8.0F},
                                    {Action::ViewMoisture, "3 Moisture", kRowA, 8.0F},
                                    {Action::ViewShade, "4 Shade", kRowA, 8.0F},
                                    {Action::Hillshade, "H Hillshade", kRowA, 24.0F},
                                    {Action::NewSeed, "N New seed", kRowA, 24.0F},
                                    {Action::ModeHeight, "5 Height only", kRowB, 0.0F},
                                    {Action::ModeBoth, "6 Height + moisture", kRowB, 8.0F},
                                    {Action::Hard, "7 Hard edges", kRowB, 24.0F},
                                    {Action::Soft, "8 Soft edges", kRowB, 8.0F}}};
    float x = 20.0F;
    float lastRow = 0.0F;
    for (const Def& d : defs) {
        if (d.row != lastRow) {
            x = 20.0F;
            lastRow = d.row;
        }
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.row, w, kChipH}});
        x += w;
    }
}

void layoutSliders(App& app) {
    constexpr float kColX[3] = {20.0F, 340.0F, 660.0F};
    constexpr float kTrackY[2] = {466.0F, 512.0F};
    constexpr float kW = 280.0F;
    app.sliders = {
        {"moisture offset", &app.params.moistureOffset, -0.4, 0.4, 2, false, false, {}},
        {"shore temperature", &app.params.tempBase, 0.2, 1.0, 2, false, false, {}},
        {"blend width", &app.params.width, 0.0, 1.0, 2, false, true, {}},
        {"sun azimuth", &app.azimuth, 0.0, 360.0, 0, true, false, {}},
        {"sun altitude", &app.altitude, 5.0, 85.0, 0, true, false, {}},
    };
    for (std::size_t i = 0; i < app.sliders.size(); ++i) {
        app.sliders[i].track = SDL_FRect{kColX[i % 3], kTrackY[i / 3], kW, 10.0F};
    }
}

void setSliderFromX(App& app, Slider& s, float x) {
    const double t = std::clamp(static_cast<double>((x - s.track.x) / s.track.w), 0.0, 1.0);
    *s.value = s.lo + t * (s.hi - s.lo);
    app.dirty = true;
}

void recolor(App& app) {
    const bio::World& w = app.world;
    const bio::Params& p = app.params;
    for (std::size_t i = 0; i < w.height.size(); ++i) {
        const double h = static_cast<double>(w.height[i]);
        const double m = static_cast<double>(w.moisture[i]);
        const double lit = bio::shade(static_cast<double>(w.slope[i]), static_cast<double>(w.aspect[i]), app.azimuth, app.altitude);
        bio::Rgb c = bio::cellColor(h, m, p);
        switch (app.view) {
            case View::Biomes:
                if (app.hillshade) c = bio::scale(c, bio::shadeFactor(lit, app.altitude));
                break;
            case View::Height:
                if (h >= bio::kSeaLevel) {
                    const auto g = static_cast<float>(0.2 + 0.8 * (h - bio::kSeaLevel) / (1.0 - bio::kSeaLevel));
                    c = bio::Rgb{g, g, g};
                }
                break;
            case View::Moisture:
                if (h >= bio::kSeaLevel) c = bio::lerp(bio::rgb(214, 190, 140), bio::rgb(30, 90, 190), static_cast<float>(bio::effectiveMoisture(m, p)));
                break;
            case View::Shade: {
                const auto g = static_cast<float>(lit);
                c = bio::Rgb{g, g, g};
                break;
            }
        }
        put(&app.pixels[i * 4], c);
    }
    SDL_UpdateTexture(app.mapTex, nullptr, app.pixels.data(), kGridW * 4);

    for (int y = 0; y < kTableRes; ++y) {
        for (int x = 0; x < kTableRes; ++x) {
            const double m = (x + 0.5) / kTableRes;
            const double t = 1.0 - (y + 0.5) / kTableRes;
            put(&app.tablePixels[static_cast<std::size_t>(y * kTableRes + x) * 4], bio::biomeColor(m, t, bio::effectiveWidth(p)));
        }
    }
    SDL_UpdateTexture(app.tableTex, nullptr, app.tablePixels.data(), kTableRes * 4);
    app.biomeCount = bio::countBiomes(w, p);
    app.dirty = false;
}

void regenerate(App& app) {
    bio::generate(app.world, kGridW, kGridH, app.seed);
    app.home = bio::nearestLandCell(app.world, kGridW * 5 / 12, kGridH / 2, bio::kSeaLevel + 0.08);
    app.dirty = true;
}

void newSeed(App& app) {
    app.seed = app.seed * 1664525U + 1013904223U;
    regenerate(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ViewBiomes: app.view = View::Biomes; break;
        case Action::ViewHeight: app.view = View::Height; break;
        case Action::ViewMoisture: app.view = View::Moisture; break;
        case Action::ViewShade: app.view = View::Shade; break;
        case Action::Hillshade: app.hillshade = !app.hillshade; break;
        case Action::NewSeed: newSeed(app); break;
        case Action::ModeHeight: app.params.mode = bio::Classify::HeightOnly; break;
        case Action::ModeBoth: app.params.mode = bio::Classify::HeightMoisture; break;
        case Action::Hard: app.params.soft = false; break;
        case Action::Soft: app.params.soft = true; break;
    }
    app.dirty = true;
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ViewBiomes: return app.view == View::Biomes;
        case Action::ViewHeight: return app.view == View::Height;
        case Action::ViewMoisture: return app.view == View::Moisture;
        case Action::ViewShade: return app.view == View::Shade;
        case Action::Hillshade: return app.hillshade;
        case Action::NewSeed: return false;
        case Action::ModeHeight: return app.params.mode == bio::Classify::HeightOnly;
        case Action::ModeBoth: return app.params.mode == bio::Classify::HeightMoisture;
        case Action::Hard: return !app.params.soft;
        case Action::Soft: return app.params.soft;
    }
    return false;
}

void drawSlider(const App& app, const Slider& s) {
    SDL_Renderer* r = app.renderer;
    const bool off = s.softOnly && !app.params.soft;
    const gd::Color label = off ? gd::kGrid : gd::kMuted;
    const gd::Color body = off ? gd::kGrid : gd::kTeal;
    app.hud.text(r, s.track.x, s.track.y - 28.0F, s.label, label);
    char value[24];
    SDL_snprintf(value, sizeof value, s.degrees ? "%.*f°" : "%.*f", s.decimals, *s.value);
    app.hud.text(r, s.track.x + s.track.w - app.hud.width(value, gd::Hud::Style::Bold), s.track.y - 28.0F, value, off ? gd::kGrid : gd::kInk, gd::Hud::Style::Bold);
    const float t = static_cast<float>((*s.value - s.lo) / (s.hi - s.lo));
    gd::fillRect(r, s.track.x, s.track.y, s.track.w, s.track.h, gd::kWhite);
    gd::fillRect(r, s.track.x, s.track.y, s.track.w * t, s.track.h, body);
    gd::strokeRect(r, s.track.x, s.track.y, s.track.w, s.track.h, off ? gd::kGrid : gd::kInk);
    if (!off) gd::card(r, s.track.x + s.track.w * t - 5.0F, s.track.y - 5.0F, 10.0F, 20.0F, gd::kWhite, gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const bio::Params& p = app.params;
    const std::size_t pc = probeCell(app);
    const double ph = static_cast<double>(app.world.height[pc]);
    const double pm = static_cast<double>(app.world.moisture[pc]);
    const double effM = bio::effectiveMoisture(pm, p);
    const double pt = bio::temperature(ph, p.tempBase);
    const int pcell = bio::cellIndex(ph, pm, p);

    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "%s, %s edges", p.mode == bio::Classify::HeightOnly ? "height only" : "height + moisture", p.soft ? "soft" : "hard");
    app.hud.titleBar(r, kWidth, "[ BIOMES AND HILLSHADE ]", right);

    // The map.
    gd::fillRect(r, kMapX + 4.0F, kMapY + 4.0F, kMapW, kMapH, gd::Color{17, 24, 39, 40});
    const SDL_FRect mapDst{kMapX, kMapY, kMapW, kMapH};
    SDL_RenderTexture(r, app.mapTex, nullptr, &mapDst);
    gd::strokeRect(r, kMapX, kMapY, kMapW, kMapH, gd::kInk);
    const float px = kMapX + (static_cast<float>(pc % kGridW) + 0.5F) * 2.0F;
    const float py = kMapY + (static_cast<float>(pc / kGridW) + 0.5F) * 2.0F;
    gd::ring(r, px, py, 7.0F, gd::kWhite);
    gd::ring(r, px, py, 8.0F, gd::kInk);

    // The Whittaker table with the probe.
    gd::card(r, 620.0F, kMapY, 320.0F, 210.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 632.0F, 46.0F, "WHITTAKER TABLE", gd::kInk, gd::Hud::Style::Bold);
    const float tx = 700.0F;
    const float ty = 72.0F;
    const SDL_FRect tableDst{tx, ty, kTableW, kTableH};
    SDL_RenderTexture(r, app.tableTex, nullptr, &tableDst);
    for (int i = 1; i < 4; ++i) {
        gd::fillRect(r, tx + kTableW * static_cast<float>(i) / 4.0F, ty, 1.0F, kTableH, gd::Color{17, 24, 39, 70});
        gd::fillRect(r, tx, ty + kTableH * static_cast<float>(i) / 4.0F, kTableW, 1.0F, gd::Color{17, 24, 39, 70});
    }
    gd::strokeRect(r, tx, ty, kTableW, kTableH, gd::kInk);
    app.hud.text(r, tx - 44.0F, ty, "hot", gd::kMuted);
    app.hud.text(r, tx - 44.0F, ty + kTableH - 22.0F, "cold", gd::kMuted);
    app.hud.text(r, tx, ty + kTableH + 4.0F, "dry", gd::kMuted);
    app.hud.text(r, tx + (kTableW - app.hud.width("moisture")) / 2.0F, ty + kTableH + 4.0F, "moisture", gd::kMuted);
    app.hud.text(r, tx + kTableW - app.hud.width("wet"), ty + kTableH + 4.0F, "wet", gd::kMuted);
    if (p.mode == bio::Classify::HeightOnly) {
        // Without the second map the whole land sits on one vertical line.
        gd::fillRect(r, tx + kTableW * static_cast<float>(effM) - 1.0F, ty, 3.0F, kTableH, gd::kOrange);
    }
    const float mx = tx + kTableW * static_cast<float>(effM);
    const float my = ty + kTableH * (1.0F - static_cast<float>(pt));
    if (pcell >= 0) {
        gd::ring(r, mx, my, 7.0F, gd::kWhite);
        gd::ring(r, mx, my, 8.0F, gd::kInk);
        gd::disc(r, mx, my, 3.0F, gd::kInk);
    }

    // Readout of the probe.
    gd::card(r, 620.0F, 258.0F, 320.0F, 106.0F, gd::kWhite, gd::kInk);
    char line[96];
    if (pcell >= 0) {
        const bio::Biome& biome = bio::table()[static_cast<std::size_t>(pcell)];
        gd::fillRect(r, 632.0F, 268.0F, 16.0F, 16.0F, toColor(biome.color));
        gd::strokeRect(r, 632.0F, 268.0F, 16.0F, 16.0F, gd::kInk);
        app.hud.text(r, 656.0F, 265.0F, biome.name, gd::kInk, gd::Hud::Style::Bold);
    } else {
        gd::fillRect(r, 632.0F, 268.0F, 16.0F, 16.0F, toColor(bio::kShallow));
        gd::strokeRect(r, 632.0F, 268.0F, 16.0F, 16.0F, gd::kInk);
        app.hud.text(r, 656.0F, 265.0F, "Water", gd::kInk, gd::Hud::Style::Bold);
    }
    SDL_snprintf(line, sizeof line, "height %.2f  moisture %.2f", ph, effM);
    app.hud.text(r, 632.0F, 289.0F, line, gd::kMuted);
    SDL_snprintf(line, sizeof line, "temp %.2f  shade %.2f", pt, bio::shade(static_cast<double>(app.world.slope[pc]), static_cast<double>(app.world.aspect[pc]), app.azimuth, app.altitude));
    app.hud.text(r, 632.0F, 311.0F, line, gd::kMuted);
    SDL_snprintf(line, sizeof line, "%d of 16 biomes on this map", app.biomeCount);
    app.hud.text(r, 632.0F, 333.0F, line, app.biomeCount <= 4 ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 3.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    for (const Slider& s : app.sliders) drawSlider(app, s);
    app.hud.text(r, 660.0F, 490.0F, "hover the map to probe", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("terrain-biomes", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->mapTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kGridW, kGridH);
    app->tableTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kTableRes, kTableRes);
    if (app->mapTex == nullptr || app->tableTex == nullptr) {
        SDL_Log("texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->mapTex, SDL_SCALEMODE_NEAREST);
    SDL_SetTextureScaleMode(app->tableTex, SDL_SCALEMODE_LINEAR);
    app->pixels.assign(static_cast<std::size_t>(kGridW) * kGridH * 4, 255);
    app->tablePixels.assign(static_cast<std::size_t>(kTableRes) * kTableRes * 4, 255);
    regenerate(*app);
    layoutChips(*app);
    layoutSliders(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = SDL_FPoint{event->motion.x, event->motion.y};
            if (app.dragSlider >= 0) setSliderFromX(app, app.sliders[static_cast<std::size_t>(app.dragSlider)], app.mouse.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c.action);
            }
            for (std::size_t i = 0; i < app.sliders.size(); ++i) {
                const SDL_FRect grab{app.sliders[i].track.x - 8.0F, app.sliders[i].track.y - 10.0F, app.sliders[i].track.w + 16.0F, 30.0F};
                if (SDL_PointInRectFloat(&p, &grab)) {
                    app.dragSlider = static_cast<int>(i);
                    setSliderFromX(app, app.sliders[i], p.x);
                }
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragSlider = -1;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.mouse = SDL_FPoint{-100.0F, -100.0F};
            app.dragSlider = -1;
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragSlider = -1; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::ViewBiomes); break;
                case SDLK_2: apply(app, Action::ViewHeight); break;
                case SDLK_3: apply(app, Action::ViewMoisture); break;
                case SDLK_4: apply(app, Action::ViewShade); break;
                case SDLK_5: apply(app, Action::ModeHeight); break;
                case SDLK_6: apply(app, Action::ModeBoth); break;
                case SDLK_7: apply(app, Action::Hard); break;
                case SDLK_8: apply(app, Action::Soft); break;
                case SDLK_H: apply(app, Action::Hillshade); break;
                case SDLK_N: apply(app, Action::NewSeed); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    if (app.dirty) recolor(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        SDL_DestroyTexture(app->mapTex);
        SDL_DestroyTexture(app->tableTex);
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
