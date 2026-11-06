// Rivers and lakes from the shape of the land, with SDL3 main callbacks.
//
// Rivers are not drawn, they fall out of the heightmap: every cell drains into its steepest lower neighbor (D8), the
// cells are processed from high to low and each one passes the rain it collected on, and a river is wherever enough
// rain has gathered. On raw noise terrain that goes wrong at once, because noise is full of small pits where the
// water has no way down. The orange squares are those pits, the rivers stop there. "Fill sinks" raises every
// depression up to its lowest way out (Priority-Flood), the raised cells are the lakes, and now every raindrop finds
// the sea. Hover over the map to follow one drop. The fill, the flow directions, the accumulation and the slider and
// color mapping live in logic.hpp and are covered by test.cpp.

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
constexpr int kMapW = 160;
constexpr int kMapH = 100;
constexpr int kCell = 4;  // px per map cell
constexpr float kMapX = 24.0F;
constexpr float kMapY = 44.0F;
constexpr float kPanelX = 688.0F;
constexpr float kPanelW = 248.0F;
constexpr float kTrackX = 696.0F;
constexpr float kTrackW = 232.0F;
constexpr float kChipY = 484.0F;
constexpr float kChipH = 34.0F;
constexpr unsigned kStartSeed = 7;
constexpr int kStartThreshold = 24;
constexpr gd::Color kRiver{29, 78, 216};
constexpr gd::Color kLake{74, 144, 226};

enum class Action { Raw, Fill, Water, NewSeed, Reset };
enum class Drag { None, Threshold, Sea };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* mapTexture = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    rv::World world;
    unsigned seed = kStartSeed;
    double sea = rv::kDefaultSea;
    int threshold = kStartThreshold;
    bool filled = false;
    bool water = true;
    int drop = -1;  // the raindrop: last cell the mouse was over, at first the most telling one
    Drag drag = Drag::None;
    bool worldDirty = true;
    bool textureDirty = true;
    bool riversDirty = true;
    bool dropDirty = true;
    std::vector<SDL_Vertex> riverVerts;
    std::vector<int> riverIdx;
    std::vector<SDL_Vertex> dropVerts;
    std::vector<int> dropIdx;
    std::vector<int> dropPath;
    int riverCells = 0;
    int deadEnds = 0;
    long maxFrames = -1;
    long frames = 0;
};

const std::vector<int>& activeDir(const App& app) { return app.filled ? app.world.dir : app.world.rawDir; }
const std::vector<int>& activeAcc(const App& app) { return app.filled ? app.world.acc : app.world.rawAcc; }

float cellCenterX(int i) { return kMapX + (static_cast<float>(i % kMapW) + 0.5F) * kCell; }
float cellCenterY(int i) { return kMapY + (static_cast<float>(i / kMapW) + 0.5F) * kCell; }

void markAllDirty(App& app) {
    app.textureDirty = true;
    app.riversDirty = true;
    app.dropDirty = true;
}

void rebuildWorld(App& app) {
    app.world = rv::buildWorld(kMapW, kMapH, rv::makeTerrain(kMapW, kMapH, app.seed), app.sea);
    if (app.drop < 0 || app.world.outlet[static_cast<std::size_t>(app.drop)] != 0) app.drop = rv::pickDemoDrop(app.world);
    app.worldDirty = false;
    markAllDirty(app);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 5> defs{{{Action::Raw, "1 Raw terrain", 0.0F},
                                   {Action::Fill, "2 Fill sinks", 8.0F},
                                   {Action::Water, "W Water", 24.0F},
                                   {Action::NewSeed, "N New terrain", 8.0F},
                                   {Action::Reset, "R Reset", 24.0F}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Raw:
            app.filled = false;
            markAllDirty(app);
            break;
        case Action::Fill:
            app.filled = true;
            markAllDirty(app);
            break;
        case Action::Water:
            app.water = !app.water;
            markAllDirty(app);
            break;
        case Action::NewSeed:
            ++app.seed;
            app.drop = -1;
            app.worldDirty = true;
            break;
        case Action::Reset:
            app.seed = kStartSeed;
            app.sea = rv::kDefaultSea;
            app.threshold = kStartThreshold;
            app.filled = false;
            app.water = true;
            app.drop = -1;
            app.drag = Drag::None;
            app.worldDirty = true;
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Raw: return !app.filled;
        case Action::Fill: return app.filled;
        case Action::Water: return app.water;
        case Action::NewSeed:
        case Action::Reset: return false;
    }
    return false;
}

void setThreshold(App& app, int value) {
    const int v = std::clamp(value, rv::kThresholdMin, rv::kThresholdMax);
    if (v != app.threshold) {
        app.threshold = v;
        app.riversDirty = true;
    }
}

void setSea(App& app, double value) {
    const double v = rv::clampSea(value);
    if (v != app.sea) {
        app.sea = v;
        app.worldDirty = true;
    }
}

void sliderFromMouse(App& app, float mx) {
    const double t = (static_cast<double>(mx) - static_cast<double>(kTrackX)) / static_cast<double>(kTrackW);
    if (app.drag == Drag::Threshold) setThreshold(app, rv::thresholdFromT(t));
    if (app.drag == Drag::Sea) setSea(app, rv::stepValue(t, rv::kSeaMin, rv::kSeaMax, 0.01));
}

// --- geometry ------------------------------------------------------------------------------------------------------

/// A thick segment as a quad, both ends pushed out by half the width so that neighbors overlap without gaps.
void addSegment(std::vector<SDL_Vertex>& verts, std::vector<int>& idx, float x0, float y0, float x1, float y1, float width, gd::Color color) {
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6F) return;
    const float ux = dx / len;
    const float uy = dy / len;
    const float half = width / 2.0F;
    const float nx = -uy * half;
    const float ny = ux * half;
    const float ax = x0 - ux * half;
    const float ay = y0 - uy * half;
    const float bx = x1 + ux * half;
    const float by = y1 + uy * half;
    const SDL_FColor c = gd::fcolor(color);
    const int base = static_cast<int>(verts.size());
    verts.push_back({{ax + nx, ay + ny}, c, {0, 0}});
    verts.push_back({{ax - nx, ay - ny}, c, {0, 0}});
    verts.push_back({{bx - nx, by - ny}, c, {0, 0}});
    verts.push_back({{bx + nx, by + ny}, c, {0, 0}});
    idx.insert(idx.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

void rebuildRivers(App& app) {
    app.riverVerts.clear();
    app.riverIdx.clear();
    app.riverCells = 0;
    app.deadEnds = 0;
    app.riversDirty = false;
    const rv::World& w = app.world;
    const std::vector<int>& dir = activeDir(app);
    const std::vector<int>& acc = activeAcc(app);
    app.deadEnds = rv::countDeadEnds(dir, w.outlet, acc, app.threshold);
    for (int i = 0; i < kMapW * kMapH; ++i) {
        const auto s = static_cast<std::size_t>(i);
        if (w.outlet[s] != 0 || dir[s] < 0 || !rv::isRiver(acc[s], app.threshold)) continue;
        const int to = i + rv::kDy[static_cast<std::size_t>(dir[s])] * kMapW + rv::kDx[static_cast<std::size_t>(dir[s])];
        const bool inLake = app.filled && w.lake[s] != 0;
        const bool toLake = app.filled && w.lake[static_cast<std::size_t>(to)] != 0;
        if (inLake && toLake) continue;  // the lake covers it
        if (!inLake) ++app.riverCells;
        const auto width = static_cast<float>(rv::riverWidth(acc[s], app.threshold));
        addSegment(app.riverVerts, app.riverIdx, cellCenterX(i), cellCenterY(i), cellCenterX(to), cellCenterY(to), width, kRiver);
    }
}

void rebuildDrop(App& app) {
    app.dropVerts.clear();
    app.dropIdx.clear();
    app.dropPath.clear();
    app.dropDirty = false;
    if (app.drop < 0) return;
    app.dropPath = rv::tracePath(activeDir(app), kMapW, app.drop);
    for (std::size_t k = 1; k < app.dropPath.size(); ++k) {
        addSegment(app.dropVerts, app.dropIdx, cellCenterX(app.dropPath[k - 1]), cellCenterY(app.dropPath[k - 1]), cellCenterX(app.dropPath[k]),
                   cellCenterY(app.dropPath[k]), 2.0F, gd::kOrange);
    }
}

void rebuildTexture(App& app) {
    void* pixels = nullptr;
    int pitch = 0;
    app.textureDirty = false;
    if (!SDL_LockTexture(app.mapTexture, nullptr, &pixels, &pitch)) {
        SDL_Log("lock map failed: %s", SDL_GetError());
        return;
    }
    const rv::World& w = app.world;
    for (int y = 0; y < kMapH; ++y) {
        auto* row = static_cast<Uint8*>(pixels) + static_cast<std::ptrdiff_t>(y) * pitch;
        for (int x = 0; x < kMapW; ++x) {
            const auto s = static_cast<std::size_t>(y) * kMapW + static_cast<std::size_t>(x);
            const double h = w.heights[s];
            rv::Rgb c = rv::terrainColor(h, app.sea);
            if (h > app.sea) {
                const double shade = rv::hillshade(w.heights, kMapW, kMapH, x, y);
                c = {static_cast<std::uint8_t>(std::clamp(std::lround(c.r * shade), 0L, 255L)), static_cast<std::uint8_t>(std::clamp(std::lround(c.g * shade), 0L, 255L)),
                     static_cast<std::uint8_t>(std::clamp(std::lround(c.b * shade), 0L, 255L))};
            }
            if (app.water && app.filled && w.lake[s] != 0) c = {kLake.r, kLake.g, kLake.b};
            Uint8* px = row + 4 * x;
            px[0] = c.r;
            px[1] = c.g;
            px[2] = c.b;
            px[3] = 255;
        }
    }
    SDL_UnlockTexture(app.mapTexture);
}

// --- drawing -------------------------------------------------------------------------------------------------------

void drawSlider(App& app, float y, const char* label, const char* value, double t, bool held) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kPanelX + 4.0F, y, label, gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kPanelX + kPanelW - 4.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, gd::kTealDark, gd::Hud::Style::Bold);
    const float ty = y + 29.0F;
    const auto knobX = kTrackX + kTrackW * static_cast<float>(std::clamp(t, 0.0, 1.0));
    gd::fillRect(r, kTrackX, ty - 2.0F, kTrackW, 4.0F, gd::kGrid);
    gd::strokeRect(r, kTrackX, ty - 2.0F, kTrackW, 4.0F, gd::kMuted);
    gd::fillRect(r, kTrackX, ty - 2.0F, knobX - kTrackX, 4.0F, gd::kTeal);
    gd::card(r, knobX - 6.0F, ty - 9.0F, 12.0F, 18.0F, held ? gd::kTeal : gd::kWhite, gd::kInk);
}

void statRow(App& app, float y, const char* label, const char* value, gd::Color color) {
    app.hud.text(app.renderer, kPanelX + 12.0F, y, label, gd::kInk);
    app.hud.text(app.renderer, kPanelX + kPanelW - 12.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, color, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const rv::World& w = app.world;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ RIVERS AND LAKES ]", app.filled ? "sinks filled, D8 flow" : "raw terrain, D8 flow");

    // Map.
    constexpr float mapPxW = static_cast<float>(kMapW * kCell);
    constexpr float mapPxH = static_cast<float>(kMapH * kCell);
    gd::fillRect(r, kMapX + 4.0F, kMapY + 4.0F, mapPxW, mapPxH, gd::Color{17, 24, 39, 40});
    const SDL_FRect dst{kMapX, kMapY, mapPxW, mapPxH};
    SDL_RenderTexture(r, app.mapTexture, nullptr, &dst);
    if (app.water) {
        if (!app.riverVerts.empty()) {
            SDL_RenderGeometry(r, nullptr, app.riverVerts.data(), static_cast<int>(app.riverVerts.size()), app.riverIdx.data(), static_cast<int>(app.riverIdx.size()));
        }
        if (!app.filled) {
            for (int i = 0; i < kMapW * kMapH; ++i) {
                const auto s = static_cast<std::size_t>(i);
                if (w.outlet[s] != 0 || w.rawDir[s] != -1) continue;
                gd::fillRect(r, cellCenterX(i) - 3.0F, cellCenterY(i) - 3.0F, 6.0F, 6.0F, gd::kOrange);
                gd::strokeRect(r, cellCenterX(i) - 3.0F, cellCenterY(i) - 3.0F, 6.0F, 6.0F, gd::kInk);
            }
        }
        if (!app.dropVerts.empty()) {
            SDL_RenderGeometry(r, nullptr, app.dropVerts.data(), static_cast<int>(app.dropVerts.size()), app.dropIdx.data(), static_cast<int>(app.dropIdx.size()));
        }
        if (!app.dropPath.empty()) {
            const int start = app.dropPath.front();
            const int end = app.dropPath.back();
            gd::disc(r, cellCenterX(start), cellCenterY(start), 5.0F, gd::kOrange);
            gd::ring(r, cellCenterX(start), cellCenterY(start), 5.0F, gd::kInk);
            if (w.outlet[static_cast<std::size_t>(end)] == 0) {
                gd::ring(r, cellCenterX(end), cellCenterY(end), 8.0F, gd::kOrange);
                gd::ring(r, cellCenterX(end), cellCenterY(end), 9.0F, gd::kInk);
            }
        }
    }
    gd::strokeRect(r, kMapX, kMapY, mapPxW, mapPxH, gd::kInk);

    // Result panel.
    gd::card(r, kPanelX, kMapY, kPanelW, 176.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 12.0F, kMapY + 8.0F, app.filled ? "SINKS FILLED" : "RAW TERRAIN", app.filled ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
    char buf[32];
    const int pits = app.filled ? 0 : w.rawPits;
    SDL_snprintf(buf, sizeof buf, "%d", pits);
    statRow(app, kMapY + 34.0F, "Pits", buf, pits > 0 ? gd::kOrange : gd::kTealDark);
    SDL_snprintf(buf, sizeof buf, "%d", app.filled ? w.lakeCount : 0);
    statRow(app, kMapY + 56.0F, "Lakes", buf, app.filled ? gd::kTealDark : gd::kMuted);
    SDL_snprintf(buf, sizeof buf, "%d", app.riverCells);
    statRow(app, kMapY + 78.0F, "River cells", buf, gd::kInk);
    SDL_snprintf(buf, sizeof buf, "%d", app.deadEnds);
    statRow(app, kMapY + 100.0F, "Dead ends", buf, app.deadEnds > 0 ? gd::kOrange : gd::kTealDark);
    if (!app.dropPath.empty()) {
        const bool stuck = w.outlet[static_cast<std::size_t>(app.dropPath.back())] == 0;
        SDL_snprintf(buf, sizeof buf, "drop: %d cells", static_cast<int>(app.dropPath.size()));
        app.hud.text(r, kPanelX + 12.0F, kMapY + 128.0F, buf, gd::kInk);
        app.hud.text(r, kPanelX + 12.0F, kMapY + 148.0F, stuck ? "stuck in a pit" : "reaches the sea", stuck ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);
    }

    // Sliders.
    SDL_snprintf(buf, sizeof buf, "%d cells", app.threshold);
    drawSlider(app, 238.0F, "Threshold", buf, rv::tFromThreshold(app.threshold), app.drag == Drag::Threshold);
    SDL_snprintf(buf, sizeof buf, "%.2f", app.sea);
    drawSlider(app, 290.0F, "Sea level", buf, (app.sea - rv::kSeaMin) / (rv::kSeaMax - rv::kSeaMin), app.drag == Drag::Sea);

    // Legend.
    const float ly = 350.0F;
    gd::fillRect(r, kPanelX + 4.0F, ly + 8.0F, 22.0F, 3.0F, kRiver);
    app.hud.text(r, kPanelX + 36.0F, ly, "river", gd::kInk);
    gd::fillRect(r, kPanelX + 112.0F, ly + 2.0F, 14.0F, 14.0F, kLake);
    gd::strokeRect(r, kPanelX + 112.0F, ly + 2.0F, 14.0F, 14.0F, gd::kInk);
    app.hud.text(r, kPanelX + 136.0F, ly, "lake", gd::kInk);
    gd::fillRect(r, kPanelX + 8.0F, ly + 28.0F, 12.0F, 12.0F, gd::kOrange);
    gd::strokeRect(r, kPanelX + 8.0F, ly + 28.0F, 12.0F, 12.0F, gd::kInk);
    app.hud.text(r, kPanelX + 36.0F, ly + 26.0F, "pit", gd::kInk);
    gd::fillRect(r, kPanelX + 112.0F, ly + 33.0F, 22.0F, 3.0F, gd::kOrange);
    app.hud.text(r, kPanelX + 144.0F, ly + 26.0F, "drop", gd::kInk);

    app.hud.text(r, 24.0F, 452.0F, "hover: raindrop   Left/Right: threshold   Up/Down: sea level   Esc: quit", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

bool inRect(float x, float y, float rx, float ry, float rw, float rh) { return x >= rx && x < rx + rw && y >= ry && y < ry + rh; }

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
    if (!SDL_CreateWindowAndRenderer("terrain-rivers", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->mapTexture = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kMapW, kMapH);
    if (app->mapTexture == nullptr) {
        SDL_Log("map texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->mapTexture, SDL_SCALEMODE_NEAREST);
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
            const float mx = event->motion.x;
            const float my = event->motion.y;
            if (app.drag != Drag::None) {
                sliderFromMouse(app, mx);
            } else if (const auto cell = rv::cellAt(static_cast<double>(mx), static_cast<double>(my), static_cast<double>(kMapX), static_cast<double>(kMapY), kCell, kMapW, kMapH)) {
                // Only land is worth following; the sea keeps the previous drop.
                if (*cell != app.drop && app.world.outlet[static_cast<std::size_t>(*cell)] == 0) {
                    app.drop = *cell;
                    app.dropDirty = true;
                }
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const float mx = event->button.x;
            const float my = event->button.y;
            const SDL_FPoint p{mx, my};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inRect(mx, my, kPanelX, 236.0F, kPanelW, 46.0F)) {
                app.drag = Drag::Threshold;
                sliderFromMouse(app, mx);
            } else if (inRect(mx, my, kPanelX, 288.0F, kPanelW, 46.0F)) {
                app.drag = Drag::Sea;
                sliderFromMouse(app, mx);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.drag = Drag::None; break;
        case SDL_EVENT_KEY_DOWN: {
            const bool fresh = !event->key.repeat;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: if (fresh) apply(app, Action::Raw); break;
                case SDLK_2: if (fresh) apply(app, Action::Fill); break;
                case SDLK_W: if (fresh) apply(app, Action::Water); break;
                case SDLK_N: if (fresh) apply(app, Action::NewSeed); break;
                case SDLK_R: if (fresh) apply(app, Action::Reset); break;
                case SDLK_RIGHT: setThreshold(app, rv::thresholdFromT(rv::tFromThreshold(app.threshold) + 0.04)); break;
                case SDLK_LEFT: setThreshold(app, rv::thresholdFromT(rv::tFromThreshold(app.threshold) - 0.04)); break;
                case SDLK_UP: setSea(app, rv::stepValue((app.sea + 0.01 - rv::kSeaMin) / (rv::kSeaMax - rv::kSeaMin), rv::kSeaMin, rv::kSeaMax, 0.01)); break;
                case SDLK_DOWN: setSea(app, rv::stepValue((app.sea - 0.01 - rv::kSeaMin) / (rv::kSeaMax - rv::kSeaMin), rv::kSeaMin, rv::kSeaMax, 0.01)); break;
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
    if (app.worldDirty) rebuildWorld(app);
    if (app.textureDirty) rebuildTexture(app);
    if (app.riversDirty) rebuildRivers(app);
    if (app.dropDirty) rebuildDrop(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->mapTexture != nullptr) SDL_DestroyTexture(app->mapTexture);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
