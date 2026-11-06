// Caves from cellular automata with SDL3 main callbacks.
//
// Random walls, then the 4-5 rule again and again: every cell looks at its eight neighbors and adopts the majority, so the
// noise smooths into blobs. The first picture is already a finished cave, "R Reroll" starts again from noise and plays the
// passes. Two mistakes are shown next to the right thing: updating the map in place (chip 2, the rule reads cells it has
// already rewritten) and trusting that the result is connected (the orange pockets are cut off from the biggest cave, a click
// starts a flood fill that proves it). The rules and the flood fill live in logic.hpp and are covered by test.cpp.

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
#include <utility>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCols = 150;
constexpr int kRows = 62;
constexpr float kCell = 6.0F;
constexpr float kMapX = 30.0F;
constexpr float kMapY = 36.0F;
constexpr float kStatsY = 414.0F;
constexpr float kSliderY = 444.0F;
constexpr float kTrackX = 300.0F;
constexpr float kTrackW = 280.0F;
constexpr float kChipY = 480.0F;
constexpr float kChipH = 34.0F;

constexpr double kDensityLo = 0.30;
constexpr double kDensityHi = 0.60;
constexpr double kDensityStep = 0.01;
constexpr double kDensityStart = 0.45;
constexpr int kStartPasses = 4;
constexpr int kMaxPasses = 60;
constexpr double kAutoInterval = 0.55;   // seconds per pass while Auto plays
constexpr double kFloodSpeed = 80.0;     // flood fill steps revealed per second
constexpr std::uint64_t kStartSeed = 22;  // a seed whose first picture has a big cut-off cave

constexpr std::array<gd::Color, 3> kPocketTints{{{253, 211, 160}, {252, 176, 100}, {250, 150, 60}}};
constexpr gd::Color kWallColor{31, 41, 55};
constexpr gd::Color kMainCave{167, 243, 228};
constexpr gd::Color kFloodFront{94, 234, 212};
constexpr gd::Color kCutOff{234, 88, 12};

enum class Action { Step, Auto, Reroll, Buffered, InPlace, Regions, KeepLargest };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Flood {
    bool active = false;
    int sx = 0;
    int sy = 0;
    std::vector<int> dist;
    int maxDist = 0;
    int reached = 0;
    double front = 0.0;  // cells with dist <= front are revealed
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* mapTexture = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    std::uint64_t seed = kStartSeed;
    std::vector<double> noise;
    double density = kDensityStart;
    int passes = kStartPasses;
    ca::Update mode = ca::Update::Buffered;
    bool showRegions = true;
    bool keepLargest = false;
    bool autoPlay = false;
    double autoClock = 0.0;

    ca::Grid grid{kCols, kRows};
    bool stable = false;
    ca::Regions regions;
    ca::Stats stats;
    Flood flood;
    bool dirtyMap = true;
    bool dirtyTexture = true;

    bool dragSlider = false;
    std::vector<std::uint32_t> pixels;
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

std::uint32_t pack(gd::Color c) {
    // SDL_PIXELFORMAT_RGBA32 is the byte order R, G, B, A in memory on every platform.
    std::uint32_t out = 0;
    const std::uint8_t bytes[4] = {c.r, c.g, c.b, 255};
    std::memcpy(&out, bytes, sizeof out);
    return out;
}

double clampDensity(double v) {
    const double stepped = std::round(v / kDensityStep) * kDensityStep;
    return std::clamp(stepped, kDensityLo, kDensityHi);
}

void clearFlood(App& app) {
    app.flood = Flood{};
    app.dirtyTexture = true;
}

/// Rebuilds the map from noise, density, update mode and pass count. The same noise is reused, so moving the density
/// slider shows how the very same random picture turns out.
void rebuild(App& app) {
    ca::Result result = ca::generate(kCols, kRows, app.noise, app.density, app.passes, app.mode, app.keepLargest);
    app.grid = std::move(result.grid);
    app.stable = result.stable;
    if (result.stable) {
        app.passes = result.changedPasses;
        app.autoPlay = false;
    }
    app.regions = ca::findRegions(app.grid);
    app.stats = ca::measure(app.grid);
    clearFlood(app);
    app.dirtyMap = false;
    app.dirtyTexture = true;
}

void newNoise(App& app) { app.noise = ca::makeNoise(kCols, kRows, app.seed); }

void reroll(App& app) {
    app.seed = app.seed * 6364136223846793005ULL + 1442695040888963407ULL;
    newNoise(app);
    app.passes = 0;
    app.autoPlay = true;
    app.autoClock = 0.0;
    app.dirtyMap = true;
}

void stepOnce(App& app) {
    if (app.stable || app.passes >= kMaxPasses) return;
    ++app.passes;
    app.dirtyMap = true;
}

void setDensity(App& app, double v) {
    const double next = clampDensity(v);
    if (next == app.density) return;
    app.density = next;
    app.dirtyMap = true;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Step: stepOnce(app); break;
        case Action::Auto:
            app.autoPlay = !app.autoPlay;
            app.autoClock = 0.0;
            break;
        case Action::Reroll: reroll(app); break;
        case Action::Buffered:
        case Action::InPlace: {
            const ca::Update want = action == Action::Buffered ? ca::Update::Buffered : ca::Update::InPlace;
            if (want != app.mode) {
                app.mode = want;
                app.dirtyMap = true;
            }
            break;
        }
        case Action::Regions:
            app.showRegions = !app.showRegions;
            app.dirtyTexture = true;
            break;
        case Action::KeepLargest:
            app.keepLargest = !app.keepLargest;
            app.dirtyMap = true;
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Step: return false;
        case Action::Auto: return app.autoPlay;
        case Action::Reroll: return false;
        case Action::Buffered: return app.mode == ca::Update::Buffered;
        case Action::InPlace: return app.mode == ca::Update::InPlace;
        case Action::Regions: return app.showRegions;
        case Action::KeepLargest: return app.keepLargest;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::Step, "Space Step", 0.0F},
                                   {Action::Auto, "A Auto", 8.0F},
                                   {Action::Reroll, "R Reroll", 8.0F},
                                   {Action::Buffered, "1 Double buffer", 20.0F},
                                   {Action::InPlace, "2 In place", 8.0F},
                                   {Action::Regions, "F Regions", 20.0F},
                                   {Action::KeepLargest, "C Keep largest", 8.0F}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 16.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

bool onSlider(float x, float y) { return x >= kTrackX - 10.0F && x <= kTrackX + kTrackW + 10.0F && y >= kSliderY - 4.0F && y <= kSliderY + 30.0F; }

void dragSliderTo(App& app, float mx) {
    const double t = std::clamp(static_cast<double>((mx - kTrackX) / kTrackW), 0.0, 1.0);
    setDensity(app, kDensityLo + t * (kDensityHi - kDensityLo));
}

bool cellAt(float mx, float my, int& cx, int& cy) {
    const float fx = (mx - kMapX) / kCell;
    const float fy = (my - kMapY) / kCell;
    if (fx < 0.0F || fy < 0.0F || fx >= static_cast<float>(kCols) || fy >= static_cast<float>(kRows)) return false;
    cx = static_cast<int>(fx);
    cy = static_cast<int>(fy);
    return true;
}

void startFlood(App& app, int cx, int cy) {
    Flood f;
    if (app.grid.isWall(cx, cy)) {
        clearFlood(app);
        return;
    }
    f.active = true;
    f.sx = cx;
    f.sy = cy;
    f.dist = ca::floodDistances(app.grid, cx, cy);
    for (int d : f.dist) {
        if (d >= 0) ++f.reached;
        f.maxDist = std::max(f.maxDist, d);
    }
    app.flood = std::move(f);
    app.dirtyTexture = true;
}

bool floodFinished(const Flood& f) { return f.active && f.front > static_cast<double>(f.maxDist) + 1.0; }

void paintTexture(App& app) {
    const bool finished = floodFinished(app.flood);
    for (int i = 0; i < kCols * kRows; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        if (app.grid.cells[idx] != 0) {
            app.pixels[idx] = pack(kWallColor);
            continue;
        }
        gd::Color c = gd::kWhite;
        if (app.showRegions) {
            const int label = app.regions.label[idx];
            c = label == app.regions.largest ? kMainCave : kPocketTints[static_cast<std::size_t>(label) % kPocketTints.size()];
        }
        if (app.flood.active) {
            const int d = app.flood.dist[idx];
            if (d >= 0 && static_cast<double>(d) <= app.flood.front) {
                c = app.flood.front - static_cast<double>(d) < 3.0 && !finished ? kFloodFront : gd::kTeal;
            } else if (d < 0 && finished) {
                c = kCutOff;
            }
        }
        app.pixels[idx] = pack(c);
    }
    SDL_UpdateTexture(app.mapTexture, nullptr, app.pixels.data(), kCols * static_cast<int>(sizeof(std::uint32_t)));
    app.dirtyTexture = false;
}

void drawSlider(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, 40.0F, kSliderY, "Wall density", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, 166.0F, kSliderY, "Left/Right", gd::kMuted);
    const float cy = kSliderY + 11.0F;
    const double t = (app.density - kDensityLo) / (kDensityHi - kDensityLo);
    const float knobX = kTrackX + kTrackW * static_cast<float>(t);
    gd::fillRect(r, kTrackX, cy - 2.0F, kTrackW, 4.0F, gd::kGrid);
    gd::strokeRect(r, kTrackX, cy - 2.0F, kTrackW, 4.0F, gd::kMuted);
    gd::fillRect(r, kTrackX, cy - 2.0F, knobX - kTrackX, 4.0F, gd::kTeal);
    gd::card(r, knobX - 6.0F, cy - 9.0F, 12.0F, 18.0F, app.dragSlider ? gd::kTeal : gd::kWhite, gd::kInk);
    char value[16];
    SDL_snprintf(value, sizeof value, "%.2f", app.density);
    app.hud.text(r, kTrackX + kTrackW + 20.0F, kSliderY, value, gd::kTealDark, gd::Hud::Style::Bold);
    const char* hint = "click a cave: flood fill";
    app.hud.text(r, 930.0F - app.hud.width(hint), kSliderY, hint, gd::kMuted);
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const int total = kCols * kRows;
    const int wallPct = static_cast<int>(std::lround(100.0 * app.stats.walls / total));
    const int mainPct = app.stats.floor > 0 ? 100 * app.stats.largest / app.stats.floor : 0;
    char pass[40];
    SDL_snprintf(pass, sizeof pass, "pass %d%s", app.passes, app.stable ? " (stable)" : "");
    char line[120];
    SDL_snprintf(line, sizeof line, "%s · walls %d%% · caves %d · biggest %d%%", pass, wallPct, app.stats.regions, mainPct);
    app.hud.text(r, 30.0F, kStatsY, line, gd::kInk, gd::Hud::Style::Bold);

    char right[96];
    gd::Color color = gd::kMuted;
    if (app.flood.active) {
        const int cut = app.stats.floor - app.flood.reached;
        if (floodFinished(app.flood)) {
            SDL_snprintf(right, sizeof right, "flood reached %d of %d · %d cut off", app.flood.reached, app.stats.floor, cut);
            color = cut > 0 ? gd::kOrange : gd::kTealDark;
        } else {
            SDL_snprintf(right, sizeof right, "flooding …");
        }
    } else {
        SDL_snprintf(right, sizeof right, "teal: biggest cave  orange: cut off");
    }
    app.hud.text(r, 930.0F - app.hud.width(right), kStatsY, right, color, floodFinished(app.flood) ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[48];
    SDL_snprintf(right, sizeof right, "4-5 rule · %s", app.mode == ca::Update::Buffered ? "double buffer" : "in place");
    app.hud.titleBar(r, kWidth, "[ CAVES FROM CELLULAR AUTOMATA ]", right);

    const SDL_FRect dst{kMapX, kMapY, kCell * kCols, kCell * kRows};
    SDL_RenderTexture(r, app.mapTexture, nullptr, &dst);
    gd::strokeRect(r, kMapX - 1.0F, kMapY - 1.0F, dst.w + 2.0F, dst.h + 2.0F, gd::kInk);
    if (app.flood.active) {
        const float sx = kMapX + (static_cast<float>(app.flood.sx) + 0.5F) * kCell;
        const float sy = kMapY + (static_cast<float>(app.flood.sy) + 0.5F) * kCell;
        gd::ring(r, sx, sy, 7.0F, gd::kInk);
        gd::ring(r, sx, sy, 8.0F, gd::kWhite);
    }

    drawStats(app);
    drawSlider(app);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool naive = c.action == Action::InPlace;
        const gd::Color fill = on ? (naive ? gd::kOrange : gd::kTeal) : gd::kWhite;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void update(App& app, double dt) {
    if (app.autoPlay) {
        app.autoClock += dt;
        if (app.autoClock >= kAutoInterval) {
            app.autoClock = 0.0;
            if (app.stable || app.passes >= kMaxPasses) {
                app.autoPlay = false;
            } else {
                stepOnce(app);
            }
        }
    }
    if (app.dirtyMap) rebuild(app);
    if (app.flood.active && !floodFinished(app.flood)) {
        app.flood.front += dt * kFloodSpeed;
        app.dirtyTexture = true;
    }
    if (app.dirtyTexture) paintTexture(app);
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
    if (!SDL_CreateWindowAndRenderer("ca-caves", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    app->mapTexture = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kCols, kRows);
    if (app->mapTexture == nullptr) {
        SDL_Log("map texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->mapTexture, SDL_SCALEMODE_NEAREST);
    app->pixels.assign(static_cast<std::size_t>(kCols * kRows), 0);
    newNoise(*app);
    layoutChips(*app);
    rebuild(*app);
    paintTexture(*app);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.dragSlider) dragSliderTo(app, event->motion.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (onSlider(p.x, p.y)) {
                app.dragSlider = true;
                dragSliderTo(app, p.x);
                break;
            }
            int cx = 0;
            int cy = 0;
            if (cellAt(p.x, p.y, cx, cy)) {
                if (app.dirtyMap) rebuild(app);
                startFlood(app, cx, cy);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragSlider = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragSlider = false; break;
        case SDL_EVENT_KEY_DOWN:
            // Left and right may repeat (hold to sweep the density), every toggle must not.
            if (event->key.key == SDLK_LEFT) {
                setDensity(app, app.density - kDensityStep);
            } else if (event->key.key == SDLK_RIGHT) {
                setDensity(app, app.density + kDensityStep);
            } else if (!event->key.repeat) {
                switch (event->key.key) {
                    case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                    case SDLK_SPACE: apply(app, Action::Step); break;
                    case SDLK_A: apply(app, Action::Auto); break;
                    case SDLK_R: apply(app, Action::Reroll); break;
                    case SDLK_1: apply(app, Action::Buffered); break;
                    case SDLK_2: apply(app, Action::InPlace); break;
                    case SDLK_F: apply(app, Action::Regions); break;
                    case SDLK_C: apply(app, Action::KeepLargest); break;
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
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(0.25, static_cast<double>(now - app.lastTicks) * 1e-9);
    app.lastTicks = now;
    update(app, dt);
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
