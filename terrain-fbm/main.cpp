// Terrain from noise with SDL3 main callbacks.
//
// Two heightmaps from the same settings, side by side: value noise (left, one random number per lattice point) and
// gradient noise (right, one random direction per lattice point), each summed over several octaves (fbm). Value
// noise shows the grid as blocks and diamonds, gradient noise hides it. Below, the layers of the gradient noise one
// by one: a single octave looks like clouds, only the sum looks like land. The fade chips switch the blend curve
// (linear shows the grid in both noises), Normalize divides the sum by the amplitude sum (off: the sum leaves 0..1
// and the part above 1 turns orange). Dragging a map pans the view. The noise, the octave sum, the coordinate and
// slider mapping and the colors live in logic.hpp and are covered by test.cpp.

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
constexpr int kPanel = 280;        // map size on screen in px
constexpr int kTex = 140;          // map resolution; every texel is drawn 2x2 px
constexpr int kLayerSize = 48;     // thumbnail size in px (1:1)
constexpr float kPanelY = 58.0F;
constexpr std::array<float, 2> kPanelX{24.0F, 316.0F};
constexpr float kSliderX = 620.0F;
constexpr float kSliderW = 316.0F;
constexpr float kTrackX = 628.0F;
constexpr float kTrackW = 300.0F;
constexpr float kRowH = 40.0F;
constexpr float kChipY = 490.0F;
constexpr float kChipH = 34.0F;

struct SliderDef {
    const char* label;
    double lo, hi, step;
};
constexpr std::array<SliderDef, 5> kSliders{{{"Octaves", 1.0, 8.0, 1.0},
                                             {"Lacunarity", 1.0, 4.0, 0.1},
                                             {"Persistence", 0.1, 0.9, 0.05},
                                             {"Frequency", 1.0, 16.0, 0.5},
                                             {"Seed", 0.0, 99.0, 1.0}}};

enum class Action { FadeLinear, FadeCubic, FadeQuintic, Terrain, Normalize, Reset };
enum class Drag { None, Slider, Pan };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

/// The naive start: linear blending, so the first frame already shows the grid. Chip 3 turns it into the real thing.
tn::Params startParams() {
    tn::Params p;
    p.octaves = 4;
    p.frequency = 3.0;
    p.fade = tn::Fade::Linear;
    return p;
}

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* maps[2] = {nullptr, nullptr};
    SDL_Texture* layers = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<float> heights;
    tn::Params params = startParams();
    double panX = 0.0;
    double panY = 0.0;
    bool terrain = true;
    bool dirty = true;
    double lo[2] = {0.0, 1.0};
    double hi[2] = {0.0, 1.0};
    Drag drag = Drag::None;
    int slider = 0;
    float mouseX = 0.0F;
    float mouseY = 0.0F;
    long maxFrames = -1;
    long frames = 0;
};

double sliderGet(const tn::Params& p, int i) {
    switch (i) {
        case 0: return p.octaves;
        case 1: return p.lacunarity;
        case 2: return p.persistence;
        case 3: return p.frequency;
        default: return p.seed;
    }
}

void sliderSet(App& app, int i, double v) {
    tn::Params& p = app.params;
    const tn::Params before = p;
    switch (i) {
        case 0: p.octaves = static_cast<int>(std::lround(v)); break;
        case 1: p.lacunarity = v; break;
        case 2: p.persistence = v; break;
        case 3: p.frequency = v; break;
        default: p.seed = static_cast<int>(std::lround(v)); break;
    }
    if (p.octaves != before.octaves || p.lacunarity != before.lacunarity || p.persistence != before.persistence || p.frequency != before.frequency ||
        p.seed != before.seed) {
        app.dirty = true;
    }
}

void sliderNudge(App& app, int i, int direction) {
    const SliderDef& d = kSliders[static_cast<std::size_t>(i)];
    sliderSet(app, i, std::clamp(sliderGet(app.params, i) + direction * d.step, d.lo, d.hi));
}

void sliderFromMouse(App& app, float mx) {
    const SliderDef& d = kSliders[static_cast<std::size_t>(app.slider)];
    const double t = (static_cast<double>(mx) - static_cast<double>(kTrackX)) / static_cast<double>(kTrackW);
    sliderSet(app, app.slider, tn::sliderValue(t, d.lo, d.hi, d.step));
}

float rowTop(int i) { return kPanelY + kRowH * static_cast<float>(i); }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 6> defs{{{Action::FadeLinear, "1 Linear", 0.0F},
                                   {Action::FadeCubic, "2 Cubic", 8.0F},
                                   {Action::FadeQuintic, "3 Quintic", 8.0F},
                                   {Action::Terrain, "T Terrain", 32.0F},
                                   {Action::Normalize, "N Normalize", 8.0F},
                                   {Action::Reset, "R Reset", 32.0F}}};
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
        case Action::FadeLinear: app.params.fade = tn::Fade::Linear; break;
        case Action::FadeCubic: app.params.fade = tn::Fade::Cubic; break;
        case Action::FadeQuintic: app.params.fade = tn::Fade::Quintic; break;
        case Action::Terrain: app.terrain = !app.terrain; break;
        case Action::Normalize: app.params.normalize = !app.params.normalize; break;
        case Action::Reset:
            app.params = startParams();
            app.panX = 0.0;
            app.panY = 0.0;
            app.terrain = true;
            app.drag = Drag::None;
            break;
    }
    app.dirty = true;
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::FadeLinear: return app.params.fade == tn::Fade::Linear;
        case Action::FadeCubic: return app.params.fade == tn::Fade::Cubic;
        case Action::FadeQuintic: return app.params.fade == tn::Fade::Quintic;
        case Action::Terrain: return app.terrain;
        case Action::Normalize: return app.params.normalize;
        case Action::Reset: return false;
    }
    return false;
}

/// Computes the heights of one map (with a one texel margin for the shading), writes the pixels and the value range.
void fillMap(App& app, int index, tn::Kind kind) {
    constexpr int n = kTex + 2;
    constexpr double scale = static_cast<double>(kPanel) / kTex;
    app.heights.resize(static_cast<std::size_t>(n) * n);
    for (int j = 0; j < n; ++j) {
        const double uy = tn::unitCoord((j - 1 + 0.5) * scale, app.panY, kPanel);
        for (int i = 0; i < n; ++i) {
            const double ux = tn::unitCoord((i - 1 + 0.5) * scale, app.panX, kPanel);
            app.heights[static_cast<std::size_t>(j) * n + static_cast<std::size_t>(i)] = static_cast<float>(tn::heightAt(app.params, kind, ux, uy));
        }
    }
    void* pixels = nullptr;
    int pitch = 0;
    if (!SDL_LockTexture(app.maps[index], nullptr, &pixels, &pitch)) {
        SDL_Log("lock map failed: %s", SDL_GetError());
        return;
    }
    double lo = 1e9;
    double hi = -1e9;
    for (int j = 0; j < kTex; ++j) {
        auto* row = static_cast<Uint8*>(pixels) + static_cast<std::ptrdiff_t>(j) * pitch;
        for (int i = 0; i < kTex; ++i) {
            const auto at = [&](int di, int dj) { return static_cast<double>(app.heights[static_cast<std::size_t>(j + 1 + dj) * n + static_cast<std::size_t>(i + 1 + di)]); };
            const double h = at(0, 0);
            lo = std::min(lo, h);
            hi = std::max(hi, h);
            tn::Rgb c = app.terrain ? tn::shade(tn::terrainColor(h), tn::shadeFactor(at(1, 1) - at(-1, -1))) : tn::grayColor(h);
            if (h > 1.0 + 1e-9) c = tn::Rgb{234, 88, 12};  // the sum left [0, 1]
            Uint8* px = row + 4 * i;
            px[0] = c.r;
            px[1] = c.g;
            px[2] = c.b;
            px[3] = 255;
        }
    }
    SDL_UnlockTexture(app.maps[index]);
    app.lo[index] = lo;
    app.hi[index] = hi;
}

/// One thumbnail per octave of the gradient noise, side by side in one texture (unused octaves stay transparent).
void fillLayers(App& app) {
    void* pixels = nullptr;
    int pitch = 0;
    if (!SDL_LockTexture(app.layers, nullptr, &pixels, &pitch)) {
        SDL_Log("lock layers failed: %s", SDL_GetError());
        return;
    }
    const int octaves = tn::clampOctaves(app.params.octaves);
    constexpr double scale = static_cast<double>(kPanel) / kLayerSize;
    // 3x3 samples per pixel: a layer finer than the box can show averages out to gray instead of aliasing (sampled
    // once per pixel, the lattice of a fine octave can line up with the samples and the layer shows nothing at all).
    constexpr int kSub = 3;
    for (int j = 0; j < kLayerSize; ++j) {
        auto* row = static_cast<Uint8*>(pixels) + static_cast<std::ptrdiff_t>(j) * pitch;
        for (int k = 0; k < tn::kMaxOctaves; ++k) {
            for (int i = 0; i < kLayerSize; ++i) {
                Uint8* px = row + 4 * (k * kLayerSize + i);
                if (k >= octaves) {
                    px[0] = px[1] = px[2] = px[3] = 0;
                    continue;
                }
                double sum = 0.0;
                for (int sj = 0; sj < kSub; ++sj) {
                    const double uy = tn::unitCoord((j + (sj + 0.5) / kSub) * scale, app.panY, kPanel);
                    for (int si = 0; si < kSub; ++si) {
                        const double ux = tn::unitCoord((i + (si + 0.5) / kSub) * scale, app.panX, kPanel);
                        sum += tn::layerAt(app.params, tn::Kind::Gradient, k, ux, uy);
                    }
                }
                const tn::Rgb c = tn::grayColor(sum / (kSub * kSub));
                px[0] = c.r;
                px[1] = c.g;
                px[2] = c.b;
                px[3] = 255;
            }
        }
    }
    SDL_UnlockTexture(app.layers);
}

void rebuild(App& app) {
    fillMap(app, 0, tn::Kind::Value);
    fillMap(app, 1, tn::Kind::Gradient);
    fillLayers(app);
    app.dirty = false;
}

void drawSliders(App& app) {
    SDL_Renderer* r = app.renderer;
    for (int i = 0; i < static_cast<int>(kSliders.size()); ++i) {
        const SliderDef& d = kSliders[static_cast<std::size_t>(i)];
        const float y = rowTop(i);
        const double v = sliderGet(app.params, i);
        char value[24];
        if (i == 0 || i == 4) {
            SDL_snprintf(value, sizeof value, "%d", static_cast<int>(std::lround(v)));
        } else {
            SDL_snprintf(value, sizeof value, i == 2 ? "%.2f" : "%.1f", v);
        }
        app.hud.text(r, kSliderX + 4.0F, y, d.label, gd::kInk, gd::Hud::Style::Bold);
        app.hud.text(r, kSliderX + kSliderW - 4.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, gd::kTealDark, gd::Hud::Style::Bold);
        const float ty = y + 29.0F;
        const auto knobX = kTrackX + kTrackW * static_cast<float>(tn::tOfValue(v, d.lo, d.hi));
        gd::fillRect(r, kTrackX, ty - 2.0F, kTrackW, 4.0F, gd::kGrid);
        gd::strokeRect(r, kTrackX, ty - 2.0F, kTrackW, 4.0F, gd::kMuted);
        gd::fillRect(r, kTrackX, ty - 2.0F, knobX - kTrackX, 4.0F, gd::kTeal);
        const bool held = app.drag == Drag::Slider && app.slider == i;
        gd::card(r, knobX - 6.0F, ty - 9.0F, 12.0F, 18.0F, held ? gd::kTeal : gd::kWhite, gd::kInk);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ TERRAIN FROM NOISE ]", "fbm = sum of octaves");

    // Maps with their names and value ranges.
    const char* names[2] = {"VALUE NOISE", "GRADIENT NOISE"};
    const gd::Color nameColor[2] = {gd::kOrange, gd::kTeal};
    for (int m = 0; m < 2; ++m) {
        const float x = kPanelX[static_cast<std::size_t>(m)];
        app.hud.text(r, x, 33.0F, names[m], nameColor[m], gd::Hud::Style::Bold);
        char range[32];
        SDL_snprintf(range, sizeof range, "%.2f .. %.2f", app.lo[m], app.hi[m]);
        app.hud.text(r, x + static_cast<float>(kPanel) - app.hud.width(range), 33.0F, range, app.hi[m] > 1.0 + 1e-9 ? gd::kOrange : gd::kMuted);
        gd::fillRect(r, x + 4.0F, kPanelY + 4.0F, static_cast<float>(kPanel), static_cast<float>(kPanel), gd::Color{17, 24, 39, 40});
        const SDL_FRect dst{x, kPanelY, static_cast<float>(kPanel), static_cast<float>(kPanel)};
        SDL_RenderTexture(r, app.maps[m], nullptr, &dst);
        gd::strokeRect(r, x, kPanelY, static_cast<float>(kPanel), static_cast<float>(kPanel), gd::kInk);
    }

    drawSliders(app);

    // Amplitude sum and what normalizing does with it.
    const double sum = tn::amplitudeSum(app.params.octaves, app.params.persistence);
    gd::card(r, kSliderX, 266.0F, kSliderW, 72.0F, gd::kWhite, gd::kInk);
    char line[64];
    SDL_snprintf(line, sizeof line, "amplitude sum %.2f", sum);
    app.hud.text(r, kSliderX + 12.0F, 270.0F, line, gd::kInk, gd::Hud::Style::Bold);
    if (app.params.normalize) {
        SDL_snprintf(line, sizeof line, "sum / %.2f stays in 0..1", sum);
        app.hud.text(r, kSliderX + 12.0F, 292.0F, line, gd::kTealDark);
    } else {
        app.hud.text(r, kSliderX + 12.0F, 292.0F, "raw sum: orange = above 1", gd::kOrange);
    }
    app.hud.text(r, kSliderX + 12.0F, 314.0F, "drag a map to pan", gd::kMuted);

    // The octave layers.
    app.hud.text(r, 24.0F, 344.0F, "gradient noise, one octave per box: f = frequency, a = amplitude", gd::kMuted);
    const int octaves = tn::clampOctaves(app.params.octaves);
    double f = app.params.frequency;
    double a = 1.0;
    const double persistence = tn::clampPersistence(app.params.persistence);
    for (int k = 0; k < tn::kMaxOctaves; ++k) {
        const float x = 24.0F + 76.0F * static_cast<float>(k);
        const float y = 370.0F;
        if (k < octaves) {
            const SDL_FRect src{static_cast<float>(k * kLayerSize), 0.0F, static_cast<float>(kLayerSize), static_cast<float>(kLayerSize)};
            const SDL_FRect dst{x, y, static_cast<float>(kLayerSize), static_cast<float>(kLayerSize)};
            SDL_RenderTexture(r, app.layers, &src, &dst);
            gd::strokeRect(r, x, y, static_cast<float>(kLayerSize), static_cast<float>(kLayerSize), gd::kInk);
            char cap[24];
            if (f >= 100.0) {
                SDL_snprintf(cap, sizeof cap, "f%.0f", f);
            } else {
                SDL_snprintf(cap, sizeof cap, "f%.1f", f);
            }
            app.hud.text(r, x, y + 50.0F, cap, gd::kInk);
            SDL_snprintf(cap, sizeof cap, "a%.2f", a);
            app.hud.text(r, x, y + 70.0F, cap, a < 0.1 ? gd::kMuted : gd::kTealDark);
        } else {
            gd::fillRect(r, x, y, static_cast<float>(kLayerSize), static_cast<float>(kLayerSize), gd::kGrid);
            gd::strokeRect(r, x, y, static_cast<float>(kLayerSize), static_cast<float>(kLayerSize), gd::kMuted);
        }
        f *= app.params.lacunarity;
        a *= persistence;
    }

    app.hud.text(r, 24.0F, 464.0F, "Up/Down: octaves   Left/Right: seed   Esc: quit", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

bool inRect(float x, float y, float rx, float ry, float rw, float rh) { return x >= rx && x < rx + rw && y >= ry && y < ry + rh; }

SDL_Texture* makeTexture(SDL_Renderer* renderer, int w, int h, SDL_BlendMode blend) {
    SDL_Texture* t = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, w, h);
    if (t == nullptr) return nullptr;
    SDL_SetTextureScaleMode(t, SDL_SCALEMODE_NEAREST);
    SDL_SetTextureBlendMode(t, blend);
    return t;
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
    if (!SDL_CreateWindowAndRenderer("terrain-fbm", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    for (SDL_Texture*& t : app->maps) {
        t = makeTexture(app->renderer, kTex, kTex, SDL_BLENDMODE_NONE);
        if (t == nullptr) {
            SDL_Log("map texture failed: %s", SDL_GetError());
            return SDL_APP_FAILURE;
        }
    }
    app->layers = makeTexture(app->renderer, kLayerSize * tn::kMaxOctaves, kLayerSize, SDL_BLENDMODE_BLEND);
    if (app->layers == nullptr) {
        SDL_Log("layer texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
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
            const float mx = event->motion.x;
            const float my = event->motion.y;
            if (app.drag == Drag::Slider) {
                sliderFromMouse(app, mx);
            } else if (app.drag == Drag::Pan) {
                tn::panBy(app.panX, app.panY, static_cast<double>(mx - app.mouseX), static_cast<double>(my - app.mouseY));
                app.dirty = true;
            }
            app.mouseX = mx;
            app.mouseY = my;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const float mx = event->button.x;
            const float my = event->button.y;
            app.mouseX = mx;
            app.mouseY = my;
            const SDL_FPoint p{mx, my};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int i = 0; i < static_cast<int>(kSliders.size()); ++i) {
                if (inRect(mx, my, kSliderX, rowTop(i) - 4.0F, kSliderW, kRowH - 2.0F)) {
                    app.drag = Drag::Slider;
                    app.slider = i;
                    sliderFromMouse(app, mx);
                    return SDL_APP_CONTINUE;
                }
            }
            for (const float x : kPanelX) {
                if (inRect(mx, my, x, kPanelY, static_cast<float>(kPanel), static_cast<float>(kPanel))) app.drag = Drag::Pan;
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
                case SDLK_1: if (fresh) apply(app, Action::FadeLinear); break;
                case SDLK_2: if (fresh) apply(app, Action::FadeCubic); break;
                case SDLK_3: if (fresh) apply(app, Action::FadeQuintic); break;
                case SDLK_T: if (fresh) apply(app, Action::Terrain); break;
                case SDLK_N: if (fresh) apply(app, Action::Normalize); break;
                case SDLK_R: if (fresh) apply(app, Action::Reset); break;
                case SDLK_UP: sliderNudge(app, 0, 1); break;
                case SDLK_DOWN: sliderNudge(app, 0, -1); break;
                case SDLK_RIGHT: sliderNudge(app, 4, 1); break;
                case SDLK_LEFT: sliderNudge(app, 4, -1); break;
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
    if (app.dirty) rebuild(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        for (SDL_Texture* t : app->maps) {
            if (t != nullptr) SDL_DestroyTexture(t);
        }
        if (app->layers != nullptr) SDL_DestroyTexture(app->layers);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
