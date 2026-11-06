// Islands and coastlines with SDL3 main callbacks.
//
// The terrain is plain fractal noise, so it fills the whole map. An island needs sea at the border, and the cheapest
// way there is a falloff: a number that is 0 in the middle and 1 at the edge, mixed into the height. The left side
// shows the map, the right side shows one row of it as a profile, so you see the three curves that are combined
// (noise, the mask 1 - f, the result) and where the sea level cuts the result. The view chips show each ingredient
// alone: the noise alone is a continent without a coast, the falloff alone is a cone. The logic is in logic.hpp
// and covered by test.cpp.

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
constexpr int kGrid = 160;      // cells per side of the height map
constexpr float kCell = 3.0F;   // screen pixels per cell
constexpr float kMapX = 24.0F;
constexpr float kMapY = 40.0F;
constexpr float kMapSize = static_cast<float>(kGrid) * kCell;  // 480
constexpr float kColX = 520.0F;                                  // right column
constexpr float kColW = 416.0F;
constexpr float kChipH = 30.0F;
constexpr float kSliderX = 690.0F;
constexpr float kSliderW = 190.0F;
constexpr float kSliderH = 26.0F;

enum class Action { Circle, Square, Diamond, Subtract, Lerp, ViewIsland, ViewNoise, ViewFalloff, NewSeed };
enum class View { Island, Noise, Falloff };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct SliderDef {
    const char* label;
    float lo;
    float hi;
    float y;
};

constexpr std::array<SliderDef, 3> kSliders{{{"sea level", 0.0F, 0.8F, 376.0F}, {"falloff strength", 0.0F, 1.0F, 410.0F}, {"falloff curve", 0.5F, 4.0F, 444.0F}}};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* mapTexture = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    isl::Params params;
    isl::Heightmap map;
    std::vector<std::uint8_t> pixels;
    float sea = 0.42F;
    View view = View::Island;
    int row = kGrid / 2;
    int dragSlider = -1;
    bool needBuild = true;
    bool needPaint = true;
    float landPercent = 0.0F;
    bool touches = false;
    SDL_FPoint mouse{-100.0F, -100.0F};
    long maxFrames = -1;
    long frames = 0;
};

bool onMap(SDL_FPoint p) { return p.x >= kMapX && p.x < kMapX + kMapSize && p.y >= kMapY && p.y < kMapY + kMapSize; }

float sliderGet(const App& app, int i) {
    switch (i) {
        case 0: return app.sea;
        case 1: return app.params.strength;
        default: return app.params.exponent;
    }
}

void sliderSet(App& app, int i, float v) {
    switch (i) {
        case 0: app.sea = v; app.needPaint = true; break;
        case 1: app.params.strength = v; app.needBuild = true; break;
        default: app.params.exponent = v; app.needBuild = true; break;
    }
}

SDL_FRect sliderRect(int i) { return SDL_FRect{kSliderX - 8.0F, kSliders[static_cast<std::size_t>(i)].y, kSliderW + 16.0F, kSliderH}; }

void dragTo(App& app, int i, float px) {
    const SliderDef& s = kSliders[static_cast<std::size_t>(i)];
    sliderSet(app, i, isl::sliderValue(px, kSliderX, kSliderW, s.lo, s.hi));
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
    };
    struct Row {
        float y;
        std::vector<Def> defs;
    };
    const std::vector<Row> rows{{260.0F, {{Action::Circle, "1 Circle"}, {Action::Square, "2 Square"}, {Action::Diamond, "3 Diamond"}}},
                                {296.0F, {{Action::Subtract, "4 Subtract"}, {Action::Lerp, "5 Lerp"}, {Action::NewSeed, "R New seed"}}},
                                {332.0F, {{Action::ViewIsland, "6 Island"}, {Action::ViewNoise, "7 Noise"}, {Action::ViewFalloff, "8 Falloff"}}}};
    for (const Row& row : rows) {
        float x = 590.0F;
        for (const Def& d : row.defs) {
            const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 20.0F;
            app.chips.push_back({d.action, d.label, SDL_FRect{x, row.y, w, kChipH}});
            x += w + 8.0F;
        }
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Circle: app.params.shape = isl::Shape::Circle; app.needBuild = true; break;
        case Action::Square: app.params.shape = isl::Shape::Square; app.needBuild = true; break;
        case Action::Diamond: app.params.shape = isl::Shape::Diamond; app.needBuild = true; break;
        case Action::Subtract: app.params.mode = isl::Mode::Subtract; app.needBuild = true; break;
        case Action::Lerp: app.params.mode = isl::Mode::Lerp; app.needBuild = true; break;
        case Action::ViewIsland: app.view = View::Island; app.needPaint = true; break;
        case Action::ViewNoise: app.view = View::Noise; app.needPaint = true; break;
        case Action::ViewFalloff: app.view = View::Falloff; app.needPaint = true; break;
        case Action::NewSeed:
            app.params.seed += 1;
            app.needBuild = true;
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Circle: return app.params.shape == isl::Shape::Circle;
        case Action::Square: return app.params.shape == isl::Shape::Square;
        case Action::Diamond: return app.params.shape == isl::Shape::Diamond;
        case Action::Subtract: return app.params.mode == isl::Mode::Subtract;
        case Action::Lerp: return app.params.mode == isl::Mode::Lerp;
        case Action::ViewIsland: return app.view == View::Island;
        case Action::ViewNoise: return app.view == View::Noise;
        case Action::ViewFalloff: return app.view == View::Falloff;
        case Action::NewSeed: return false;
    }
    return false;
}

std::array<std::uint8_t, 3> bandColor(isl::Band band) {
    switch (band) {
        case isl::Band::Deep: return {25, 60, 105};
        case isl::Band::Shallow: return {70, 150, 175};
        case isl::Band::Beach: return {231, 214, 158};
        case isl::Band::Grass: return {126, 176, 92};
        case isl::Band::Forest: return {64, 123, 72};
        case isl::Band::Rock: return {134, 128, 122};
        case isl::Band::Snow: return {246, 246, 246};
    }
    return {0, 0, 0};
}

std::uint8_t toByte(float v) { return static_cast<std::uint8_t>(std::clamp(v, 0.0F, 255.0F)); }

void paint(App& app) {
    const int n = app.map.n;
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const auto i = static_cast<std::size_t>(y * n + x);
            std::array<float, 3> c{};
            if (app.view == View::Island) {
                const float h = app.map.height[i];
                const isl::Band band = isl::classify(h, app.sea);
                const auto base = bandColor(band);
                if (band == isl::Band::Deep || band == isl::Band::Shallow) {
                    // Water gets darker with depth.
                    const float depth = std::clamp((app.sea - h) / 0.3F, 0.0F, 1.0F);
                    const auto shallow = bandColor(isl::Band::Shallow);
                    const auto deep = bandColor(isl::Band::Deep);
                    for (std::size_t k = 0; k < 3; ++k) c[k] = isl::lerp(static_cast<float>(shallow[k]), static_cast<float>(deep[k]), depth);
                } else {
                    // Light from the upper left: compare the cell with its lower right neighbour.
                    const auto lit = app.map.height[static_cast<std::size_t>(std::max(0, y - 1) * n + std::max(0, x - 1))];
                    const auto dark = app.map.height[static_cast<std::size_t>(std::min(n - 1, y + 1) * n + std::min(n - 1, x + 1))];
                    const float shade = 1.0F + std::clamp((lit - dark) * 7.0F, -0.35F, 0.35F);
                    for (std::size_t k = 0; k < 3; ++k) c[k] = static_cast<float>(base[k]) * shade;
                }
            } else {
                const float v = (app.view == View::Noise ? app.map.noise[i] : app.map.falloff[i]) * 235.0F + 10.0F;
                c = {v, v, v};
            }
            const std::size_t o = i * 4;
            app.pixels[o] = toByte(c[0]);
            app.pixels[o + 1] = toByte(c[1]);
            app.pixels[o + 2] = toByte(c[2]);
            app.pixels[o + 3] = 255;
        }
    }
    SDL_UpdateTexture(app.mapTexture, nullptr, app.pixels.data(), n * 4);
    app.landPercent = 100.0F * isl::landFraction(app.map.height, app.sea);
    app.touches = isl::touchesBorder(app.map.height, n, app.sea);
    app.needPaint = false;
}

void refresh(App& app) {
    if (app.needBuild) {
        app.map = isl::build(app.params, kGrid);
        app.needBuild = false;
        app.needPaint = true;
    }
    if (app.needPaint) paint(app);
}

void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c) {
    gd::setColor(r, c);
    SDL_RenderLine(r, x0, y0, x1, y1);
    SDL_RenderLine(r, x0, y0 + 1.0F, x1, y1 + 1.0F);
}

void drawProfile(App& app) {
    SDL_Renderer* r = app.renderer;
    constexpr float kPy0 = 70.0F;  // y of value 1
    constexpr float kPh = 100.0F;  // pixels between value 1 and value 0
    constexpr float kPx0 = 536.0F;
    constexpr float kPw = 384.0F;
    const int n = app.map.n;
    const auto rowBase = static_cast<std::size_t>(app.row * n);
    const auto yOf = [](float v) { return kPy0 + kPh * (1.0F - std::clamp(v, 0.0F, 1.0F)); };
    const auto xOf = [n](int j) { return kPx0 + (static_cast<float>(j) + 0.5F) / static_cast<float>(n) * kPw; };

    gd::card(r, kColX, 40.0F, kColW, 164.0F, gd::kWhite, gd::kInk);
    char title[48];
    SDL_snprintf(title, sizeof title, "profile of row y = %.2f", static_cast<double>(isl::cellToNorm(app.row, n) * 0.5F + 0.5F));
    app.hud.text(r, kColX + 14.0F, 46.0F, title, gd::kInk, gd::Hud::Style::Bold);

    gd::setColor(r, gd::kGrid);
    SDL_RenderLine(r, kPx0, yOf(0.0F), kPx0 + kPw, yOf(0.0F));
    SDL_RenderLine(r, kPx0, yOf(1.0F), kPx0 + kPw, yOf(1.0F));

    // Land part of the result, filled between the sea line and the curve.
    const float colW = kPw / static_cast<float>(n);
    for (int j = 0; j < n; ++j) {
        const float h = app.map.height[rowBase + static_cast<std::size_t>(j)];
        if (isl::isLand(h, app.sea)) gd::fillRect(r, xOf(j) - colW / 2.0F, yOf(h), colW + 0.5F, yOf(app.sea) - yOf(h), gd::kTealSoft);
    }
    // Sea level, dashed.
    gd::setColor(r, gd::kInk);
    for (float x = kPx0; x < kPx0 + kPw; x += 10.0F) SDL_RenderLine(r, x, yOf(app.sea), std::min(x + 5.0F, kPx0 + kPw), yOf(app.sea));

    for (int j = 0; j + 1 < n; ++j) {
        const auto a = rowBase + static_cast<std::size_t>(j);
        thickLine(r, xOf(j), yOf(app.map.noise[a]), xOf(j + 1), yOf(app.map.noise[a + 1]), gd::kMuted);
        thickLine(r, xOf(j), yOf(1.0F - app.map.falloff[a]), xOf(j + 1), yOf(1.0F - app.map.falloff[a + 1]), gd::kOrange);
        thickLine(r, xOf(j), yOf(app.map.height[a]), xOf(j + 1), yOf(app.map.height[a + 1]), gd::kTeal);
    }

    float x = kColX + 14.0F;
    const float ly = 178.0F;
    for (const auto& [text, color] : {std::pair<const char*, gd::Color>{"noise", gd::kMuted}, {"mask 1-f", gd::kOrange}, {"height", gd::kTeal}, {"sea level", gd::kInk}}) {
        app.hud.text(r, x, ly, text, color, gd::Hud::Style::Bold);
        x += app.hud.width(text, gd::Hud::Style::Bold) + 22.0F;
    }
}

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kMapX, kMapY, kMapSize, kMapSize, gd::kWhite, gd::kInk);
    const SDL_FRect dst{kMapX, kMapY, kMapSize, kMapSize};
    SDL_RenderTexture(r, app.mapTexture, nullptr, &dst);
    gd::strokeRect(r, kMapX, kMapY, kMapSize, kMapSize, gd::kInk);
    // The row of the profile.
    const float y = kMapY + (static_cast<float>(app.row) + 0.5F) * kCell;
    gd::setColor(r, gd::Color{17, 24, 39, 200});
    for (float x = kMapX; x < kMapX + kMapSize; x += 12.0F) SDL_RenderLine(r, x, y, std::min(x + 6.0F, kMapX + kMapSize), y);
    gd::fillRect(r, kMapX - 8.0F, y - 4.0F, 8.0F, 8.0F, gd::kTeal);
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    char line[96];
    SDL_snprintf(line, sizeof line, "land %.0f %%", static_cast<double>(app.landPercent));
    app.hud.text(r, kColX, 212.0F, line, gd::kInk, gd::Hud::Style::Bold);
    const bool cone = app.params.mode == isl::Mode::Lerp && app.params.strength >= 0.95F;
    const char* status = "island, sea all around";
    gd::Color statusColor = gd::kTeal;
    if (app.touches) {
        status = "land runs off the map edge";
        statusColor = gd::kOrange;
    } else if (cone) {
        status = "only the cone is left";
        statusColor = gd::kOrange;
    } else if (app.landPercent < 1.0F) {
        status = "no land at all";
        statusColor = gd::kOrange;
    }
    app.hud.text(r, kColX + 130.0F, 212.0F, status, statusColor, gd::Hud::Style::Bold);

    if (onMap(app.mouse)) {
        const int n = app.map.n;
        const int cx = std::clamp(static_cast<int>((app.mouse.x - kMapX) / kMapSize * static_cast<float>(n)), 0, n - 1);
        const auto i = static_cast<std::size_t>(isl::rowForY(app.mouse.y, kMapY, kMapSize, n) * n + cx);
        SDL_snprintf(line, sizeof line, "cursor  noise %.2f  mask %.2f  height %.2f", static_cast<double>(app.map.noise[i]), static_cast<double>(1.0F - app.map.falloff[i]),
                     static_cast<double>(app.map.height[i]));
    } else {
        SDL_snprintf(line, sizeof line, "hover the map for the numbers");
    }
    app.hud.text(r, kColX, 234.0F, line, gd::kMuted);

    app.hud.text(r, kColX, 264.0F, "shape", gd::kMuted);
    app.hud.text(r, kColX, 300.0F, "mode", gd::kMuted);
    app.hud.text(r, kColX, 336.0F, "view", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }

    for (int i = 0; i < static_cast<int>(kSliders.size()); ++i) {
        const SliderDef& s = kSliders[static_cast<std::size_t>(i)];
        const float v = sliderGet(app, i);
        const float t = std::clamp((v - s.lo) / (s.hi - s.lo), 0.0F, 1.0F);
        app.hud.text(r, kColX, s.y + 2.0F, s.label, gd::kInk);
        gd::fillRect(r, kSliderX, s.y + 10.0F, kSliderW, 6.0F, gd::kGrid);
        gd::fillRect(r, kSliderX, s.y + 10.0F, kSliderW * t, 6.0F, gd::kTeal);
        gd::strokeRect(r, kSliderX, s.y + 10.0F, kSliderW, 6.0F, gd::kInk);
        gd::card(r, kSliderX + kSliderW * t - 5.0F, s.y + 2.0F, 10.0F, 22.0F, app.dragSlider == i ? gd::kOrange : gd::kWhite, gd::kInk);
        SDL_snprintf(line, sizeof line, "%.2f", static_cast<double>(v));
        app.hud.text(r, kSliderX + kSliderW + 16.0F, s.y + 2.0F, line, gd::kInk, gd::Hud::Style::Bold);
    }

    app.hud.text(r, kColX, 488.0F, "hover the map: pick the row", gd::kMuted);
    app.hud.text(r, kColX, 508.0F, "drag the sliders · keys 1-8 · R", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    const char* shapes[] = {"circle", "square", "diamond"};
    char right[64];
    SDL_snprintf(right, sizeof right, "%s · %s · seed %u", shapes[static_cast<int>(app.params.shape)], app.params.mode == isl::Mode::Lerp ? "lerp" : "subtract",
                 static_cast<unsigned>(app.params.seed));
    app.hud.titleBar(r, kWidth, "[ ISLANDS AND COASTLINES ]", right);
    drawMap(app);
    drawProfile(app);
    drawPanel(app);
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
    if (!SDL_CreateWindowAndRenderer("terrain-island", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->mapTexture = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kGrid, kGrid);
    if (app->mapTexture == nullptr) {
        SDL_Log("map texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->mapTexture, SDL_SCALEMODE_NEAREST);
    app->pixels.assign(static_cast<std::size_t>(kGrid) * kGrid * 4U, 255);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    refresh(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {event->motion.x, event->motion.y};
            if (app.dragSlider >= 0) dragTo(app, app.dragSlider, app.mouse.x);
            if (app.dragSlider < 0 && onMap(app.mouse)) app.row = isl::rowForY(app.mouse.y, kMapY, kMapSize, kGrid);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c.action);
            }
            for (int i = 0; i < static_cast<int>(kSliders.size()); ++i) {
                const SDL_FRect rect = sliderRect(i);
                if (SDL_PointInRectFloat(&p, &rect)) {
                    app.dragSlider = i;
                    dragTo(app, i, p.x);
                }
            }
            if (onMap(p)) app.row = isl::rowForY(p.y, kMapY, kMapSize, kGrid);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragSlider = -1;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.dragSlider = -1;
            app.mouse = {-100.0F, -100.0F};
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Circle); break;
                case SDLK_2: apply(app, Action::Square); break;
                case SDLK_3: apply(app, Action::Diamond); break;
                case SDLK_4: apply(app, Action::Subtract); break;
                case SDLK_5: apply(app, Action::Lerp); break;
                case SDLK_6: apply(app, Action::ViewIsland); break;
                case SDLK_7: apply(app, Action::ViewNoise); break;
                case SDLK_8: apply(app, Action::ViewFalloff); break;
                case SDLK_R: apply(app, Action::NewSeed); break;
                default: break;
            }
            break;
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
        app->hud.destroy();
        if (app->mapTexture != nullptr) SDL_DestroyTexture(app->mapTexture);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
