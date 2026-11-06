// Gamma and linear light with SDL3 main callbacks.
//
// Top: two bars blend the same two colors. One interpolates the sRGB code values (what a lerp on the stored pixel
// does), the other interpolates the light and encodes the result. The panel next to them prints what actually leaves
// the screen. Bottom: four spheres lit by the same light, one correct pipeline and three typical mistakes.
// Everything that can be wrong (the sRGB curve, the mixes, the four pipelines, the geometry of the spheres) lives in
// logic.hpp and is covered by test.cpp. The mouse over the spheres moves the light; the sphere pixels are only
// recomputed when the light or the texture changes.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

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

// Bars.
constexpr float kBarX = 24.0F;
constexpr float kBarW = 536.0F;
constexpr float kBarH = 38.0F;
constexpr float kBar1Y = 78.0F;
constexpr float kBar2Y = 148.0F;
constexpr float kHandleY = 202.0F;
constexpr float kGrabTop = 72.0F;
constexpr float kGrabBottom = 214.0F;

// Spheres.
constexpr int kSphereCount = 4;
constexpr float kRadius = 66.0F;
constexpr int kCell = 140;  // texture cell per sphere, a few transparent pixels of margin around the rim
constexpr float kCardY = 246.0F;
constexpr float kCardH = 196.0F;
constexpr float kCardW = 212.0F;
constexpr float kSphereCy = 346.0F;
constexpr double kAmbient = 0.03;
constexpr int kCheckerCells = 6;
constexpr float kLightZoneTop = 240.0F;
constexpr float kLightZoneBottom = 480.0F;

constexpr float kChipY = 484.0F;
constexpr float kChipH = 34.0F;

struct Pair {
    const char* label;
    gam::Rgb a;
    gam::Rgb b;
};

const std::array<Pair, 3> kPairs{{
    {"Black/White", {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}},
    {"Red/Green", {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
    {"Teal/Orange", {13.0 / 255.0, 148.0 / 255.0, 136.0 / 255.0}, {234.0 / 255.0, 88.0 / 255.0, 12.0 / 255.0}},
}};

struct Albedo {
    gam::Rgb even;
    gam::Rgb odd;
};

// 0: a colored checker, 1: one plain mid gray (code value 128).
const std::array<Albedo, 2> kAlbedos{{
    {{214.0 / 255.0, 96.0 / 255.0, 48.0 / 255.0}, {238.0 / 255.0, 222.0 / 255.0, 190.0 / 255.0}},
    {{128.0 / 255.0, 128.0 / 255.0, 128.0 / 255.0}, {128.0 / 255.0, 128.0 / 255.0, 128.0 / 255.0}},
}};

struct SphereInfo {
    gam::Pipeline pipeline;
    const char* name;
    const char* formula;
    bool correct;
};

const std::array<SphereInfo, kSphereCount> kSpheres{{
    {gam::Pipeline::Naive, "NAIVE", "a * L", false},
    {gam::Pipeline::Correct, "CORRECT", "enc(dec(a) * L)", true},
    {gam::Pipeline::RawAlbedo, "TEXTURE NOT DECODED", "enc(a * L)", false},
    {gam::Pipeline::EncodedTwice, "ENCODED TWICE", "enc(enc(dec(a) * L))", false},
}};

enum class Action { Pair0, Pair1, Pair2, AlbedoColor, AlbedoGray };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* spheres = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<std::uint8_t> pixels;  // RGBA, kSphereCount cells side by side
    int pair = 0;
    int albedo = 0;
    double t = 0.5;
    gam::Vec3 light = gam::lightFromMouse(300.0, 250.0);
    bool dragging = false;
    bool dirty = true;
    long maxFrames = -1;
    long frames = 0;
};

gd::Color toColor(gam::Rgb c) { return gd::Color{gam::toByte(c.r), gam::toByte(c.g), gam::toByte(c.b)}; }

int percent(double v) { return static_cast<int>(std::lround(v * 100.0)); }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 5> defs{{{Action::Pair0, "1 Black/White", 0.0F},
                                   {Action::Pair1, "2 Red/Green", 8.0F},
                                   {Action::Pair2, "3 Teal/Orange", 8.0F},
                                   {Action::AlbedoColor, "4 Colored sphere", 32.0F},
                                   {Action::AlbedoGray, "5 Gray sphere", 8.0F}}};
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
        case Action::Pair0: app.pair = 0; break;
        case Action::Pair1: app.pair = 1; break;
        case Action::Pair2: app.pair = 2; break;
        case Action::AlbedoColor:
            app.albedo = 0;
            app.dirty = true;
            break;
        case Action::AlbedoGray:
            app.albedo = 1;
            app.dirty = true;
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Pair0: return app.pair == 0;
        case Action::Pair1: return app.pair == 1;
        case Action::Pair2: return app.pair == 2;
        case Action::AlbedoColor: return app.albedo == 0;
        case Action::AlbedoGray: return app.albedo == 1;
    }
    return false;
}

void setT(App& app, double x) { app.t = gam::sliderT(x, static_cast<double>(kBarX), static_cast<double>(kBarW)); }

/// Redraws all four spheres into the shared texture.
bool renderSpheres(App& app) {
    std::fill(app.pixels.begin(), app.pixels.end(), static_cast<std::uint8_t>(0));
    const Albedo& tex = kAlbedos[static_cast<std::size_t>(app.albedo)];
    const double radius = static_cast<double>(kRadius);
    const double center = static_cast<double>(kCell) / 2.0;
    const std::size_t stride = static_cast<std::size_t>(kSphereCount * kCell) * 4;
    for (int y = 0; y < kCell; ++y) {
        for (int x = 0; x < kCell; ++x) {
            const double px = static_cast<double>(x) + 0.5 - center;
            const double py = center - (static_cast<double>(y) + 0.5);
            const double dist = std::sqrt(px * px + py * py);
            const double cov = gam::coverage(dist, radius);
            if (cov <= 0.0) continue;
            const double pull = dist > radius ? radius / dist : 1.0;  // rim pixels use the nearest point on the circle
            gam::Vec3 n{};
            if (!gam::sphereNormal(px * pull, py * pull, radius, n)) continue;
            const double light = gam::lambert(n, app.light, kAmbient);
            const gam::Rgb a = gam::checkerCell(n, kCheckerCells) ? tex.even : tex.odd;
            for (int s = 0; s < kSphereCount; ++s) {
                const gam::Pipeline p = kSpheres[static_cast<std::size_t>(s)].pipeline;
                std::uint8_t* out = &app.pixels[static_cast<std::size_t>(y) * stride + static_cast<std::size_t>(s * kCell + x) * 4];
                out[0] = gam::toByte(gam::shade(p, a.r, light));
                out[1] = gam::toByte(gam::shade(p, a.g, light));
                out[2] = gam::toByte(gam::shade(p, a.b, light));
                out[3] = gam::toByte(cov);
            }
        }
    }
    if (!SDL_UpdateTexture(app.spheres, nullptr, app.pixels.data(), static_cast<int>(stride))) {
        SDL_Log("SDL_UpdateTexture failed: %s", SDL_GetError());
        return false;
    }
    app.dirty = false;
    return true;
}

void drawBar(App& app, float y, bool linear) {
    SDL_Renderer* r = app.renderer;
    const Pair& pr = kPairs[static_cast<std::size_t>(app.pair)];
    constexpr float kColumn = 2.0F;
    const int columns = static_cast<int>(kBarW / kColumn);
    for (int i = 0; i < columns; ++i) {
        const double t = (static_cast<double>(i) + 0.5) / static_cast<double>(columns);
        const gam::Rgb c = linear ? gam::mixLinear(pr.a, pr.b, t) : gam::mixSrgb(pr.a, pr.b, t);
        gd::fillRect(r, kBarX + static_cast<float>(i) * kColumn, y, kColumn, kBarH, toColor(c));
    }
    gd::strokeRect(r, kBarX, y, kBarW, kBarH, gd::kInk);
}

void drawMixRow(App& app, float y, const char* title, gd::Color titleColor, gam::Rgb mixed) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, 598.0F, y, 70.0F, 56.0F, toColor(mixed));
    gd::strokeRect(r, 598.0F, y, 70.0F, 56.0F, gd::kInk);
    app.hud.text(r, 680.0F, y, title, titleColor, gd::Hud::Style::Bold);
    char line[64];
    SDL_snprintf(line, sizeof line, "rgb %d %d %d", gam::toByte(mixed.r), gam::toByte(mixed.g), gam::toByte(mixed.b));
    app.hud.text(r, 680.0F, y + 22.0F, line, gd::kInk);
    SDL_snprintf(line, sizeof line, "light %d %%", percent(gam::luminance(gam::decode(mixed))));
    app.hud.text(r, 680.0F, y + 44.0F, line, gd::kInk, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const Pair& pr = kPairs[static_cast<std::size_t>(app.pair)];
    const gam::Rgb naive = gam::mixSrgb(pr.a, pr.b, app.t);
    const gam::Rgb right = gam::mixLinear(pr.a, pr.b, app.t);

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ GAMMA AND LINEAR LIGHT ]", "exact sRGB curve");

    // Bars.
    app.hud.text(r, kBarX, 52.0F, "mixed as sRGB values (naive)", gd::kOrange, gd::Hud::Style::Bold);
    drawBar(app, kBar1Y, false);
    app.hud.text(r, kBarX, 122.0F, "mixed as light, then encoded", gd::kTealDark, gd::Hud::Style::Bold);
    drawBar(app, kBar2Y, true);

    const float mx = kBarX + static_cast<float>(app.t) * kBarW;
    // The marker skips the caption of the lower bar so it does not cross the text.
    gd::fillRect(r, mx - 1.0F, kBar1Y - 4.0F, 3.0F, kBarH + 8.0F, gd::kInk);
    gd::fillRect(r, mx - 1.0F, kBar2Y - 4.0F, 3.0F, kHandleY - kBar2Y + 4.0F, gd::kInk);
    gd::disc(r, mx, kHandleY, 8.0F, app.dragging ? gd::kTeal : gd::kWhite);
    gd::ring(r, mx, kHandleY, 8.0F, gd::kInk);
    gd::ring(r, mx, kHandleY, 9.0F, gd::kInk);
    app.hud.text(r, kBarX, 222.0F, "drag on the bars, or press left / right", gd::kMuted);

    // Result panel.
    gd::card(r, 584.0F, 44.0F, 352.0F, 176.0F, gd::kWhite, gd::kInk);
    char head[48];
    SDL_snprintf(head, sizeof head, "%d %% of the way to B", percent(app.t));
    app.hud.text(r, 598.0F, 52.0F, head, gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, 820.0F, 52.0F, "A", gd::kMuted, gd::Hud::Style::Bold);
    gd::fillRect(r, 834.0F, 52.0F, 18.0F, 18.0F, toColor(pr.a));
    gd::strokeRect(r, 834.0F, 52.0F, 18.0F, 18.0F, gd::kInk);
    app.hud.text(r, 868.0F, 52.0F, "B", gd::kMuted, gd::Hud::Style::Bold);
    gd::fillRect(r, 882.0F, 52.0F, 18.0F, 18.0F, toColor(pr.b));
    gd::strokeRect(r, 882.0F, 52.0F, 18.0F, 18.0F, gd::kInk);
    drawMixRow(app, 82.0F, "sRGB values mixed", gd::kOrange, naive);
    drawMixRow(app, 152.0F, "light mixed", gd::kTealDark, right);

    // Spheres.
    for (int s = 0; s < kSphereCount; ++s) {
        const SphereInfo& info = kSpheres[static_cast<std::size_t>(s)];
        const float cx = 138.0F + 228.0F * static_cast<float>(s);
        gd::card(r, cx - kCardW / 2.0F, kCardY, kCardW, kCardH, info.correct ? gd::kTealSoft : gd::kWhite, info.correct ? gd::kTealDark : gd::kInk);
        const gd::Color nameColor = info.correct ? gd::kTealDark : gd::kOrange;
        app.hud.text(r, cx - app.hud.width(info.name, gd::Hud::Style::Bold) / 2.0F, kCardY + 8.0F, info.name, nameColor, gd::Hud::Style::Bold);
        const SDL_FRect src{static_cast<float>(s * kCell), 0.0F, static_cast<float>(kCell), static_cast<float>(kCell)};
        const SDL_FRect dst{cx - static_cast<float>(kCell) / 2.0F, kSphereCy - static_cast<float>(kCell) / 2.0F, static_cast<float>(kCell), static_cast<float>(kCell)};
        SDL_RenderTexture(r, app.spheres, &src, &dst);
        app.hud.text(r, cx - app.hud.width(info.formula) / 2.0F, kCardY + kCardH - 30.0F, info.formula, gd::kInk);
    }
    app.hud.text(r, 24.0F, 452.0F, "mouse over the spheres: move the light     a = texture color, L = light", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

bool inBars(float x, float y) { return x >= kBarX - 12.0F && x <= kBarX + kBarW + 12.0F && y >= kGrabTop && y <= kGrabBottom; }

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
    if (!SDL_CreateWindowAndRenderer("gamma-linear", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->spheres = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kSphereCount * kCell, kCell);
    if (app->spheres == nullptr) {
        SDL_Log("sphere texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureBlendMode(app->spheres, SDL_BLENDMODE_BLEND);
    app->pixels.assign(static_cast<std::size_t>(kSphereCount * kCell * kCell) * 4, 0);
    layoutChips(*app);
    if (!renderSpheres(*app)) return SDL_APP_FAILURE;
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            const float x = event->motion.x;
            const float y = event->motion.y;
            if (app.dragging) {
                setT(app, static_cast<double>(x));
            } else if (y >= kLightZoneTop && y < kLightZoneBottom) {
                app.light = gam::lightFromMouse(static_cast<double>(x), static_cast<double>(y));
                app.dirty = true;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inBars(p.x, p.y)) {
                app.dragging = true;
                setT(app, static_cast<double>(p.x));
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging = false; break;
        case SDL_EVENT_KEY_DOWN:
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                // Stepping the slider is meant to repeat when the key is held.
                case SDLK_LEFT: app.t = gam::clamp01(app.t - 0.02); break;
                case SDLK_RIGHT: app.t = gam::clamp01(app.t + 0.02); break;
                default: break;
            }
            if (!event->key.repeat) {
                switch (event->key.key) {
                    case SDLK_1: apply(app, Action::Pair0); break;
                    case SDLK_2: apply(app, Action::Pair1); break;
                    case SDLK_3: apply(app, Action::Pair2); break;
                    case SDLK_4: apply(app, Action::AlbedoColor); break;
                    case SDLK_5: apply(app, Action::AlbedoGray); break;
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
    if (app.dirty && !renderSpheres(app)) return SDL_APP_FAILURE;
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->spheres != nullptr) SDL_DestroyTexture(app->spheres);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
