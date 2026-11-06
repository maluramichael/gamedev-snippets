// Ordered dithering with SDL3 main callbacks.
//
// A generated test image (gray ramp, lit sphere, hue chart) is reduced to a tiny palette with a tiled Bayer threshold
// map. The mouse moves a split: original on the left, the palette version on the right, and a loupe shows both
// side by side. The same image can be dithered in sRGB code values (the usual mistake, midtones come out too bright)
// or in linear light (the share of bright pixels matches the light of the original). All math lives in logic.hpp and
// is covered by test.cpp; this file only builds the textures and draws.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
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
constexpr int kImgW = 312;
constexpr int kImgH = 192;
constexpr int kScale = 2;
constexpr float kImgX = 24.0F;
constexpr float kImgY = 44.0F;
constexpr float kImgDispW = static_cast<float>(kImgW * kScale);
constexpr float kImgDispH = static_cast<float>(kImgH * kScale);
constexpr int kLoupe = 16;  // source pixels per loupe side
constexpr float kLoupeSize = 112.0F;
constexpr float kPanelX = 668.0F;
constexpr float kPanelW = 268.0F;
constexpr int kBlock = 8;  // block size of the error measure
constexpr float kChipH = 30.0F;

enum class Kind { Matrix, Palette, Space };

struct Chip {
    Kind kind;
    int value;
    std::string label;
    SDL_FRect rect;
};

constexpr std::array<od::Pattern, 5> kPatterns{od::Pattern::Off, od::Pattern::Bayer2, od::Pattern::Bayer4, od::Pattern::Bayer8, od::Pattern::Noise};
constexpr std::array<od::Palette, 4> kPalettes{{{2, true}, {4, true}, {2, false}, {4, false}}};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    SDL_Surface* scratch = nullptr;  // RGBA staging surface, its pitch is what we write rows with
    SDL_Texture* srcTex = nullptr;
    SDL_Texture* outTex = nullptr;
    od::Image src;
    od::Image out;
    std::vector<Chip> chips;
    int pattern = 2;  // index into kPatterns
    int palette = 0;  // index into kPalettes
    od::Space space = od::Space::Linear;
    double error = 0.0;  // percent, see od::blockError
    int split = 96;      // image pixels left of the divider show the original
    bool hover = false;
    int focusX = 0;
    int focusY = 0;
    long maxFrames = -1;
    long frames = 0;
};

const char* patternName(od::Pattern p) {
    switch (p) {
        case od::Pattern::Off: return "no matrix";
        case od::Pattern::Bayer2: return "Bayer 2x2";
        case od::Pattern::Bayer4: return "Bayer 4x4";
        case od::Pattern::Bayer8: return "Bayer 8x8";
        case od::Pattern::Noise: return "white noise";
    }
    return "";
}

bool upload(App& app, const od::Image& img, SDL_Texture* tex) {
    od::toRgba(img, static_cast<std::uint8_t*>(app.scratch->pixels), app.scratch->pitch);
    if (!SDL_UpdateTexture(tex, nullptr, app.scratch->pixels, app.scratch->pitch)) {
        SDL_Log("texture update failed: %s", SDL_GetError());
        return false;
    }
    return true;
}

bool rebuild(App& app) {
    app.out = od::dither(app.src, kPalettes[static_cast<std::size_t>(app.palette)], kPatterns[static_cast<std::size_t>(app.pattern)], app.space);
    app.error = od::blockError(app.src, app.out, kBlock) * 100.0;
    return upload(app, app.out, app.outTex);
}

void layoutChips(App& app) {
    struct Def {
        Kind kind;
        int value;
        const char* label;
        int row;
        float gapBefore;
    };
    const std::array<Def, 11> defs{{{Kind::Matrix, 0, "1 Off", 0, 0.0F},
                                    {Kind::Matrix, 1, "2 Bayer 2x2", 0, 8.0F},
                                    {Kind::Matrix, 2, "3 Bayer 4x4", 0, 8.0F},
                                    {Kind::Matrix, 3, "4 Bayer 8x8", 0, 8.0F},
                                    {Kind::Matrix, 4, "5 Noise", 0, 8.0F},
                                    {Kind::Palette, 0, "6 B/W", 1, 0.0F},
                                    {Kind::Palette, 1, "7 4 grays", 1, 8.0F},
                                    {Kind::Palette, 2, "8 8 colors", 1, 8.0F},
                                    {Kind::Palette, 3, "9 64 colors", 1, 8.0F},
                                    {Kind::Space, 0, "S sRGB", 1, 74.0F},
                                    {Kind::Space, 1, "L Linear", 1, 8.0F}}};
    const std::array<float, 2> rowY{458.0F, 496.0F};
    std::array<float, 2> x{124.0F, 124.0F};
    for (const Def& d : defs) {
        const auto row = static_cast<std::size_t>(d.row);
        x[row] += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.kind, d.value, d.label, SDL_FRect{x[row], rowY[row], w, kChipH}});
        x[row] += w;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.kind) {
        case Kind::Matrix: return app.pattern == c.value;
        case Kind::Palette: return app.palette == c.value;
        case Kind::Space: return (c.value == 0) == (app.space == od::Space::Srgb);
    }
    return false;
}

bool apply(App& app, Kind kind, int value) {
    switch (kind) {
        case Kind::Matrix: app.pattern = value; break;
        case Kind::Palette: app.palette = value; break;
        case Kind::Space: app.space = value == 0 ? od::Space::Srgb : od::Space::Linear; break;
    }
    return rebuild(app);
}

/// Image pixel under a point in canvas coordinates, or false when the point is outside the image.
bool imagePixel(float x, float y, int& px, int& py) {
    if (x < kImgX || y < kImgY || x >= kImgX + kImgDispW || y >= kImgY + kImgDispH) return false;
    px = std::clamp(static_cast<int>((x - kImgX) / static_cast<float>(kScale)), 0, kImgW - 1);
    py = std::clamp(static_cast<int>((y - kImgY) / static_cast<float>(kScale)), 0, kImgH - 1);
    return true;
}

void pointAt(App& app, float x, float y) {
    int px = 0;
    int py = 0;
    app.hover = imagePixel(x, y, px, py);
    if (app.hover) {
        app.split = px;
        app.focusX = px;
        app.focusY = py;
    }
}

void labelBox(App& app, float x, float y, const char* text, gd::Color ink, gd::Hud::Style style = gd::Hud::Style::Bold) {
    const float w = app.hud.width(text, style) + 10.0F;
    gd::fillRect(app.renderer, x, y, w, 26.0F, gd::Color{255, 255, 255, 225});
    app.hud.text(app.renderer, x + 5.0F, y + 2.0F, text, ink, style);
}

void drawImage(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kImgX, kImgY, kImgDispW, kImgDispH, gd::kWhite, gd::kInk);
    const int s = std::clamp(app.split, 0, kImgW);
    if (s > 0) {
        const SDL_FRect src{0.0F, 0.0F, static_cast<float>(s), static_cast<float>(kImgH)};
        const SDL_FRect dst{kImgX, kImgY, static_cast<float>(s * kScale), kImgDispH};
        SDL_RenderTexture(r, app.srcTex, &src, &dst);
    }
    if (s < kImgW) {
        const SDL_FRect src{static_cast<float>(s), 0.0F, static_cast<float>(kImgW - s), static_cast<float>(kImgH)};
        const SDL_FRect dst{kImgX + static_cast<float>(s * kScale), kImgY, static_cast<float>((kImgW - s) * kScale), kImgDispH};
        SDL_RenderTexture(r, app.outTex, &src, &dst);
    }
    const float dx = kImgX + static_cast<float>(s * kScale);
    gd::fillRect(r, dx - 1.0F, kImgY, 2.0F, kImgDispH, gd::kInk);
    gd::fillRect(r, dx - 6.0F, kImgY + kImgDispH / 2.0F - 16.0F, 12.0F, 32.0F, gd::kWhite);
    gd::strokeRect(r, dx - 6.0F, kImgY + kImgDispH / 2.0F - 16.0F, 12.0F, 32.0F, gd::kInk);
    gd::strokeRect(r, kImgX, kImgY, kImgDispW, kImgDispH, gd::kInk);

    if (s * kScale >= 110) labelBox(app, kImgX + 6.0F, kImgY + 6.0F, "ORIGINAL", gd::kInk);
    if ((kImgW - s) * kScale >= 110) {
        const float w = app.hud.width("PALETTE", gd::Hud::Style::Bold) + 10.0F;
        labelBox(app, kImgX + kImgDispW - w - 6.0F, kImgY + 6.0F, "PALETTE", app.space == od::Space::Srgb ? gd::kOrange : gd::kTealDark);
    }
}

void drawLoupe(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 44.0F, kPanelW, 148.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 8.0F, 48.0F, "original", gd::kMuted);
    app.hud.text(r, kPanelX + 8.0F + kLoupeSize + 12.0F, 48.0F, "palette", gd::kMuted);
    const int fx = std::clamp(app.focusX - kLoupe / 2, 0, kImgW - kLoupe);
    const int fy = std::clamp(app.focusY - kLoupe / 2, 0, kImgH - kLoupe);
    const SDL_FRect src{static_cast<float>(fx), static_cast<float>(fy), static_cast<float>(kLoupe), static_cast<float>(kLoupe)};
    const SDL_FRect left{kPanelX + 8.0F, 72.0F, kLoupeSize, kLoupeSize};
    const SDL_FRect right{kPanelX + 8.0F + kLoupeSize + 12.0F, 72.0F, kLoupeSize, kLoupeSize};
    SDL_RenderTexture(r, app.srcTex, &src, &left);
    SDL_RenderTexture(r, app.outTex, &src, &right);
    gd::strokeRect(r, left.x, left.y, left.w, left.h, gd::kInk);
    gd::strokeRect(r, right.x, right.y, right.w, right.h, gd::kInk);
}

void drawMatrixCard(App& app) {
    SDL_Renderer* r = app.renderer;
    const od::Pattern p = kPatterns[static_cast<std::size_t>(app.pattern)];
    const od::Palette pal = kPalettes[static_cast<std::size_t>(app.palette)];
    const float y0 = 206.0F;
    gd::card(r, kPanelX, y0, kPanelW, 112.0F, gd::kWhite, gd::kInk);
    const float mx = kPanelX + 8.0F;
    const float my = y0 + 8.0F;
    constexpr float kMatrixSize = 96.0F;
    const int n = od::patternSize(p);
    if (n > 0) {
        const float cell = kMatrixSize / static_cast<float>(n);
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const double th = od::threshold(p, x, y);
                const auto g = static_cast<Uint8>(40.0 + th * 215.0);
                gd::fillRect(r, mx + static_cast<float>(x) * cell, my + static_cast<float>(y) * cell, cell, cell, gd::Color{g, g, g});
                if (n <= 4) {
                    char num[8];
                    SDL_snprintf(num, sizeof num, "%d", od::bayerIndex(n, x, y));
                    const gd::Color c = th < 0.5 ? gd::kWhite : gd::kInk;
                    app.hud.text(r, mx + static_cast<float>(x) * cell + (cell - app.hud.width(num)) / 2.0F, my + static_cast<float>(y) * cell + (cell - 22.0F) / 2.0F, num, c);
                }
            }
        }
    } else if (p == od::Pattern::Noise) {
        constexpr int kNoiseCells = 8;
        const float cell = kMatrixSize / static_cast<float>(kNoiseCells);
        for (int y = 0; y < kNoiseCells; ++y) {
            for (int x = 0; x < kNoiseCells; ++x) {
                const auto g = static_cast<Uint8>(40.0 + od::noiseThreshold(x, y) * 215.0);
                gd::fillRect(r, mx + static_cast<float>(x) * cell, my + static_cast<float>(y) * cell, cell, cell, gd::Color{g, g, g});
            }
        }
    } else {
        gd::fillRect(r, mx, my, kMatrixSize, kMatrixSize, gd::Color{147, 147, 147});
        app.hud.text(r, mx + (kMatrixSize - app.hud.width("0.5")) / 2.0F, my + 37.0F, "0.5", gd::kWhite);
    }
    gd::strokeRect(r, mx, my, kMatrixSize, kMatrixSize, gd::kInk);

    const float tx = mx + kMatrixSize + 12.0F;
    char line[48];
    app.hud.text(r, tx, y0 + 8.0F, patternName(p), gd::kInk, gd::Hud::Style::Bold);
    const int tones = od::tones(p, pal.levels);
    if (tones < 0) {
        SDL_snprintf(line, sizeof line, "random");
    } else {
        SDL_snprintf(line, sizeof line, "%d tones/ch", tones);
    }
    app.hud.text(r, tx, y0 + 32.0F, line, gd::kMuted);
    app.hud.text(r, tx, y0 + 62.0F, "block error", gd::kMuted);
    SDL_snprintf(line, sizeof line, "%.2f%%", app.error);
    app.hud.text(r, tx, y0 + 82.0F, line, app.space == od::Space::Srgb ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);
}

void drawPaletteCard(App& app) {
    SDL_Renderer* r = app.renderer;
    const od::Palette pal = kPalettes[static_cast<std::size_t>(app.palette)];
    const float y0 = 332.0F;
    gd::card(r, kPanelX, y0, kPanelW, 96.0F, gd::kWhite, gd::kInk);
    char line[48];
    SDL_snprintf(line, sizeof line, "palette: %d colors", od::paletteSize(pal));
    app.hud.text(r, kPanelX + 8.0F, y0 + 4.0F, line, gd::kMuted);

    std::vector<od::Rgb> colors;
    const double steps = static_cast<double>(pal.levels - 1);
    for (int a = 0; a < pal.levels; ++a) {
        const double va = static_cast<double>(a) / steps;
        if (pal.gray) {
            colors.push_back({va, va, va});
            continue;
        }
        for (int b = 0; b < pal.levels; ++b) {
            for (int c = 0; c < pal.levels; ++c) colors.push_back({va, static_cast<double>(b) / steps, static_cast<double>(c) / steps});
        }
    }
    const float size = colors.size() <= 8 ? 28.0F : 13.0F;
    const std::size_t perRow = colors.size() <= 8 ? 8 : 16;
    for (std::size_t i = 0; i < colors.size(); ++i) {
        const float x = kPanelX + 8.0F + static_cast<float>(i % perRow) * (size + 2.0F);
        const float y = y0 + 32.0F + static_cast<float>(i / perRow) * (size + 2.0F);
        gd::fillRect(r, x, y, size, size, gd::Color{od::toByte(colors[i].r), od::toByte(colors[i].g), od::toByte(colors[i].b)});
        gd::strokeRect(r, x, y, size, size, gd::kInk);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "%s, %s", patternName(kPatterns[static_cast<std::size_t>(app.pattern)]), app.space == od::Space::Srgb ? "sRGB (naive)" : "linear light");
    app.hud.titleBar(r, kWidth, "[ ORDERED DITHERING ]", right);

    drawImage(app);
    drawLoupe(app);
    drawMatrixCard(app);
    drawPaletteCard(app);

    app.hud.text(r, 24.0F, 432.0F, "move the mouse over the image: original left, palette right", gd::kMuted);
    app.hud.text(r, 24.0F, 463.0F, "MATRIX", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, 24.0F, 501.0F, "PALETTE", gd::kInk, gd::Hud::Style::Bold);
    for (const Chip& c : app.chips) {
        if (c.kind == Kind::Space && c.value == 0) app.hud.text(r, c.rect.x - 74.0F + 14.0F, 501.0F, "SPACE", gd::kInk, gd::Hud::Style::Bold);
        const bool on = active(app, c);
        const gd::Color fill = !on ? gd::kWhite : (c.kind == Kind::Space && c.value == 0) ? gd::kOrange : gd::kTeal;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("ordered-dithering", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->scratch = SDL_CreateSurface(kImgW, kImgH, SDL_PIXELFORMAT_RGBA32);
    app->srcTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kImgW, kImgH);
    app->outTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kImgW, kImgH);
    if (app->scratch == nullptr || app->srcTex == nullptr || app->outTex == nullptr) {
        SDL_Log("image resources failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->srcTex, SDL_SCALEMODE_NEAREST);
    SDL_SetTextureScaleMode(app->outTex, SDL_SCALEMODE_NEAREST);
    app->src = od::makeTestImage(kImgW, kImgH);
    app->focusX = app->split;
    app->focusY = 112;
    if (!upload(*app, app->src, app->srcTex) || !rebuild(*app)) return SDL_APP_FAILURE;
    layoutChips(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: pointAt(app, event->motion.x, event->motion.y); break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            pointAt(app, p.x, p.y);  // a tap on the image moves the split too (touch has no hover)
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect) && !apply(app, c.kind, c.value)) return SDL_APP_FAILURE;
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            if (event->key.repeat) break;
            bool ok = true;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: ok = apply(app, Kind::Matrix, 0); break;
                case SDLK_2: ok = apply(app, Kind::Matrix, 1); break;
                case SDLK_3: ok = apply(app, Kind::Matrix, 2); break;
                case SDLK_4: ok = apply(app, Kind::Matrix, 3); break;
                case SDLK_5: ok = apply(app, Kind::Matrix, 4); break;
                case SDLK_6: ok = apply(app, Kind::Palette, 0); break;
                case SDLK_7: ok = apply(app, Kind::Palette, 1); break;
                case SDLK_8: ok = apply(app, Kind::Palette, 2); break;
                case SDLK_9: ok = apply(app, Kind::Palette, 3); break;
                case SDLK_S: ok = apply(app, Kind::Space, 0); break;
                case SDLK_L: ok = apply(app, Kind::Space, 1); break;
                default: break;
            }
            if (!ok) return SDL_APP_FAILURE;
            break;
        }
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->srcTex != nullptr) SDL_DestroyTexture(app->srcTex);
        if (app->outTex != nullptr) SDL_DestroyTexture(app->outTex);
        if (app->scratch != nullptr) SDL_DestroySurface(app->scratch);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
