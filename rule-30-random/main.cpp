// Rule 30 as a random bit source, with SDL3 main callbacks.
//
// Left: the textbook picture, one black cell and the rule "left xor (center or right)". The left edge is regular,
// the middle looks like noise. The orange cells are the ones that feed the bit stream. Right: those bits as a
// 256 x 256 image (65,536 bits, left to right, top to bottom) with the numbers a quick randomness check looks at:
// share of ones, longest run, repeat period. The same panel can be fed by a bad LCG for comparison, which has the
// same share of ones and still fails on runs and period. The row update runs 64 cells per word or one cell at a
// time, both give the same bits, and the demo measures how long each takes to fill the image. The automaton, the
// statistics and the number formatting live in logic.hpp and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <array>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kImage = 256;                                              // bit image side
constexpr std::size_t kBits = static_cast<std::size_t>(kImage) * kImage;  // 65,536
constexpr std::size_t kTriRows = 190;
constexpr long kTriHalf = 99;
constexpr int kTriCols = static_cast<int>(2 * kTriHalf + 1);
constexpr float kTriScale = 2.0F;
constexpr double kBudgetMs = 8.0;  // compute time per frame, the rest is left for drawing
constexpr float kChipH = 30.0F;
constexpr gd::Color kOrangeSoft{254, 215, 170};
constexpr gd::Color kUnwritten{231, 229, 228};

enum class Action { SrcCenter, SrcLeft, SrcLcgLow, SrcLcgHigh, EnginePerCell, EngineBits, Pause, Restart };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* triTex = nullptr;
    SDL_Texture* bitTex = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    r30::Triangle tri;
    std::vector<unsigned char> pixels;  // RGBA of the bit image
    std::vector<unsigned char> bits;
    std::unique_ptr<r30::Stream> stream;
    r30::BitStats stats;
    r30::PeriodFinder period;
    std::size_t produced = 0;
    std::size_t uploaded = 0;  // bits whose pixels are already in the texture
    bool fullUpload = true;
    double computeMs = 0.0;
    r30::Source source = r30::Source::Center;
    r30::Engine engine = r30::Engine::BitParallel;
    bool paused = false;
    long maxFrames = -1;
    long frames = 0;
};

double nowMs() {
#ifdef __EMSCRIPTEN__
    return emscripten_get_now();  // SDL_GetPerformanceCounter only has millisecond steps in the browser
#else
    return static_cast<double>(SDL_GetPerformanceCounter()) * 1000.0 / static_cast<double>(SDL_GetPerformanceFrequency());
#endif
}

bool isRule(r30::Source s) { return s == r30::Source::Center || s == r30::Source::LeftSide; }

void put(std::vector<unsigned char>& px, std::size_t i, gd::Color c) {
    px[4 * i] = c.r;
    px[4 * i + 1] = c.g;
    px[4 * i + 2] = c.b;
    px[4 * i + 3] = 255;
}

/// The cell that feeds the stream in row t at column x (offset from the start cell).
bool isRead(r30::Source s, std::size_t t, long x) {
    if (s == r30::Source::Center) return x == 0;
    if (s == r30::Source::LeftSide) return t >= r30::kLeftOffset && x == static_cast<long>(r30::kLeftOffset) - static_cast<long>(t);
    return false;
}

void buildTriangleTexture(App& app) {
    std::vector<unsigned char> px(static_cast<std::size_t>(kTriCols) * kTriRows * 4);
    for (std::size_t t = 0; t < kTriRows; ++t) {
        for (long x = -kTriHalf; x <= kTriHalf; ++x) {
            const bool black = app.tri.at(t, x);
            const bool read = isRead(app.source, t, x);
            const gd::Color c = read ? (black ? gd::kOrange : kOrangeSoft) : (black ? gd::kInk : gd::kWhite);
            put(px, t * static_cast<std::size_t>(kTriCols) + static_cast<std::size_t>(x + kTriHalf), c);
        }
    }
    SDL_UpdateTexture(app.triTex, nullptr, px.data(), kTriCols * 4);
}

/// Start over: new stream, empty image. Also the reset for pause, so a restarted run is never stuck.
void restart(App& app) {
    app.stream = std::make_unique<r30::Stream>(app.source, app.engine, kBits);
    app.stats = {};
    app.period = {};
    app.produced = 0;
    app.uploaded = 0;
    app.fullUpload = true;
    app.computeMs = 0.0;
    app.paused = false;
    app.bits.assign(kBits, 0);
    for (std::size_t i = 0; i < kBits; ++i) put(app.pixels, i, kUnwritten);
    buildTriangleTexture(app);
}

void generate(App& app) {
    if (app.paused || app.produced >= kBits) return;
    const bool slow = isRule(app.source) && app.engine == r30::Engine::PerCell;
    const std::size_t chunk = slow ? 1 : 64;  // a per-cell row takes long enough to check the clock after every one
    const double t0 = nowMs();
    double now = t0;
    while (app.produced < kBits && now - t0 < kBudgetMs) {
        for (std::size_t i = 0; i < chunk && app.produced < kBits; ++i) {
            const bool b = app.stream->next();
            app.bits[app.produced] = b ? 1 : 0;
            app.stats.push(b);
            app.period.push(b);
            put(app.pixels, app.produced, b ? gd::kInk : gd::kWhite);
            ++app.produced;
        }
        now = nowMs();
    }
    app.computeMs += now - t0;
}

void uploadBitImage(App& app) {
    if (app.fullUpload) {
        SDL_UpdateTexture(app.bitTex, nullptr, app.pixels.data(), kImage * 4);
        app.fullUpload = false;
    } else if (app.produced > app.uploaded) {
        const int row0 = static_cast<int>(app.uploaded / kImage);
        const int row1 = static_cast<int>((app.produced + kImage - 1) / kImage);
        const SDL_Rect rect{0, row0, kImage, row1 - row0};
        SDL_UpdateTexture(app.bitTex, &rect, app.pixels.data() + static_cast<std::size_t>(row0) * kImage * 4, kImage * 4);
    }
    app.uploaded = app.produced;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float y;
    };
    constexpr float kRow1 = 452.0F;
    constexpr float kRow2 = 494.0F;
    const std::array<Def, 8> defs{{{Action::SrcCenter, "1 Rule 30 center", 0.0F, kRow1},
                                   {Action::SrcLeft, "2 Rule 30 left", 8.0F, kRow1},
                                   {Action::SrcLcgLow, "3 LCG bit 0", 8.0F, kRow1},
                                   {Action::SrcLcgHigh, "4 LCG bit 31", 8.0F, kRow1},
                                   {Action::EnginePerCell, "5 Per cell", 0.0F, kRow2},
                                   {Action::EngineBits, "6 64 cells per word", 8.0F, kRow2},
                                   {Action::Pause, "Space Pause", 24.0F, kRow2},
                                   {Action::Restart, "R Restart", 8.0F, kRow2}}};
    float x = 0.0F;
    float lastY = -1.0F;
    for (const Def& d : defs) {
        if (d.y != lastY) x = 96.0F;
        lastY = d.y;
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, kChipH}});
        x += w;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::SrcCenter: app.source = r30::Source::Center; restart(app); break;
        case Action::SrcLeft: app.source = r30::Source::LeftSide; restart(app); break;
        case Action::SrcLcgLow: app.source = r30::Source::LcgLow; restart(app); break;
        case Action::SrcLcgHigh: app.source = r30::Source::LcgHigh; restart(app); break;
        case Action::EnginePerCell:
            if (isRule(app.source)) {
                app.engine = r30::Engine::PerCell;
                restart(app);
            }
            break;
        case Action::EngineBits:
            if (isRule(app.source)) {
                app.engine = r30::Engine::BitParallel;
                restart(app);
            }
            break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Restart: restart(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::SrcCenter: return app.source == r30::Source::Center;
        case Action::SrcLeft: return app.source == r30::Source::LeftSide;
        case Action::SrcLcgLow: return app.source == r30::Source::LcgLow;
        case Action::SrcLcgHigh: return app.source == r30::Source::LcgHigh;
        case Action::EnginePerCell: return app.engine == r30::Engine::PerCell;
        case Action::EngineBits: return app.engine == r30::Engine::BitParallel;
        case Action::Pause: return app.paused;
        case Action::Restart: return false;
    }
    return false;
}

bool enabled(const App& app, Action action) {
    if (action == Action::EnginePerCell || action == Action::EngineBits) return isRule(app.source);
    return true;
}

const char* sourceName(r30::Source s) {
    switch (s) {
        case r30::Source::Center: return "Rule 30 center";
        case r30::Source::LeftSide: return "Rule 30 left side";
        case r30::Source::LcgLow: return "LCG, bit 0";
        case r30::Source::LcgHigh: return "LCG, bit 31";
    }
    return "";
}

const char* sourceDetail(r30::Source s) {
    switch (s) {
        case r30::Source::Center: return "cell 0 of every row";
        case r30::Source::LeftSide: return "8 cells from edge";
        case r30::Source::LcgLow:
        case r30::Source::LcgHigh: return "32 bit, mod 2^32";
    }
    return "";
}

std::string formatDuration(double ms) {
    const auto whole = static_cast<unsigned long long>(ms + 0.5);
    return r30::formatThousands(whole) + " ms";
}

void label(App& app, float x, float y, const char* text) { app.hud.text(app.renderer, x, y, text, gd::kMuted); }

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    constexpr float x = 738.0F;
    gd::card(r, 724.0F, 40.0F, 216.0F, 388.0F, gd::kWhite, gd::kInk);
    label(app, x, 52.0F, "SOURCE");
    app.hud.text(r, x, 74.0F, sourceName(app.source), gd::kInk, gd::Hud::Style::Bold);
    label(app, x, 96.0F, sourceDetail(app.source));

    label(app, x, 130.0F, "BITS");
    const std::string bitsLine = r30::formatThousands(app.produced) + " / " + r30::formatThousands(kBits);
    app.hud.text(r, x, 152.0F, bitsLine, gd::kInk, gd::Hud::Style::Bold);

    label(app, x, 184.0F, "ONES");
    const std::string onesLine = r30::formatThousands(app.stats.ones) + " (" + r30::formatPercent(app.stats.ones, app.stats.n) + "%)";
    app.hud.text(r, x, 206.0F, onesLine, gd::kInk, gd::Hud::Style::Bold);

    label(app, x, 238.0F, "LONGEST RUN");
    app.hud.text(r, x, 260.0F, std::to_string(app.stats.longest), gd::kInk, gd::Hud::Style::Bold);
    const int expected = r30::expectedLongestRun(app.stats.n);
    label(app, x, 282.0F, expected > 0 ? ("fair coin ~ " + std::to_string(expected)).c_str() : "fair coin ~ -");

    label(app, x, 316.0F, "REPEATS");
    const int p = app.period.period();
    if (p > 0) {
        app.hud.text(r, x, 338.0F, "every " + std::to_string(p) + (p == 1 ? " bit" : " bits"), gd::kOrange, gd::Hud::Style::Bold);
    } else if (app.period.size() < r30::kPeriodSure) {
        app.hud.text(r, x, 338.0F, "collecting...", gd::kMuted, gd::Hud::Style::Bold);
    } else {
        app.hud.text(r, x, 338.0F, "none up to 64", gd::kTeal, gd::Hud::Style::Bold);
    }

    label(app, x, 370.0F, app.produced >= kBits ? "COMPUTE TIME" : "COMPUTE TIME SO FAR");
    app.hud.text(r, x, 392.0F, formatDuration(app.computeMs), gd::kInk, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ RULE 30 ]", "p xor (q or r)");

    // The textbook picture.
    gd::card(r, 20.0F, 40.0F, 408.0F, 388.0F, gd::kWhite, gd::kInk);
    const SDL_FRect triDst{25.0F, 44.0F, static_cast<float>(kTriCols) * kTriScale, static_cast<float>(kTriRows) * kTriScale};
    SDL_RenderTexture(r, app.triTex, nullptr, &triDst);
    label(app, 30.0F, 48.0F, "regular");
    label(app, 366.0F, 48.0F, "noise");

    // The bit image and its first bits.
    gd::card(r, 444.0F, 40.0F, 264.0F, 264.0F, gd::kWhite, gd::kInk);
    const SDL_FRect bitDst{448.0F, 44.0F, static_cast<float>(kImage), static_cast<float>(kImage)};
    SDL_RenderTexture(r, app.bitTex, nullptr, &bitDst);
    label(app, 448.0F, 314.0F, "FIRST 64 BITS");
    for (std::size_t i = 0; i < 64; ++i) {
        const float cx = 448.0F + static_cast<float>(i % 16) * 16.0F;
        const float cy = 342.0F + static_cast<float>(i / 16) * 16.0F;
        if (i < app.produced) {
            gd::fillRect(r, cx, cy, 15.0F, 15.0F, app.bits[i] != 0 ? gd::kInk : gd::kWhite);
            gd::strokeRect(r, cx, cy, 15.0F, 15.0F, gd::kInk);
        } else {
            gd::fillRect(r, cx, cy, 15.0F, 15.0F, kUnwritten);
        }
    }
    label(app, 448.0F, 410.0F, "image: 256 bits per row");

    drawStats(app);

    label(app, 20.0F, 459.0F, "SOURCE");
    label(app, 20.0F, 501.0F, "ENGINE");
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool usable = enabled(app, c.action);
        const gd::Color fill = !usable ? gd::kGrid : on ? gd::kTeal : gd::kWhite;
        const gd::Color text = !usable ? gd::kMuted : on ? gd::kWhite : gd::kInk;
        const std::string caption = (c.action == Action::Pause && app.paused) ? "Space Resume" : c.label;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + 12.0F, c.rect.y + 4.0F, caption, text, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("rule-30-random", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->triTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kTriCols, static_cast<int>(kTriRows));
    app->bitTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kImage, kImage);
    if (app->triTex == nullptr || app->bitTex == nullptr) {
        SDL_Log("texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    for (SDL_Texture* t : {app->triTex, app->bitTex}) {
        SDL_SetTextureScaleMode(t, SDL_SCALEMODE_NEAREST);
        SDL_SetTextureBlendMode(t, SDL_BLENDMODE_NONE);
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->tri = r30::makeTriangle(kTriRows, kTriHalf);
    app->pixels.assign(kBits * 4, 0);
    layoutChips(*app);
    restart(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    break;  // a restart rebuilds state, never act on a second chip with the same click
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key toggles or restarts, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::SrcCenter); break;
                case SDLK_2: apply(app, Action::SrcLeft); break;
                case SDLK_3: apply(app, Action::SrcLcgLow); break;
                case SDLK_4: apply(app, Action::SrcLcgHigh); break;
                case SDLK_5: apply(app, Action::EnginePerCell); break;
                case SDLK_6: apply(app, Action::EngineBits); break;
                case SDLK_SPACE: apply(app, Action::Pause); break;
                case SDLK_R: apply(app, Action::Restart); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    generate(app);
    uploadBitImage(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->triTex != nullptr) SDL_DestroyTexture(app->triTex);
        if (app->bitTex != nullptr) SDL_DestroyTexture(app->bitTex);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
