// Langton's ant with SDL3 main callbacks.
//
// One ant on a grid that wraps around. On a white cell it turns right, on a black one left, flips the cell and steps forward.
// That is the whole rule, and the demo is about what it makes: a short symmetric start, then about 10,000 steps of chaos, then
// the ant suddenly builds a "highway", a 104-step pattern it repeats forever while walking diagonally away. On a small torus
// the highway runs into its own earlier mess and breaks, which is why the grid size is a chip. The rule chips swap in
// multi-color ants: every letter is the turn for one color.
// The ant, the symmetry check and the highway detector live in logic.hpp and are covered by test.cpp. The demo starts 6000
// steps in, so the first frame already shows the chaos and the highway follows within seconds.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;

constexpr float kGridX = 24.0F;
constexpr float kGridY = 40.0F;
constexpr float kGridPx = 400.0F;  // the grid is always 400 canvas pixels wide, a cell is 8, 4 or 2 of them
constexpr int kMaxSize = 200;
constexpr long kHeadStart = 6000;  // steps already done when the demo opens

constexpr std::array<const char*, 5> kRules{"RL", "RLR", "LLRR", "RLLR", "LRRRRRLLR"};
constexpr std::array<int, 3> kSizes{50, 100, 200};
constexpr std::array<double, 6> kRates{1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0};
constexpr std::array<const char*, 6> kRateLabels{"1", "10", "100", "1k", "10k", "100k"};
constexpr std::size_t kDefaultSize = 1;
constexpr std::size_t kDefaultRate = 3;

constexpr float kChipH = 30.0F;
constexpr float kRowOne = 450.0F;
constexpr float kRowTwo = 488.0F;

constexpr float kPanelX = 444.0F;
constexpr float kPanelW = 492.0F;
constexpr float kPad = 16.0F;
constexpr float kStripX = kPanelX + kPad;
constexpr float kStripY = 200.0F;
constexpr float kStripH = 16.0F;

// Color 0 is the paper white, color 1 the classic black. Further colors only need to tell each other apart.
constexpr std::array<gd::Color, 10> kPalette{{{255, 255, 255, 255},
                                              {17, 24, 39, 255},
                                              {234, 88, 12, 255},
                                              {13, 148, 136, 255},
                                              {202, 138, 4, 255},
                                              {37, 99, 235, 255},
                                              {147, 51, 234, 255},
                                              {219, 39, 119, 255},
                                              {101, 163, 13, 255},
                                              {120, 113, 108, 255}}};

enum class Kind { Rule, Size, Rate, Pause, Step, Restart };

struct Chip {
    Kind kind;
    std::size_t value;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* grid = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::optional<ant::World> world;
    std::size_t ruleIdx = 0;
    std::size_t sizeIdx = kDefaultSize;
    std::size_t rateIdx = kDefaultRate;
    bool paused = false;
    bool dirty = true;  // the grid texture must be rebuilt
    double acc = 0.0;
    SDL_FPoint mouse{-100.0F, -100.0F};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

gd::Color phaseColor(ant::Phase p) { return p == ant::Phase::Symmetry ? gd::kTeal : p == ant::Phase::Chaos ? gd::kOrange : gd::kInk; }

const char* phaseName(ant::Phase p) { return p == ant::Phase::Symmetry ? "Symmetry" : p == ant::Phase::Chaos ? "Chaos" : "Highway"; }

bool inside(SDL_FPoint p, const SDL_FRect& r) { return SDL_PointInRectFloat(&p, &r); }

/// A fresh ant on an empty grid with the current rule and size. The accumulator restarts too, or the first step comes early.
bool restart(App& app, long headStart) {
    const auto rule = ant::parseRule(kRules[app.ruleIdx]);
    if (!rule) return false;
    app.world.emplace(*rule, kSizes[app.sizeIdx]);
    app.world->run(headStart);
    app.acc = 0.0;
    app.paused = false;
    app.dirty = true;
    return true;
}

void apply(App& app, Kind kind, std::size_t value) {
    switch (kind) {
        case Kind::Rule:
            app.ruleIdx = value;
            restart(app, 0);
            break;
        case Kind::Size:
            app.sizeIdx = value;
            restart(app, 0);
            break;
        case Kind::Rate:
            app.rateIdx = value;
            app.acc = 0.0;
            break;
        case Kind::Pause:
            app.paused = !app.paused;
            app.acc = 0.0;
            break;
        case Kind::Step:
            app.paused = true;
            app.acc = 0.0;
            app.world->step();
            app.dirty = true;
            break;
        case Kind::Restart: restart(app, 0); break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.kind) {
        case Kind::Rule: return app.ruleIdx == c.value;
        case Kind::Size: return app.sizeIdx == c.value;
        case Kind::Rate: return app.rateIdx == c.value;
        case Kind::Pause: return app.paused;
        default: return false;
    }
}

void layoutChips(App& app) {
    const auto add = [&app](Kind kind, std::size_t value, std::string label, float& x, float y, float gap) {
        x += gap;
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({kind, value, std::move(label), SDL_FRect{x, y, w, kChipH}});
        x += w;
    };
    float x = 24.0F + app.hud.width("RULE", gd::Hud::Style::Bold);
    for (std::size_t i = 0; i < kRules.size(); ++i) add(Kind::Rule, i, std::to_string(i + 1) + " " + kRules[i], x, kRowOne, i == 0 ? 12.0F : 8.0F);
    x += 16.0F + app.hud.width("SIZE", gd::Hud::Style::Bold);
    for (std::size_t i = 0; i < kSizes.size(); ++i) add(Kind::Size, i, std::to_string(kSizes[i]), x, kRowOne, i == 0 ? 12.0F : 8.0F);

    x = 24.0F + app.hud.width("SPEED", gd::Hud::Style::Bold);
    for (std::size_t i = 0; i < kRates.size(); ++i) add(Kind::Rate, i, kRateLabels[i], x, kRowTwo, i == 0 ? 12.0F : 8.0F);
    x += 12.0F + app.hud.width("steps/s");
    add(Kind::Pause, 0, "SPACE Pause", x, kRowTwo, 16.0F);
    add(Kind::Step, 0, "N Step", x, kRowTwo, 8.0F);
    add(Kind::Restart, 0, "R Restart", x, kRowTwo, 8.0F);
}

void drawChip(App& app, const Chip& c) {
    SDL_Renderer* r = app.renderer;
    const bool on = active(app, c);
    const bool hot = inside(app.mouse, c.rect);
    gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : (hot ? gd::kTealSoft : gd::kWhite), gd::kInk);
    const float ty = c.rect.y + (c.rect.h - app.hud.lineHeight(gd::Hud::Style::Bold)) / 2.0F;
    app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, ty, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
}

/// Colors to pixels: one pixel per cell, scaled up with nearest-neighbor when drawn. Rows are addressed by pitch.
void upload(App& app) {
    const ant::World& w = *app.world;
    const int n = w.size();
    const int pitch = n * 4;
    std::vector<Uint8> pixels(static_cast<std::size_t>(pitch) * static_cast<std::size_t>(n));
    const auto& cells = w.cells();
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const gd::Color c = kPalette[cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(n) + static_cast<std::size_t>(x)] % kPalette.size()];
            Uint8* d = &pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(pitch) + static_cast<std::size_t>(x) * 4];
            d[0] = c.r;
            d[1] = c.g;
            d[2] = c.b;
            d[3] = 255;
        }
    }
    const SDL_Rect area{0, 0, n, n};
    SDL_UpdateTexture(app.grid, &area, pixels.data(), pitch);
    app.dirty = false;
}

void drawGrid(App& app) {
    SDL_Renderer* r = app.renderer;
    const ant::World& w = *app.world;
    const float n = static_cast<float>(w.size());
    const float cell = kGridPx / n;
    gd::fillRect(r, kGridX, kGridY, kGridPx, kGridPx, gd::kWhite);
    const SDL_FRect src{0.0F, 0.0F, n, n};
    const SDL_FRect dst{kGridX, kGridY, kGridPx, kGridPx};
    SDL_RenderTexture(r, app.grid, &src, &dst);
    gd::strokeRect(r, kGridX - 1.0F, kGridY - 1.0F, kGridPx + 2.0F, kGridPx + 2.0F, gd::kInk);

    // The ant: at least 7 pixels wide even when a cell is 2, with a stroke so it shows on every color, and a nose for the heading.
    const float cx = kGridX + (static_cast<float>(w.antX()) + 0.5F) * cell;
    const float cy = kGridY + (static_cast<float>(w.antY()) + 0.5F) * cell;
    gd::fillRect(r, cx - 3.5F, cy - 3.5F, 7.0F, 7.0F, gd::kOrange);
    gd::strokeRect(r, cx - 3.5F, cy - 3.5F, 7.0F, 7.0F, gd::kInk);
    static constexpr float kNoseX[4] = {0.0F, 1.0F, 0.0F, -1.0F};
    static constexpr float kNoseY[4] = {-1.0F, 0.0F, 1.0F, 0.0F};
    gd::fillRect(r, cx + kNoseX[w.antDir()] * 6.0F - 1.5F, cy + kNoseY[w.antDir()] * 6.0F - 1.5F, 3.0F, 3.0F, gd::kInk);
}

void drawPhaseCard(App& app) {
    SDL_Renderer* r = app.renderer;
    const ant::World& w = *app.world;
    gd::card(r, kPanelX, 40.0F, kPanelW, 124.0F, gd::kWhite, gd::kInk);
    const float x = kPanelX + kPad;
    app.hud.text(r, x, 50.0F, "STEP", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, x, 70.0F, ant::formatCount(w.steps()), gd::kTealDark, gd::Hud::Style::Large);
    const ant::Phase phase = w.phase();
    app.hud.text(r, x + 250.0F, 50.0F, "PHASE", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, x + 250.0F, 70.0F, phaseName(phase), phaseColor(phase), gd::Hud::Style::Large);

    char buf[96];
    if (const auto h = w.highway()) {
        SDL_snprintf(buf, sizeof buf, "period %d, shift (%+d,%+d) per period", h->period, h->dx, h->dy);
        app.hud.text(r, x, 120.0F, buf, gd::kInk);
        app.hud.text(r, x, 140.0F, "repeating since step " + ant::formatCount(h->onset), gd::kMuted);
    } else {
        SDL_snprintf(buf, sizeof buf, "no highway (looks for periods up to %d)", ant::kLongestPeriod);
        app.hud.text(r, x, 120.0F, buf, gd::kInk);
        const unsigned bits = w.symmetries();
        if (bits != 0) {
            SDL_snprintf(buf, sizeof buf, "symmetric now: %d of %d reflections", std::popcount(bits), ant::kSymmetryCount);
            app.hud.text(r, x, 140.0F, buf, gd::kMuted);
        } else {
            app.hud.text(r, x, 140.0F, "last symmetric step: " + ant::formatCount(w.lastSymmetric()), gd::kMuted);
        }
    }
}

void drawTimeline(App& app) {
    SDL_Renderer* r = app.renderer;
    const ant::World& w = *app.world;
    gd::card(r, kPanelX, 172.0F, kPanelW, 84.0F, gd::kWhite, gd::kInk);
    const float x = kPanelX + kPad;
    app.hud.text(r, x, 178.0F, "PHASES", gd::kMuted, gd::Hud::Style::Bold);  // 64 steps per pixel of the strip
    float lx = x + 90.0F;
    for (ant::Phase p : {ant::Phase::Symmetry, ant::Phase::Chaos, ant::Phase::Highway}) {
        gd::fillRect(r, lx, 183.0F, 10.0F, 10.0F, phaseColor(p));
        app.hud.text(r, lx + 14.0F, 178.0F, phaseName(p), gd::kMuted);
        lx += 14.0F + app.hud.width(phaseName(p)) + 18.0F;
    }
    gd::fillRect(r, kStripX, kStripY, static_cast<float>(ant::kLogBuckets), kStripH, gd::kPaper);
    const auto& log = w.phaseLog();
    for (std::size_t i = 0; i < log.size(); ++i) gd::fillRect(r, kStripX + static_cast<float>(i), kStripY, 1.0F, kStripH, phaseColor(log[i]));
    gd::strokeRect(r, kStripX - 1.0F, kStripY - 1.0F, static_cast<float>(ant::kLogBuckets) + 2.0F, kStripH + 2.0F, gd::kInk);
    // Where the ant is now: a mark through the strip. Past the right end there is none.
    const long bucket = w.steps() / ant::kBucket;
    if (bucket < static_cast<long>(ant::kLogBuckets)) gd::fillRect(r, kStripX + static_cast<float>(bucket), kStripY - 4.0F, 2.0F, kStripH + 8.0F, gd::kInk);
    for (long mark = 0; mark <= 20000; mark += 10000) {
        const float mx = kStripX + static_cast<float>(mark / ant::kBucket);
        gd::fillRect(r, mx, kStripY + kStripH + 1.0F, 1.0F, 4.0F, gd::kMuted);
        const std::string label = mark == 0 ? "0" : std::to_string(mark / 1000) + "k";
        app.hud.text(r, mx - (mark == 0 ? 0.0F : app.hud.width(label) / 2.0F), kStripY + kStripH + 5.0F, label, gd::kMuted);
    }
    app.hud.text(r, kStripX + 345.0F, kStripY + kStripH + 5.0F, "steps", gd::kMuted);
}

void drawColors(App& app) {
    SDL_Renderer* r = app.renderer;
    const ant::World& w = *app.world;
    gd::card(r, kPanelX, 264.0F, kPanelW, 80.0F, gd::kWhite, gd::kInk);
    const float x = kPanelX + kPad;
    app.hud.text(r, x, 270.0F, "COLORS", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, x + 90.0F, 270.0F, "the turn made on each color", gd::kMuted);
    for (int c = 0; c < w.rule().colors(); ++c) {
        const float sx = x + static_cast<float>(c) * 36.0F;
        gd::fillRect(r, sx, 294.0F, 24.0F, 24.0F, kPalette[static_cast<std::size_t>(c) % kPalette.size()]);
        gd::strokeRect(r, sx, 294.0F, 24.0F, 24.0F, gd::kInk);
        const char letter[2] = {w.rule().turnsRight(c) ? 'R' : 'L', '\0'};
        app.hud.text(r, sx + 7.0F, 318.0F, letter, gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const ant::World& w = *app.world;
    gd::card(r, kPanelX, 352.0F, kPanelW, 88.0F, gd::kWhite, gd::kInk);
    const float x = kPanelX + kPad;
    static constexpr const char* kHeading[4] = {"up", "right", "down", "left"};
    char buf[96];
    SDL_snprintf(buf, sizeof buf, "colored cells: %s", ant::formatCount(w.colored()).c_str());
    app.hud.text(r, x, 358.0F, buf, gd::kInk);
    SDL_snprintf(buf, sizeof buf, "ant at (%d,%d), heading %s", w.antX(), w.antY(), kHeading[w.antDir()]);
    app.hud.text(r, x, 382.0F, buf, gd::kInk);
    app.hud.text(r, x, 406.0F, "keys: 1-5 rule, G size, left/right speed", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    char buf[64];
    gd::backdrop(r, kWidth, kHeight);
    const int size = app.world->size();
    SDL_snprintf(buf, sizeof buf, "%s on a %dx%d torus", kRules[app.ruleIdx], size, size);
    app.hud.titleBar(r, kWidth, "[ LANGTON'S ANT ]", buf);

    drawGrid(app);
    drawPhaseCard(app);
    drawTimeline(app);
    drawColors(app);
    drawStats(app);

    const auto label = [&app, r](float x, float y, const char* s) { app.hud.text(r, x, y + (kChipH - app.hud.lineHeight(gd::Hud::Style::Bold)) / 2.0F, s, gd::kMuted, gd::Hud::Style::Bold); };
    label(24.0F, kRowOne, "RULE");
    label(24.0F, kRowTwo, "SPEED");
    for (const Chip& c : app.chips) {
        if (c.kind == Kind::Size && c.value == 0) label(c.rect.x - 12.0F - app.hud.width("SIZE", gd::Hud::Style::Bold), kRowOne, "SIZE");
        if (c.kind == Kind::Pause) app.hud.text(r, c.rect.x - 16.0F - app.hud.width("steps/s"), kRowTwo + (kChipH - app.hud.lineHeight()) / 2.0F, "steps/s", gd::kMuted);
    }
    for (const Chip& c : app.chips) drawChip(app, c);
}

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
    if (!SDL_CreateWindowAndRenderer("langtons-ant", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->grid = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kMaxSize, kMaxSize);
    if (app->grid == nullptr) {
        SDL_Log("grid texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->grid, SDL_SCALEMODE_NEAREST);
    if (!restart(*app, kHeadStart)) {
        SDL_Log("invalid rule %s", kRules[app->ruleIdx]);
        return SDL_APP_FAILURE;
    }
    layoutChips(*app);
    app->lastTicks = SDL_GetTicks();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: app.mouse = SDL_FPoint{event->motion.x, event->motion.y}; break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE: app.mouse = SDL_FPoint{-100.0F, -100.0F}; break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            for (const Chip& c : app.chips) {
                if (inside(p, c.rect)) {
                    apply(app, c.kind, c.value);
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode key = event->key.key;
            // Everything toggles or restarts, only N is meant to be held down to walk the ant slowly.
            if (event->key.repeat && key != SDLK_N) break;
            if (key >= SDLK_1 && key < SDLK_1 + static_cast<SDL_Keycode>(kRules.size())) {
                apply(app, Kind::Rule, static_cast<std::size_t>(key - SDLK_1));
                break;
            }
            switch (key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_SPACE: apply(app, Kind::Pause, 0); break;
                case SDLK_N: apply(app, Kind::Step, 0); break;
                case SDLK_R: apply(app, Kind::Restart, 0); break;
                case SDLK_G: apply(app, Kind::Size, (app.sizeIdx + 1) % kSizes.size()); break;
                case SDLK_LEFT:
                    if (app.rateIdx > 0) apply(app, Kind::Rate, app.rateIdx - 1);
                    break;
                case SDLK_RIGHT:
                    if (app.rateIdx + 1 < kRates.size()) apply(app, Kind::Rate, app.rateIdx + 1);
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
    const Uint64 now = SDL_GetTicks();
    const double dt = static_cast<double>(now - app.lastTicks) / 1000.0;
    app.lastTicks = now;
    if (!app.paused) {
        const long n = ant::stepsDue(app.acc, kRates[app.rateIdx], dt);
        if (n > 0) {
            app.world->run(n);
            app.dirty = true;
        }
    }
    if (app.dirty) upload(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->grid != nullptr) SDL_DestroyTexture(app->grid);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
