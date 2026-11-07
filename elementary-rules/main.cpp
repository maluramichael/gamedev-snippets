// Elementary cellular automata with SDL3 main callbacks.
//
// The eight cards on top are the whole rule: each shows a neighborhood (left, center, right cell) and, below it, what the
// middle cell becomes. Clicking the lower cell flips that bit, and the rule number (0 to 255) is just those eight bits read
// as a binary number. The picture is the rule at work on a ring of cells, one row per step, time running downward.
// The point of the demo is the "Variants" view: mirror and complement turn a rule into another one that draws the same
// picture flipped or inverted, so the 256 rules are only 88 different worlds. Rule 110 shows up again as 124, 137 and 193.
// The automaton and the rule symmetries live in logic.hpp and are covered by test.cpp.

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

// The big picture: one cell is 4x4 canvas pixels. An odd column count puts the single seed exactly in the middle.
constexpr int kMainCols = 225;
constexpr int kMainRows = 80;
constexpr int kMainCell = 4;
// Variants view: four smaller pictures side by side.
constexpr int kPanelCols = 107;
constexpr int kPanelRows = 148;
constexpr int kPanelCell = 2;

constexpr float kPicX = 30.0F;
constexpr float kPicY = 122.0F;
constexpr float kPanelX0 = 31.0F;
constexpr float kPanelStep = 228.0F;
constexpr float kPanelY = 146.0F;
constexpr float kRevealSeconds = 0.8F;  // time a fresh picture needs to grow from the first row to the last
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;

constexpr float kCardX0 = 24.0F;
constexpr float kCardStep = 62.0F;
constexpr float kCardW = 56.0F;
constexpr float kCardY = 38.0F;
constexpr float kCardH = 78.0F;

enum class Action { Preset30, Preset90, Preset110, Preset184, Mirror, Complement, Variants, Single, Random, Dec, Inc };
enum class SeedMode { Single, Random };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

/// A texture the picture is drawn into once whenever the rule changes, instead of thousands of rectangles every frame.
struct Picture {
    SDL_Texture* texture = nullptr;
    int w = 0;
    int h = 0;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    Picture main;
    std::array<Picture, 4> panels;
    eca::Rule rule = 110;
    SeedMode seedMode = SeedMode::Random;
    std::uint64_t randomIndex = 1;
    bool variantsView = false;
    bool dirty = true;               // the textures must be rebuilt
    float revealT = kRevealSeconds;  // starts full: the first frame already shows a finished picture
    SDL_FPoint mouse{-100.0F, -100.0F};
    Uint64 lastTicks = 0;
    int worlds = 0;
    long maxFrames = -1;
    long frames = 0;
};

bool createPicture(SDL_Renderer* renderer, Picture& p, int w, int h) {
    p.texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, w, h);
    if (p.texture == nullptr) return false;
    p.w = w;
    p.h = h;
    SDL_SetTextureScaleMode(p.texture, SDL_SCALEMODE_NEAREST);
    return true;
}

void upload(Picture& p, const std::vector<eca::Row>& rows) {
    const int pitch = p.w * 4;
    std::vector<Uint8> pixels(static_cast<std::size_t>(pitch) * static_cast<std::size_t>(p.h));
    for (int y = 0; y < p.h; ++y) {
        for (int x = 0; x < p.w; ++x) {
            const auto ry = static_cast<std::size_t>(y);
            const auto rx = static_cast<std::size_t>(x);
            const bool on = ry < rows.size() && rx < rows[ry].size() && rows[ry][rx] != 0;
            const gd::Color c = on ? gd::kInk : gd::kWhite;
            Uint8* d = &pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(pitch) + static_cast<std::size_t>(x) * 4];
            d[0] = c.r;
            d[1] = c.g;
            d[2] = c.b;
            d[3] = 255;
        }
    }
    SDL_UpdateTexture(p.texture, nullptr, pixels.data(), pitch);
}

eca::Row seedRow(const App& app, int width) {
    return app.seedMode == SeedMode::Single ? eca::singleSeed(width) : eca::randomSeed(width, 1000 + app.randomIndex);
}

/// Anything that changes the picture: rebuild the textures and let the picture grow from the first row again.
void changed(App& app) {
    app.dirty = true;
    app.revealT = 0.0F;
}

void rebuild(App& app) {
    if (!app.variantsView) {
        upload(app.main, eca::evolve(app.rule, seedRow(app, kMainCols), kMainRows));
    } else {
        // Each variant starts from the correspondingly transformed row, so its picture is the transformed picture of the original.
        const eca::Row seed = seedRow(app, kPanelCols);
        const std::array<eca::Row, 4> seeds{seed, eca::mirrorRow(seed), eca::complementRow(seed), eca::complementRow(eca::mirrorRow(seed))};
        const auto rules = eca::variants(app.rule);
        for (std::size_t i = 0; i < 4; ++i) upload(app.panels[i], eca::evolve(rules[i], seeds[i], kPanelRows));
    }
    app.dirty = false;
}

void setRule(App& app, eca::Rule rule) {
    app.rule = rule;
    changed(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Preset30: setRule(app, 30); break;
        case Action::Preset90: setRule(app, 90); break;
        case Action::Preset110: setRule(app, 110); break;
        case Action::Preset184: setRule(app, 184); break;
        case Action::Mirror: setRule(app, eca::mirror(app.rule)); break;
        case Action::Complement: setRule(app, eca::complement(app.rule)); break;
        case Action::Variants:
            app.variantsView = !app.variantsView;
            changed(app);
            break;
        case Action::Single:
            app.seedMode = SeedMode::Single;
            changed(app);
            break;
        case Action::Random:
            if (app.seedMode == SeedMode::Random) ++app.randomIndex;  // pressing it again rolls a new row
            app.seedMode = SeedMode::Random;
            changed(app);
            break;
        case Action::Dec: setRule(app, eca::nextRule(app.rule, -1)); break;
        case Action::Inc: setRule(app, eca::nextRule(app.rule, 1)); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Preset30: return app.rule == 30;
        case Action::Preset90: return app.rule == 90;
        case Action::Preset110: return app.rule == 110;
        case Action::Preset184: return app.rule == 184;
        case Action::Variants: return app.variantsView;
        case Action::Single: return app.seedMode == SeedMode::Single;
        case Action::Random: return app.seedMode == SeedMode::Random;
        default: return false;
    }
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 9> defs{{{Action::Preset30, "30", 0.0F},
                                   {Action::Preset90, "90", 8.0F},
                                   {Action::Preset110, "110", 8.0F},
                                   {Action::Preset184, "184", 8.0F},
                                   {Action::Mirror, "M Mirror", 24.0F},
                                   {Action::Complement, "C Complement", 8.0F},
                                   {Action::Variants, "V Variants", 24.0F},
                                   {Action::Single, "S Single", 24.0F},
                                   {Action::Random, "R Random", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
    // Rule number stepping, next to the big number.
    app.chips.push_back({Action::Dec, "-1", SDL_FRect{562.0F, 90.0F, 34.0F, 24.0F}});
    app.chips.push_back({Action::Inc, "+1", SDL_FRect{602.0F, 90.0F, 34.0F, 24.0F}});
}

/// Card i (0 = leftmost) shows neighborhood 7 - i, so the order is 111 down to 000 like in every Wolfram table.
SDL_FRect resultCell(int i) { return SDL_FRect{kCardX0 + kCardStep * static_cast<float>(i) + 15.0F, kCardY + 48.0F, 26.0F, 26.0F}; }

bool inside(SDL_FPoint p, const SDL_FRect& r) { return SDL_PointInRectFloat(&p, &r); }

void drawChip(App& app, const Chip& c) {
    SDL_Renderer* r = app.renderer;
    const bool on = active(app, c.action);
    const bool hot = inside(app.mouse, c.rect);
    gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : (hot ? gd::kTealSoft : gd::kWhite), gd::kInk);
    const float ty = c.rect.y + (c.rect.h - app.hud.lineHeight(gd::Hud::Style::Bold)) / 2.0F;
    app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, ty, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
}

void drawRuleCards(App& app) {
    SDL_Renderer* r = app.renderer;
    for (int i = 0; i < 8; ++i) {
        const int n = 7 - i;
        const float x = kCardX0 + kCardStep * static_cast<float>(i);
        gd::card(r, x, kCardY, kCardW, kCardH, gd::kWhite, gd::kInk);
        // The neighborhood: left, center, right.
        for (int k = 0; k < 3; ++k) {
            const bool on = ((n >> (2 - k)) & 1) != 0;
            const float cx = x + 7.0F + 14.0F * static_cast<float>(k);
            gd::fillRect(r, cx, kCardY + 8.0F, 14.0F, 14.0F, on ? gd::kInk : gd::kWhite);
            gd::strokeRect(r, cx, kCardY + 8.0F, 14.0F, 14.0F, gd::kInk);
        }
        char label[8];
        SDL_snprintf(label, sizeof label, "%d%d%d", (n >> 2) & 1, (n >> 1) & 1, n & 1);
        app.hud.text(r, x + (kCardW - app.hud.width(label)) / 2.0F, kCardY + 24.0F, label, gd::kMuted);
        // What the middle cell becomes: this is the clickable bit.
        const SDL_FRect cell = resultCell(i);
        const bool on = eca::ruleBit(app.rule, n) != 0;
        const bool hot = inside(app.mouse, cell);
        gd::fillRect(r, cell.x, cell.y, cell.w, cell.h, on ? gd::kInk : gd::kWhite);
        gd::strokeRect(r, cell.x, cell.y, cell.w, cell.h, hot ? gd::kOrange : gd::kInk);
        if (hot) gd::strokeRect(r, cell.x + 1.0F, cell.y + 1.0F, cell.w - 2.0F, cell.h - 2.0F, gd::kOrange);
    }
}

void drawInfo(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 548.0F, kCardY, 388.0F, kCardH, gd::kWhite, gd::kInk);
    char buf[48];
    SDL_snprintf(buf, sizeof buf, "%d", static_cast<int>(app.rule));
    app.hud.text(r, 562.0F, 44.0F, buf, gd::kTealDark, gd::Hud::Style::Large);
    const auto v = eca::variants(app.rule);
    const auto members = eca::world(app.rule);
    app.hud.text(r, 650.0F, 46.0F, eca::bits(app.rule), gd::kInk, gd::Hud::Style::Bold);
    SDL_snprintf(buf, sizeof buf, "mirror %d", static_cast<int>(v[1]));
    app.hud.text(r, 650.0F, 68.0F, buf, gd::kMuted);
    SDL_snprintf(buf, sizeof buf, "compl. %d", static_cast<int>(v[2]));
    app.hud.text(r, 650.0F, 88.0F, buf, gd::kMuted);
    SDL_snprintf(buf, sizeof buf, "world %d/%d", eca::worldIndex(app.rule), app.worlds);
    app.hud.text(r, 790.0F, 46.0F, buf, gd::kOrange, gd::Hud::Style::Bold);
    SDL_snprintf(buf, sizeof buf, "%d rule%s", static_cast<int>(members.size()), members.size() == 1 ? "" : "s");
    app.hud.text(r, 790.0F, 68.0F, buf, gd::kMuted);
    if (v[1] == app.rule) app.hud.text(r, 790.0F, 88.0F, "self-mirror", gd::kMuted);
}

void drawPicture(App& app, const Picture& p, float x, float y, int cell, float t) {
    SDL_Renderer* r = app.renderer;
    const float fw = static_cast<float>(p.w * cell);
    const float fh = static_cast<float>(p.h * cell);
    gd::fillRect(r, x, y, fw, fh, gd::kWhite);
    const int shown = std::clamp(static_cast<int>(static_cast<float>(p.h) * std::min(t / kRevealSeconds, 1.0F)), 0, p.h);
    if (shown > 0) {
        const SDL_FRect src{0.0F, 0.0F, static_cast<float>(p.w), static_cast<float>(shown)};
        const SDL_FRect dst{x, y, fw, static_cast<float>(shown * cell)};
        SDL_RenderTexture(r, p.texture, &src, &dst);
    }
    gd::strokeRect(r, x - 1.0F, y - 1.0F, fw + 2.0F, fh + 2.0F, gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    char buf[64];
    gd::backdrop(r, kWidth, kHeight);
    SDL_snprintf(buf, sizeof buf, "256 rules, %d worlds", app.worlds);
    app.hud.titleBar(r, kWidth, "[ ELEMENTARY RULES ]", buf);

    drawRuleCards(app);
    drawInfo(app);

    // Time axis: the first row is the start, every row below is one step later.
    app.hud.text(r, 10.0F, kPicY - 2.0F, "t", gd::kMuted, gd::Hud::Style::Bold);
    gd::fillRect(r, 14.0F, kPicY + 24.0F, 2.0F, 292.0F, gd::kMuted);
    for (int k = 0; k < 6; ++k) {  // arrow head pointing down, one pixel row per step, widest at the top
        const auto narrow = static_cast<float>(k);
        gd::fillRect(r, 15.0F - (6.0F - narrow), kPicY + 316.0F - 7.0F + narrow, 2.0F * (6.0F - narrow), 1.0F, gd::kMuted);
    }

    if (!app.variantsView) {
        drawPicture(app, app.main, kPicX, kPicY, kMainCell, app.revealT);
    } else {
        const auto v = eca::variants(app.rule);
        const char* names[4] = {"original", "mirrored", "inverted", "both"};
        for (std::size_t i = 0; i < 4; ++i) {
            const float x = kPanelX0 + kPanelStep * static_cast<float>(i);
            SDL_snprintf(buf, sizeof buf, "%d %s%s", static_cast<int>(v[i]), names[i], (i > 0 && v[i] == app.rule) ? " (same)" : "");
            app.hud.text(r, x, kPicY - 2.0F, buf, i == 0 ? gd::kTealDark : gd::kInk, gd::Hud::Style::Bold);
            drawPicture(app, app.panels[i], x, kPanelY, kPanelCell, app.revealT);
        }
    }

    app.hud.text(r, 24.0F, 446.0F, "click a card's lower cell to flip a bit   keys: 1-4 presets, left/right rule, M C V S R", gd::kMuted);
    for (const Chip& c : app.chips) drawChip(app, c);
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
    if (!SDL_CreateWindowAndRenderer("elementary-rules", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!createPicture(app->renderer, app->main, kMainCols, kMainRows)) {
        SDL_Log("texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    for (Picture& p : app->panels) {
        if (!createPicture(app->renderer, p, kPanelCols, kPanelRows)) {
            SDL_Log("texture failed: %s", SDL_GetError());
            return SDL_APP_FAILURE;
        }
    }
    app->worlds = eca::worldCount();
    layoutChips(*app);
    rebuild(*app);
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
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int i = 0; i < 8; ++i) {
                if (inside(p, resultCell(i))) {
                    setRule(app, eca::toggleBit(app.rule, 7 - i));
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode key = event->key.key;
            // Everything toggles or rolls except the rule stepping, which is meant to be held down to scrub through rules.
            if (event->key.repeat && key != SDLK_LEFT && key != SDLK_RIGHT) break;
            switch (key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Preset30); break;
                case SDLK_2: apply(app, Action::Preset90); break;
                case SDLK_3: apply(app, Action::Preset110); break;
                case SDLK_4: apply(app, Action::Preset184); break;
                case SDLK_M: apply(app, Action::Mirror); break;
                case SDLK_C: apply(app, Action::Complement); break;
                case SDLK_V: apply(app, Action::Variants); break;
                case SDLK_S: apply(app, Action::Single); break;
                case SDLK_R: apply(app, Action::Random); break;
                case SDLK_LEFT: apply(app, Action::Dec); break;
                case SDLK_RIGHT: apply(app, Action::Inc); break;
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
    const float dt = std::min(static_cast<float>(now - app.lastTicks) / 1000.0F, 0.25F);
    app.lastTicks = now;
    if (app.dirty) rebuild(app);
    app.revealT = std::min(app.revealT + dt, kRevealSeconds);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        SDL_DestroyTexture(app->main.texture);
        for (Picture& p : app->panels) SDL_DestroyTexture(p.texture);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
