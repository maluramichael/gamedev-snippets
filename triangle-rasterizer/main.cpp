// Triangle rasterizer with SDL3 main callbacks.
//
// A magnified pixel grid and a fan of triangles that share their edges. Each pixel is drawn when its centre is
// inside a triangle; the question is what to do with a centre that lies exactly ON a shared edge. "Inclusive" counts
// it for both neighbours (orange pixels: drawn twice, a bright seam under additive blending), "Exclusive" for
// neither (dark pixels: holes), "Top-left" gives it to exactly one. The corners are draggable and start on pixel
// centres, which makes edges run through centres everywhere. The geometry lives in logic.hpp and is covered by
// test.cpp (any split of a rectangle into triangles covers each pixel exactly once).

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
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
constexpr int kGridW = 36;
constexpr int kGridH = 22;
constexpr float kCell = 20.0F;
constexpr float kGridX = 20.0F;
constexpr float kGridY = 40.0F;
constexpr float kGrab = 12.0F;  // px around a corner that grabs it
constexpr float kChipY = 492.0F;
constexpr float kChipH = 34.0F;
constexpr int kMinTris = 3;
constexpr int kMaxTris = 12;
constexpr int kStartTris = 8;

constexpr gd::Color kShade[3] = {{13, 148, 136, 255}, {74, 184, 172, 255}, {128, 212, 202, 255}};
constexpr gd::Color kInkSoft{17, 24, 39, 170};

enum class Action { Inclusive, Exclusive, TopLeft, Snap, Fewer, More, Reset };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    tr::Fan fan;
    int count = kStartTris;
    tr::Rule rule = tr::Rule::Inclusive;
    bool snap = true;
    SDL_FPoint mouse{-100.0F, -100.0F};
    int dragging = -1;  // -1 none, 0 centre, i + 1 = rim[i]
    long maxFrames = -1;
    long frames = 0;
};

// Per-frame result of rasterizing the fan under the current rule.
struct Frame {
    std::vector<tr::Tri> tris;
    std::vector<int> counts;
    std::vector<int> reference;
    std::vector<int> owner;  // index of the last triangle that drew the pixel, -1 none
    tr::Tally tally;
};

const char* ruleName(tr::Rule rule) {
    switch (rule) {
        case tr::Rule::Inclusive: return "Inclusive";
        case tr::Rule::Exclusive: return "Exclusive";
        case tr::Rule::TopLeft: return "Top-left";
    }
    return "";
}

const char* ruleTest(tr::Rule rule) {
    switch (rule) {
        case tr::Rule::Inclusive: return "E >= 0";
        case tr::Rule::Exclusive: return "E > 0";
        case tr::Rule::TopLeft: return "E > 0, or E = 0 on top/left";
    }
    return "";
}

float toScreenX(int sub) { return kGridX + static_cast<float>(sub) / static_cast<float>(tr::kSub) * kCell; }
float toScreenY(int sub) { return kGridY + static_cast<float>(sub) / static_cast<float>(tr::kSub) * kCell; }

bool inGrid(SDL_FPoint p) { return p.x >= kGridX && p.x < kGridX + kGridW * kCell && p.y >= kGridY && p.y < kGridY + kGridH * kCell; }

void resetFan(App& app) {
    app.fan = tr::presetFan(app.count, kGridW, kGridH);
    app.dragging = -1;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::Inclusive, "1 Inclusive", 0.0F},
                                   {Action::Exclusive, "2 Exclusive", 8.0F},
                                   {Action::TopLeft, "3 Top-left", 8.0F},
                                   {Action::Snap, "S Snap to centres", 20.0F},
                                   {Action::Fewer, "- Tri", 20.0F},
                                   {Action::More, "+ Tri", 6.0F},
                                   {Action::Reset, "R Reset", 14.0F}}};
    float x = kGridX;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void setCount(App& app, int count) {
    app.count = std::clamp(count, kMinTris, kMaxTris);
    resetFan(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Inclusive: app.rule = tr::Rule::Inclusive; break;
        case Action::Exclusive: app.rule = tr::Rule::Exclusive; break;
        case Action::TopLeft: app.rule = tr::Rule::TopLeft; break;
        case Action::Snap: {
            app.snap = !app.snap;
            if (app.snap) {
                // Move all corners onto pixel centres, unless that would fold the fan.
                const tr::Fan snapped = tr::snappedFan(app.fan);
                if (tr::fanValid(snapped)) app.fan = snapped;
            }
            app.dragging = -1;
            break;
        }
        case Action::Fewer: setCount(app, app.count - 1); break;
        case Action::More: setCount(app, app.count + 1); break;
        case Action::Reset: resetFan(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Inclusive: return app.rule == tr::Rule::Inclusive;
        case Action::Exclusive: return app.rule == tr::Rule::Exclusive;
        case Action::TopLeft: return app.rule == tr::Rule::TopLeft;
        case Action::Snap: return app.snap;
        default: return false;
    }
}

tr::Pt& corner(tr::Fan& fan, int index) { return index == 0 ? fan.center : fan.rim[static_cast<std::size_t>(index - 1)]; }

/// The corner nearest to the cursor within the grab radius: 0 centre, i + 1 = rim[i], -1 none.
int nearestCorner(const App& app, SDL_FPoint p) {
    int best = -1;
    float bestD = kGrab * kGrab;
    const int n = static_cast<int>(app.fan.rim.size()) + 1;
    for (int i = 0; i < n; ++i) {
        const tr::Pt c = i == 0 ? app.fan.center : app.fan.rim[static_cast<std::size_t>(i - 1)];
        const float dx = toScreenX(c.x) - p.x;
        const float dy = toScreenY(c.y) - p.y;
        const float d = dx * dx + dy * dy;
        if (d <= bestD) {
            bestD = d;
            best = i;
        }
    }
    return best;
}

void dragTo(App& app, SDL_FPoint p) {
    // Keep the corner inside the grid, a hair away from the far edge.
    const double gx = std::clamp(static_cast<double>((p.x - kGridX) / kCell), 0.0, static_cast<double>(kGridW) - 0.01);
    const double gy = std::clamp(static_cast<double>((p.y - kGridY) / kCell), 0.0, static_cast<double>(kGridH) - 0.01);
    tr::Fan trial = app.fan;
    corner(trial, app.dragging) = app.snap ? tr::snapToCentre(gx, gy) : tr::snapToSub(gx, gy);
    // A move that would flip or tangle a triangle is ignored; the corner waits at its last valid spot.
    if (tr::fanValid(trial)) app.fan = trial;
}

Frame rasterizeFan(const App& app) {
    Frame f;
    f.tris = tr::triangles(app.fan);
    f.counts = tr::coverage(f.tris, app.rule, kGridW, kGridH);
    f.reference = tr::coverage(f.tris, tr::Rule::TopLeft, kGridW, kGridH);
    f.owner.assign(f.counts.size(), -1);
    for (std::size_t i = 0; i < f.tris.size(); ++i) {
        tr::rasterize(f.tris[i], app.rule, kGridW, kGridH, [&](int x, int y) { f.owner[static_cast<std::size_t>(y * kGridW + x)] = static_cast<int>(i); });
    }
    f.tally = tr::tally(f.counts, f.reference);
    return f;
}

void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = 0; o < 2; ++o) {
        const auto off = static_cast<float>(o) - 0.5F;
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

void drawGrid(App& app, const Frame& f) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, kGridX, kGridY, kGridW * kCell, kGridH * kCell, gd::kWhite);
    for (int y = 0; y < kGridH; ++y) {
        for (int x = 0; x < kGridW; ++x) {
            const auto i = static_cast<std::size_t>(y * kGridW + x);
            const int c = f.counts[i];
            const float px = kGridX + static_cast<float>(x) * kCell;
            const float py = kGridY + static_cast<float>(y) * kCell;
            if (c >= 2) {
                gd::fillRect(r, px, py, kCell, kCell, gd::kOrange);
            } else if (c == 1) {
                gd::fillRect(r, px, py, kCell, kCell, kShade[f.owner[i] % 3]);
            } else if (f.reference[i] > 0) {
                gd::fillRect(r, px, py, kCell, kCell, gd::kInk);  // a hole: the correct rule would draw this pixel
            }
        }
    }
    // Cell lines, then a dot in every pixel centre.
    gd::setColor(r, gd::Color{17, 24, 39, 40});
    for (int x = 0; x <= kGridW; ++x) SDL_RenderLine(r, kGridX + static_cast<float>(x) * kCell, kGridY, kGridX + static_cast<float>(x) * kCell, kGridY + kGridH * kCell);
    for (int y = 0; y <= kGridH; ++y) SDL_RenderLine(r, kGridX, kGridY + static_cast<float>(y) * kCell, kGridX + kGridW * kCell, kGridY + static_cast<float>(y) * kCell);
    gd::setColor(r, gd::Color{17, 24, 39, 110});
    for (int y = 0; y < kGridH; ++y) {
        for (int x = 0; x < kGridW; ++x) {
            const SDL_FRect dot{kGridX + (static_cast<float>(x) + 0.5F) * kCell - 1.0F, kGridY + (static_cast<float>(y) + 0.5F) * kCell - 1.0F, 2.0F, 2.0F};
            SDL_RenderFillRect(r, &dot);
        }
    }
    gd::strokeRect(r, kGridX, kGridY, kGridW * kCell, kGridH * kCell, gd::kInk);
}

void drawGeometry(App& app, const Frame& f) {
    SDL_Renderer* r = app.renderer;
    for (const tr::Tri& t : f.tris) {
        const tr::Pt p[3] = {t.a, t.b, t.c};
        for (int k = 0; k < 3; ++k) {
            const tr::Pt a = p[k];
            const tr::Pt b = p[(k + 1) % 3];
            thickLine(r, toScreenX(a.x), toScreenY(a.y), toScreenX(b.x), toScreenY(b.y), kInkSoft);
        }
    }
    const int hot = app.dragging >= 0 ? app.dragging : nearestCorner(app, app.mouse);
    const int n = static_cast<int>(app.fan.rim.size()) + 1;
    for (int i = 0; i < n; ++i) {
        const tr::Pt c = i == 0 ? app.fan.center : app.fan.rim[static_cast<std::size_t>(i - 1)];
        const float cx = toScreenX(c.x);
        const float cy = toScreenY(c.y);
        gd::fillRect(r, cx - 6.0F, cy - 6.0F, 12.0F, 12.0F, i == hot ? gd::kTeal : gd::kWhite);
        gd::strokeRect(r, cx - 6.0F, cy - 6.0F, 12.0F, 12.0F, gd::kInk);
    }
}

void drawHover(App& app) {
    if (!inGrid(app.mouse) || app.dragging >= 0) return;
    const int x = static_cast<int>((app.mouse.x - kGridX) / kCell);
    const int y = static_cast<int>((app.mouse.y - kGridY) / kCell);
    const float px = kGridX + static_cast<float>(x) * kCell;
    const float py = kGridY + static_cast<float>(y) * kCell;
    gd::strokeRect(app.renderer, px, py, kCell, kCell, gd::kInk);
    gd::strokeRect(app.renderer, px + 1.0F, py + 1.0F, kCell - 2.0F, kCell - 2.0F, gd::kWhite);
}

void drawPanel(App& app, const Frame& f) {
    SDL_Renderer* r = app.renderer;
    constexpr float px = 760.0F;
    constexpr float py = 40.0F;
    constexpr float pw = 180.0F;
    gd::card(r, px, py, pw, kGridH * kCell, gd::kWhite, gd::kInk);
    float y = py + 10.0F;
    const float x = px + 12.0F;
    app.hud.text(r, x, y, "RULE", gd::kMuted);
    y += 24.0F;
    app.hud.text(r, x, y, ruleName(app.rule), app.rule == tr::Rule::TopLeft ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
    y += 24.0F;
    // The test sentence wraps at 16 characters.
    std::string test = ruleTest(app.rule);
    if (app.rule == tr::Rule::TopLeft) {
        app.hud.text(r, x, y, "E > 0, or E = 0", gd::kInk);
        app.hud.text(r, x, y + 22.0F, "on top/left edge", gd::kInk);
        y += 44.0F;
    } else {
        app.hud.text(r, x, y, test, gd::kInk);
        y += 22.0F;
    }
    y += 16.0F;

    const struct {
        const char* label;
        int value;
        gd::Color swatch;
        gd::Color text;
    } rows[3] = {{"once", f.tally.once, kShade[0], gd::kInk}, {"twice", f.tally.twice, gd::kOrange, gd::kOrange}, {"holes", f.tally.holes, gd::kInk, gd::kInk}};
    for (const auto& row : rows) {
        gd::fillRect(r, x, y + 3.0F, 14.0F, 14.0F, row.swatch);
        gd::strokeRect(r, x, y + 3.0F, 14.0F, 14.0F, gd::kInk);
        app.hud.text(r, x + 22.0F, y, row.label, gd::kMuted);
        char num[16];
        SDL_snprintf(num, sizeof num, "%d", row.value);
        app.hud.text(r, x + pw - 24.0F - app.hud.width(num, gd::Hud::Style::Bold), y, num, row.value > 0 ? row.text : gd::kMuted, gd::Hud::Style::Bold);
        y += 26.0F;
    }
    y += 10.0F;
    char line[32];
    SDL_snprintf(line, sizeof line, "%d triangles", static_cast<int>(f.tris.size()));
    app.hud.text(r, x, y, line, gd::kMuted);
    y += 30.0F;

    // The pixel under the cursor.
    if (inGrid(app.mouse)) {
        const int cx = static_cast<int>((app.mouse.x - kGridX) / kCell);
        const int cy = static_cast<int>((app.mouse.y - kGridY) / kCell);
        SDL_snprintf(line, sizeof line, "pixel %d,%d", cx, cy);
        app.hud.text(r, x, y, line, gd::kInk, gd::Hud::Style::Bold);
        y += 22.0F;
        const tr::Pt centre = tr::pixelCentre(cx, cy);
        std::string by;
        int hits = 0;
        for (std::size_t i = 0; i < f.tris.size(); ++i) {
            if (tr::covers(tr::oriented(f.tris[i]), centre, app.rule)) {
                by += (hits > 0 ? " " : "") + std::to_string(i + 1);
                ++hits;
            }
        }
        SDL_snprintf(line, sizeof line, "drawn %dx", hits);
        app.hud.text(r, x, y, line, hits >= 2 ? gd::kOrange : gd::kInk);
        y += 20.0F;
        if (!by.empty()) app.hud.text(r, x, y, "by " + by, gd::kMuted);
    } else {
        app.hud.text(r, x, y, "hover a pixel", gd::kMuted);
    }
    app.hud.text(r, x, py + kGridH * kCell - 52.0F, "drag the corners", gd::kMuted);
    app.hud.text(r, x, py + kGridH * kCell - 30.0F, "dots = centres", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const Frame f = rasterizeFan(app);
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ TRIANGLE RASTERIZER ]", "pixel centre on a shared edge: who draws it?");
    drawGrid(app, f);
    drawGeometry(app, f);
    drawHover(app);
    drawPanel(app, f);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
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
    if (!SDL_CreateWindowAndRenderer("triangle-rasterizer", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    resetFan(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {event->motion.x, event->motion.y};
            if (app.dragging >= 0) dragTo(app, app.mouse);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                    break;
                }
            }
            if (!onChip) app.dragging = nearestCorner(app, p);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = -1;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.dragging = -1;
            app.mouse = {-100.0F, -100.0F};
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Inclusive); break;
                case SDLK_2: apply(app, Action::Exclusive); break;
                case SDLK_3: apply(app, Action::TopLeft); break;
                case SDLK_S: apply(app, Action::Snap); break;
                case SDLK_MINUS:
                case SDLK_KP_MINUS: apply(app, Action::Fewer); break;
                case SDLK_EQUALS:
                case SDLK_PLUS:
                case SDLK_KP_PLUS: apply(app, Action::More); break;
                case SDLK_R: apply(app, Action::Reset); break;
                default: break;
            }
            break;
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
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
