// From a mouse position to a hex with SDL3 main callbacks.
//
// The left card follows the two steps of the conversion for the mouse position: pixel to fractional q, r, s, then
// rounding. "Naive" rounds the three components on their own, which breaks q + r + s = 0 on about a quarter of the
// plane (the orange tiles show where). "Rounded" resets the component that moved the most, and always finds a hex.
// The second mode draws a line between two hexes: N + 1 samples of a straight lerp, each rounded. The math lives in
// logic.hpp and is covered by test.cpp.

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
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kTile = 6.0F;  // size of the tiles that mark where the naive rounding fails

constexpr gd::Color kHexEdge{107, 114, 128, 170};
constexpr gd::Color kHexFill{255, 255, 255, 190};
constexpr gd::Color kInkSoft{17, 24, 39, 150};
constexpr gd::Color kBadTile{234, 88, 12, 95};

enum class Action { Naive, Rounded, Pick, Line, Pointy, Flat, Nudge };

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
    bool flat = false;
    hpr::Layout layout = hpr::makeLayout(false);
    std::vector<hpr::Cube> cells;
    bool naive = false;
    bool lineMode = false;
    bool nudge = true;
    hpr::Vec2 mouse = hpr::kStartMouse;
    hpr::Cube lineStart = hpr::kStartLine;
    long maxFrames = -1;
    long frames = 0;
};

/// What is under the mouse right now.
struct View {
    hpr::Frac frac{};
    hpr::Rounding rounding{};
    std::optional<hpr::Cube> picked;       // the hex the current mode finds, inside the field
    std::optional<hpr::Cube> fixedInside;  // the corrected hex, inside the field (the end of the line)
};

View viewOf(const App& app) {
    View v;
    v.frac = hpr::toFractional(app.layout, app.mouse);
    v.rounding = hpr::roundDetail(v.frac);
    if (hpr::inBounds(app.layout, hpr::kField, v.rounding.fixed)) v.fixedInside = v.rounding.fixed;
    if (app.naive) {
        if (hpr::valid(v.rounding.naive) && hpr::inBounds(app.layout, hpr::kField, v.rounding.naive)) v.picked = v.rounding.naive;
    } else {
        v.picked = v.fixedInside;
    }
    return v;
}

void setOrientation(App& app, bool flat) {
    app.flat = flat;
    app.layout = hpr::makeLayout(flat);
    app.cells = hpr::cells(app.layout);
    if (!hpr::inBounds(app.layout, hpr::kField, app.lineStart)) app.lineStart = hpr::kStartLine;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::Naive, "1 Naive", 0.0F},
                                   {Action::Rounded, "2 Rounded", 8.0F},
                                   {Action::Pick, "3 Pick", 24.0F},
                                   {Action::Line, "4 Line", 8.0F},
                                   {Action::Pointy, "5 Pointy", 24.0F},
                                   {Action::Flat, "6 Flat", 8.0F},
                                   {Action::Nudge, "N Nudge", 24.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Naive: app.naive = true; break;
        case Action::Rounded: app.naive = false; break;
        case Action::Pick: app.lineMode = false; break;
        case Action::Line: app.lineMode = true; break;
        case Action::Pointy: setOrientation(app, false); break;
        case Action::Flat: setOrientation(app, true); break;
        case Action::Nudge: app.nudge = !app.nudge; break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Naive: return app.naive;
        case Action::Rounded: return !app.naive;
        case Action::Pick: return !app.lineMode;
        case Action::Line: return app.lineMode;
        case Action::Pointy: return !app.flat;
        case Action::Flat: return app.flat;
        case Action::Nudge: return app.nudge;
    }
    return false;
}

// ---- drawing helpers ----

void fillHex(SDL_Renderer* r, hpr::Vec2 center, const std::array<hpr::Vec2, 6>& k, gd::Color c) {
    std::array<SDL_Vertex, 7> v{};
    v[0] = {{static_cast<float>(center.x), static_cast<float>(center.y)}, gd::fcolor(c), {0, 0}};
    for (std::size_t i = 0; i < 6; ++i) v[i + 1] = {{static_cast<float>(k[i].x), static_cast<float>(k[i].y)}, gd::fcolor(c), {0, 0}};
    const std::array<int, 18> idx{0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 1};
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

void strokeHex(SDL_Renderer* r, hpr::Vec2 center, const std::array<hpr::Vec2, 6>& k, gd::Color c, double scale = 1.0) {
    gd::setColor(r, c);
    for (std::size_t i = 0; i < 6; ++i) {
        const hpr::Vec2 a = k[i];
        const hpr::Vec2 b = k[(i + 1) % 6];
        SDL_RenderLine(r, static_cast<float>(center.x + (a.x - center.x) * scale), static_cast<float>(center.y + (a.y - center.y) * scale),
                       static_cast<float>(center.x + (b.x - center.x) * scale), static_cast<float>(center.y + (b.y - center.y) * scale));
    }
}

/// Fill and a double outline for a hex that is picked or part of the line.
void markHex(App& app, hpr::Cube h, gd::Color fill, gd::Color edge) {
    const hpr::Vec2 c = hpr::toPixel(app.layout, h);
    const auto k = hpr::corners(app.layout, h);
    fillHex(app.renderer, c, k, fill);
    strokeHex(app.renderer, c, k, edge);
    strokeHex(app.renderer, c, k, edge, 0.93);
}

void centered(App& app, float cx, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Bold) {
    app.hud.text(app.renderer, cx - app.hud.width(s, style) / 2.0F, y, s, c, style);
}

/// "%.3f" without a minus sign in front of zero.
std::string fixed3(double v) {
    char buf[32];
    SDL_snprintf(buf, sizeof buf, "%.3f", std::fabs(v) < 0.0005 ? 0.0 : v);
    return buf;
}

std::string integer(int v) {
    char buf[16];
    SDL_snprintf(buf, sizeof buf, "%d", v);
    return buf;
}

std::string triple(hpr::Cube c) {
    char buf[48];
    SDL_snprintf(buf, sizeof buf, "%d, %d, %d", c.q, c.r, c.s);
    return buf;
}

hpr::Vec2 pixelOf(const App& app, hpr::Frac f) { return hpr::toPixel(app.layout, f.q, f.r); }

// ---- the field ----

void drawField(App& app, const View& view) {
    SDL_Renderer* r = app.renderer;
    for (const hpr::Cube& h : app.cells) {
        const hpr::Vec2 c = hpr::toPixel(app.layout, h);
        const auto k = hpr::corners(app.layout, h);
        fillHex(r, c, k, kHexFill);
        strokeHex(r, c, k, kHexEdge);
    }

    if (!app.lineMode) {
        if (app.naive) {
            // Every orange tile is a mouse position where the naive rounding gives something that is not a hex.
            for (float y = static_cast<float>(hpr::kField.y0); y < static_cast<float>(hpr::kField.y1); y += kTile) {
                for (float x = static_cast<float>(hpr::kField.x0); x < static_cast<float>(hpr::kField.x1); x += kTile) {
                    const hpr::Vec2 center{static_cast<double>(x + kTile / 2.0F), static_cast<double>(y + kTile / 2.0F)};
                    const hpr::Rounding o = hpr::roundDetail(hpr::toFractional(app.layout, center));
                    if (!hpr::valid(o.naive)) gd::fillRect(r, x, y, kTile, kTile, kBadTile);
                }
            }
        }
        if (view.picked) markHex(app, *view.picked, gd::kTealSoft, gd::kTeal);
        if (!app.naive && view.picked) {
            const hpr::Vec2 c = hpr::toPixel(app.layout, *view.picked);
            gd::setColor(r, gd::kTealDark);
            SDL_RenderLine(r, static_cast<float>(app.mouse.x), static_cast<float>(app.mouse.y), static_cast<float>(c.x), static_cast<float>(c.y));
        }
        const bool bad = app.naive && !hpr::valid(view.rounding.naive);
        const gd::Color dot = bad ? gd::kOrange : gd::kTeal;
        gd::disc(r, static_cast<float>(app.mouse.x), static_cast<float>(app.mouse.y), 6.0F, dot);
        gd::ring(r, static_cast<float>(app.mouse.x), static_cast<float>(app.mouse.y), 6.0F, gd::kInk);
        gd::ring(r, static_cast<float>(app.mouse.x), static_cast<float>(app.mouse.y), 7.0F, gd::kInk);
        return;
    }

    // Line mode: the end of the line follows the mouse.
    const hpr::Cube a = app.lineStart;
    const hpr::Cube b = view.fixedInside ? *view.fixedInside : a;
    const std::vector<hpr::Frac> samples = hpr::lineSamples(a, b, app.nudge);
    std::vector<std::optional<hpr::Cube>> hit;
    for (const hpr::Frac& f : samples) {
        const hpr::Rounding o = hpr::roundDetail(f);
        hit.push_back(app.naive ? (hpr::valid(o.naive) ? std::optional<hpr::Cube>(o.naive) : std::nullopt) : std::optional<hpr::Cube>(o.fixed));
    }
    for (const auto& h : hit) {
        if (h) markHex(app, *h, gd::kTealSoft, gd::kTeal);
    }
    markHex(app, a, gd::Color{13, 148, 136, 0}, gd::kInk);
    markHex(app, b, gd::Color{13, 148, 136, 0}, gd::kInk);

    const hpr::Vec2 pa = hpr::toPixel(app.layout, a);
    const hpr::Vec2 pb = hpr::toPixel(app.layout, b);
    gd::setColor(r, kInkSoft);
    SDL_RenderLine(r, static_cast<float>(pa.x), static_cast<float>(pa.y), static_cast<float>(pb.x), static_cast<float>(pb.y));

    for (std::size_t i = 0; i < samples.size(); ++i) {
        const hpr::Vec2 p = pixelOf(app, samples[i]);
        if (hit[i]) {
            gd::disc(r, static_cast<float>(p.x), static_cast<float>(p.y), 4.0F, gd::kWhite);
            gd::ring(r, static_cast<float>(p.x), static_cast<float>(p.y), 4.0F, gd::kInk);
        } else {
            gd::disc(r, static_cast<float>(p.x), static_cast<float>(p.y), 6.0F, gd::kOrange);
            gd::ring(r, static_cast<float>(p.x), static_cast<float>(p.y), 6.0F, gd::kInk);
        }
        if (hit[i]) {
            const hpr::Vec2 c = hpr::toPixel(app.layout, *hit[i]);
            std::string label = integer(static_cast<int>(i));
            if (i == 0) label = "A";
            if (i + 1 == samples.size() && samples.size() > 1) label = "B";
            // The line runs close to the center, so the label goes on the side of the hex away from the sample dot.
            const float ly = p.y > c.y ? static_cast<float>(c.y) - 20.0F : static_cast<float>(c.y) + 6.0F;
            centered(app, static_cast<float>(c.x), ly, label, gd::kTealDark);
        }
    }
}

// ---- the cards ----

void drawConversionCard(App& app, const View& view) {
    SDL_Renderer* r = app.renderer;
    const hpr::Rounding& o = view.rounding;
    gd::card(r, 24.0F, 44.0F, 320.0F, 250.0F, gd::kWhite, gd::kInk);

    char line[96];
    SDL_snprintf(line, sizeof line, "mouse  %d, %d px", hpr::toInt(app.mouse.x), hpr::toInt(app.mouse.y));
    app.hud.text(r, 38.0F, 52.0F, line, gd::kMuted);

    const float fracX = 62.0F;
    const float roundX = 146.0F;
    const float diffX = 206.0F;
    const float fixedX = 278.0F;
    app.hud.text(r, fracX, 76.0F, "frac", gd::kMuted);
    app.hud.text(r, roundX, 76.0F, "round", gd::kMuted);
    app.hud.text(r, diffX, 76.0F, "diff", gd::kMuted);
    app.hud.text(r, fixedX, 76.0F, "fixed", gd::kMuted);

    const std::array<const char*, 3> names{"q", "r", "s"};
    const std::array<double, 3> fracs{view.frac.q, view.frac.r, view.frac.s};
    const std::array<int, 3> naives{o.naive.q, o.naive.r, o.naive.s};
    const std::array<int, 3> fixeds{o.fixed.q, o.fixed.r, o.fixed.s};
    const std::array<double, 3> diffs{o.dq, o.dr, o.ds};
    for (int i = 0; i < 3; ++i) {
        const auto k = static_cast<std::size_t>(i);
        const float y = 100.0F + 24.0F * static_cast<float>(i);
        const bool changed = !app.naive && o.fixedAxis == i;
        app.hud.text(r, 38.0F, y, names[k], gd::kInk, gd::Hud::Style::Bold);
        app.hud.text(r, fracX, y, fixed3(fracs[k]), gd::kInk);
        app.hud.text(r, roundX, y, integer(naives[k]), gd::kInk);
        const bool diffMatters = o.fixedAxis == i;
        app.hud.text(r, diffX, y, fixed3(diffs[k]), diffMatters ? gd::kOrange : gd::kMuted, diffMatters ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        if (app.naive) {
            app.hud.text(r, fixedX, y, "-", gd::kMuted);
        } else {
            app.hud.text(r, fixedX, y, integer(fixeds[k]), changed ? gd::kOrange : gd::kTealDark, changed ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        }
    }

    const int sum = o.naive.q + o.naive.r + o.naive.s;
    const bool sumOk = sum == 0;
    SDL_snprintf(line, sizeof line, sumOk ? "sum of round = %d" : "sum of round = %d, not 0", sum);
    app.hud.text(r, 38.0F, 178.0F, line, sumOk ? gd::kTealDark : gd::kOrange, gd::Hud::Style::Bold);

    if (app.naive) {
        app.hud.text(r, 38.0F, 202.0F, sumOk ? "lucky, this one is a hex" : "no hex here, naive fails", sumOk ? gd::kMuted : gd::kOrange);
    } else if (o.fixedAxis >= 0) {
        static const std::array<const char*, 3> formulas{"q = -r - s", "r = -q - s", "s = -q - r"};
        SDL_snprintf(line, sizeof line, "biggest diff: %s", names[static_cast<std::size_t>(o.fixedAxis)]);
        app.hud.text(r, 38.0F, 202.0F, line, gd::kOrange);
        SDL_snprintf(line, sizeof line, "reset %s", formulas[static_cast<std::size_t>(o.fixedAxis)]);
        app.hud.text(r, 38.0F, 224.0F, line, gd::kOrange);
    } else {
        app.hud.text(r, 38.0F, 202.0F, "sum is 0, nothing to fix", gd::kMuted);
    }

    std::string verdict;
    gd::Color color = gd::kTeal;
    if (view.picked) {
        verdict = "HEX " + triple(*view.picked);
    } else if (app.naive && !sumOk) {
        verdict = "NO HEX";
        color = gd::kOrange;
    } else {
        verdict = "OUTSIDE";
        color = gd::kMuted;
    }
    app.hud.text(r, 38.0F, 248.0F, verdict, color, gd::Hud::Style::Large);
}

void drawInfoCard(App& app, const View& view) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 24.0F, 306.0F, 320.0F, 128.0F, gd::kWhite, gd::kInk);
    char line[96];
    if (!app.lineMode) {
        app.hud.text(r, 38.0F, 314.0F, "1 pixel to fractional q r s", gd::kInk);
        app.hud.text(r, 38.0F, 338.0F, "2 round each component", gd::kInk);
        app.hud.text(r, 38.0F, 362.0F, "3 sum not 0: reset the one", gd::kInk);
        app.hud.text(r, 38.0F, 386.0F, "  that moved the most", gd::kInk);
        app.hud.text(r, 38.0F, 410.0F, app.naive ? "orange area: naive fails" : "key 1: where naive fails", app.naive ? gd::kOrange : gd::kMuted, gd::Hud::Style::Bold);
        return;
    }
    const hpr::Cube a = app.lineStart;
    const hpr::Cube b = view.fixedInside ? *view.fixedInside : a;
    const int n = hpr::distance(a, b);
    app.hud.text(r, 38.0F, 314.0F, "A  " + triple(a), gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, 38.0F, 338.0F, "B  " + triple(b), gd::kInk, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "N = %d, %d samples", n, n + 1);
    app.hud.text(r, 38.0F, 362.0F, line, gd::kInk);
    app.hud.text(r, 38.0F, 386.0F, "t = i / N, i = 0 to N", gd::kMuted);
    if (app.naive) {
        int bad = 0;
        for (const hpr::Frac& f : hpr::lineSamples(a, b, app.nudge)) bad += hpr::valid(hpr::roundDetail(f).naive) ? 0 : 1;
        SDL_snprintf(line, sizeof line, "%d of %d are no hex", bad, n + 1);
        app.hud.text(r, 38.0F, 410.0F, line, bad > 0 ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);
    } else {
        app.hud.text(r, 38.0F, 410.0F, app.nudge ? "nudge on: 1e-6 shift" : "nudge off", app.nudge ? gd::kTealDark : gd::kMuted, gd::Hud::Style::Bold);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const View view = viewOf(app);

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ MOUSE TO HEX ]", app.lineMode ? "line: lerp + round" : "pixel to cube_round");

    drawField(app, view);
    drawConversionCard(app, view);
    drawInfoCard(app, view);

    app.hud.text(r, 40.0F, 446.0F, app.lineMode ? "move: end B (rounded)   click: set start A   keys 1-6, N" : "move the mouse over the hexes   keys 1-6, N", gd::kMuted);

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
        if (std::strcmp(argv[i], "--frames") == 0) {
            app->maxFrames = std::atol(argv[i + 1]);
        }
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("hex-pixel-runden", kWidth, kHeight, 0, &app->window, &app->renderer)) {
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
    setOrientation(*app, false);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)}; break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = {static_cast<double>(p.x), static_cast<double>(p.y)};
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                }
            }
            if (!onChip && app.lineMode) {
                if (const auto h = viewOf(app).fixedInside) app.lineStart = *h;
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Naive); break;
                case SDLK_2: apply(app, Action::Rounded); break;
                case SDLK_3: apply(app, Action::Pick); break;
                case SDLK_4: apply(app, Action::Line); break;
                case SDLK_5: apply(app, Action::Pointy); break;
                case SDLK_6: apply(app, Action::Flat); break;
                case SDLK_N: apply(app, Action::Nudge); break;
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
