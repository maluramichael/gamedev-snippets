// Cyclic cellular automaton with SDL3 main callbacks.
//
// Every cell has one of n colors on a circle and is eaten by the next color as soon as enough neighbors carry it. From noise this
// grows blocks of color, then spirals that turn around "demons" (a 2x2 block around which the color circle winds), until every cell repeats its
// colors with the same period. The counter on the right counts the demons step by step, the status line waits for the moment the whole
// grid repeats itself. Two switches show the typical mistakes next to the right thing: updating in place instead of with a second
// buffer, and a color circle that forgot to wrap around (a line). The automaton, the demon count and the period detector live in
// logic.hpp and are covered by test.cpp.

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

constexpr int kCols = 160;
constexpr int kRows = 96;
constexpr float kCell = 4.0F;
constexpr float kFieldX = 24.0F;
constexpr float kFieldY = 42.0F;
constexpr float kFieldW = static_cast<float>(kCols) * kCell;
constexpr float kFieldH = static_cast<float>(kRows) * kCell;

constexpr float kPanelX = 688.0F;
constexpr float kPanelW = 248.0F;

constexpr float kRowOne = 442.0F;
constexpr float kRowTwo = 482.0F;
constexpr float kChipH = 30.0F;

constexpr int kWarmupSteps = 40;     // steps already taken when the demo opens, so the first picture shows blocks and the first spirals
constexpr int kMaxStepsPerFrame = 8;
constexpr int kMaxMarks = 200;       // demon rings are only drawn when there are few of them, in noise they would cover everything
constexpr std::size_t kSeriesPoints = 600;
constexpr std::array<int, 5> kRates{{5, 15, 30, 60, 120}};  // steps per second
constexpr std::uint64_t kFirstSeed = 11;

enum class Action {
    ColorsDec, ColorsInc, ThresholdDec, ThresholdInc, Neumann, Moore, Buffered, InPlace, Ring, Line, Demons, Pause, Reroll, SlowerDown, FasterUp
};

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

enum class Value { Colors, Threshold, Rate };

struct Slot {  // a number between two chips, drawn every frame
    Value value;
    float x;
    float y;
    float w;
};

struct Caption {
    std::string text;
    float x;
    float y;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* field = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<Slot> slots;
    std::vector<Caption> captions;

    cca::Rule rule{12, 1, cca::Hood::VonNeumann, true};
    bool inPlace = false;
    bool showDemons = true;
    bool paused = false;
    int rateIdx = 2;

    cca::Grid cur;
    cca::Grid nxt;
    std::uint64_t seed = kFirstSeed;
    long stepCount = 0;
    cca::Demons demons;
    int startDemons = 0;
    cca::Series series{kSeriesPoints};
    cca::PeriodDetector detector{32};
    int period = 0;
    std::vector<cca::DemonSite> sites;

    std::array<std::array<Uint8, 3>, cca::kMaxColors> palette{};
    std::vector<Uint8> pixels = std::vector<Uint8>(static_cast<std::size_t>(kCols) * kRows * 4, 255);
    bool dirty = true;

    double acc = 0.0;
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

std::array<Uint8, 3> hsv(float hue, float sat, float val) {
    const float h = std::fmod(hue, 360.0F) / 60.0F;
    const float c = val * sat;
    const float x = c * (1.0F - std::fabs(std::fmod(h, 2.0F) - 1.0F));
    float r = 0.0F;
    float g = 0.0F;
    float b = 0.0F;
    if (h < 1.0F) { r = c; g = x; }
    else if (h < 2.0F) { r = x; g = c; }
    else if (h < 3.0F) { g = c; b = x; }
    else if (h < 4.0F) { g = x; b = c; }
    else if (h < 5.0F) { r = x; b = c; }
    else { r = c; b = x; }
    const float m = val - c;
    const auto to8 = [](float v) { return static_cast<Uint8>(std::lround(std::clamp(v, 0.0F, 1.0F) * 255.0F)); };
    return {to8(r + m), to8(g + m), to8(b + m)};
}

/// The n colors are n steps around the hue circle, so "the next color" is also the next hue.
void buildPalette(App& app) {
    for (int i = 0; i < app.rule.colors; ++i) {
        app.palette[static_cast<std::size_t>(i)] = hsv(360.0F * static_cast<float>(i) / static_cast<float>(app.rule.colors) + 8.0F, 0.72F, 0.96F);
    }
}

/// 1234567 as "1,234,567". snprintf would use the locale of the machine.
std::string grouped(long v) {
    std::string digits = std::to_string(std::labs(v));
    std::string out;
    int count = 0;
    for (auto it = digits.rbegin(); it != digits.rend(); ++it) {
        if (count > 0 && count % 3 == 0) out.push_back(',');
        out.push_back(*it);
        ++count;
    }
    if (v < 0) out.push_back('-');
    std::reverse(out.begin(), out.end());
    return out;
}

const char* hoodName(cca::Hood hood) { return hood == cca::Hood::Moore ? "Moore" : "von Neumann"; }

void advance(App& app) {
    if (app.inPlace) {
        cca::stepInPlace(app.cur, app.rule);
    } else {
        cca::stepInto(app.cur, app.nxt, app.rule);
        std::swap(app.cur, app.nxt);
    }
    ++app.stepCount;
    app.demons = cca::countDemons(app.cur, app.rule.colors);
    app.series.push(app.demons.total());
    app.period = app.detector.push(app.cur);
    app.dirty = true;
}

/// New noise with the current rule. Everything that belongs to the old run restarts with it: the step count, the curve, the period
/// detector and the time accumulator (or the first step would come early).
void restart(App& app, int warmup) {
    app.rule.colors = cca::clampColors(app.rule.colors);
    app.rule.threshold = cca::clampThreshold(app.rule.threshold, app.rule.hood);
    buildPalette(app);
    app.cur = cca::randomGrid(kCols, kRows, app.rule.colors, app.seed++);
    app.nxt = cca::Grid(kCols, kRows);
    app.stepCount = 0;
    app.acc = 0.0;
    app.series.clear();
    app.detector.reset();
    app.period = 0;
    app.demons = cca::countDemons(app.cur, app.rule.colors);
    app.startDemons = app.demons.total();
    app.series.push(app.startDemons);
    for (int i = 0; i < warmup; ++i) advance(app);
    app.dirty = true;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ColorsDec: if (app.rule.colors > cca::kMinColors) { --app.rule.colors; restart(app, 0); } break;
        case Action::ColorsInc: if (app.rule.colors < cca::kMaxColors) { ++app.rule.colors; restart(app, 0); } break;
        case Action::ThresholdDec: if (app.rule.threshold > 1) { --app.rule.threshold; restart(app, 0); } break;
        case Action::ThresholdInc:
            if (app.rule.threshold < cca::neighborsOf(app.rule.hood)) { ++app.rule.threshold; restart(app, 0); }
            break;
        case Action::Neumann: app.rule.hood = cca::Hood::VonNeumann; restart(app, 0); break;
        case Action::Moore: app.rule.hood = cca::Hood::Moore; restart(app, 0); break;
        case Action::Buffered: app.inPlace = false; restart(app, 0); break;
        case Action::InPlace: app.inPlace = true; restart(app, 0); break;
        case Action::Ring: app.rule.ring = true; restart(app, 0); break;
        case Action::Line: app.rule.ring = false; restart(app, 0); break;
        case Action::Demons: app.showDemons = !app.showDemons; break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Reroll: restart(app, 0); break;
        case Action::SlowerDown: app.rateIdx = std::max(app.rateIdx - 1, 0); app.acc = 0.0; break;
        case Action::FasterUp: app.rateIdx = std::min(app.rateIdx + 1, static_cast<int>(kRates.size()) - 1); app.acc = 0.0; break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Neumann: return app.rule.hood == cca::Hood::VonNeumann;
        case Action::Moore: return app.rule.hood == cca::Hood::Moore;
        case Action::Buffered: return !app.inPlace;
        case Action::InPlace: return app.inPlace;
        case Action::Ring: return app.rule.ring;
        case Action::Line: return !app.rule.ring;
        case Action::Demons: return app.showDemons;
        case Action::Pause: return app.paused;
        default: return false;
    }
}

bool enabled(const App& app, Action action) {
    switch (action) {
        case Action::ColorsDec: return app.rule.colors > cca::kMinColors;
        case Action::ColorsInc: return app.rule.colors < cca::kMaxColors;
        case Action::ThresholdDec: return app.rule.threshold > 1;
        case Action::ThresholdInc: return app.rule.threshold < cca::neighborsOf(app.rule.hood);
        case Action::SlowerDown: return app.rateIdx > 0;
        case Action::FasterUp: return app.rateIdx + 1 < static_cast<int>(kRates.size());
        default: return true;
    }
}

float chipWidth(const App& app, const std::string& label) { return app.hud.width(label, gd::Hud::Style::Bold) + 24.0F; }

void layoutChips(App& app) {
    const auto chip = [&app](Action action, const std::string& label, float x, float y, float w) {
        app.chips.push_back({action, label, SDL_FRect{x, y, w, kChipH}});
        return x + w;
    };
    // Steppers: caption, minus chip, a slot for the number, plus chip. Returns the x after the plus chip.
    const auto stepper = [&](const std::string& caption, Action dec, Action inc, Value value, float x, float y, float slotW) {
        app.captions.push_back({caption, x, y + (kChipH - app.hud.lineHeight()) / 2.0F});
        x += app.hud.width(caption) + 10.0F;
        x = chip(dec, "-", x, y, 30.0F);
        app.slots.push_back({value, x, y, slotW});
        x += slotW;
        return chip(inc, "+", x, y, 30.0F);
    };

    float x = stepper("colors n", Action::ColorsDec, Action::ColorsInc, Value::Colors, 24.0F, kRowOne, 40.0F);
    x = stepper("threshold", Action::ThresholdDec, Action::ThresholdInc, Value::Threshold, x + 28.0F, kRowOne, 34.0F);
    x += 28.0F;
    x = chip(Action::Neumann, "1 von Neumann", x, kRowOne, chipWidth(app, "1 von Neumann")) + 8.0F;
    x = chip(Action::Moore, "2 Moore", x, kRowOne, chipWidth(app, "2 Moore"));
    stepper("speed", Action::SlowerDown, Action::FasterUp, Value::Rate, x + 28.0F, kRowOne, 56.0F);

    x = 24.0F;
    x = chip(Action::Buffered, "3 Buffered", x, kRowTwo, chipWidth(app, "3 Buffered")) + 8.0F;
    x = chip(Action::InPlace, "4 In place", x, kRowTwo, chipWidth(app, "4 In place")) + 20.0F;
    x = chip(Action::Ring, "5 Ring", x, kRowTwo, chipWidth(app, "5 Ring")) + 8.0F;
    x = chip(Action::Line, "6 Line", x, kRowTwo, chipWidth(app, "6 Line")) + 20.0F;
    x = chip(Action::Demons, "D Demons", x, kRowTwo, chipWidth(app, "D Demons")) + 20.0F;
    x = chip(Action::Pause, "Space Pause", x, kRowTwo, chipWidth(app, "Space Pause")) + 8.0F;
    chip(Action::Reroll, "R Reroll", x, kRowTwo, chipWidth(app, "R Reroll"));
}

void upload(App& app) {
    for (int i = 0; i < kCols * kRows; ++i) {
        const auto& c = app.palette[app.cur.cells[static_cast<std::size_t>(i)]];
        Uint8* p = &app.pixels[static_cast<std::size_t>(i) * 4];
        p[0] = c[0];
        p[1] = c[1];
        p[2] = c[2];
        p[3] = 255;
    }
    SDL_UpdateTexture(app.field, nullptr, app.pixels.data(), kCols * 4);
    app.dirty = false;
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Caption& c : app.captions) app.hud.text(r, c.x, c.y, c.text, gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool ok = enabled(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, ok ? gd::kInk : gd::kGrid);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label,
                     on ? gd::kWhite : (ok ? gd::kInk : gd::kMuted), gd::Hud::Style::Bold);
    }
    for (const Slot& s : app.slots) {
        char text[16];
        switch (s.value) {
            case Value::Colors: SDL_snprintf(text, sizeof text, "%d", app.rule.colors); break;
            case Value::Threshold: SDL_snprintf(text, sizeof text, "%d", app.rule.threshold); break;
            case Value::Rate: SDL_snprintf(text, sizeof text, "%d/s", kRates[static_cast<std::size_t>(app.rateIdx)]); break;
        }
        app.hud.text(r, s.x + (s.w - app.hud.width(text, gd::Hud::Style::Bold)) / 2.0F, s.y + 4.0F, text, gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawDemonMarks(App& app) {
    if (!app.showDemons || app.demons.total() > kMaxMarks || app.demons.total() == 0) return;
    cca::collectDemons(app.cur, app.rule.colors, app.sites);
    for (const cca::DemonSite& s : app.sites) {
        const float cx = kFieldX + static_cast<float>((s.x + 1) % kCols) * kCell;
        const float cy = kFieldY + static_cast<float>((s.y + 1) % kRows) * kCell;
        const gd::Color c = s.sign > 0 ? gd::kTeal : gd::kOrange;
        gd::ring(app.renderer, cx, cy, 9.0F, gd::kWhite);
        gd::ring(app.renderer, cx, cy, 8.0F, c);
        gd::ring(app.renderer, cx, cy, 7.0F, c);
        gd::ring(app.renderer, cx, cy, 6.0F, gd::kWhite);
    }
}

/// The demon count over time on a log scale (white noise starts in the thousands and ends in the dozens).
void drawCurve(App& app, float x, float y, float w, float h) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, x, y, w, h, gd::kPaper);
    gd::strokeRect(r, x, y, w, h, gd::kInk);
    const std::vector<int>& v = app.series.values();
    if (v.size() < 2) return;
    const double top = std::log1p(static_cast<double>(std::max(app.startDemons, 1)));
    const auto px = [&](std::size_t i) { return x + 2.0F + (w - 4.0F) * static_cast<float>(i) / static_cast<float>(kSeriesPoints - 1); };
    const auto py = [&](int value) { return y + h - 3.0F - (h - 6.0F) * static_cast<float>(std::log1p(static_cast<double>(value)) / top); };
    gd::setColor(r, gd::kTeal);
    for (std::size_t i = 1; i < v.size(); ++i) {
        for (float o = 0.0F; o < 2.0F; o += 1.0F) SDL_RenderLine(r, px(i - 1), py(v[i - 1]) + o, px(i), py(v[i]) + o);
    }
}

void drawWheel(App& app, float cx, float cy, float radius) {
    SDL_Renderer* r = app.renderer;
    const int n = app.rule.colors;
    const auto pos = [&](int i) {
        const float a = -1.5707963F + 6.2831853F * static_cast<float>(i) / static_cast<float>(n);
        return SDL_FPoint{cx + std::cos(a) * radius, cy + std::sin(a) * radius};
    };
    // The way "eaten by the next one" goes around. A line has no way from the last color back to the first.
    gd::setColor(r, gd::kInk);
    for (int i = 0; i < n; ++i) {
        if (i == n - 1 && !app.rule.ring) break;
        const SDL_FPoint a = pos(i);
        const SDL_FPoint b = pos((i + 1) % n);
        SDL_RenderLine(r, a.x, a.y, b.x, b.y);
    }
    for (int i = 0; i < n; ++i) {
        const SDL_FPoint p = pos(i);
        const auto& c = app.palette[static_cast<std::size_t>(i)];
        gd::disc(r, p.x, p.y, 5.0F, gd::Color{c[0], c[1], c[2]});
        gd::ring(r, p.x, p.y, 5.0F, gd::kInk);
    }
    app.hud.text(r, cx - app.hud.width("0", gd::Hud::Style::Bold) / 2.0F, cy - 8.0F, "0", gd::kInk, gd::Hud::Style::Bold);
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    const float x = kPanelX + 12.0F;

    gd::card(r, kPanelX, kFieldY, kPanelW, 142.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, x, kFieldY + 6.0F, "DEMONS", gd::kMuted);
    app.hud.text(r, x + kPanelW - 24.0F - app.hud.width("log scale"), kFieldY + 6.0F, "log scale", gd::kMuted);
    app.hud.text(r, x, kFieldY + 22.0F, grouped(app.demons.total()), gd::kInk, gd::Hud::Style::Large);
    const std::string plus = "+" + grouped(app.demons.positive);
    app.hud.text(r, x, kFieldY + 68.0F, plus, gd::kTeal, gd::Hud::Style::Bold);
    app.hud.text(r, x + app.hud.width(plus, gd::Hud::Style::Bold) + 20.0F, kFieldY + 68.0F, "-" + grouped(app.demons.negative), gd::kOrange, gd::Hud::Style::Bold);
    drawCurve(app, x, kFieldY + 94.0F, kPanelW - 24.0F, 36.0F);

    const float by = kFieldY + 154.0F;
    gd::card(r, kPanelX, by, kPanelW, 62.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, x, by + 6.0F, "step " + grouped(app.stepCount), gd::kInk, gd::Hud::Style::Bold);
    std::string status = "settling...";
    gd::Color statusColor = gd::kMuted;
    if (app.period == 1) {
        status = "frozen";
        statusColor = gd::kOrange;
    } else if (app.period > 1) {
        status = "period " + std::to_string(app.period) + (app.period == app.rule.colors ? " = n" : (app.period == app.rule.colors + 1 ? " = n+1" : ""));
        statusColor = app.period == app.rule.colors ? gd::kTeal : gd::kOrange;
    }
    app.hud.text(r, x, by + 32.0F, status, statusColor, gd::Hud::Style::Bold);

    drawWheel(app, kPanelX + 40.0F, 299.0F, 26.0F);
    app.hud.text(r, kPanelX + 86.0F, 272.0F, "eaten by the", gd::kMuted);
    app.hud.text(r, kPanelX + 86.0F, 292.0F, "next color", gd::kMuted);
    app.hud.text(r, kPanelX + 86.0F, 312.0F, app.rule.ring ? "n-1 wraps to 0" : "n-1 has no eater", app.rule.ring ? gd::kMuted : gd::kOrange);

    app.hud.text(r, x, 342.0F, "Left/Right  n", gd::kMuted);
    app.hud.text(r, x, 362.0F, "Up/Down  threshold", gd::kMuted);
    app.hud.text(r, x, 382.0F, "+/-  speed", gd::kMuted);
    app.hud.text(r, x, 402.0F, "click field: new noise", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[96];
    SDL_snprintf(right, sizeof right, "n=%d  threshold %d  %s", app.rule.colors, app.rule.threshold, hoodName(app.rule.hood));
    app.hud.titleBar(r, kWidth, "[ CYCLIC CELLULAR AUTOMATON ]", right);

    const SDL_FRect dst{kFieldX, kFieldY, kFieldW, kFieldH};
    SDL_RenderTexture(r, app.field, nullptr, &dst);
    gd::strokeRect(r, kFieldX - 1.0F, kFieldY - 1.0F, kFieldW + 2.0F, kFieldH + 2.0F, gd::kInk);
    drawDemonMarks(app);
    drawPanel(app);
    drawChips(app);
}

bool inField(float x, float y) { return x >= kFieldX && x < kFieldX + kFieldW && y >= kFieldY && y < kFieldY + kFieldH; }

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
    if (!SDL_CreateWindowAndRenderer("cyclic-ca-spirals", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->field = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kCols, kRows);
    if (app->field == nullptr) {
        SDL_Log("field texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->field, SDL_SCALEMODE_NEAREST);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    restart(*app, kWarmupSteps);
    app->lastTicks = SDL_GetTicks();
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
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                    break;
                }
            }
            if (!onChip && inField(p.x, p.y)) restart(app, 0);
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key restarts the run or toggles something, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_LEFT: apply(app, Action::ColorsDec); break;
                case SDLK_RIGHT: apply(app, Action::ColorsInc); break;
                case SDLK_DOWN: apply(app, Action::ThresholdDec); break;
                case SDLK_UP: apply(app, Action::ThresholdInc); break;
                case SDLK_1: apply(app, Action::Neumann); break;
                case SDLK_2: apply(app, Action::Moore); break;
                case SDLK_3: apply(app, Action::Buffered); break;
                case SDLK_4: apply(app, Action::InPlace); break;
                case SDLK_5: apply(app, Action::Ring); break;
                case SDLK_6: apply(app, Action::Line); break;
                case SDLK_D: apply(app, Action::Demons); break;
                case SDLK_SPACE: apply(app, Action::Pause); break;
                case SDLK_R: apply(app, Action::Reroll); break;
                case SDLK_MINUS: apply(app, Action::SlowerDown); break;
                case SDLK_PLUS:
                case SDLK_EQUALS: apply(app, Action::FasterUp); break;
                default: break;
            }
            break;
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
        const int due = cca::stepsDue(app.acc, kRates[static_cast<std::size_t>(app.rateIdx)], dt, kMaxStepsPerFrame);
        for (int i = 0; i < due; ++i) advance(app);
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
        if (app->field != nullptr) SDL_DestroyTexture(app->field);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
