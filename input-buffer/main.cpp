// Coyote time and input buffer with SDL3 main callbacks.
//
// A tester runs over stepping stones and presses jump a little too late at a ledge and a little too early before a
// landing, like a human does. Two windows forgive that: coyote time lets a jump still work for a moment after the
// ground is gone, the input buffer keeps an early press alive until the player can use it. The timeline at the
// bottom shows the last three seconds: when the player could jump, when jump was pressed and what became of it.
//
// The simulation runs in fixed ticks of 1/120 s (logic.hpp), fed from an accumulator, so a window of 100 ms is the
// same 12 ticks at any frame rate. Frame time is clamped, otherwise a stalled tab would trigger a burst of ticks.
// Keys: space or click jumps, C and B cycle the windows, M switches bot and you, X the reset on jump, T slow motion.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
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
constexpr double kMaxFrameTime = 0.25;  // clamp: no burst of ticks after a stall
constexpr int kMaxTicksPerFrame = 40;
constexpr long kPreroll = 960;  // ticks simulated before the first frame so the timeline is not empty

// Scene.
constexpr float kSceneTop = 30.0F;
constexpr float kSceneBottom = 294.0F;
constexpr float kGroundY = 236.0F;  // screen y of the top line of the stones
constexpr float kPlayerX = 420.0F;  // the camera keeps the player here
constexpr float kStoneH = 26.0F;
constexpr float kPlayerW = 18.0F;
constexpr float kPlayerH = 24.0F;

// Timeline.
constexpr float kTlTop = 298.0F;
constexpr float kTlHeight = 136.0F;
constexpr float kTlLeft = 120.0F;
constexpr float kTlRight = 924.0F;
constexpr int kTlTicks = 360;  // three seconds
constexpr float kPpt = (kTlRight - kTlLeft) / static_cast<float>(kTlTicks);
constexpr float kRowH = 22.0F;
constexpr float kRowGroundY = 324.0F;
constexpr float kRowInputY = 354.0F;
constexpr float kRowJumpY = 384.0F;

constexpr std::array<int, 5> kWindowMs{0, 50, 100, 150, 200};
constexpr float kChipH = 30.0F;
constexpr gd::Color kRowBg{243, 244, 246};
constexpr gd::Color kOrangeSoft{234, 88, 12, 45};
constexpr gd::Color kOrangeMid{234, 88, 12, 150};
constexpr gd::Color kTealMid{13, 148, 136, 150};

enum class Act { Coyote, Buffer, Bot, You, Consume, Slow, Restart };

struct Chip {
    Act act;
    int value;
    std::string label;
    SDL_FRect rect;
};

struct Label {
    float x;
    float y;
    std::string text;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    ib::Sim sim;
    std::vector<Chip> chips;
    std::vector<Label> labels;
    int coyoteIdx = 2;
    int bufferIdx = 2;
    bool slow = false;
    bool pressQueued = false;
    Uint64 last = 0;
    double accumulator = 0.0;
    long lastFall = -100000;
    long maxFrames = -1;
    long frames = 0;
};

void applyWindows(App& app) {
    app.sim.params.coyote = ib::ticksFromMs(kWindowMs[static_cast<std::size_t>(app.coyoteIdx)]);
    app.sim.params.buffer = ib::ticksFromMs(kWindowMs[static_cast<std::size_t>(app.bufferIdx)]);
}

void runTick(App& app, bool press) {
    if (app.sim.tickOnce(press)) app.lastFall = app.sim.tick;
}

bool windowActive(const App& app, Act act, int value) {
    const int ticks = ib::ticksFromMs(kWindowMs[static_cast<std::size_t>(value)]);
    return act == Act::Coyote ? app.sim.params.coyote == ticks : app.sim.params.buffer == ticks;
}

bool active(const App& app, const Chip& c) {
    switch (c.act) {
        case Act::Coyote:
        case Act::Buffer: return windowActive(app, c.act, c.value);
        case Act::Bot: return app.sim.botOn;
        case Act::You: return !app.sim.botOn;
        case Act::Consume: return app.sim.params.consume;
        case Act::Slow: return app.slow;
        case Act::Restart: return false;
    }
    return false;
}

void restart(App& app) {
    app.sim.restart();
    app.accumulator = 0.0;
    app.pressQueued = false;
    app.lastFall = -100000;
}

void setBot(App& app, bool on) {
    if (app.sim.botOn == on) return;
    app.sim.botOn = on;
    app.sim.bot.onRespawn();
    app.accumulator = 0.0;
    app.pressQueued = false;
}

void apply(App& app, const Chip& c) {
    switch (c.act) {
        case Act::Coyote: app.coyoteIdx = c.value; applyWindows(app); break;
        case Act::Buffer: app.bufferIdx = c.value; applyWindows(app); break;
        case Act::Bot: setBot(app, true); break;
        case Act::You: setBot(app, false); break;
        case Act::Consume: app.sim.params.consume = !app.sim.params.consume; break;
        case Act::Slow: app.slow = !app.slow; break;
        case Act::Restart: restart(app); break;
    }
}

void layout(App& app) {
    float x = 40.0F;
    float y = 466.0F;
    auto label = [&](const char* text) {
        app.labels.push_back({x, y + 4.0F, text});
        x += app.hud.width(text) + 12.0F;
    };
    auto chip = [&](Act act, int value, std::string text) {
        const float w = app.hud.width(text, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({act, value, std::move(text), SDL_FRect{x, y, w, kChipH}});
        x += w + 8.0F;
    };
    label("coyote ms (C)");
    for (int i = 0; i < 5; ++i) chip(Act::Coyote, i, std::to_string(kWindowMs[static_cast<std::size_t>(i)]));
    x += 24.0F;
    label("buffer ms (B)");
    for (int i = 0; i < 5; ++i) chip(Act::Buffer, i, std::to_string(kWindowMs[static_cast<std::size_t>(i)]));

    x = 40.0F;
    y = 504.0F;
    chip(Act::Bot, 0, "bot plays");
    chip(Act::You, 0, "you play");
    x += 16.0F;
    chip(Act::Consume, 0, "reset on jump: OFF (BUG)");  // widest text, the label changes with the state
    x += 16.0F;
    chip(Act::Slow, 0, "slow-mo (T)");
    chip(Act::Restart, 0, "restart (R)");
}

// ---- drawing ----

float screenX(const App& app, double worldX) { return kPlayerX + static_cast<float>(worldX - app.sim.world.x); }

void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

void cross(SDL_Renderer* r, float cx, float cy, float s, gd::Color c) {
    thickLine(r, cx - s, cy - s, cx + s, cy + s, c, 1);
    thickLine(r, cx - s, cy + s, cx + s, cy - s, c, 1);
}

std::string msText(int ticks) { return std::to_string(ib::msFromTicks(ticks)) + " ms"; }

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const ib::World& w = app.sim.world;
    const SDL_Rect clip{0, static_cast<int>(kSceneTop), kWidth, static_cast<int>(kSceneBottom - kSceneTop)};
    SDL_SetRenderClipRect(r, &clip);

    const long first = ib::cycleOf(w.x - static_cast<double>(kPlayerX)) - 1;
    const long lastCycle = ib::cycleOf(w.x + static_cast<double>(kWidth - kPlayerX)) + 1;
    for (long c = first; c <= lastCycle; ++c) {
        const double base = static_cast<double>(c) * ib::kPeriod;
        const std::array<std::pair<double, double>, 2> stones{{{0.0, ib::kLongLen}, {ib::kShortStart, ib::kShortLen}}};
        for (const auto& s : stones) {
            const float x = screenX(app, base + s.first);
            const auto len = static_cast<float>(s.second);
            gd::fillRect(r, x, kGroundY, len, kStoneH, gd::kInk);
            gd::fillRect(r, x, kGroundY, len, 4.0F, gd::kTealDark);
        }
    }

    // Player: a box with the feet at the world y. Its two bars show what the controller would accept right now.
    const float px = kPlayerX;
    const float feet = kGroundY + static_cast<float>(w.y);
    gd::fillRect(r, px - kPlayerW / 2.0F, feet - kPlayerH, kPlayerW, kPlayerH, gd::kTeal);
    gd::strokeRect(r, px - kPlayerW / 2.0F, feet - kPlayerH, kPlayerW, kPlayerH, gd::kInk);
    gd::fillRect(r, px + 2.0F, feet - kPlayerH + 5.0F, 5.0F, 5.0F, gd::kWhite);

    const ib::Params& p = app.sim.params;
    const int since = app.sim.ctrl.sinceGround();
    float barY = feet - kPlayerH - 14.0F;
    if (!w.onGround && since <= p.coyote) {
        const float frac = static_cast<float>(p.coyote - since + 1) / static_cast<float>(p.coyote + 1);
        gd::fillRect(r, px - 20.0F, barY, 40.0F, 6.0F, gd::kTealSoft);
        gd::fillRect(r, px - 20.0F, barY, 40.0F * frac, 6.0F, gd::kTeal);
        gd::strokeRect(r, px - 20.0F, barY, 40.0F, 6.0F, gd::kInk);
        app.hud.text(r, px + 26.0F, barY - 8.0F, "coyote", gd::kTealDark);
        barY -= 12.0F;
    }
    if (app.sim.ctrl.pending()) {
        const int left = std::max(0, p.buffer - app.sim.ctrl.pressAge());
        const float frac = static_cast<float>(left + 1) / static_cast<float>(p.buffer + 1);
        gd::fillRect(r, px - 20.0F, barY, 40.0F, 6.0F, kOrangeSoft);
        gd::fillRect(r, px - 20.0F, barY, 40.0F * frac, 6.0F, gd::kOrange);
        gd::strokeRect(r, px - 20.0F, barY, 40.0F, 6.0F, gd::kInk);
        app.hud.text(r, px + 26.0F, barY - 8.0F, "buffer", gd::kOrange);
    }
    SDL_SetRenderClipRect(r, nullptr);

    if (app.sim.tick - app.lastFall < 100) {
        const char* msg = "FELL";
        app.hud.text(r, 480.0F - app.hud.width(msg, gd::Hud::Style::Large) / 2.0F, 150.0F, msg, gd::kOrange, gd::Hud::Style::Large);
    }
}

struct Verdict {
    std::string head;
    std::string line1;
    std::string line2;
    gd::Color color;
};

Verdict describe(const App& app, const ib::PressRec* r) {
    const ib::Params& p = app.sim.params;
    if (r == nullptr) return {"NO INPUT YET", "wait for the first jump", "", gd::kMuted};
    if (r->state == ib::PressRec::State::Fired) {
        switch (r->kind) {
            case ib::Jump::Coyote:
                return {"COYOTE JUMP", msText(r->sinceGround + r->age) + " after leaving the ledge", "coyote time saved it", gd::kTealDark};
            case ib::Jump::Air:
                return {"DOUBLE JUMP (BUG)", "a jump in mid-air", "coyote timer was not reset", gd::kOrange};
            default:
                if (r->age > 0) return {"BUFFERED JUMP", "pressed " + msText(r->age) + " before landing", "the buffer saved it", gd::kTealDark};
                return {"JUMP", "pressed on the ground", "no help needed", gd::kTealDark};
        }
    }
    const ib::Miss m = ib::classifyMiss(*r);
    switch (m.kind) {
        case ib::Lost::Early: return {"INPUT LOST", "pressed " + msText(m.ticks) + " before landing", "buffer is only " + msText(p.buffer), gd::kOrange};
        case ib::Lost::Late: return {"INPUT LOST", msText(m.ticks) + " after leaving the ledge", "coyote is only " + msText(p.coyote), gd::kOrange};
        case ib::Lost::Extra: break;
    }
    return {"TAP IGNORED", "already in the air", "nothing to wait for", gd::kMuted};
}

void drawPanels(App& app) {
    SDL_Renderer* r = app.renderer;
    const ib::PressRec* last = app.sim.hist.lastResolved();
    const Verdict v = describe(app, last);
    gd::card(r, 24.0F, 44.0F, 340.0F, 98.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, 50.0F, "last input", gd::kMuted);
    app.hud.text(r, 38.0F, 70.0F, v.head, v.color, gd::Hud::Style::Bold);
    app.hud.text(r, 38.0F, 94.0F, v.line1, gd::kInk);
    app.hud.text(r, 38.0F, 114.0F, v.line2, gd::kMuted);

    const ib::Stats& s = app.sim.hist.stats();
    gd::card(r, 736.0F, 44.0F, 200.0F, 118.0F, gd::kWhite, gd::kInk);
    struct Row {
        const char* name;
        int value;
        gd::Color color;
    };
    const std::array<Row, 5> rows{{{"jumps", s.jumps, gd::kInk},
                                    {"saved", s.saved, gd::kTealDark},
                                    {"lost", s.lost, s.lost > 0 ? gd::kOrange : gd::kInk},
                                    {"falls", s.falls, s.falls > 0 ? gd::kOrange : gd::kInk},
                                    {"air jumps", s.airJumps, s.airJumps > 0 ? gd::kOrange : gd::kInk}}};
    float y = 50.0F;
    for (const Row& row : rows) {
        app.hud.text(r, 750.0F, y, row.name, gd::kMuted);
        const std::string num = std::to_string(row.value);
        app.hud.text(r, 922.0F - app.hud.width(num, gd::Hud::Style::Bold), y, num, row.color, gd::Hud::Style::Bold);
        y += 21.0F;
    }
}

float tickX(long now, long tick) { return kTlRight - static_cast<float>(now - tick) * kPpt; }

void drawTimeline(App& app) {
    SDL_Renderer* r = app.renderer;
    const ib::History& h = app.sim.hist;
    const long now = app.sim.tick - 1;
    const int buffer = app.sim.params.buffer;

    gd::card(r, 24.0F, kTlTop, 912.0F, kTlHeight, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, kTlTop + 5.0F, "last 3 s", gd::kMuted);
    const char* legend = "light teal: coyote window | orange: buffer window | dot: jump";
    app.hud.text(r, kTlRight - app.hud.width(legend) + 12.0F, kTlTop + 5.0F, legend, gd::kMuted);
    const std::array<std::pair<const char*, float>, 3> rowLabels{{{"ground", kRowGroundY}, {"input", kRowInputY}, {"jumps", kRowJumpY}}};
    for (const auto& rl : rowLabels) {
        gd::fillRect(r, kTlLeft, rl.second, kTlRight - kTlLeft, kRowH, kRowBg);
        app.hud.text(r, 38.0F, rl.second + 1.0F, rl.first, gd::kMuted);
    }
    // Half-second lines, "now" is the right edge.
    for (int i = 0; i <= 6; ++i) {
        const float x = kTlRight - static_cast<float>(i) * 0.5F * ib::kTickRate * kPpt;
        gd::fillRect(r, x, kRowGroundY - 4.0F, 1.0F, kRowJumpY + kRowH - kRowGroundY + 8.0F, gd::kGrid);
        const std::string label = i == 0 ? "now" : "-" + std::to_string(i / 2) + (i % 2 == 0 ? ".0 s" : ".5 s");
        app.hud.text(r, x - app.hud.width(label) / 2.0F, kRowJumpY + kRowH + 5.0F, label, gd::kMuted);
    }

    // Row 1: grounded (solid) and the coyote window after it (light).
    for (long t = std::max(0L, now - kTlTicks + 1); t <= now; ++t) {
        const ib::TickRec* rec = h.at(t);
        if (rec == nullptr || !rec->open) continue;
        const float x = tickX(now, t) - kPpt;
        if (rec->grounded) {
            gd::fillRect(r, x, kRowGroundY, kPpt + 0.6F, kRowH, gd::kTeal);
        } else {
            gd::fillRect(r, x, kRowGroundY, kPpt + 0.6F, kRowH, gd::kTealSoft);
            gd::fillRect(r, x, kRowGroundY, kPpt + 0.6F, 3.0F, gd::kTeal);
            gd::fillRect(r, x, kRowGroundY + kRowH - 3.0F, kPpt + 0.6F, 3.0F, gd::kTeal);
        }
    }

    // Row 2: each press with the buffer window behind it and what became of it.
    for (const ib::PressRec& p : h.presses()) {
        if (now - p.tick >= kTlTicks) continue;
        const float xp = tickX(now, p.tick) - kPpt;
        const float xWindow = std::min(kTlRight, xp + std::max(3.0F, static_cast<float>(buffer + 1) * kPpt));
        const bool pending = p.state == ib::PressRec::State::Pending;
        const float xEnd = pending ? kTlRight : tickX(now, p.endTick);
        gd::fillRect(r, xp, kRowInputY, xWindow - xp, kRowH, kOrangeSoft);
        gd::strokeRect(r, xp, kRowInputY, xWindow - xp, kRowH, kOrangeMid);
        const gd::Color used = p.state == ib::PressRec::State::Fired ? kTealMid : kOrangeMid;
        gd::fillRect(r, xp, kRowInputY + 8.0F, std::max(2.0F, xEnd - xp), kRowH - 16.0F, used);
        gd::fillRect(r, xp, kRowInputY - 3.0F, 2.0F, kRowH + 6.0F, gd::kInk);
        const float my = kRowInputY + kRowH / 2.0F;
        if (p.state == ib::PressRec::State::Fired) {
            gd::disc(r, xEnd - kPpt / 2.0F, my, 4.5F, gd::kTeal);
            gd::ring(r, xEnd - kPpt / 2.0F, my, 4.5F, gd::kInk);
        } else if (p.state == ib::PressRec::State::Lost) {
            const bool miss = ib::classifyMiss(p).kind != ib::Lost::Extra;
            cross(r, xEnd - kPpt / 2.0F, my, miss ? 5.0F : 3.0F, miss ? gd::kOrange : gd::kMuted);
        }
    }

    // Row 3: the jumps that fired, and the falls.
    const float jy = kRowJumpY + kRowH / 2.0F;
    for (const ib::PressRec& p : h.presses()) {
        if (p.state != ib::PressRec::State::Fired || now - p.endTick >= kTlTicks) continue;
        const float x = tickX(now, p.endTick) - kPpt / 2.0F;
        std::string text = "jump";
        if (p.kind == ib::Jump::Coyote) text = "coyote " + msText(p.sinceGround + p.age);
        else if (p.kind == ib::Jump::Air) text = "";
        else if (p.age > 0) text = "buffer " + msText(p.age);
        if (!text.empty() && x - 10.0F - app.hud.width(text) >= kTlLeft + 2.0F) {
            // Right-aligned to the left of the dot, the press that caused the jump lies there as well. Jumps are at
            // least 0.6 s apart, so neighbours do not collide. A label that would run out of the row is left out.
            app.hud.text(r, x - 10.0F - app.hud.width(text), kRowJumpY + 1.0F, text, p.kind == ib::Jump::Coyote || p.age > 0 ? gd::kTealDark : gd::kMuted);
        }
        const bool air = p.kind == ib::Jump::Air;
        gd::disc(r, x, jy, air ? 7.0F : 5.5F, air ? gd::kOrange : gd::kTeal);
        gd::ring(r, x, jy, air ? 7.0F : 5.5F, gd::kInk);
    }
    for (const long f : h.falls()) {
        if (now - f >= kTlTicks) continue;
        const float x = tickX(now, f) - kPpt / 2.0F;
        cross(r, x, jy, 5.0F, gd::kOrange);
        const float tx = x - 10.0F - app.hud.width("fell");
        if (tx >= kTlLeft + 2.0F) app.hud.text(r, tx, kRowJumpY + 1.0F, "fell", gd::kOrange);
    }
    gd::fillRect(r, kTlRight, kRowGroundY - 4.0F, 2.0F, kRowJumpY + kRowH - kRowGroundY + 8.0F, gd::kInk);
}

void drawFrame(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ COYOTE TIME + INPUT BUFFER ]", app.sim.botOn ? "bot: sloppy tester" : "you play: space / click");
    drawScene(app);
    drawPanels(app);
    drawTimeline(app);
    app.hud.text(r, 40.0F, 442.0F, "space/click jump | C coyote | B buffer | M bot/you | X reset | T slow | R restart", gd::kMuted);
    for (const Label& l : app.labels) app.hud.text(r, l.x, l.y, l.text, gd::kInk);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const bool bug = c.act == Act::Consume && !on;
        const std::string text = c.act == Act::Consume ? (on ? "reset on jump: ON" : "reset on jump: OFF (BUG)") : c.label;
        const float w = c.rect.w;
        gd::card(r, c.rect.x, c.rect.y, w, c.rect.h, on ? gd::kTeal : (bug ? gd::kOrange : gd::kWhite), gd::kInk);
        app.hud.text(r, c.rect.x + (w - app.hud.width(text, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, text,
                     on || bug ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("input-buffer", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    applyWindows(*app);
    layout(*app);
    for (long t = 0; t < kPreroll; ++t) runTick(*app, false);
    app->last = SDL_GetPerformanceCounter();
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
                    apply(app, c);
                    return SDL_APP_CONTINUE;
                }
            }
            if (p.y > kSceneTop && p.y < kSceneBottom) app.pressQueued = true;
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_SPACE:
                case SDLK_UP: app.pressQueued = true; break;
                case SDLK_C: app.coyoteIdx = (app.coyoteIdx + 1) % 5; applyWindows(app); break;
                case SDLK_B: app.bufferIdx = (app.bufferIdx + 1) % 5; applyWindows(app); break;
                case SDLK_M: setBot(app, !app.sim.botOn); break;
                case SDLK_X: app.sim.params.consume = !app.sim.params.consume; break;
                case SDLK_T: app.slow = !app.slow; break;
                case SDLK_R: restart(app); break;
                default: break;
            }
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.pressQueued = false; break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetPerformanceCounter();
    const double frame = std::min(static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency()), kMaxFrameTime);
    app.last = now;
    app.accumulator += frame * (app.slow ? 0.25 : 1.0);
    int steps = 0;
    while (app.accumulator >= ib::kDt && steps < kMaxTicksPerFrame) {
        runTick(app, app.pressQueued);
        app.pressQueued = false;
        app.accumulator -= ib::kDt;
        ++steps;
    }
    if (steps == kMaxTicksPerFrame) app.accumulator = 0.0;

    drawFrame(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
