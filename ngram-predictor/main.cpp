// N-gram predictor with SDL3 main callbacks.
//
// Rock paper scissors against an AI that only counts. It remembers which move followed which window of earlier
// moves and expects the most frequent one next, then plays the move that beats it. People are not random, so the
// hit rate climbs far above the 33 percent a coin flip would give within a few dozen rounds. Three simulated
// players with a habit (and a "you" mode to try to fool it) make that visible in seconds. The counting, the
// fallback to shorter windows and the hit rate live in logic.hpp and are covered by test.cpp.

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
constexpr double kRoundSeconds = 0.4;  // time between two rounds of a simulated player
constexpr double kSlip = 0.12;         // how often a simulated player plays a random move
constexpr int kWarmupRounds = 30;      // rounds played before the first frame, so the chart is not empty
constexpr std::size_t kTableRows = 6;
constexpr std::array<double, 3> kMistakeRates{0.0, 0.25, 0.5};

constexpr gd::Color kOrangeSoft{255, 228, 204};
constexpr gd::Color kInkSoft{17, 24, 39, 120};
constexpr gd::Color kRuler{214, 211, 209};
constexpr gd::Color kBarBest{94, 234, 212};

enum class Mode { You, Loop, Biased, Cycle };
enum class Action { Order, Mistake, Player, Reset };

struct Chip {
    Action action;
    int arg;
    std::string label;
    SDL_FRect rect;
};

struct Label {
    float x, y;
    std::string text;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<Label> labels;
    float describeX = 0.0F;
    ngp::Game game{7};
    ngp::Rng simRng{12345};
    ngp::SimPlayer sim{ngp::Habit::Loop, kSlip};
    Mode mode = Mode::Loop;
    int mistakeIndex = 0;
    std::optional<ngp::RoundResult> last;
    double accumulator = 0.0;
    Uint64 lastTicks = 0;
    SDL_FPoint mouse{-100.0F, -100.0F};
    long maxFrames = -1;
    long frames = 0;
};

const char* moveName(ngp::Move m) { return m == ngp::Rock ? "Rock" : (m == ngp::Paper ? "Paper" : "Scissors"); }

SDL_FRect moveButton(int i) { return SDL_FRect{24.0F + 96.0F * static_cast<float>(i), 262.0F, 92.0F, 88.0F}; }

// ---------------------------------------------------------------- game flow

void play(App& app, ngp::Move m) { app.last = app.game.play(m); }

void setMode(App& app, Mode mode) {
    if (mode != app.mode) app.game.resetStats();  // the hit rate belongs to one player
    app.mode = mode;
    app.accumulator = 0.0;
    ngp::Habit habit = ngp::Habit::Loop;
    if (mode == Mode::Biased) habit = ngp::Habit::Biased;
    if (mode == Mode::Cycle) habit = ngp::Habit::Cycle;
    app.sim = ngp::SimPlayer{habit, kSlip};
}

void setMistake(App& app, int index) {
    index = std::clamp(index, 0, static_cast<int>(kMistakeRates.size()) - 1);
    if (index != app.mistakeIndex) app.game.score = ngp::Score();  // who wins depends on the mistake rate
    app.mistakeIndex = index;
    app.game.mistake = kMistakeRates[static_cast<std::size_t>(app.mistakeIndex)];
}

void resetGame(App& app) {
    app.game.reset();
    app.last.reset();
    app.accumulator = 0.0;
    app.sim.step = 0;
}

void apply(App& app, Action action, int arg) {
    switch (action) {
        case Action::Order:
            arg = std::clamp(arg, 1, ngp::kMaxOrder);
            if (arg != app.game.order) app.game.resetStats();  // the hit rate belongs to one order
            app.game.order = arg;
            break;
        case Action::Mistake: setMistake(app, arg); break;
        case Action::Player: setMode(app, static_cast<Mode>(arg)); break;
        case Action::Reset: resetGame(app); break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Order: return app.game.order == c.arg;
        case Action::Mistake: return app.mistakeIndex == c.arg;
        case Action::Player: return static_cast<int>(app.mode) == c.arg;
        case Action::Reset: return false;
    }
    return false;
}

const char* describe(Mode mode) {
    switch (mode) {
        case Mode::You: return "R, P or S: try to fool it";
        case Mode::Loop: return "R R P S on repeat, 12% slips";
        case Mode::Biased: return "50% R, 30% P, 20% S, 12% slips";
        case Mode::Cycle: return "R P S in turn, 12% slips";
    }
    return "";
}

// ---------------------------------------------------------------- layout

void addChips(App& app, float y, const std::string& title, const std::vector<std::pair<std::string, std::pair<Action, int>>>& defs, float& x) {
    app.labels.push_back({x, y + 6.0F, title});
    x += app.hud.width(title) + 10.0F;
    for (const auto& d : defs) {
        const float w = app.hud.width(d.first, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.second.first, d.second.second, d.first, SDL_FRect{x, y, w, 30.0F}});
        x += w + 6.0F;
    }
}

void layoutChips(App& app) {
    float x = 24.0F;
    addChips(app, 452.0F, "Order", {{"1 N=1", {Action::Order, 1}}, {"2 N=2", {Action::Order, 2}}, {"3 N=3", {Action::Order, 3}}, {"4 N=4", {Action::Order, 4}}}, x);
    x += 18.0F;
    addChips(app, 452.0F, "AI mistakes", {{"5 0%", {Action::Mistake, 0}}, {"6 25%", {Action::Mistake, 1}}, {"7 50%", {Action::Mistake, 2}}}, x);
    x += 18.0F;
    addChips(app, 452.0F, "", {{"Del Reset", {Action::Reset, 0}}}, x);
    float x2 = 24.0F;
    addChips(app, 490.0F, "Player", {{"Y You", {Action::Player, 0}}, {"L Loop", {Action::Player, 1}}, {"B Biased", {Action::Player, 2}}, {"C Cycle", {Action::Player, 3}}}, x2);
    app.describeX = x2 + 18.0F;
}

// ---------------------------------------------------------------- drawing helpers

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

void drawRock(SDL_Renderer* r, float cx, float cy, float s) {
    constexpr std::array<float, 10> kRadius{1.0F, 0.86F, 0.97F, 0.90F, 1.0F, 0.88F, 0.96F, 0.85F, 0.98F, 0.90F};
    std::vector<SDL_Vertex> v;
    std::vector<int> idx;
    v.push_back({{cx - 6.0F * s, cy - 8.0F * s}, gd::fcolor(gd::Color{236, 238, 242}), {0, 0}});
    const int n = static_cast<int>(kRadius.size());
    std::vector<SDL_FPoint> pts;
    for (int i = 0; i < n; ++i) {
        const float a = 6.2831853F * (static_cast<float>(i) + 0.3F) / static_cast<float>(n);
        const float rad = 32.0F * s * kRadius[static_cast<std::size_t>(i)];
        pts.push_back({cx + std::cos(a) * rad, cy + 3.0F * s + std::sin(a) * rad * 0.9F});
        v.push_back({pts.back(), gd::fcolor(gd::Color{148, 155, 168}), {0, 0}});
        idx.insert(idx.end(), {0, 1 + i, 1 + (i + 1) % n});
    }
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
    const int half = s > 0.8F ? 1 : 0;
    for (int i = 0; i < n; ++i) {
        const SDL_FPoint a = pts[static_cast<std::size_t>(i)];
        const SDL_FPoint b = pts[static_cast<std::size_t>((i + 1) % n)];
        thickLine(r, a.x, a.y, b.x, b.y, gd::kInk, half);
    }
    // cracks
    thickLine(r, cx - 4.0F * s, cy - 14.0F * s, cx + 4.0F * s, cy - 2.0F * s, kInkSoft, 0);
    thickLine(r, cx + 4.0F * s, cy - 2.0F * s, cx - 2.0F * s, cy + 10.0F * s, kInkSoft, 0);
    thickLine(r, cx + 4.0F * s, cy - 2.0F * s, cx + 16.0F * s, cy + 2.0F * s, kInkSoft, 0);
}

void drawPaper(SDL_Renderer* r, float cx, float cy, float s) {
    const float w = 50.0F * s;
    const float h = 64.0F * s;
    const float x = cx - w / 2.0F;
    const float y = cy - h / 2.0F;
    const float fold = 15.0F * s;
    gd::fillRect(r, x, y + fold, w, h - fold, gd::kWhite);
    gd::fillRect(r, x, y, w - fold, fold, gd::kWhite);
    // folded corner
    const SDL_FColor c = gd::fcolor(gd::Color{226, 232, 240});
    const SDL_Vertex tri[3] = {{{x + w - fold, y}, c, {0, 0}}, {{x + w, y + fold}, c, {0, 0}}, {{x + w - fold, y + fold}, c, {0, 0}}};
    SDL_RenderGeometry(r, nullptr, tri, 3, nullptr, 0);
    const int half = s > 0.8F ? 1 : 0;
    thickLine(r, x, y, x + w - fold, y, gd::kInk, half);
    thickLine(r, x + w - fold, y, x + w, y + fold, gd::kInk, half);
    thickLine(r, x + w, y + fold, x + w, y + h, gd::kInk, half);
    thickLine(r, x + w, y + h, x, y + h, gd::kInk, half);
    thickLine(r, x, y + h, x, y, gd::kInk, half);
    thickLine(r, x + w - fold, y, x + w - fold, y + fold, gd::kInk, 0);
    thickLine(r, x + w - fold, y + fold, x + w, y + fold, gd::kInk, 0);
    for (int i = 0; i < 4; ++i) {
        const float ly = y + (24.0F + 9.0F * static_cast<float>(i)) * s;
        thickLine(r, x + 8.0F * s, ly, x + w - 8.0F * s - (i == 3 ? 14.0F * s : 0.0F), ly, kInkSoft, 0);
    }
}

void drawScissors(SDL_Renderer* r, float cx, float cy, float s) {
    const int outer = s > 0.8F ? 2 : 1;
    const int inner = s > 0.8F ? 1 : 0;
    // two blades cross at the pivot, the handles are the rings at the lower ends
    for (int side = -1; side <= 1; side += 2) {
        const auto f = static_cast<float>(side);
        const float tipX = cx + f * 17.0F * s;
        const float tipY = cy - 40.0F * s;
        const float ringX = cx - f * 19.0F * s;
        const float ringY = cy + 36.0F * s;
        thickLine(r, tipX, tipY, ringX, ringY, gd::kInk, outer);
        thickLine(r, tipX, tipY, ringX, ringY, gd::Color{203, 213, 225}, inner);
    }
    for (int side = -1; side <= 1; side += 2) {
        const auto f = static_cast<float>(side);
        const float ringX = cx + f * 19.0F * s;
        const float ringY = cy + 38.0F * s;
        gd::ring(r, ringX, ringY, 9.0F * s, gd::kOrange);
        gd::ring(r, ringX, ringY, 8.0F * s, gd::kOrange);
        gd::ring(r, ringX, ringY, 10.0F * s, gd::kInk);
    }
    gd::disc(r, cx, cy - 3.0F * s, 4.0F * s, gd::kInk);
}

void drawMove(SDL_Renderer* r, ngp::Move m, float cx, float cy, float s) {
    switch (m) {
        case ngp::Rock: drawRock(r, cx, cy, s); break;
        case ngp::Paper: drawPaper(r, cx, cy, s); break;
        case ngp::Scissors: drawScissors(r, cx, cy, s); break;
    }
}

std::string percent(double p) {
    char buf[16];
    SDL_snprintf(buf, sizeof buf, "%d%%", static_cast<int>(std::lround(p * 100.0)));
    return buf;
}

void centeredText(App& app, float cx, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, std::round(cx - app.hud.width(s, style) / 2.0F), y, s, c, style);
}

// ---------------------------------------------------------------- panels

void drawDuel(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 24.0F, 44.0F, 288.0F, 208.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 44.0F, 48.0F, "YOU", gd::kOrange, gd::Hud::Style::Bold);
    app.hud.text(r, 168.0F, 48.0F, "AI", gd::kTeal, gd::Hud::Style::Bold);
    const bool playerWon = app.last && app.last->outcome > 0;
    const bool aiWon = app.last && app.last->outcome < 0;
    gd::card(r, 44.0F, 74.0F, 100.0F, 96.0F, playerWon ? kOrangeSoft : gd::kWhite, gd::kInk);
    gd::card(r, 168.0F, 74.0F, 100.0F, 96.0F, aiWon ? gd::kTealSoft : gd::kWhite, gd::kInk);
    app.hud.text(r, 148.0F, 110.0F, "vs", gd::kMuted);
    if (!app.last) {
        centeredText(app, 94.0F, 100.0F, "?", gd::kMuted, gd::Hud::Style::Large);
        centeredText(app, 218.0F, 100.0F, "?", gd::kMuted, gd::Hud::Style::Large);
        centeredText(app, 168.0F, 182.0F, "no round yet", gd::kMuted);
        return;
    }
    drawMove(r, app.last->player, 94.0F, 122.0F, 1.0F);
    drawMove(r, app.last->ai, 218.0F, 122.0F, 1.0F);
    if (playerWon) {
        centeredText(app, 168.0F, 174.0F, "YOU WIN", gd::kOrange, gd::Hud::Style::Large);
    } else if (aiWon) {
        centeredText(app, 168.0F, 174.0F, "AI WINS", gd::kTeal, gd::Hud::Style::Large);
    } else {
        centeredText(app, 168.0F, 174.0F, "DRAW", gd::kMuted, gd::Hud::Style::Large);
    }
    const ngp::Prediction& p = app.last->prediction;
    std::string line = std::string("AI expected ") + moveName(p.move) + " " + percent(p.prob[static_cast<std::size_t>(p.move)]);
    if (app.last->slipped) line = std::string("expected ") + moveName(p.move) + ", slipped";
    centeredText(app, 168.0F, 224.0F, line, app.last->hit ? gd::kTeal : gd::kMuted, app.last->hit ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
}

void drawMoveButtons(App& app) {
    SDL_Renderer* r = app.renderer;
    for (int i = 0; i < ngp::kMoves; ++i) {
        const auto m = static_cast<ngp::Move>(i);
        const SDL_FRect b = moveButton(i);
        const bool hover = SDL_PointInRectFloat(&app.mouse, &b);
        const bool lastPlayed = app.last && app.last->player == m;
        gd::card(r, b.x, b.y, b.w, b.h, hover ? gd::kTealSoft : (lastPlayed ? kOrangeSoft : gd::kWhite), gd::kInk);
        const char key[2] = {ngp::letter(m), '\0'};
        app.hud.text(r, b.x + 6.0F, b.y + 3.0F, key, gd::kMuted, gd::Hud::Style::Bold);
        drawMove(r, m, b.x + b.w / 2.0F, b.y + 33.0F, 0.55F);
        centeredText(app, b.x + b.w / 2.0F, b.y + 62.0F, moveName(m), gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawScore(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 24.0F, 358.0F, 288.0F, 84.0F, gd::kWhite, gd::kInk);
    const ngp::Tracker& t = app.game.tracker;
    char line[64];
    SDL_snprintf(line, sizeof line, "AI predicted %lu of %lu", static_cast<unsigned long>(t.hits()), static_cast<unsigned long>(t.rounds()));
    app.hud.text(r, 38.0F, 364.0F, line, gd::kTeal, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "overall %s, chance 33%%", percent(t.cumulative()).c_str());
    app.hud.text(r, 38.0F, 386.0F, line, gd::kInk);
    const ngp::Score& s = app.game.score;
    SDL_snprintf(line, sizeof line, "You %lu · Draw %lu · AI %lu", static_cast<unsigned long>(s.player), static_cast<unsigned long>(s.draws), static_cast<unsigned long>(s.ai));
    app.hud.text(r, 38.0F, 408.0F, line, gd::kMuted);
}

void drawChart(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 332.0F, 44.0F, 604.0F, 208.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 346.0F, 48.0F, "AI hit rate, last 20 rounds", gd::kInk, gd::Hud::Style::Bold);

    constexpr float px = 388.0F;
    constexpr float py = 100.0F;
    constexpr float pw = 520.0F;
    constexpr float ph = 126.0F;
    auto yOf = [&](double v) { return py + ph * static_cast<float>(1.0 - v); };

    const std::deque<double>& curve = app.game.tracker.curve();
    if (!curve.empty()) {
        const std::string now = percent(curve.back());
        app.hud.text(r, 922.0F - app.hud.width(now, gd::Hud::Style::Large), 46.0F, now, gd::kTeal, gd::Hud::Style::Large);
    }

    for (const double v : {0.0, 0.5, 1.0}) {
        gd::fillRect(r, px, yOf(v), pw, 1.0F, kRuler);
        const std::string label = percent(v);
        app.hud.text(r, px - 8.0F - app.hud.width(label), yOf(v) - 11.0F, label, gd::kMuted);
    }
    // the coin-flip line
    const float chanceY = yOf(1.0 / 3.0);
    for (float x = px; x < px + pw; x += 12.0F) gd::fillRect(r, x, chanceY, 6.0F, 2.0F, gd::kOrange);
    app.hud.text(r, px + 6.0F, chanceY + 3.0F, "chance 33%", gd::kOrange);

    if (curve.empty()) {
        centeredText(app, px + pw / 2.0F, py + 50.0F, "play a round", gd::kMuted);
    } else {
        const float step = pw / static_cast<float>(ngp::Tracker::kCurve - 1);
        std::vector<SDL_FPoint> pts;
        for (std::size_t i = 0; i < curve.size(); ++i) pts.push_back({px + step * static_cast<float>(i), yOf(curve[i])});
        const SDL_FColor top = gd::fcolor(gd::kTeal, 0.28F);
        const SDL_FColor bottom = gd::fcolor(gd::kTeal, 0.04F);
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
            const SDL_Vertex q[4] = {{pts[i], top, {0, 0}}, {pts[i + 1], top, {0, 0}}, {{pts[i + 1].x, py + ph}, bottom, {0, 0}}, {{pts[i].x, py + ph}, bottom, {0, 0}}};
            const int idx[6] = {0, 1, 2, 0, 2, 3};
            SDL_RenderGeometry(r, nullptr, q, 4, idx, 6);
        }
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) thickLine(r, pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y, gd::kTeal, 1);
        gd::disc(r, pts.back().x, pts.back().y, 5.0F, gd::kTeal);
        gd::ring(r, pts.back().x, pts.back().y, 5.0F, gd::kInk);
    }
    app.hud.text(r, px, 230.0F, "older", gd::kMuted);
    app.hud.text(r, px + pw - app.hud.width("now"), 230.0F, "now", gd::kMuted);
}

void drawTable(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 332.0F, 258.0F, 604.0F, 184.0F, gd::kWhite, gd::kInk);
    const int order = app.game.order;
    char title[96];
    if (order == 1) {
        SDL_snprintf(title, sizeof title, "AI memory, N=1 (no window)");
    } else {
        SDL_snprintf(title, sizeof title, "AI memory, N=%d (window: last %d)", order, order - 1);
    }
    app.hud.text(r, 346.0F, 262.0F, title, gd::kInk, gd::Hud::Style::Bold);

    const ngp::Prediction next = app.game.model.predict(order, 0);
    if (next.usedOrder < order) {
        char note[48];
        SDL_snprintf(note, sizeof note, "window unseen: using N=%d", next.usedOrder);
        app.hud.text(r, 922.0F - app.hud.width(note), 262.0F, note, gd::kOrange);
    }

    constexpr std::array<float, 3> cellX{440.0F, 540.0F, 640.0F};
    constexpr float cellW = 92.0F;
    app.hud.text(r, 346.0F, 286.0F, "window", gd::kMuted);
    for (int k = 0; k < ngp::kMoves; ++k) {
        const std::string h = std::string("then ") + ngp::letter(static_cast<ngp::Move>(k));
        app.hud.text(r, cellX[static_cast<std::size_t>(k)], 286.0F, h, gd::kMuted);
    }
    app.hud.text(r, 748.0F, 286.0F, "AI picks", gd::kMuted);

    const std::vector<ngp::Model::Row> rows = app.game.model.rows(order, kTableRows);
    const std::size_t all = app.game.model.rows(order, 1000).size();
    if (all > rows.size()) {
        char more[24];
        SDL_snprintf(more, sizeof more, "+%lu more", static_cast<unsigned long>(all - rows.size()));
        app.hud.text(r, 922.0F - app.hud.width(more), 286.0F, more, gd::kMuted);
    }
    if (rows.empty()) app.hud.text(r, 346.0F, 312.0F, "nothing learned yet", gd::kMuted);

    float y = 310.0F;
    for (const ngp::Model::Row& row : rows) {
        if (row.current) gd::fillRect(r, 338.0F, y, 592.0F, 22.0F, gd::kTealSoft);
        const std::string window = order == 1 ? "(none)" : ngp::windowLetters(order, row.window);
        app.hud.text(r, 346.0F, y, window, gd::kInk, gd::Hud::Style::Bold);
        if (row.current && order > 1) app.hud.text(r, 346.0F + app.hud.width(window, gd::Hud::Style::Bold) + 10.0F, y, "now", gd::kOrange, gd::Hud::Style::Bold);

        const std::uint32_t best = std::max({row.counts[0], row.counts[1], row.counts[2]});
        int bestCount = 0;
        int bestIndex = 0;
        for (int k = 0; k < ngp::kMoves; ++k) {
            const std::uint32_t c = row.counts[static_cast<std::size_t>(k)];
            if (c == best) {
                ++bestCount;
                bestIndex = k;
            }
        }
        for (int k = 0; k < ngp::kMoves; ++k) {
            const std::uint32_t c = row.counts[static_cast<std::size_t>(k)];
            const double prob = static_cast<double>(c + 1) / static_cast<double>(row.total + ngp::kMoves);
            const float x = cellX[static_cast<std::size_t>(k)];
            gd::fillRect(r, x, y + 2.0F, cellW, 18.0F, gd::kWhite);
            gd::fillRect(r, x, y + 2.0F, cellW * static_cast<float>(prob), 18.0F, (c == best && row.total > 0) ? kBarBest : kRuler);
            gd::strokeRect(r, x, y + 2.0F, cellW, 18.0F, kRuler);
            char num[16];
            SDL_snprintf(num, sizeof num, "%u", static_cast<unsigned>(c));
            app.hud.text(r, x + 6.0F, y, num, gd::kInk, gd::Hud::Style::Bold);
            const std::string pc = percent(prob);
            app.hud.text(r, x + cellW - 4.0F - app.hud.width(pc), y, pc, gd::kMuted);
        }
        if (row.total == 0) {
            app.hud.text(r, 748.0F, y, "unseen", gd::kOrange);
        } else if (bestCount == 1) {
            app.hud.text(r, 748.0F, y, moveName(static_cast<ngp::Move>(bestIndex)), gd::kTeal, gd::Hud::Style::Bold);
        } else {
            app.hud.text(r, 748.0F, y, "tie", gd::kMuted);
        }
        y += 22.0F;
    }
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Label& l : app.labels) app.hud.text(r, l.x, l.y, l.text, gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const bool hover = SDL_PointInRectFloat(&app.mouse, &c.rect);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : (hover ? gd::kTealSoft : gd::kWhite), gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, app.describeX, 494.0F, describe(app.mode), gd::kMuted);
}

void drawScene(App& app) {
    gd::backdrop(app.renderer, kWidth, kHeight);
    char right[48];
    SDL_snprintf(right, sizeof right, "round %lu · N=%d", static_cast<unsigned long>(app.game.tracker.rounds()), app.game.order);
    app.hud.titleBar(app.renderer, kWidth, "[ N-GRAM PREDICTOR ]", right);
    drawDuel(app);
    drawMoveButtons(app);
    drawScore(app);
    drawChart(app);
    drawTable(app);
    drawChips(app);
}

// ---------------------------------------------------------------- input

void clickAt(App& app, SDL_FPoint p) {
    for (const Chip& c : app.chips) {
        if (SDL_PointInRectFloat(&p, &c.rect)) {
            apply(app, c.action, c.arg);
            return;
        }
    }
    for (int i = 0; i < ngp::kMoves; ++i) {
        const SDL_FRect b = moveButton(i);
        if (SDL_PointInRectFloat(&p, &b)) {
            if (app.mode != Mode::You) setMode(app, Mode::You);
            play(app, static_cast<ngp::Move>(i));
            return;
        }
    }
}

void keyDown(App& app, SDL_Keycode key) {
    auto human = [&](ngp::Move m) {
        if (app.mode != Mode::You) setMode(app, Mode::You);
        play(app, m);
    };
    switch (key) {
        case SDLK_R: human(ngp::Rock); break;
        case SDLK_P: human(ngp::Paper); break;
        case SDLK_S: human(ngp::Scissors); break;
        case SDLK_1: apply(app, Action::Order, 1); break;
        case SDLK_2: apply(app, Action::Order, 2); break;
        case SDLK_3: apply(app, Action::Order, 3); break;
        case SDLK_4: apply(app, Action::Order, 4); break;
        case SDLK_5: apply(app, Action::Mistake, 0); break;
        case SDLK_6: apply(app, Action::Mistake, 1); break;
        case SDLK_7: apply(app, Action::Mistake, 2); break;
        case SDLK_Y: apply(app, Action::Player, 0); break;
        case SDLK_L: apply(app, Action::Player, 1); break;
        case SDLK_B: apply(app, Action::Player, 2); break;
        case SDLK_C: apply(app, Action::Player, 3); break;
        case SDLK_DELETE:
        case SDLK_BACKSPACE: apply(app, Action::Reset, 0); break;
        default: break;
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
    if (!SDL_CreateWindowAndRenderer("ngram-predictor", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    setMode(*app, Mode::Loop);
    setMistake(*app, 0);
    // The first frame already shows a learned habit instead of an empty chart.
    for (int i = 0; i < kWarmupRounds; ++i) play(*app, app->sim.next(app->simRng));
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: app.mouse = {event->motion.x, event->motion.y}; break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            app.mouse = {event->button.x, event->button.y};
            if (event->button.button == SDL_BUTTON_LEFT) clickAt(app, app.mouse);
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE: app.mouse = {-100.0F, -100.0F}; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            keyDown(app, event->key.key);
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(static_cast<double>(now - app.lastTicks) * 1e-9, 0.25);
    app.lastTicks = now;
    if (app.mode != Mode::You) {
        app.accumulator += dt;
        if (app.accumulator >= kRoundSeconds) {
            // one round per frame at most, the rest of a long frame is dropped instead of played as a burst
            app.accumulator = std::fmod(app.accumulator, kRoundSeconds);
            play(app, app.sim.next(app.simRng));
        }
    }
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
