// FSM vs behavior tree with SDL3 main callbacks.
//
// Two guards walk the same corridor and read the same senses, one is decided by a state machine (left diagram), one by a
// behavior tree (right diagram, nodes lit by what they returned this tick). The mouse is the player, a click makes noise.
// The three chips on the left add requirements: the state machine needs edges from every state, the tree gets one branch
// at the top, and the counters show the difference. The selector chip makes the tree passive (a running branch keeps
// control, the guard ignores you) and the history chip lets the state machine return to where a radio call interrupted
// it. All decisions live in logic.hpp and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

using gbt::State;

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr double kStep = 1.0 / 60.0;
constexpr float kArenaX = 20.0F;  // canvas x of corridor position 0
constexpr float kCardX = 10.0F;
constexpr float kCardY = 276.0F;
constexpr float kCardW = 940.0F;
constexpr float kCardH = 124.0F;
constexpr std::array<float, 2> kLaneY{282.0F, 340.0F};
constexpr float kLaneH = 54.0F;
constexpr float kChipH = 32.0F;
constexpr std::array<float, 2> kChipY{430.0F, 470.0F};

constexpr gd::Color kGreen{34, 197, 94};
constexpr gd::Color kRed{220, 38, 38};
constexpr gd::Color kYellow{250, 204, 21};
constexpr gd::Color kBlue{37, 99, 235};
constexpr gd::Color kPurple{147, 51, 234};

enum class Action { Noise, LowHealth, Radio, Selector, History, Hurt, Ring, Reset };

struct Chip {
    Action action;
    int row;
    SDL_FRect rect;
};

struct NoiseRing {
    double x;
    double age;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    gbt::Features features;
    bool history = false;
    bool active = true;
    gbt::FsmBrain fsm;
    gbt::BtBrain bt;
    double playerX = 560.0;
    std::vector<NoiseRing> rings;
    int lastFired = -1;
    double firedAge = 99.0;
    bool hasLastEdge = false;
    gbt::Edge lastEdge{State::Patrol, State::Patrol, gbt::When::Sees, gbt::Group::Base};
    std::string note;
    double accumulator = 0.0;
    Uint64 lastNs = 0;
    long maxFrames = -1;
    long frames = 0;
};

// ---- state changes -------------------------------------------------------------------------------------------------

void reconfigure(App& app) {
    app.fsm.configure(app.features, app.history);
    app.bt.configure(app.features, app.active);
    for (gbt::Body* b : {&app.fsm.body, &app.bt.body}) {
        b->hasNoise = false;  // whatever was pending belonged to the old rules
        b->radioRinging = false;
    }
    app.lastFired = -1;
    app.hasLastEdge = false;
}

void setFeatures(App& app, const gbt::Features& next, const char* what) {
    const gbt::Cost c = gbt::changeCost(app.features, next);
    const bool added = c.btNodes > 0;
    app.features = next;
    reconfigure(app);
    char buf[160];
    SDL_snprintf(buf, sizeof buf, "%s %s:  FSM %s%d edges, %d states edited   BT %s%d nodes, %d place", added ? "+" : "-", what, c.fsmEdges < 0 ? "-" : "+",
                 std::abs(c.fsmEdges), c.fsmStatesEdited, c.btNodes < 0 ? "-" : "+", std::abs(c.btNodes), c.btPlaces);
    app.note = buf;
}

void resetGuards(App& app) {
    app.fsm = gbt::FsmBrain();
    app.bt = gbt::BtBrain();
    reconfigure(app);
    app.rings.clear();
    app.accumulator = 0.0;
    app.note.clear();
}

void makeNoiseAt(App& app, double x) {
    gbt::makeNoise(app.fsm.body, x);
    gbt::makeNoise(app.bt.body, x);
    app.rings.push_back({x, 0.0});
    if (app.rings.size() > 6) app.rings.erase(app.rings.begin());
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Noise:
            setFeatures(app, {!app.features.noise, app.features.lowHealth, app.features.radio}, "Noise");
            break;
        case Action::LowHealth:
            setFeatures(app, {app.features.noise, !app.features.lowHealth, app.features.radio}, "Low health");
            break;
        case Action::Radio:
            setFeatures(app, {app.features.noise, app.features.lowHealth, !app.features.radio}, "Radio call");
            break;
        case Action::Selector:
            app.active = !app.active;
            reconfigure(app);
            app.note = app.active ? "active selector: asks from the top every tick" : "passive selector: a running branch keeps control until it is done";
            break;
        case Action::History:
            app.history = !app.history;
            reconfigure(app);
            app.note = app.history ? "history on: after the call the FSM goes back to where it was" : "history off: after the call the FSM only knows the way to Patrol";
            break;
        case Action::Hurt:
            for (gbt::Body* b : {&app.fsm.body, &app.bt.body}) b->hp = std::max(0.0, b->hp - 35.0);
            app.note = app.features.lowHealth ? "guards hurt: below 30 hp they head for the medkit" : "guards hurt, but nobody handles it yet (add 2)";
            break;
        case Action::Ring:
            for (gbt::Body* b : {&app.fsm.body, &app.bt.body}) b->radioRinging = true;
            app.note = app.features.radio ? "radio rings" : "radio rings, nobody listens yet (add 3)";
            break;
        case Action::Reset: resetGuards(app); break;
    }
}

bool isOn(const App& app, Action action) {
    switch (action) {
        case Action::Noise: return app.features.noise;
        case Action::LowHealth: return app.features.lowHealth;
        case Action::Radio: return app.features.radio;
        case Action::Selector: return app.active;
        case Action::History: return app.history;
        default: return false;
    }
}

const char* chipLabel(const App& app, Action action) {
    switch (action) {
        case Action::Noise: return "1 + Noise";
        case Action::LowHealth: return "2 + Low health";
        case Action::Radio: return "3 + Radio call";
        case Action::Selector: return app.active ? "4 Selector: active" : "4 Selector: passive";
        case Action::History: return app.history ? "5 History: on" : "5 History: off";
        case Action::Hurt: return "H Hurt guards";
        case Action::Ring: return "P Ring radio";
        case Action::Reset: return "R Reset";
    }
    return "";
}

float chipWidth(const App& app, Action action) {
    // the widest label of a chip decides, so the row does not jump when it toggles
    const char* wide = action == Action::Selector ? "4 Selector: passive" : action == Action::History ? "5 History: off" : chipLabel(app, action);
    return app.hud.width(wide, gd::Hud::Style::Bold) + 24.0F;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        int row;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Action::Noise, 0, 0.0F},
                                   {Action::LowHealth, 0, 8.0F},
                                   {Action::Radio, 0, 8.0F},
                                   {Action::Selector, 0, 24.0F},
                                   {Action::History, 0, 8.0F},
                                   {Action::Hurt, 1, 0.0F},
                                   {Action::Ring, 1, 8.0F},
                                   {Action::Reset, 1, 8.0F}}};
    float x = 0.0F;
    int row = -1;
    for (const Def& d : defs) {
        if (d.row != row) {
            row = d.row;
            x = 24.0F;
        }
        x += d.gapBefore;
        const float w = chipWidth(app, d.action);
        app.chips.push_back({d.action, d.row, SDL_FRect{x, kChipY[static_cast<std::size_t>(d.row)], w, kChipH}});
        x += w;
    }
}

// ---- simulation ----------------------------------------------------------------------------------------------------

void step(App& app, double dt) {
    app.fsm.tick(app.playerX, dt);
    app.bt.tick(app.playerX, dt);
    if (app.fsm.fired >= 0) {
        app.lastFired = app.fsm.fired;
        app.firedAge = 0.0;
        app.lastEdge = app.fsm.edges[static_cast<std::size_t>(app.fsm.fired)];
        app.hasLastEdge = true;
    } else {
        app.firedAge += dt;
    }
    for (NoiseRing& ring : app.rings) ring.age += dt;
    std::erase_if(app.rings, [](const NoiseRing& ring) { return ring.age > 1.2; });
}

// ---- drawing -------------------------------------------------------------------------------------------------------

gd::Color withAlpha(gd::Color c, int a) {
    c.a = static_cast<Uint8>(a);
    return c;
}

void thickLine(SDL_Renderer* r, gbt::Pt a, gbt::Pt b, float width, gd::Color c) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len < 1e-6) return;
    const auto nx = static_cast<float>(-dy / len * width * 0.5);
    const auto ny = static_cast<float>(dx / len * width * 0.5);
    const SDL_FColor col = gd::fcolor(c);
    const SDL_Vertex v[4] = {{{static_cast<float>(a.x) + nx, static_cast<float>(a.y) + ny}, col, {0, 0}},
                             {{static_cast<float>(a.x) - nx, static_cast<float>(a.y) - ny}, col, {0, 0}},
                             {{static_cast<float>(b.x) - nx, static_cast<float>(b.y) - ny}, col, {0, 0}},
                             {{static_cast<float>(b.x) + nx, static_cast<float>(b.y) + ny}, col, {0, 0}}};
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
}

void arrow(SDL_Renderer* r, gbt::Pt from, gbt::Pt to, float width, gd::Color c) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double len = std::hypot(dx, dy);
    if (len < 1.0) return;
    const gbt::Pt d{dx / len, dy / len};
    const double head = width > 2.0F ? 11.0 : 8.0;
    const gbt::Pt base{to.x - d.x * head, to.y - d.y * head};
    thickLine(r, from, base, width, c);
    const double half = head * 0.45;
    const SDL_FColor col = gd::fcolor(c);
    const SDL_Vertex v[3] = {{{static_cast<float>(to.x), static_cast<float>(to.y)}, col, {0, 0}},
                             {{static_cast<float>(base.x - d.y * half), static_cast<float>(base.y + d.x * half)}, col, {0, 0}},
                             {{static_cast<float>(base.x + d.y * half), static_cast<float>(base.y - d.x * half)}, col, {0, 0}}};
    SDL_RenderGeometry(r, nullptr, v, 3, nullptr, 0);
}

gd::Color stateColor(State s) {
    switch (s) {
        case State::Patrol: return gd::Color{209, 213, 219};
        case State::Chase: return gd::kOrange;
        case State::Search: return kYellow;
        case State::Attack: return kRed;
        case State::Inspect: return gd::Color{96, 165, 250};
        case State::Heal: return gd::Color{74, 222, 128};
        case State::Radio: return gd::Color{192, 132, 252};
    }
    return gd::kMuted;
}

gd::Color groupColor(gbt::Group g) {
    switch (g) {
        case gbt::Group::Base: return withAlpha(gd::kInk, 150);
        case gbt::Group::Noise: return withAlpha(kBlue, 170);
        case gbt::Group::Heal: return withAlpha(kGreen, 170);
        case gbt::Group::Radio: return withAlpha(kPurple, 170);
    }
    return gd::kInk;
}

// Diagram layout of the state machine, canvas coordinates.
constexpr std::array<gbt::Pt, 7> kNodePos{{{80, 86}, {250, 86}, {165, 146}, {420, 86}, {335, 146}, {80, 206}, {420, 206}}};  // by State
constexpr gbt::Pt kHistoryPos{335, 206};
constexpr double kBoxHalfW = 38.0;
constexpr double kBoxHalfH = 13.0;
constexpr double kHistoryR = 13.0;

gbt::Pt nodePos(State s) { return kNodePos[static_cast<std::size_t>(s)]; }

void drawEdge(App& app, std::size_t index, bool lit) {
    const gbt::Edge& e = app.fsm.edges[index];
    const bool viaMarker = e.viaHistory && app.features.radio && app.history;
    const gbt::Pt a = nodePos(e.from);
    const gbt::Pt b = viaMarker ? kHistoryPos : nodePos(e.to);
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len < 1.0) return;
    const gbt::Pt d{dx / len, dy / len};
    const gbt::Pt off{-d.y * 5.0, d.x * 5.0};  // opposite directions end up on opposite sides
    const gbt::Pt start = gbt::boxEdgePoint(a, d, kBoxHalfW, kBoxHalfH, off);
    const gbt::Pt end = viaMarker ? gbt::Pt{b.x - d.x * kHistoryR + off.x, b.y - d.y * kHistoryR + off.y}
                                  : gbt::boxEdgePoint(b, {-d.x, -d.y}, kBoxHalfW, kBoxHalfH, off);
    arrow(app.renderer, start, end, lit ? 3.0F : 1.6F, lit ? gd::kOrange : groupColor(e.group));
}

void drawFsm(App& app) {
    SDL_Renderer* r = app.renderer;
    const gbt::Features& f = app.features;
    app.hud.text(r, 14.0F, 36.0F, "STATE MACHINE", gd::kInk, gd::Hud::Style::Bold);
    char counts[48];
    SDL_snprintf(counts, sizeof counts, "%d states, %d edges", gbt::stateCount(f), static_cast<int>(app.fsm.edges.size()));
    app.hud.text(r, 470.0F - app.hud.width(counts), 36.0F, counts, gd::kMuted);

    const bool litNow = app.lastFired >= 0 && app.firedAge < 0.6;
    for (std::size_t i = 0; i < app.fsm.edges.size(); ++i) {
        if (!litNow || static_cast<int>(i) != app.lastFired) drawEdge(app, i, false);
    }
    if (litNow) drawEdge(app, static_cast<std::size_t>(app.lastFired), true);

    const bool marker = f.radio && app.history;
    if (marker) {
        gd::disc(r, static_cast<float>(kHistoryPos.x), static_cast<float>(kHistoryPos.y), static_cast<float>(kHistoryR), gd::kWhite);
        gd::ring(r, static_cast<float>(kHistoryPos.x), static_cast<float>(kHistoryPos.y), static_cast<float>(kHistoryR), gd::kInk);
        app.hud.text(r, static_cast<float>(kHistoryPos.x) - 5.0F, static_cast<float>(kHistoryPos.y) - 11.0F, "H", gd::kInk, gd::Hud::Style::Bold);
        if (app.fsm.state == State::Radio) {
            const gbt::Pt to = nodePos(app.fsm.remembered);
            const gbt::Pt d{to.x - kHistoryPos.x, to.y - kHistoryPos.y};
            const double len = std::hypot(d.x, d.y);
            if (len > 1.0) {
                const gbt::Pt from{kHistoryPos.x + d.x / len * kHistoryR, kHistoryPos.y + d.y / len * kHistoryR};
                const gbt::Pt tip = gbt::boxEdgePoint(to, {-d.x / len, -d.y / len}, kBoxHalfW, kBoxHalfH);
                arrow(r, from, tip, 3.0F, gd::kOrange);
            }
        }
    }

    for (State s : gbt::kAllStates) {
        if (!gbt::stateExists(s, f)) continue;
        const gbt::Pt c = nodePos(s);
        const bool current = app.fsm.state == s;
        const auto x = static_cast<float>(c.x - kBoxHalfW);
        const auto y = static_cast<float>(c.y - kBoxHalfH);
        gd::card(r, x, y, static_cast<float>(kBoxHalfW * 2.0), static_cast<float>(kBoxHalfH * 2.0), current ? gd::kTeal : gd::kWhite, gd::kInk);
        const char* label = gbt::name(s);
        app.hud.text(r, static_cast<float>(c.x) - app.hud.width(label) / 2.0F, y + 2.0F, label, current ? gd::kWhite : gd::kInk);
    }

    if (app.hasLastEdge) {
        char line[80];
        SDL_snprintf(line, sizeof line, "last: %s > %s (%s)", gbt::name(app.lastEdge.from), gbt::name(app.fsm.state), gbt::whenLabel(app.lastEdge.when));
        app.hud.text(r, 14.0F, 242.0F, line, gd::kMuted);
    } else {
        app.hud.text(r, 14.0F, 242.0F, "last: no transition yet", gd::kMuted);
    }
}

gd::Color statusColor(gbt::Status s) { return s == gbt::Status::Success ? kGreen : s == gbt::Status::Failure ? kRed : kYellow; }

void node(App& app, const gbt::Node& n, float x, float y, float w, float h) {
    SDL_Renderer* r = app.renderer;
    const bool ticked = n.lastTick == app.bt.tickId;
    const gd::Color fill = ticked ? statusColor(n.last) : gd::kWhite;
    const gd::Color text = !ticked ? gd::kMuted : n.last == gbt::Status::Running ? gd::kInk : gd::kWhite;
    gd::fillRect(r, x, y, w, h, fill);
    gd::strokeRect(r, x, y, w, h, gd::kInk);
    app.hud.text(r, x + (w - app.hud.width(n.label)) / 2.0F, y + (h - app.hud.lineHeight()) / 2.0F, n.label, text);
}

void drawBt(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, 494.0F, 36.0F, "BEHAVIOR TREE", gd::kInk, gd::Hud::Style::Bold);
    const auto& root = static_cast<const gbt::Selector&>(*app.bt.root);
    char counts[32];
    SDL_snprintf(counts, sizeof counts, "%d nodes", gbt::countNodes(root));
    app.hud.text(r, 950.0F - app.hud.width(counts), 36.0F, counts, gd::kMuted);

    constexpr float kTop = 62.0F;
    constexpr float kRootX = 494.0F;
    constexpr float kRootW = 78.0F;
    constexpr float kSeqX = 588.0F;
    constexpr float kSeqW = 70.0F;
    constexpr float kLeafX = 672.0F;
    const auto rows = static_cast<float>(root.kids.size());
    const float rowH = std::min(36.0F, 206.0F / rows);
    const float boxH = rowH - 6.0F;

    // tree edges first, nodes on top
    const gd::Color tree = withAlpha(gd::kInk, 140);
    gd::setColor(r, tree);
    const float rootRight = kRootX + kRootW;
    for (std::size_t i = 0; i < root.kids.size(); ++i) {
        const float cy = kTop + rowH * static_cast<float>(i) + boxH / 2.0F;
        SDL_RenderLine(r, rootRight, kTop + rowH * rows / 2.0F - 3.0F, kSeqX, cy);
        const gbt::Node& branch = *root.kids[i];
        float lx = kLeafX;
        for (const auto& leaf : branch.kids) {
            const float w = app.hud.width(leaf->label) + 12.0F;
            SDL_RenderLine(r, lx == kLeafX ? kSeqX + kSeqW : lx - 8.0F, cy, lx, cy);
            lx += w + 8.0F;
        }
    }

    // the root as a tall bar, orange when it is the passive kind
    const float rootH = rowH * rows - 6.0F;
    gd::card(r, kRootX, kTop, kRootW, rootH, app.active ? gd::kTeal : gd::kOrange, gd::kInk);
    const char* rootLabel = root.label.c_str();
    app.hud.text(r, kRootX + (kRootW - app.hud.width(rootLabel)) / 2.0F, kTop + rootH / 2.0F - 11.0F, rootLabel, gd::kWhite, gd::Hud::Style::Bold);

    for (std::size_t i = 0; i < root.kids.size(); ++i) {
        const float y = kTop + rowH * static_cast<float>(i);
        const gbt::Node& branch = *root.kids[i];
        if (branch.kids.empty()) {  // a single action sits in the first leaf column
            node(app, branch, kSeqX, y, app.hud.width(branch.label) + 12.0F, boxH);
            continue;
        }
        node(app, branch, kSeqX, y, kSeqW, boxH);
        float lx = kLeafX;
        for (const auto& leaf : branch.kids) {
            const float w = app.hud.width(leaf->label) + 12.0F;
            node(app, *leaf, lx, y, w, boxH);
            lx += w + 8.0F;
        }
    }
}

void drawGuard(App& app, const gbt::Body& body, State state, float laneY, const char* label) {
    SDL_Renderer* r = app.renderer;
    const float floorY = laneY + 46.0F;
    const auto gx = kArenaX + static_cast<float>(body.x);
    const gd::Color sc = stateColor(state);
    gd::fillRect(r, kCardX + 1.0F, laneY, kCardW - 2.0F, kLaneH, withAlpha(gd::kPaper, 150));
    // sight band
    const float bandL = std::max(kArenaX, gx - static_cast<float>(gbt::kSight));
    const float bandR = std::min(kArenaX + static_cast<float>(gbt::kArena), gx + static_cast<float>(gbt::kSight));
    gd::fillRect(r, bandL, laneY + 4.0F, bandR - bandL, kLaneH - 8.0F, withAlpha(sc, 38));
    // floor, waypoints, medkit
    gd::fillRect(r, kArenaX, floorY, static_cast<float>(gbt::kArena), 1.0F, withAlpha(gd::kInk, 120));
    for (double wp : gbt::kWaypoints) gd::fillRect(r, kArenaX + static_cast<float>(wp) - 1.0F, floorY - 4.0F, 2.0F, 8.0F, withAlpha(gd::kInk, 120));
    if (app.features.lowHealth) {
        const float mx = kArenaX + static_cast<float>(gbt::kMedkitX);
        gd::fillRect(r, mx - 9.0F, floorY - 18.0F, 18.0F, 18.0F, gd::kWhite);
        gd::strokeRect(r, mx - 9.0F, floorY - 18.0F, 18.0F, 18.0F, gd::kInk);
        gd::fillRect(r, mx - 6.0F, floorY - 10.0F, 12.0F, 3.0F, kRed);
        gd::fillRect(r, mx - 1.5F, floorY - 15.0F, 3.0F, 12.0F, kRed);
    }
    // what the guard remembers: last seen (square), noise (cross)
    if (body.hasLead) {
        const float lx = kArenaX + static_cast<float>(body.leadX);
        gd::strokeRect(r, lx - 5.0F, floorY - 10.0F, 10.0F, 10.0F, gd::kMuted);
    }
    if (body.hasNoise) {
        const float nx = kArenaX + static_cast<float>(body.noiseX);
        thickLine(r, {nx - 5.0, floorY - 10.0}, {nx + 5.0, floorY}, 2.0F, gd::kOrange);
        thickLine(r, {nx - 5.0, floorY}, {nx + 5.0, floorY - 10.0}, 2.0F, gd::kOrange);
    }
    // the player
    {
        const float px = kArenaX + static_cast<float>(app.playerX);
        const SDL_FColor col = gd::fcolor(gd::kTeal);
        const SDL_Vertex v[4] = {{{px, floorY - 22.0F}, col, {0, 0}}, {{px + 8.0F, floorY - 12.0F}, col, {0, 0}}, {{px, floorY - 2.0F}, col, {0, 0}}, {{px - 8.0F, floorY - 12.0F}, col, {0, 0}}};
        const int idx[6] = {0, 1, 2, 0, 2, 3};
        SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
        thickLine(r, {px, floorY - 22.0}, {px + 8.0, floorY - 12.0}, 1.5F, gd::kInk);
        thickLine(r, {px + 8.0, floorY - 12.0}, {px, floorY - 2.0}, 1.5F, gd::kInk);
        thickLine(r, {px, floorY - 2.0}, {px - 8.0, floorY - 12.0}, 1.5F, gd::kInk);
        thickLine(r, {px - 8.0, floorY - 12.0}, {px, floorY - 22.0}, 1.5F, gd::kInk);
    }
    // the guard: disc in the color of its state, hp bar, eye, strike flash
    const float gy = floorY - 12.0F;
    gd::disc(r, gx, gy, 11.0F, sc);
    gd::ring(r, gx, gy, 11.0F, gd::kInk);
    gd::ring(r, gx, gy, 10.0F, gd::kInk);
    gd::disc(r, gx + static_cast<float>(body.facing) * 4.0F, gy - 3.0F, 2.5F, gd::kInk);
    gd::fillRect(r, gx - 14.0F, gy - 22.0F, 28.0F, 4.0F, withAlpha(gd::kInk, 60));
    gd::fillRect(r, gx - 14.0F, gy - 22.0F, 28.0F * static_cast<float>(body.hp / gbt::kMaxHp), 4.0F, body.hp < gbt::kLowHp ? kRed : gd::kTeal);
    if (body.flash > 0.0) {
        const float f = static_cast<float>(body.flash / 0.25);
        const float reach = 20.0F + 16.0F * (1.0F - f);
        thickLine(r, {gx + body.facing * 14.0, gy - 8.0}, {gx + body.facing * reach, gy + 4.0}, 3.0F, withAlpha(kRed, static_cast<int>(255 * f)));
        thickLine(r, {gx + body.facing * 14.0, gy + 4.0}, {gx + body.facing * reach, gy - 8.0}, 3.0F, withAlpha(kRed, static_cast<int>(255 * f)));
    }
    // lane label and state name
    app.hud.text(r, kCardX + 6.0F, laneY + 2.0F, label, gd::kMuted, gd::Hud::Style::Bold);
    const char* sn = gbt::name(state);
    const float tw = app.hud.width(sn, gd::Hud::Style::Bold);
    const float tx = gx + 18.0F + tw > kCardX + kCardW - 6.0F ? gx - 18.0F - tw : gx + 18.0F;
    app.hud.text(r, tx, laneY + 1.0F, sn, gd::kInk, gd::Hud::Style::Bold);  // above the player marker, which is 22 px high
}

void drawArena(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kCardX, kCardY, kCardW, kCardH, gd::kWhite, gd::kInk);
    // noise rings span both lanes
    for (const NoiseRing& ring : app.rings) {
        const auto cx = kArenaX + static_cast<float>(ring.x);
        const auto a = static_cast<int>(200.0 * (1.0 - ring.age / 1.2));
        gd::setColor(r, withAlpha(gd::kOrange, a));
        const float rad = static_cast<float>(ring.age * 230.0);
        for (int i = 0; i < 48; ++i) {
            const float t0 = 6.2831853F * static_cast<float>(i) / 48.0F;
            const float t1 = 6.2831853F * static_cast<float>(i + 1) / 48.0F;
            const float cy = kCardY + kCardH / 2.0F;
            SDL_RenderLine(r, cx + std::cos(t0) * rad, cy + std::sin(t0) * rad * 0.25F, cx + std::cos(t1) * rad, cy + std::sin(t1) * rad * 0.25F);
        }
    }
    drawGuard(app, app.fsm.body, app.fsm.state, kLaneY[0], "FSM");
    gd::fillRect(r, kCardX + 1.0F, kLaneY[0] + kLaneH + 1.0F, kCardW - 2.0F, 1.0F, gd::kGrid);
    drawGuard(app, app.bt.body, app.bt.state, kLaneY[1], "BT");
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ FSM VS BEHAVIOR TREE ]", "same guard, two brains");
    drawFsm(app);
    drawBt(app);
    drawArena(app);

    const char* note = app.note.empty() ? "add a requirement with 1, 2 or 3 and watch the counters" : app.note.c_str();
    app.hud.text(r, 14.0F, 406.0F, note, app.note.empty() ? gd::kMuted : gd::kInk);

    for (const Chip& c : app.chips) {
        const bool on = isOn(app, c.action);
        const char* label = chipLabel(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }

    app.hud.text(r, 14.0F, 514.0F, "mouse: you   click: make noise", gd::kMuted);
    float lx = 950.0F - 226.0F;
    app.hud.text(r, lx, 514.0F, "tree:", gd::kMuted);
    lx += 56.0F;
    const std::array<std::pair<gd::Color, const char*>, 3> legend{{{kGreen, "ok"}, {kRed, "fail"}, {kYellow, "run"}}};
    for (const auto& [color, text] : legend) {
        gd::fillRect(r, lx, 520.0F, 12.0F, 12.0F, color);
        gd::strokeRect(r, lx, 520.0F, 12.0F, 12.0F, gd::kInk);
        app.hud.text(r, lx + 17.0F, 514.0F, text, gd::kMuted);
        lx += 17.0F + app.hud.width(text) + 14.0F;
    }
}

bool inArena(float x, float y) { return x >= kCardX && x <= kCardX + kCardW && y >= kCardY && y <= kCardY + kCardH; }

double playerFromCanvas(float x) { return std::clamp(static_cast<double>(x - kArenaX), 0.0, gbt::kArena); }

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
    if (!SDL_CreateWindowAndRenderer("behavior-tree-vs-fsm", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    reconfigure(*app);
    app->lastNs = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (inArena(event->motion.x, event->motion.y)) app.playerX = playerFromCanvas(event->motion.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inArena(p.x, p.y)) {
                app.playerX = playerFromCanvas(p.x);
                makeNoiseAt(app, app.playerX);
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here toggles or triggers
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Noise); break;
                case SDLK_2: apply(app, Action::LowHealth); break;
                case SDLK_3: apply(app, Action::Radio); break;
                case SDLK_4: apply(app, Action::Selector); break;
                case SDLK_5: apply(app, Action::History); break;
                case SDLK_H: apply(app, Action::Hurt); break;
                case SDLK_P: apply(app, Action::Ring); break;
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
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(0.25, static_cast<double>(now - app.lastNs) / 1e9);
    app.lastNs = now;
    app.accumulator += dt;
    for (int steps = 0; app.accumulator >= kStep && steps < 8; ++steps) {
        step(app, kStep);
        app.accumulator -= kStep;
    }
    if (app.accumulator >= kStep) app.accumulator = 0.0;  // fell behind: drop the rest instead of a burst next frame
    drawScene(app);
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
