// GOAP planner with SDL3 main callbacks.
//
// An agent has a goal (the guard is dead) and a handful of actions, each with preconditions, effects and a cost. There is no
// script that says "get the rifle, open the door, shoot": the order falls out of an A* search over world states, so when you
// lock the door the same agent finds another route by itself. The "Script" mode runs the plan the designer would have hard
// coded for the free door and shows what happens to it when the world changes. The logic (world, actions, both searches) is
// in logic.hpp and covered by test.cpp.

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

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr double kWalkSpeed = 200.0;     // px per second
constexpr double kActionTime = 0.55;     // seconds for everything that is not walking
constexpr double kDoneTime = 2.2;        // seconds the finished scene stays before it restarts
constexpr double kMaxDt = 0.25;
constexpr float kRoomL = 560.0F;
constexpr float kRoomT = 60.0F;
constexpr float kRoomR = 930.0F;
constexpr float kRoomB = 370.0F;
constexpr float kDoorTop = 190.0F;
constexpr float kDoorBottom = 240.0F;
constexpr float kWindowL = 700.0F;
constexpr float kWindowR = 740.0F;
constexpr goap::Pos kEnemy{860.0, 200.0};
constexpr SDL_FRect kDoorHit{512.0F, 168.0F, 100.0F, 96.0F};
constexpr float kChipH = 30.0F;
constexpr int kPlanRows = 6;

enum class Mode { Goap, Script };
enum class Phase { Idle, Acting, Patrol, Stuck, Done };
enum class Act { Soldier, Assassin, Rat, Goap, Script, Door, Crates, Restart };

struct Chip {
    Act act;
    SDL_FRect rect;
};

struct Agent {
    goap::State s = goap::startState();  // the agent's world state, loc is the last place it reached
    goap::Pos pos = goap::placePos(goap::kStart);
    Phase phase = Phase::Idle;
    goap::Problem problem;       // actions of the world as it is now, rebuilt whenever the agent re-plans
    goap::Plan plan;             // forward A*, the plan the agent follows in GOAP mode
    goap::Plan planBack;         // backward A* over the same problem, only for the comparison on screen
    goap::Problem scriptProblem;  // the world the script was written for: free door, no crates
    goap::Plan scriptPlan;
    int step = 0;  // next step of plan (GOAP) or script
    int planNumber = 0;
    bool replanNeeded = true;
    int actionIdx = -1;
    double t = 0.0;
    double dur = 1.0;
    goap::Pos from{};
    goap::Pos to{};
    int patrolTarget = goap::kStart;
    double doneTimer = 0.0;
    std::string note;  // reason shown when stuck
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    goap::World world;
    Mode mode = Mode::Goap;
    Agent agent;
    goap::Pos mouse{-100.0, -100.0};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

goap::Pos lerp(goap::Pos a, goap::Pos b, double k) { return {a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k}; }
double dist(goap::Pos a, goap::Pos b) { return std::hypot(b.x - a.x, b.y - a.y); }

const char* roleName(goap::Role r) {
    switch (r) {
        case goap::Role::Soldier: return "Soldier";
        case goap::Role::Assassin: return "Assassin";
        case goap::Role::Rat: return "Rat";
    }
    return "";
}

const char* doorName(goap::Door d) {
    switch (d) {
        case goap::Door::Free: return "free";
        case goap::Door::Locked: return "locked";
        case goap::Door::Barricaded: return "barricaded";
    }
    return "";
}

// ---- agent ----------------------------------------------------------------------------------------------------

bool isDone(const App& app) { return goap::goalMet(app.agent.s, app.agent.problem.goal); }

void beginAction(App& app, int idx) {
    Agent& a = app.agent;
    const goap::Action& act = a.problem.actions[static_cast<std::size_t>(idx)];
    if (!goap::applicable(a.s, act)) {  // the world changed under the plan
        a.replanNeeded = true;
        return;
    }
    a.actionIdx = idx;
    a.t = 0.0;
    a.from = a.pos;
    a.to = act.to >= 0 ? goap::placePos(act.to) : a.pos;
    a.dur = act.kind == goap::Kind::Walk ? std::max(0.2, dist(a.from, a.to) / kWalkSpeed) : kActionTime;
    a.phase = Phase::Acting;
}

void startPatrol(App& app, const char* why) {
    Agent& a = app.agent;
    a.note = why;
    a.phase = Phase::Patrol;
    a.patrolTarget = a.s.v[goap::kVarLoc] == goap::kStart ? goap::kLocker : goap::kStart;
}

void decide(App& app) {
    Agent& a = app.agent;
    if (a.replanNeeded) {
        a.problem = goap::buildProblem(app.world, a.s);
        if (app.mode == Mode::Goap) {
            a.plan = goap::planForward(a.problem);
            a.planBack = goap::planBackward(a.problem);
            a.step = 0;
            ++a.planNumber;
        }
        a.replanNeeded = false;
    }
    if (isDone(app)) {
        a.phase = Phase::Done;
        a.doneTimer = 0.0;
        return;
    }
    if (app.mode == Mode::Goap) {
        if (!a.plan.found) {
            startPatrol(app, "NO PLAN: patrol");
            return;
        }
        if (a.step >= static_cast<int>(a.plan.steps.size())) {
            a.replanNeeded = true;
            return;
        }
        beginAction(app, a.plan.steps[static_cast<std::size_t>(a.step)]);
        return;
    }
    if (a.step >= static_cast<int>(a.scriptPlan.steps.size())) {
        a.note = "STUCK: script ended";
        a.phase = Phase::Stuck;
        return;
    }
    const std::string& name = a.scriptProblem.actions[static_cast<std::size_t>(a.scriptPlan.steps[static_cast<std::size_t>(a.step)])].name;
    const int idx = goap::findAction(a.problem, name);
    if (idx < 0 || !goap::applicable(a.s, a.problem.actions[static_cast<std::size_t>(idx)])) {
        a.note = "STUCK: cannot " + name;
        a.phase = Phase::Stuck;
        return;
    }
    beginAction(app, idx);
}

void resetAgent(App& app) {
    Agent& a = app.agent;
    a = Agent{};
    // The script is what the designer wrote for the world without trouble: free door, no distractors.
    a.scriptProblem = goap::buildProblem(goap::World{app.world.role, goap::Door::Free, 0}, goap::startState());
    a.scriptPlan = goap::planForward(a.scriptProblem);
    a.problem = goap::buildProblem(app.world, a.s);
    decide(app);
}

void updateAgent(App& app, double dt) {
    Agent& a = app.agent;
    switch (a.phase) {
        case Phase::Idle: decide(app); break;
        case Phase::Acting: {
            a.t += dt;
            a.pos = lerp(a.from, a.to, std::min(1.0, a.t / a.dur));
            if (a.t >= a.dur) {
                a.s = goap::apply(a.s, a.problem.actions[static_cast<std::size_t>(a.actionIdx)]);
                a.pos = a.to;
                ++a.step;
                a.phase = Phase::Idle;
            }
            break;
        }
        case Phase::Patrol: {
            const goap::Pos target = goap::placePos(a.patrolTarget);
            const double d = dist(a.pos, target);
            const double move = kWalkSpeed * 0.6 * dt;
            if (d <= move) {
                a.pos = target;
                a.s.v[goap::kVarLoc] = static_cast<std::int8_t>(a.patrolTarget);
                if (a.replanNeeded) {
                    a.phase = Phase::Idle;
                } else {
                    a.patrolTarget = a.patrolTarget == goap::kStart ? goap::kLocker : goap::kStart;
                }
            } else {
                a.pos = lerp(a.pos, target, move / d);
            }
            break;
        }
        case Phase::Stuck:
            if (a.replanNeeded) a.phase = Phase::Idle;
            break;
        case Phase::Done:
            a.doneTimer += dt;
            if (a.doneTimer >= kDoneTime) resetAgent(app);
            break;
    }
}

// ---- input ----------------------------------------------------------------------------------------------------

std::string chipLabel(const App& app, Act act) {
    switch (act) {
        case Act::Soldier: return "1 Soldier";
        case Act::Assassin: return "2 Assassin";
        case Act::Rat: return "3 Rat";
        case Act::Goap: return "4 GOAP";
        case Act::Script: return "5 Script";
        case Act::Door: return std::string("D Door: ") + doorName(app.world.door);
        case Act::Crates: return "C Crates: " + std::to_string(app.world.crates);
        case Act::Restart: return "R Restart";
    }
    return "";
}

bool chipOn(const App& app, Act act) {
    switch (act) {
        case Act::Soldier: return app.world.role == goap::Role::Soldier;
        case Act::Assassin: return app.world.role == goap::Role::Assassin;
        case Act::Rat: return app.world.role == goap::Role::Rat;
        case Act::Goap: return app.mode == Mode::Goap;
        case Act::Script: return app.mode == Mode::Script;
        case Act::Door: return app.world.door != goap::Door::Free;
        case Act::Crates: return app.world.crates > 0;
        case Act::Restart: return false;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Act act;
        const char* widest;  // fixed width for labels that change
        float y;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Act::Soldier, "", 468.0F, 0.0F},
                                   {Act::Assassin, "", 468.0F, 8.0F},
                                   {Act::Rat, "", 468.0F, 8.0F},
                                   {Act::Goap, "", 468.0F, 28.0F},
                                   {Act::Script, "", 468.0F, 8.0F},
                                   {Act::Door, "D Door: barricaded", 504.0F, 0.0F},
                                   {Act::Crates, "C Crates: 9", 504.0F, 8.0F},
                                   {Act::Restart, "", 504.0F, 28.0F}}};
    float x = 40.0F;
    float lastY = 0.0F;
    for (const Def& d : defs) {
        if (d.y != lastY) x = 40.0F;
        lastY = d.y;
        x += d.gapBefore;
        const std::string label = d.widest[0] != '\0' ? std::string(d.widest) : chipLabel(app, d.act);
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.act, SDL_FRect{x, d.y, w, kChipH}});
        x += w;
    }
}

void restart(App& app) { resetAgent(app); }

void setRole(App& app, goap::Role role) {
    app.world.role = role;
    restart(app);
}

void setMode(App& app, Mode mode) {
    app.mode = mode;
    restart(app);
}

void cycleDoor(App& app) {
    switch (app.world.door) {
        case goap::Door::Free: app.world.door = goap::Door::Locked; break;
        case goap::Door::Locked: app.world.door = goap::Door::Barricaded; break;
        case goap::Door::Barricaded: app.world.door = goap::Door::Free; break;
    }
    app.agent.replanNeeded = true;  // the agent notices at its next step
}

void cycleCrates(App& app) {
    app.world.crates = (app.world.crates + 3) % (goap::kMaxCrates + 3);
    restart(app);
}

void apply(App& app, Act act) {
    switch (act) {
        case Act::Soldier: setRole(app, goap::Role::Soldier); break;
        case Act::Assassin: setRole(app, goap::Role::Assassin); break;
        case Act::Rat: setRole(app, goap::Role::Rat); break;
        case Act::Goap: setMode(app, Mode::Goap); break;
        case Act::Script: setMode(app, Mode::Script); break;
        case Act::Door: cycleDoor(app); break;
        case Act::Crates: cycleCrates(app); break;
        case Act::Restart: restart(app); break;
    }
}

// ---- drawing --------------------------------------------------------------------------------------------------

void thickLine(SDL_Renderer* r, goap::Pos a, goap::Pos b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F),
                       static_cast<float>(b.x) + (flat ? 0.0F : off), static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

void dashed(SDL_Renderer* r, goap::Pos a, goap::Pos b, gd::Color c) {
    const double len = dist(a, b);
    for (double d = 0.0; d < len; d += 14.0) {
        thickLine(r, lerp(a, b, d / len), lerp(a, b, std::min(len, d + 8.0) / len), c, 1);
    }
}

void text(App& app, float x, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, x, y, s, c, style);
}

void textCentered(App& app, float cx, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    text(app, cx - app.hud.width(s, style) / 2.0F, y, s, c, style);
}

void drawRoom(App& app) {
    SDL_Renderer* r = app.renderer;
    const Agent& a = app.agent;
    gd::fillRect(r, kRoomL, kRoomT, kRoomR - kRoomL, kRoomB - kRoomT, gd::kWhite);
    constexpr gd::Color wall = gd::kInk;
    gd::fillRect(r, kRoomL - 2, kRoomT - 2, kRoomR - kRoomL + 4, 4, wall);                    // top
    gd::fillRect(r, kRoomR - 2, kRoomT - 2, 4, kRoomB - kRoomT + 4, wall);                    // right
    gd::fillRect(r, kRoomL - 2, kRoomT - 2, 4, kDoorTop - kRoomT + 2, wall);                  // left, above the door
    gd::fillRect(r, kRoomL - 2, kDoorBottom, 4, kRoomB - kDoorBottom + 2, wall);              // left, below the door
    gd::fillRect(r, kRoomL - 2, kRoomB - 2, kWindowL - kRoomL + 2, 4, wall);                  // bottom, left of the window
    gd::fillRect(r, kWindowR, kRoomB - 2, kRoomR - kWindowR + 2, 4, wall);                    // bottom, right of the window

    // Door: open leaf swung into the yard, otherwise a bar whose look says why nobody gets through.
    const bool opened = a.s.v[goap::kVarDoor] == 1;
    if (opened) {
        thickLine(r, {kRoomL, kDoorTop}, {kRoomL - 28.0, kDoorTop - 22.0}, gd::kTeal, 2);
    } else if (app.world.door == goap::Door::Free) {
        gd::fillRect(r, kRoomL - 3, kDoorTop, 6, kDoorBottom - kDoorTop, gd::kTeal);
    } else {
        gd::fillRect(r, kRoomL - 3, kDoorTop, 6, kDoorBottom - kDoorTop, gd::kOrange);
        if (app.world.door == goap::Door::Barricaded) {
            thickLine(r, {kRoomL - 22.0, kDoorTop}, {kRoomL + 4.0, kDoorBottom}, gd::kOrange, 2);
            thickLine(r, {kRoomL - 22.0, kDoorBottom}, {kRoomL + 4.0, kDoorTop}, gd::kOrange, 2);
        }
    }
    const std::string doorLabel = opened ? "open" : doorName(app.world.door);
    const bool blocked = !opened && app.world.door != goap::Door::Free;
    text(app, kRoomL - 12.0F - app.hud.width(doorLabel), kDoorBottom + 8.0F, doorLabel, blocked ? gd::kOrange : gd::kMuted);
    text(app, kRoomL - 12.0F - app.hud.width("door"), kDoorTop - 30.0F, "door", gd::kMuted);

    // Window.
    if (a.s.v[goap::kVarWindow] == 1) {
        for (float x = kWindowL; x < kWindowR; x += 10.0F) gd::fillRect(r, x, kRoomB - 2, 5, 4, gd::kTeal);
    } else {
        gd::fillRect(r, kWindowL, kRoomB - 3, kWindowR - kWindowL, 6, gd::kTealSoft);
        gd::strokeRect(r, kWindowL, kRoomB - 3, kWindowR - kWindowL, 6, gd::kInk);
    }
    textCentered(app, (kWindowL + kWindowR) / 2.0F, kRoomB - 28.0F, "window", gd::kMuted);

    // Enemy.
    const auto ex = static_cast<float>(kEnemy.x);
    const auto ey = static_cast<float>(kEnemy.y);
    if (a.s.v[goap::kVarDead] == 1) {
        thickLine(r, {kEnemy.x - 10.0, kEnemy.y - 10.0}, {kEnemy.x + 10.0, kEnemy.y + 10.0}, gd::kMuted, 1);
        thickLine(r, {kEnemy.x - 10.0, kEnemy.y + 10.0}, {kEnemy.x + 10.0, kEnemy.y - 10.0}, gd::kMuted, 1);
        textCentered(app, ex, ey + 18.0F, "Guard (dead)", gd::kMuted);
    } else {
        gd::disc(r, ex, ey, 12.0F, gd::kOrange);
        gd::ring(r, ex, ey, 12.0F, gd::kInk);
        textCentered(app, ex, ey + 18.0F, "Guard", gd::kInk);
    }
}

void drawPlanPath(App& app) {
    const Agent& a = app.agent;
    const goap::Problem& prob = app.mode == Mode::Goap ? a.problem : a.scriptProblem;
    const goap::Plan& plan = app.mode == Mode::Goap ? a.plan : a.scriptPlan;
    if (!plan.found) return;
    for (int idx : plan.steps) {
        const goap::Action& act = prob.actions[static_cast<std::size_t>(idx)];
        if (act.from >= 0 && act.to >= 0) dashed(app.renderer, goap::placePos(act.from), goap::placePos(act.to), gd::Color{13, 148, 136, 150});
    }
}

void drawWorld(App& app) {
    SDL_Renderer* r = app.renderer;
    const Agent& a = app.agent;
    drawRoom(app);
    drawPlanPath(app);

    // Crates, the distractors.
    for (int i = 0; i < app.world.crates; ++i) {
        const goap::Pos p = goap::placePos(goap::kFirstCrate + i);
        const bool looted = a.s.v[static_cast<std::size_t>(goap::kVarLooted + i)] == 1;
        gd::fillRect(r, static_cast<float>(p.x) - 9.0F, static_cast<float>(p.y) - 9.0F, 18.0F, 18.0F, looted ? gd::kTealSoft : gd::kWhite);
        gd::strokeRect(r, static_cast<float>(p.x) - 9.0F, static_cast<float>(p.y) - 9.0F, 18.0F, 18.0F, gd::kInk);
    }

    // Start and locker.
    const goap::Pos st = goap::placePos(goap::kStart);
    gd::ring(r, static_cast<float>(st.x), static_cast<float>(st.y), 16.0F, gd::kMuted);
    textCentered(app, static_cast<float>(st.x), static_cast<float>(st.y) + 22.0F, "Start", gd::kMuted);
    const goap::Pos lk = goap::placePos(goap::kLocker);
    gd::card(r, static_cast<float>(lk.x) - 17.0F, static_cast<float>(lk.y) - 13.0F, 34.0F, 26.0F, gd::kWhite, gd::kInk);
    if (a.s.v[goap::kVarWeapon] == 0 && app.world.role == goap::Role::Soldier) {
        thickLine(r, {lk.x - 9.0, lk.y}, {lk.x + 9.0, lk.y}, gd::kInk, 1);
    }
    textCentered(app, static_cast<float>(lk.x), static_cast<float>(lk.y) - 38.0F, "Locker", gd::kMuted);

    // Hover frame around the clickable door.
    const SDL_FPoint mp{static_cast<float>(app.mouse.x), static_cast<float>(app.mouse.y)};
    if (SDL_PointInRectFloat(&mp, &kDoorHit)) gd::strokeRect(r, kDoorHit.x, kDoorHit.y, kDoorHit.w, kDoorHit.h, gd::kTeal);

    // Agent.
    const bool rat = app.world.role == goap::Role::Rat;
    const gd::Color body = app.world.role == goap::Role::Soldier ? gd::kTeal : (rat ? gd::kOrange : gd::kInk);
    const float radius = rat ? 8.0F : 12.0F;
    const auto ax = static_cast<float>(a.pos.x);
    const auto ay = static_cast<float>(a.pos.y);
    gd::disc(r, ax, ay, radius, body);
    gd::ring(r, ax, ay, radius, gd::kInk);
    if (a.s.v[goap::kVarWeapon] == 1) thickLine(r, {a.pos.x + radius, a.pos.y}, {a.pos.x + radius + 12.0, a.pos.y - 4.0}, gd::kInk, 1);
    std::string tag;
    gd::Color tagColor = gd::kMuted;
    if (a.phase == Phase::Acting) {
        tag = a.problem.actions[static_cast<std::size_t>(a.actionIdx)].name;
    } else if (a.phase == Phase::Stuck) {
        tag = "!";
        tagColor = gd::kOrange;
    } else if (a.phase == Phase::Patrol) {
        tag = "patrol";
        tagColor = gd::kOrange;
    }
    if (!tag.empty()) textCentered(app, ax, ay - radius - 26.0F, tag, tagColor, gd::Hud::Style::Bold);
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    const Agent& a = app.agent;
    gd::card(r, 20.0F, 44.0F, 296.0F, 250.0F, gd::kWhite, gd::kInk);
    const bool goapMode = app.mode == Mode::Goap;
    const goap::Problem& prob = goapMode ? a.problem : a.scriptProblem;
    const goap::Plan& plan = goapMode ? a.plan : a.scriptPlan;

    std::string head;
    gd::Color headColor = gd::kInk;
    if (goapMode) {
        if (plan.found) {
            head = "PLAN  cost " + std::to_string(plan.cost);
        } else {
            head = "NO PLAN";
            headColor = gd::kOrange;
        }
    } else {
        head = "SCRIPT  fixed order";
    }
    text(app, 32.0F, 52.0F, head, headColor, gd::Hud::Style::Bold);

    const int stuckAt = (!goapMode && a.phase == Phase::Stuck) ? a.step : -1;
    float y = 78.0F;
    for (int i = 0; i < kPlanRows; ++i, y += 22.0F) {
        if (!plan.found || i >= static_cast<int>(plan.steps.size())) continue;
        const goap::Action& act = prob.actions[static_cast<std::size_t>(plan.steps[static_cast<std::size_t>(i)])];
        gd::Color c = gd::kInk;
        auto style = gd::Hud::Style::Regular;
        if (i < a.step) {
            c = gd::kMuted;
        } else if (i == a.step) {
            style = gd::Hud::Style::Bold;
            c = i == stuckAt ? gd::kOrange : gd::kTeal;
        }
        if (a.phase == Phase::Done) c = gd::kMuted;
        text(app, 32.0F, y, std::to_string(i + 1) + " " + act.name, c, style);
        if (goapMode) {
            const std::string cost = std::to_string(act.cost);
            text(app, 300.0F - app.hud.width(cost), y, cost, c, style);
        }
    }

    y = 78.0F + 22.0F * static_cast<float>(kPlanRows) + 6.0F;
    if (goapMode) {
        text(app, 32.0F, y, "forward A*  " + std::to_string(a.plan.expanded) + " nodes", gd::kInk);
        text(app, 32.0F, y + 22.0F, "backward A* " + std::to_string(a.planBack.expanded) + " nodes", gd::kMuted);
    } else {
        text(app, 32.0F, y, "no search, no replan", gd::kMuted);
    }
    std::string status = "running";
    gd::Color statusColor = gd::kInk;
    switch (a.phase) {
        case Phase::Idle:
        case Phase::Acting: status = "step " + std::to_string(std::min(a.step + 1, static_cast<int>(plan.steps.size()))) + "/" + std::to_string(plan.steps.size()); break;
        case Phase::Patrol:
        case Phase::Stuck:
            status = a.note;
            statusColor = gd::kOrange;
            break;
        case Phase::Done:
            status = "DONE: guard dead";
            statusColor = gd::kTeal;
            break;
    }
    text(app, 32.0F, y + 46.0F - 2.0F, status, statusColor, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    std::string right = std::string(roleName(app.world.role)) + " / " + (app.mode == Mode::Goap ? "GOAP" : "script");
    if (app.mode == Mode::Goap) right += " / plan " + std::to_string(app.agent.planNumber);
    app.hud.titleBar(r, kWidth, "[ GOAP PLANNER ]", right);
    drawWorld(app);
    drawPanel(app);
    text(app, 40.0F, 446.0F, "click the door: free, locked, barricaded. The agent notices at its next step.", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = chipOn(app, c.act);
        const bool problem = c.act == Act::Door || c.act == Act::Crates;
        const std::string label = chipLabel(app, c.act);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? (problem ? gd::kOrange : gd::kTeal) : gd::kWhite, gd::kInk);
        text(app, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("goap-planner", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    resetAgent(*app);
    app->lastTicks = SDL_GetTicks();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)}; break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE: app.mouse = {-100.0, -100.0}; break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = {static_cast<double>(p.x), static_cast<double>(p.y)};
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.act);
                    onChip = true;
                    break;
                }
            }
            if (!onChip && SDL_PointInRectFloat(&p, &kDoorHit)) cycleDoor(app);
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key toggles or restarts, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Act::Soldier); break;
                case SDLK_2: apply(app, Act::Assassin); break;
                case SDLK_3: apply(app, Act::Rat); break;
                case SDLK_4: apply(app, Act::Goap); break;
                case SDLK_5: apply(app, Act::Script); break;
                case SDLK_D: apply(app, Act::Door); break;
                case SDLK_C: apply(app, Act::Crates); break;
                case SDLK_R: apply(app, Act::Restart); break;
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
    const double dt = std::min(kMaxDt, static_cast<double>(now - app.lastTicks) / 1000.0);
    app.lastTicks = now;
    updateAgent(app, dt);
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
