// Petri-net arbiter with SDL3 main callbacks.
//
// A squad wants one mounted gun that only one soldier can use. The net at the bottom is the arbiter: tokens are the agents and
// the gun, a transition fires when every input place holds a token, and "Assign" is the one that needs a waiting agent AND the
// free gun. Hit the gunner (click, K, or let Auto-kill do it) and the gun token flows back, so the next agent gets it. What the
// net cannot say is WHO, that is the selection rule: FIFO (registration order, the naive one), Nearest, or Score (distance
// plus accuracy). The number under each agent is what the rule compares, the orange ring marks who is next in line. The net,
// the rules and the walking live in logic.hpp and are covered by test.cpp.

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
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kAgentRadius = 13.0F;
constexpr double kPickRadius = 20.0;     // px around an agent that a click hits
constexpr float kFieldViewTop = 36.0F;   // clicks above this are not on the field
constexpr float kFieldViewBottom = 276.0F;
constexpr float kEnemyX = 938.0F;
constexpr float kGunOffset = 36.0F;  // the MG stands this far right of the spot where its gunner stands
constexpr double kFlight = 0.35;         // seconds a token needs to travel along the arcs of a fired transition
constexpr double kFlash = 0.3;           // seconds a fired transition stays orange
constexpr float kRowA = 336.0F;
constexpr float kRowB = 410.0F;
constexpr float kPlaceR = 22.0F;
constexpr float kBarHalfW = 6.0F;
constexpr float kBarHalfH = 22.0F;
constexpr float kSliderX = 112.0F;
constexpr float kSliderW = 216.0F;
constexpr float kCardX = 24.0F;
constexpr float kCardY = 372.0F;
constexpr float kCardW = 440.0F;
constexpr float kCardH = 84.0F;
constexpr gd::Color kInkSoft{17, 24, 39, 70};

enum class Action { Fifo, Nearest, Score, KillGunner, Auto, NewRound };

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
    pa::Sim sim = pa::makeSim(pa::Policy::Fifo, 6, pa::kShowcaseRound);
    pa::Vec2 mouse{-100.0, -100.0};
    bool dragging = false;
    Uint64 lastNs = 0;
    long maxFrames = -1;
    long frames = 0;
};

// Node positions of the net, indexed by pa::Place and pa::Trans.
const std::array<SDL_FPoint, pa::kPlaceCount> kPlacePos{{{70.0F, kRowA}, {310.0F, kRowA}, {550.0F, kRowA}, {790.0F, kRowA}, {670.0F, kRowB}}};
const std::array<SDL_FPoint, pa::kTransCount> kTransPos{{{190.0F, kRowA}, {430.0F, kRowA}, {670.0F, kRowA}, {790.0F, kRowB}, {550.0F, kRowB}}};
const std::array<const char*, pa::kPlaceCount> kPlaceName{"Spawned", "Registered", "Waiting", "Gun busy", "Gun free"};
const std::array<const char*, pa::kTransCount> kTransName{"Register", "Start (n)", "Assign", "Hit", "Dropout"};

std::string number(int v) { return std::to_string(v); }

/// One decimal without printf, so no locale decides between point and comma.
std::string oneDecimal(double v) {
    const long tenths = std::lround(std::max(v, 0.0) * 10.0);
    return std::to_string(tenths / 10) + "." + std::to_string(tenths % 10);
}

const char* policyName(pa::Policy p) { return p == pa::Policy::Fifo ? "FIFO" : p == pa::Policy::Nearest ? "Nearest" : "Score"; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 6> defs{{{Action::Fifo, "1 FIFO", 0.0F},
                                   {Action::Nearest, "2 Nearest", 8.0F},
                                   {Action::Score, "3 Score", 8.0F},
                                   {Action::KillGunner, "K Hit the gunner", 24.0F},
                                   {Action::Auto, "A Auto-hit", 8.0F},
                                   {Action::NewRound, "R New round", 8.0F}}};
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
        case Action::Fifo: pa::setPolicy(app.sim, pa::Policy::Fifo); break;
        case Action::Nearest: pa::setPolicy(app.sim, pa::Policy::Nearest); break;
        case Action::Score: pa::setPolicy(app.sim, pa::Policy::Score); break;
        case Action::KillGunner: pa::killGunner(app.sim); break;
        case Action::Auto: app.sim.autoKill = !app.sim.autoKill; break;
        case Action::NewRound: pa::nextRound(app.sim); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Fifo: return app.sim.policy == pa::Policy::Fifo;
        case Action::Nearest: return app.sim.policy == pa::Policy::Nearest;
        case Action::Score: return app.sim.policy == pa::Policy::Score;
        case Action::Auto: return app.sim.autoKill;
        default: return false;
    }
}

int countFromX(float x) {
    const double t = std::clamp(static_cast<double>(x - kSliderX) / static_cast<double>(kSliderW), 0.0, 1.0);
    return pa::kMinAgents + static_cast<int>(std::lround(t * (pa::kMaxAgents - pa::kMinAgents)));
}

SDL_FRect sliderRect() { return SDL_FRect{kSliderX - 14.0F, kCardY + 2.0F, kSliderW + 28.0F, 28.0F}; }

/// The agent a click at `p` hits: the nearest one that can still be hit (waiting or at the gun).
int agentAt(const App& app, pa::Vec2 p) {
    int best = -1;
    double bestDist = kPickRadius;
    for (std::size_t i = 0; i < app.sim.agents.size(); ++i) {
        const pa::Agent& a = app.sim.agents[i];
        if (a.state != pa::State::Waiting && a.state != pa::State::Walking && a.state != pa::State::Manning) continue;
        const double d = pa::dist(a.pos, p);
        if (d <= bestDist) {
            best = static_cast<int>(i);
            bestDist = d;
        }
    }
    return best;
}

void line(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half = 0) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

void dashed(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, float on = 6.0F, float off = 6.0F) {
    const float len = std::hypot(x1 - x0, y1 - y0);
    if (len < 1.0F) return;
    const float ux = (x1 - x0) / len;
    const float uy = (y1 - y0) / len;
    for (float s = 0.0F; s < len; s += on + off) {
        const float e = std::min(s + on, len);
        line(r, x0 + ux * s, y0 + uy * s, x0 + ux * e, y0 + uy * e, c);
    }
}

void centerText(App& app, SDL_Renderer* r, float cx, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(r, cx - app.hud.width(s, style) / 2.0F, y, s, c, style);
}

SDL_FPoint lerp(SDL_FPoint a, SDL_FPoint b, double t) {
    const auto f = static_cast<float>(std::clamp(t, 0.0, 1.0));
    return {a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f};
}

/// Where an arc leaves the place (a circle) and meets the transition (a bar). All arcs run along an axis.
void arcEnds(SDL_FPoint place, SDL_FPoint trans, bool placeIsSource, SDL_FPoint& from, SDL_FPoint& to) {
    const bool vertical = std::fabs(place.x - trans.x) < 1.0F;
    const float sx = place.x < trans.x ? 1.0F : -1.0F;
    const float sy = place.y < trans.y ? 1.0F : -1.0F;
    const SDL_FPoint pe = vertical ? SDL_FPoint{place.x, place.y + sy * kPlaceR} : SDL_FPoint{place.x + sx * kPlaceR, place.y};
    const SDL_FPoint te = vertical ? SDL_FPoint{trans.x, trans.y - sy * kBarHalfH} : SDL_FPoint{trans.x - sx * kBarHalfW, trans.y};
    from = placeIsSource ? pe : te;
    to = placeIsSource ? te : pe;
}

void arrow(SDL_Renderer* r, SDL_FPoint from, SDL_FPoint to, int weight, gd::Hud& hud) {
    line(r, from.x, from.y, to.x, to.y, gd::kInk, 1);
    const float len = std::hypot(to.x - from.x, to.y - from.y);
    if (len < 1.0F) return;
    const float ux = (to.x - from.x) / len;
    const float uy = (to.y - from.y) / len;
    for (const float side : {-1.0F, 1.0F}) {
        const float hx = -ux * 10.0F - uy * 5.0F * side;
        const float hy = -uy * 10.0F + ux * 5.0F * side;
        line(r, to.x, to.y, to.x + hx, to.y + hy, gd::kInk, 0);
    }
    if (weight > 1) {
        const std::string w = number(weight);
        hud.text(r, (from.x + to.x) / 2.0F - hud.width(w, gd::Hud::Style::Bold) / 2.0F, (from.y + to.y) / 2.0F - 26.0F, w, gd::kOrange, gd::Hud::Style::Bold);
    }
}

void tokenDots(SDL_Renderer* r, App& app, SDL_FPoint c, int n, gd::Color color) {
    if (n <= 0) return;
    if (n > 9) {
        centerText(app, r, c.x, c.y - 11.0F, number(n), color, gd::Hud::Style::Bold);
        return;
    }
    const int cols = n <= 4 ? 2 : 3;
    const int rows = (n + cols - 1) / cols;
    for (int i = 0; i < n; ++i) {
        const int col = i % cols;
        const int row = i / cols;
        const float x = c.x + (static_cast<float>(col) - (static_cast<float>(cols) - 1.0F) / 2.0F) * 11.0F;
        const float y = c.y + (static_cast<float>(row) - (static_cast<float>(rows) - 1.0F) / 2.0F) * 11.0F;
        gd::disc(r, x, y, 4.5F, color);
    }
}

void drawNet(App& app) {
    SDL_Renderer* r = app.renderer;
    const pa::Sim& sim = app.sim;

    // Arcs come from the net itself, so the picture cannot drift from what runs.
    for (std::size_t t = 0; t < sim.net.transitions.size(); ++t) {
        for (const pa::Arc& a : sim.net.transitions[t].in) {
            SDL_FPoint from;
            SDL_FPoint to;
            arcEnds(kPlacePos[a.place], kTransPos[t], true, from, to);
            arrow(r, from, to, a.weight, app.hud);
        }
        for (const pa::Arc& a : sim.net.transitions[t].out) {
            SDL_FPoint from;
            SDL_FPoint to;
            arcEnds(kPlacePos[a.place], kTransPos[t], false, from, to);
            arrow(r, from, to, a.weight, app.hud);
        }
    }

    // Transitions: orange for a moment after firing, teal while enabled (fires on the next tick), white otherwise.
    for (std::size_t t = 0; t < pa::kTransCount; ++t) {
        double lastFire = -1e9;
        for (const pa::FireEvent& e : sim.events) {
            if (e.transition == t) lastFire = std::max(lastFire, e.time);
        }
        const bool flash = sim.clock - lastFire < kFlash;
        const bool on = pa::enabled(sim.net, t);
        const gd::Color fill = flash ? gd::kOrange : on ? gd::kTeal : gd::kWhite;
        const SDL_FPoint c = kTransPos[t];
        gd::card(r, c.x - kBarHalfW, c.y - kBarHalfH, kBarHalfW * 2.0F, kBarHalfH * 2.0F, fill, gd::kInk);
        const bool above = c.y < (kRowA + kRowB) / 2.0F;
        centerText(app, r, c.x, above ? c.y - kBarHalfH - 24.0F : c.y + kBarHalfH + 4.0F, kTransName[t], gd::kInk, gd::Hud::Style::Bold);
    }

    // Places and their tokens.
    for (std::size_t p = 0; p < pa::kPlaceCount; ++p) {
        const SDL_FPoint c = kPlacePos[p];
        gd::disc(r, c.x, c.y, kPlaceR, gd::kWhite);
        gd::ring(r, c.x, c.y, kPlaceR, gd::kInk);
        gd::ring(r, c.x, c.y, kPlaceR - 1.0F, gd::kInk);
        const gd::Color tone = p == pa::GunBusy ? gd::kOrange : p == pa::GunAvailable ? gd::kTeal : gd::kInk;
        tokenDots(r, app, c, sim.net.marking[p], tone);
        const bool above = c.y < (kRowA + kRowB) / 2.0F;
        centerText(app, r, c.x, above ? c.y - kPlaceR - 24.0F : c.y + kPlaceR + 4.0F, kPlaceName[p], gd::kMuted);
    }

    // Tokens in flight: in along the input arcs, then out along the output arcs of a transition that just fired.
    for (const pa::FireEvent& e : sim.events) {
        const double u = (sim.clock - e.time) / kFlight;
        if (u < 0.0 || u >= 1.0) continue;
        const pa::Transition& tr = sim.net.transitions[e.transition];
        for (const pa::Arc& a : (u < 0.5 ? tr.in : tr.out)) {
            SDL_FPoint from;
            SDL_FPoint to;
            arcEnds(kPlacePos[a.place], kTransPos[e.transition], u < 0.5, from, to);
            const SDL_FPoint p = lerp(from, to, u < 0.5 ? u * 2.0 : (u - 0.5) * 2.0);
            gd::disc(r, p.x, p.y, 6.0F, gd::kOrange);
            gd::ring(r, p.x, p.y, 6.0F, gd::kInk);
        }
    }
}

std::string agentLabel(const App& app, const pa::Agent& a) {
    if (app.sim.policy == pa::Policy::Score) return number(static_cast<int>(std::lround(100.0 * pa::score(a.pos, a.accuracy, app.sim.gun))));
    return number(static_cast<int>(std::lround(pa::dist(a.pos, app.sim.gun))));
}

void drawField(App& app) {
    SDL_Renderer* r = app.renderer;
    const pa::Sim& sim = app.sim;
    const auto gx = static_cast<float>(sim.gun.x);
    const auto gy = static_cast<float>(sim.gun.y);
    const bool busy = sim.gunner >= 0;

    // Who would get the gun next: the pick among the waiting.
    std::vector<pa::Candidate> waiting;
    for (std::size_t i = 0; i < sim.agents.size(); ++i) {
        const pa::Agent& a = sim.agents[i];
        if (a.state == pa::State::Waiting) waiting.push_back({i, static_cast<int>(i), a.pos, a.accuracy});
    }
    const auto next = pa::pick(sim.policy, waiting, sim.gun);

    for (const pa::Candidate& c : waiting) dashed(r, static_cast<float>(c.pos.x), static_cast<float>(c.pos.y), gx, gy, kInkSoft);
    if (busy) {
        const pa::Agent& g = sim.agents[static_cast<std::size_t>(sim.gunner)];
        line(r, static_cast<float>(g.pos.x), static_cast<float>(g.pos.y), gx, gy, gd::kTeal, 1);
    }

    // The enemy and, while somebody mans the gun, the tracer between the two. The gunner stands at (gx, gy), the MG right of it.
    const float mgx = gx + kGunOffset;
    const bool manned = busy && sim.agents[static_cast<std::size_t>(sim.gunner)].state == pa::State::Manning;
    if (manned) {
        const float phase = static_cast<float>(std::fmod(sim.clock * 400.0, 24.0));
        for (float x = mgx + 60.0F + phase; x < kEnemyX - 18.0F; x += 24.0F) line(r, x, gy, std::min(x + 8.0F, kEnemyX - 18.0F), gy, gd::kOrange, 1);
    }
    gd::fillRect(r, kEnemyX - 12.0F, gy - 12.0F, 24.0F, 24.0F, gd::kOrange);
    gd::strokeRect(r, kEnemyX - 12.0F, gy - 12.0F, 24.0F, 24.0F, gd::kInk);
    app.hud.text(r, static_cast<float>(kWidth) - 62.0F, gy + 20.0F, "ENEMY", gd::kOrange, gd::Hud::Style::Bold);

    // The gun.
    gd::fillRect(r, mgx + 22.0F, gy - 4.0F, 22.0F, 8.0F, gd::kInk);
    gd::card(r, mgx - 24.0F, gy - 16.0F, 48.0F, 32.0F, busy ? gd::kTealSoft : gd::kWhite, gd::kInk);
    centerText(app, r, mgx, gy - 11.0F, "MG", gd::kInk, gd::Hud::Style::Bold);
    centerText(app, r, mgx + 10.0F, gy - 46.0F, busy ? "TAKEN" : "FREE", busy ? gd::kOrange : gd::kTeal, gd::Hud::Style::Bold);
    if (manned) gd::disc(r, mgx + 46.0F, gy, 5.0F + 3.0F * static_cast<float>(std::fmod(sim.clock * 12.0, 1.0)), gd::kOrange);

    // The agents.
    for (std::size_t i = 0; i < sim.agents.size(); ++i) {
        const pa::Agent& a = sim.agents[i];
        auto x = static_cast<float>(a.pos.x);
        auto y = static_cast<float>(a.pos.y);
        if (a.state == pa::State::Dead && pa::dist(a.pos, sim.gun) < 1.0) {  // fell at the post: lie down beside it, not under the next gunner
            x = gx - 30.0F - 16.0F * static_cast<float>(i % 4);
            y = gy + 30.0F + 14.0F * static_cast<float>(i / 4);
        }
        const std::string num = number(static_cast<int>(i) + 1);
        switch (a.state) {
            case pa::State::Spawning:
                dashed(r, x - 9.0F, y - 9.0F, x + 9.0F, y - 9.0F, gd::kMuted, 3.0F, 3.0F);
                gd::ring(r, x, y, kAgentRadius, gd::kGrid);
                break;
            case pa::State::Dead:
                gd::disc(r, x, y, kAgentRadius, gd::kGrid);
                gd::ring(r, x, y, kAgentRadius, gd::kMuted);
                line(r, x - 6.0F, y - 6.0F, x + 6.0F, y + 6.0F, gd::kMuted, 0);
                line(r, x - 6.0F, y + 6.0F, x + 6.0F, y - 6.0F, gd::kMuted, 0);
                break;
            default: {
                const bool gunner = a.state == pa::State::Walking || a.state == pa::State::Manning;
                const bool ready = a.state == pa::State::Waiting;
                gd::disc(r, x, y, kAgentRadius, gunner ? gd::kTeal : gd::kWhite);
                gd::ring(r, x, y, kAgentRadius, gd::kInk);
                gd::ring(r, x, y, kAgentRadius - 1.0F, gd::kInk);
                if (ready && next && waiting[*next].id == i) {
                    gd::ring(r, x, y, kAgentRadius + 4.0F, gd::kOrange);
                    gd::ring(r, x, y, kAgentRadius + 5.0F, gd::kOrange);
                }
                centerText(app, r, x, y - 11.0F, num, gunner ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
                if (ready || a.state == pa::State::Registered) {
                    if (ready) centerText(app, r, x, y + kAgentRadius + 2.0F, agentLabel(app, a), gd::kMuted);
                    const float bw = 26.0F;
                    const float by = y + kAgentRadius + 24.0F;
                    gd::fillRect(r, x - bw / 2.0F, by, bw, 4.0F, gd::kGrid);
                    gd::fillRect(r, x - bw / 2.0F, by, bw * static_cast<float>(a.accuracy), 4.0F, gd::kTealDark);
                }
                break;
            }
        }
    }

    // The enemy's crosshair follows the cursor, a click on an agent hits it.
    if (app.mouse.y > static_cast<double>(kFieldViewTop) && app.mouse.y < static_cast<double>(kFieldViewBottom)) {
        const auto mx = static_cast<float>(app.mouse.x);
        const auto my = static_cast<float>(app.mouse.y);
        const bool onAgent = agentAt(app, app.mouse) >= 0;
        const gd::Color c = onAgent ? gd::kOrange : kInkSoft;
        gd::ring(r, mx, my, 9.0F, c);
        line(r, mx - 15.0F, my, mx - 5.0F, my, c, 0);
        line(r, mx + 5.0F, my, mx + 15.0F, my, c, 0);
        line(r, mx, my - 15.0F, mx, my - 5.0F, c, 0);
        line(r, mx, my + 5.0F, mx, my + 15.0F, c, 0);
    }
}

void drawCard(App& app) {
    SDL_Renderer* r = app.renderer;
    const pa::Sim& sim = app.sim;
    gd::card(r, kCardX, kCardY, kCardW, kCardH, gd::kWhite, gd::kInk);

    // Slider: number of agents.
    app.hud.text(r, kCardX + 12.0F, kCardY + 6.0F, "AGENTS", gd::kInk, gd::Hud::Style::Bold);
    const float ty = kCardY + 14.0F;
    gd::fillRect(r, kSliderX, ty, kSliderW, 8.0F, gd::kGrid);
    const float frac = static_cast<float>(sim.count - pa::kMinAgents) / static_cast<float>(pa::kMaxAgents - pa::kMinAgents);
    gd::fillRect(r, kSliderX, ty, kSliderW * frac, 8.0F, gd::kTeal);
    gd::strokeRect(r, kSliderX, ty, kSliderW, 8.0F, gd::kInk);
    gd::card(r, kSliderX + kSliderW * frac - 6.0F, ty - 7.0F, 12.0F, 22.0F, app.dragging ? gd::kTeal : gd::kWhite, gd::kInk);
    app.hud.text(r, kSliderX + kSliderW + 20.0F, kCardY + 6.0F, number(sim.count), gd::kInk, gd::Hud::Style::Bold);

    const char* why = sim.policy == pa::Policy::Fifo      ? "registration order, number = distance"
                      : sim.policy == pa::Policy::Nearest ? "closest first, number = distance"
                                                          : "0.6 close + 0.4 accuracy, number = x100";
    app.hud.text(r, kCardX + 12.0F, kCardY + 30.0F, why, gd::kMuted);

    const gd::Color tone = sim.policy == pa::Policy::Fifo ? gd::kOrange : gd::kTealDark;
    const std::string stats = number(sim.stats.assignments) + " given  avg " + number(static_cast<int>(std::lround(pa::meanDistance(sim.stats)))) + " px  walk " +
                              oneDecimal(pa::meanArrival(sim.stats)) + " s";
    app.hud.text(r, kCardX + 12.0F, kCardY + 54.0F, stats, tone, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    const std::string right = std::string(policyName(app.sim.policy)) + "  round " + number(static_cast<int>(app.sim.round - pa::kShowcaseRound) + 1);
    app.hud.titleBar(r, kWidth, "[ PETRI ARBITER ]", right);
    drawField(app);
    drawNet(app);
    drawCard(app);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, 40.0F, 516.0F, "click agent: hit it  up/down: agents  ring: next  bar: accuracy  teal: can fire", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("petri-arbiter", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    // Start a second in: the squad has registered and the first gunner is already on the way.
    for (int i = 0; i < 80; ++i) pa::advance(app->sim, 1.0 / 60.0);
    app->lastNs = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            if (app.dragging) pa::setCount(app.sim, countFromX(event->motion.x));
            break;
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
            if (onChip) break;
            const SDL_FRect slider = sliderRect();
            if (SDL_PointInRectFloat(&p, &slider)) {
                app.dragging = true;
                pa::setCount(app.sim, countFromX(p.x));
                break;
            }
            if (p.y > kFieldViewTop && p.y < kFieldViewBottom) {
                const int hit = agentAt(app, app.mouse);
                if (hit >= 0) pa::kill(app.sim, static_cast<std::size_t>(hit));
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.dragging = false;
            app.mouse = {-100.0, -100.0};
            break;
        case SDL_EVENT_KEY_DOWN:
            // Everything toggles or triggers, only the agent count may repeat while the key is held.
            if (event->key.key == SDLK_UP) {
                pa::setCount(app.sim, app.sim.count + 1);
            } else if (event->key.key == SDLK_DOWN) {
                pa::setCount(app.sim, app.sim.count - 1);
            } else if (!event->key.repeat) {
                switch (event->key.key) {
                    case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                    case SDLK_1: apply(app, Action::Fifo); break;
                    case SDLK_2: apply(app, Action::Nearest); break;
                    case SDLK_3: apply(app, Action::Score); break;
                    case SDLK_K: apply(app, Action::KillGunner); break;
                    case SDLK_A: apply(app, Action::Auto); break;
                    case SDLK_R: apply(app, Action::NewRound); break;
                    default: break;
                }
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = static_cast<double>(now - app.lastNs) / 1e9;
    app.lastNs = now;
    pa::advance(app.sim, dt);
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
