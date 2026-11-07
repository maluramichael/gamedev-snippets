// Play style as a vector, SDL3 main callbacks.
//
// Every move you make adds a small update vector to a profile of five numbers (fighter, tactician, storyteller,
// role-player, power gamer). The game never looks at the moves again: it scores each candidate event with a dot
// product against the profile and shows the best one. The first frame is the player from the Game AI Pro chapter,
// Grendel 0.81 against the fairy 0.09. Three story moves later the fairy wins. "Select: last move" is the naive
// rule that reacts to the latest move only, which is why one off-style move flips it and does not flip the profile.
// "Forget" pulls the profile back to neutral over time, so recent moves weigh more. The math is in logic.hpp.

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
constexpr float kRowTop = 80.0F;
constexpr float kRowPitch = 38.0F;
constexpr float kBarW = 230.0F;
constexpr float kScoreBarW = 150.0F;
constexpr float kMoveChipY = 386.0F;
constexpr float kOptionChipY = 482.0F;
constexpr float kChipH = 34.0F;
constexpr double kMaxDt = 0.25;
constexpr double kFlashSeconds = 0.18;
constexpr double kBarEase = 14.0;  // display only, the numbers on screen are the real values
constexpr gd::Color kBarSoft{110, 200, 190};

enum class Action { Move, Select, Forget, NewPlayer, Chapter };

struct Chip {
    Action action;
    int move;  // for Action::Move
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    psm::Profile profile = psm::kChapterProfile;
    psm::Profile shown = psm::kChapterProfile;
    int lastMove = -1;
    int moves = 0;
    bool naive = false;
    bool forgetOn = false;
    std::array<double, psm::kMoves> flash{};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

const char* moveLabel(int move) {
    static const std::array<const char*, psm::kMoves> labels{"1 Fight", "2 Tactics", "3 Story", "4 Roleplay", "5 Power-up"};
    return labels[static_cast<std::size_t>(move)];
}

std::string chipLabel(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Move: return moveLabel(c.move);
        case Action::Select: return app.naive ? "S Select: last move" : "S Select: profile";
        case Action::Forget: return app.forgetOn ? "F Forget: on" : "F Forget: off";
        case Action::NewPlayer: return "N New player";
        case Action::Chapter: return "E Chapter example";
    }
    return {};
}

void layoutChips(App& app) {
    const auto width = [&app](const char* s) { return app.hud.width(s, gd::Hud::Style::Bold) + 24.0F; };
    float x = 24.0F;
    for (int m = 0; m < psm::kMoves; ++m) {
        const float w = width(moveLabel(m));
        app.chips.push_back({Action::Move, m, SDL_FRect{x, kMoveChipY, w, kChipH}});
        x += w + 14.0F;
    }
    struct Def {
        Action action;
        const char* widest;
    };
    const std::array<Def, 4> defs{{{Action::Select, "S Select: last move"}, {Action::Forget, "F Forget: off"}, {Action::NewPlayer, "N New player"}, {Action::Chapter, "E Chapter example"}}};
    x = 24.0F;
    for (const Def& d : defs) {
        const float w = width(d.widest);
        app.chips.push_back({d.action, -1, SDL_FRect{x, kOptionChipY, w, kChipH}});
        x += w + 16.0F;
    }
}

void resetPlayer(App& app, const psm::Profile& start) {
    app.profile = start;
    app.lastMove = -1;
    app.moves = 0;
    app.flash.fill(0.0);
}

void play(App& app, int move) {
    app.profile = psm::learn(app.profile, psm::kMovesTable[static_cast<std::size_t>(move)].update);
    app.lastMove = move;
    ++app.moves;
    app.flash[static_cast<std::size_t>(move)] = kFlashSeconds;
}

void apply(App& app, const Chip& c) {
    switch (c.action) {
        case Action::Move: play(app, c.move); break;
        case Action::Select: app.naive = !app.naive; break;
        case Action::Forget: app.forgetOn = !app.forgetOn; break;
        case Action::NewPlayer: resetPlayer(app, psm::kNeutral); break;
        case Action::Chapter: resetPlayer(app, psm::kChapterProfile); break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Move: return app.flash[static_cast<std::size_t>(c.move)] > 0.0;
        case Action::Select: return true;
        case Action::Forget: return app.forgetOn;
        default: return false;
    }
}

void update(App& app, double dt) {
    if (app.forgetOn) app.profile = psm::forget(app.profile, psm::kNeutral, psm::kForgetHalfLife, dt);
    const auto ease = static_cast<float>(1.0 - std::exp(-kBarEase * dt));
    for (std::size_t i = 0; i < app.profile.size(); ++i) app.shown[i] += (app.profile[i] - app.shown[i]) * static_cast<double>(ease);
    for (double& f : app.flash) f = std::max(0.0, f - dt);
}

std::string num(double v, int decimals = 2) {
    char buf[24];
    SDL_snprintf(buf, sizeof buf, decimals == 1 ? "%.1f" : "%.2f", v);
    return buf;
}

void text(App& app, float x, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, x, y, s, c, style);
}

void drawProfile(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 24.0F, 44.0F, 430.0F, 300.0F, gd::kWhite, gd::kInk);
    text(app, 38.0F, 52.0F, "PLAYER PROFILE", gd::kMuted, gd::Hud::Style::Bold);
    const auto top = static_cast<std::size_t>(std::max_element(app.profile.begin(), app.profile.end()) - app.profile.begin());
    for (std::size_t i = 0; i < app.profile.size(); ++i) {
        const float y = kRowTop + kRowPitch * static_cast<float>(i);
        text(app, 38.0F, y + 2.0F, psm::kStyleNames[i], gd::kInk, gd::Hud::Style::Bold);
        gd::fillRect(r, 160.0F, y + 1.0F, kBarW, 22.0F, gd::kGrid);
        const auto w = static_cast<float>(std::clamp(app.shown[i], 0.0, 1.0)) * kBarW;
        gd::fillRect(r, 160.0F, y + 1.0F, w, 22.0F, app.profile[top] > 1e-9 && i == top ? gd::kTeal : kBarSoft);
        gd::strokeRect(r, 160.0F, y + 1.0F, kBarW, 22.0F, gd::kInk);
        text(app, 400.0F, y + 2.0F, num(app.profile[i]), gd::kInk, gd::Hud::Style::Bold);
    }
    std::string vec = "(";
    for (std::size_t i = 0; i < app.profile.size(); ++i) {
        if (i > 0) vec += " ";
        vec += psm::kStyleLetters[i];
        vec += " " + num(app.profile[i]);
    }
    text(app, 38.0F, 276.0F, vec + ")", gd::kInk);
    text(app, 38.0F, 304.0F, "moves played: " + std::to_string(app.moves), gd::kMuted);
    if (app.forgetOn) text(app, 200.0F, 304.0F, "forget: half-life " + std::to_string(static_cast<int>(psm::kForgetHalfLife)) + " s", gd::kMuted);
}

/// Explanation under the candidate list: the winning dot product written out, or what the naive rule looks at.
void drawWhy(App& app, const psm::Choice& choice, const psm::Choice& profileChoice) {
    std::string l1;
    std::string l2;
    if (app.naive) {
        if (app.lastMove < 0) {
            l1 = "no move yet";
            l2 = "the naive rule has nothing to go on";
        } else {
            l1 = std::string("last move: ") + psm::kMovesTable[static_cast<std::size_t>(app.lastMove)].name;
            if (profileChoice.index < 0) {
                l2 = "profile has no signal";
            } else if (profileChoice.index == choice.index) {
                l2 = "profile agrees";
            } else {
                l2 = std::string("profile would pick ") + psm::kCandidatesTable[static_cast<std::size_t>(profileChoice.index)].name;
            }
        }
    } else if (choice.index < 0) {
        l1 = "no signal yet";
        l2 = "play a move";
    } else {
        const auto win = static_cast<std::size_t>(choice.index);
        const auto terms = psm::topTerms(app.profile, psm::kCandidatesTable[win].fit, 3);
        for (std::size_t k = 0; k < terms.size(); ++k) {
            const auto s = static_cast<std::size_t>(terms[k].style);
            if (k > 0) l1 += " + ";
            l1 += num(psm::kCandidatesTable[win].fit[s], 1) + " × " + num(app.profile[s]) + " (" + psm::kStyleLetters[s] + ")";
        }
        l1 += " = " + num(choice.best);
        const auto second = static_cast<std::size_t>(choice.runnerUp);
        l2 = std::string("beats ") + psm::kCandidatesTable[second].name + " (" + num(psm::scoreByProfile(app.profile)[second]) + ") by " + num(choice.margin);
    }
    text(app, 502.0F, 276.0F, l1, gd::kInk, gd::Hud::Style::Bold);
    text(app, 502.0F, 304.0F, l2, gd::kMuted);
}

void drawCandidates(App& app) {
    SDL_Renderer* r = app.renderer;
    const psm::Scores profileScores = psm::scoreByProfile(app.profile);
    const psm::Scores scores = app.naive ? psm::scoreByLastMove(app.lastMove) : profileScores;
    const psm::Choice choice = psm::choose(scores);
    const psm::Choice profileChoice = psm::choose(profileScores);

    gd::card(r, 486.0F, 44.0F, 450.0F, 300.0F, gd::kWhite, gd::kInk);
    text(app, 502.0F, 52.0F, "NEXT EVENT", gd::kMuted, gd::Hud::Style::Bold);
    const std::string rule = app.naive ? "score = fit of last move" : "score = profile · fit";
    text(app, 920.0F - app.hud.width(rule), 52.0F, rule, app.naive ? gd::kOrange : gd::kMuted);

    const gd::Color accent = app.naive ? gd::kOrange : gd::kTeal;
    for (std::size_t i = 0; i < scores.size(); ++i) {
        const float y = kRowTop + kRowPitch * static_cast<float>(i);
        const bool win = static_cast<int>(i) == choice.index;
        if (win) gd::fillRect(r, 494.0F, y - 3.0F, 434.0F, 30.0F, app.naive ? gd::Color{255, 237, 213} : gd::kTealSoft);
        text(app, 502.0F, y + 2.0F, psm::kCandidatesTable[i].name, gd::kInk, win ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        gd::fillRect(r, 700.0F, y + 1.0F, kScoreBarW, 22.0F, gd::kGrid);
        const auto w = static_cast<float>(std::clamp(scores[i], 0.0, 1.0)) * kScoreBarW;
        gd::fillRect(r, 700.0F, y + 1.0F, w, 22.0F, win ? accent : kBarSoft);
        gd::strokeRect(r, 700.0F, y + 1.0F, kScoreBarW, 22.0F, gd::kInk);
        text(app, 862.0F, y + 2.0F, num(scores[i]), win ? accent : gd::kInk, gd::Hud::Style::Bold);
    }
    drawWhy(app, choice, profileChoice);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ PLAY STYLE MODEL ]", app.naive ? "select by last move (naive)" : "select by profile (dot product)");
    drawProfile(app);
    drawCandidates(app);

    text(app, 24.0F, 360.0F, "YOUR MOVE", gd::kMuted, gd::Hud::Style::Bold);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const gd::Color fill = c.action == Action::Select ? (app.naive ? gd::kOrange : gd::kTeal) : (on ? gd::kTeal : gd::kWhite);
        const bool dark = on || c.action == Action::Select;
        const std::string label = chipLabel(app, c);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        text(app, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, label, dark ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }

    std::string last = "last move: none";
    if (app.lastMove >= 0) {
        const psm::Move& m = psm::kMovesTable[static_cast<std::size_t>(app.lastMove)];
        last = std::string("last move: ") + m.name + "  adds";
        for (std::size_t i = 0; i < m.update.size(); ++i) {
            if (m.update[i] > 0.0) last += std::string("  ") + psm::kStyleLetters[i] + " +" + num(m.update[i], 1);
        }
    }
    text(app, 24.0F, 432.0F, last, gd::kInk);
    text(app, 24.0F, 454.0F, "keys: 1-5 move   S select rule   F forget   N new player   E chapter example", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("play-style-model", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c);
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            if (event->key.repeat) break;  // every key plays a move or flips a switch, none should fire while held
            const SDL_Keycode key = event->key.key;
            if (key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            if (key >= SDLK_1 && key < SDLK_1 + psm::kMoves) {
                play(app, static_cast<int>(key - SDLK_1));
                break;
            }
            for (const Chip& c : app.chips) {
                if ((key == SDLK_S && c.action == Action::Select) || (key == SDLK_F && c.action == Action::Forget) || (key == SDLK_N && c.action == Action::NewPlayer) ||
                    (key == SDLK_E && c.action == Action::Chapter)) {
                    apply(app, c);
                }
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
    const double dt = std::min(kMaxDt, static_cast<double>(now - app.lastTicks) / 1000.0);
    app.lastTicks = now;
    update(app, dt);
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
