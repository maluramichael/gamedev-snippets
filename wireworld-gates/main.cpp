// Wireworld with SDL3 main callbacks.
//
// Four cell states and one counting rule are enough for wires, diodes and logic gates. The field is a circuit you can
// paint on; the presets (clock, diode, OR, XOR, AND, half adder) are ordinary cells, nothing is special-cased. For the
// gates the demo plays the test bench: at the start of every period it injects one electron into each input that is
// switched on, and the table on the right is measured, not stored: every input combination runs on a copy of the
// current field. The two rules marked "bug" are the usual implementation mistakes, and the table shows what they do to
// the gates. The cell logic and the circuits live in logic.hpp and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "circuits.hpp"
#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCell = 12;
constexpr int kCols = 60;
constexpr int kRows = 33;
constexpr float kFieldX = 0.0F;
constexpr float kFieldY = 34.0F;
constexpr float kFieldW = static_cast<float>(kCols * kCell);
constexpr float kFieldH = static_cast<float>(kRows * kCell);
constexpr float kPanelX = 732.0F;
constexpr float kPanelW = 216.0F;
constexpr float kChipH = 34.0F;
constexpr std::array<int, 3> kSpeeds{6, 20, 60};  // steps per second
constexpr int kMaxStepsPerFrame = 8;
constexpr double kMaxFrameTime = 0.25;
constexpr int kLampSteps = 8;  // how long an output lamp stays lit after a head passed the probe

enum class Action { Preset, Clear, RunPause, Step, Speed, Tool, Rule };

struct Chip {
    Action action;
    int arg;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::vector<SDL_FRect> tableRows;  // filled while drawing, used for clicks

    ww::Machine machine{ww::Grid(kCols, kRows), ww::Ports{}, 0};
    const ww::Circuit* circuit = nullptr;
    std::size_t presetIndex = 0;
    bool edited = false;  // the field differs from the loaded preset
    std::vector<ww::Row> table;
    int clockPeriod = 0;
    bool dirty = true;  // table / clock period need to be measured again

    ww::Cell brush = ww::Cell::Wire;
    bool running = true;
    int speed = 1;
    double accumulator = 0.0;
    Uint64 lastNs = 0;

    bool painting = false;
    ww::Cell stroke = ww::Cell::Wire;
    int lastX = 0;
    int lastY = 0;

    long maxFrames = -1;
    long frames = 0;
};

const std::vector<ww::Circuit>& presets() { return ww::circuits(); }

void refresh(App& app) {
    app.dirty = false;
    app.table.clear();
    app.clockPeriod = 0;
    if (app.circuit == nullptr) return;
    if (app.circuit->expect != nullptr) {
        app.table = ww::truthTable(app.machine.grid, *app.circuit, app.machine.ports, app.machine.rule);
    } else if (!app.machine.ports.probes.empty()) {
        app.clockPeriod = ww::measurePeriod(app.machine.grid, app.machine.ports.probes[0], 400, app.machine.rule);
    }
}

void resetClock(App& app) {
    app.accumulator = 0.0;
    app.lastNs = SDL_GetTicksNS();
}

void loadPreset(App& app, std::size_t index, int bits = -1) {
    if (index >= presets().size()) return;
    const ww::Circuit& c = presets()[index];
    ww::Placed p = ww::place(c, kCols, kRows);
    const ww::Rule rule = app.machine.rule;
    app.machine = ww::Machine(std::move(p.grid), std::move(p.ports), c.period);
    app.machine.rule = rule;
    app.machine.bits = bits >= 0 ? static_cast<unsigned>(bits) : c.startBits;
    app.circuit = &c;
    app.presetIndex = index;
    app.edited = false;
    app.painting = false;
    app.dirty = true;
    app.machine.startRound();
    resetClock(app);
}

void clearField(App& app) {
    const ww::Rule rule = app.machine.rule;
    app.machine = ww::Machine(ww::Grid(kCols, kRows), ww::Ports{}, 0);
    app.machine.rule = rule;
    app.circuit = nullptr;
    app.edited = false;
    app.painting = false;
    app.dirty = true;
    resetClock(app);
}

void setRule(App& app, ww::Rule rule) {
    app.machine.rule = rule;
    app.dirty = true;
    if (app.circuit != nullptr && !app.edited) {
        loadPreset(app, app.presetIndex, static_cast<int>(app.machine.bits));  // an untouched preset starts over under the new rule, a dead clock comes back
    } else if (app.machine.period > 0) {
        app.machine.restartRound();
    }
}

/// The inputs changed: a fresh round, so electrons of the old setting cannot meet the new ones.
void setInputs(App& app, unsigned bits) {
    app.machine.bits = bits;
    if (app.machine.period > 0) app.machine.restartRound();
}

void cycleRule(App& app) {
    const int next = (static_cast<int>(app.machine.rule) + 1) % 3;
    setRule(app, static_cast<ww::Rule>(next));
}

void toggleInput(App& app, std::size_t i) {
    if (app.circuit == nullptr || i >= app.circuit->inputs.size()) return;
    setInputs(app, app.machine.bits ^ (1U << i));
}

void apply(App& app, Action action, int arg) {
    switch (action) {
        case Action::Preset: loadPreset(app, static_cast<std::size_t>(arg)); break;
        case Action::Clear: clearField(app); break;
        case Action::RunPause:
            app.running = !app.running;
            resetClock(app);
            break;
        case Action::Step:
            app.running = false;
            app.machine.tick();
            break;
        case Action::Speed:
            app.speed = arg;
            resetClock(app);
            break;
        case Action::Tool: app.brush = static_cast<ww::Cell>(arg); break;
        case Action::Rule: setRule(app, static_cast<ww::Rule>(arg)); break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Preset: return app.circuit != nullptr && app.circuit == &presets()[static_cast<std::size_t>(c.arg)];
        case Action::Speed: return app.speed == c.arg;
        case Action::Tool: return static_cast<int>(app.brush) == c.arg;
        case Action::Rule: return static_cast<int>(app.machine.rule) == c.arg;
        case Action::RunPause: return !app.running;
        default: return false;
    }
}

std::string chipLabel(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Preset: return std::to_string(c.arg + 1) + " " + presets()[static_cast<std::size_t>(c.arg)].label;
        case Action::Clear: return "C Clear";
        case Action::RunPause: return app.running ? "Space Pause" : "Space Run";
        case Action::Step: return "N Step";
        case Action::Speed: return std::to_string(kSpeeds[static_cast<std::size_t>(c.arg)]) + "/s";
        case Action::Tool:
            if (c.arg == static_cast<int>(ww::Cell::Wire)) return "W Wire";
            if (c.arg == static_cast<int>(ww::Cell::Head)) return "H Head";
            return "E Erase";
        case Action::Rule:
            if (c.arg == static_cast<int>(ww::Rule::Wireworld)) return "1 or 2 heads";
            if (c.arg == static_cast<int>(ww::Rule::AnyHead)) return ">= 1 head (bug)";
            return "in place (bug)";
    }
    return "";
}

void addChip(App& app, Action action, int arg, float x, float y, float w, float h = kChipH) { app.chips.push_back({action, arg, SDL_FRect{x, y, w, h}}); }

float chipWidth(const App& app, Action action, int arg) {
    const Chip probe{action, arg, SDL_FRect{}};
    return app.hud.width(chipLabel(app, probe), gd::Hud::Style::Bold) + 24.0F;
}

void layoutChips(App& app) {
    // Row 1: the presets and Clear.
    float x = 24.0F;
    const float y1 = 444.0F;
    for (std::size_t i = 0; i < presets().size(); ++i) {
        const float w = chipWidth(app, Action::Preset, static_cast<int>(i));
        addChip(app, Action::Preset, static_cast<int>(i), x, y1, w);
        x += w + 8.0F;
    }
    x += 16.0F;
    addChip(app, Action::Clear, 0, x, y1, chipWidth(app, Action::Clear, 0));

    // Row 2: run controls, speed, brushes.
    x = 24.0F;
    const float y2 = 486.0F;
    const float runW = app.hud.width("Space Pause", gd::Hud::Style::Bold) + 24.0F;
    addChip(app, Action::RunPause, 0, x, y2, runW);
    x += runW + 8.0F;
    addChip(app, Action::Step, 0, x, y2, chipWidth(app, Action::Step, 0));
    x += chipWidth(app, Action::Step, 0) + 24.0F;
    for (int i = 0; i < static_cast<int>(kSpeeds.size()); ++i) {
        const float w = chipWidth(app, Action::Speed, i);
        addChip(app, Action::Speed, i, x, y2, w);
        x += w + 8.0F;
    }
    x += 16.0F;
    for (const ww::Cell c : {ww::Cell::Wire, ww::Cell::Head, ww::Cell::Empty}) {
        const float w = chipWidth(app, Action::Tool, static_cast<int>(c));
        addChip(app, Action::Tool, static_cast<int>(c), x, y2, w);
        x += w + 8.0F;
    }

    // The rule chips stack in the side panel.
    float y = 96.0F;
    for (const ww::Rule r : {ww::Rule::Wireworld, ww::Rule::AnyHead, ww::Rule::InPlace}) {
        addChip(app, Action::Rule, static_cast<int>(r), kPanelX + 6.0F, y, kPanelW - 12.0F, 28.0F);
        y += 33.0F;
    }
}

gd::Color cellColor(ww::Cell c) {
    switch (c) {
        case ww::Cell::Wire: return gd::kInk;
        case ww::Cell::Head: return gd::kOrange;
        case ww::Cell::Tail: return gd::kTeal;
        case ww::Cell::Empty: break;
    }
    return gd::kWhite;
}

bool fieldCell(float px, float py, int& cx, int& cy) {
    if (px < kFieldX || py < kFieldY || px >= kFieldX + kFieldW || py >= kFieldY + kFieldH) return false;
    cx = static_cast<int>((px - kFieldX) / static_cast<float>(kCell));
    cy = static_cast<int>((py - kFieldY) / static_cast<float>(kCell));
    return true;
}

void paintAt(App& app, int cx, int cy) {
    app.edited = true;
    app.machine.grid.line(app.lastX, app.lastY, cx, cy, app.stroke);
    app.lastX = cx;
    app.lastY = cy;
}

bool nothingIn(const App& app, float x, float y, float w, float h) {
    // True when the text box lies inside the field and covers no cell (2 px of air around it).
    if (x < kFieldX + 2.0F || y < kFieldY + 2.0F || x + w > kFieldX + kFieldW - 2.0F || y + h > kFieldY + kFieldH - 2.0F) return false;
    const int x0 = static_cast<int>((x - 2.0F - kFieldX) / static_cast<float>(kCell));
    const int x1 = static_cast<int>((x + w + 2.0F - kFieldX) / static_cast<float>(kCell));
    const int y0 = static_cast<int>((y - 2.0F - kFieldY) / static_cast<float>(kCell));
    const int y1 = static_cast<int>((y + h + 2.0F - kFieldY) / static_cast<float>(kCell));
    for (int cy = y0; cy <= y1; ++cy) {
        for (int cx = x0; cx <= x1; ++cx) {
            if (app.machine.grid.at(cx, cy) != ww::Cell::Empty) return false;
        }
    }
    return true;
}

/// Puts a label next to a cell where it covers nothing: right, left, above, below, in that order.
void labelCell(App& app, ww::Port p, const std::string& text, gd::Color color) {
    const float tw = app.hud.width(text, gd::Hud::Style::Bold);
    const float th = app.hud.lineHeight(gd::Hud::Style::Bold);
    const float cx = kFieldX + static_cast<float>(p.x * kCell);
    const float cy = kFieldY + static_cast<float>(p.y * kCell);
    const std::array<std::pair<float, float>, 4> spots{{{cx + 18.0F, cy + 5.0F - th / 2.0F},
                                                        {cx - 8.0F - tw, cy + 5.0F - th / 2.0F},
                                                        {cx + 5.0F - tw / 2.0F, cy - 8.0F - th},
                                                        {cx + 5.0F - tw / 2.0F, cy + 18.0F}}};
    for (const auto& [x, y] : spots) {
        if (nothingIn(app, x, y, tw, th)) {
            app.hud.text(app.renderer, x, y, text, color, gd::Hud::Style::Bold);
            return;
        }
    }
}

void drawField(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, kFieldX, kFieldY, kFieldW, kFieldH, gd::kWhite);
    gd::setColor(r, gd::kGrid);
    for (int x = 0; x <= kCols; ++x) SDL_RenderLine(r, kFieldX + static_cast<float>(x * kCell), kFieldY, kFieldX + static_cast<float>(x * kCell), kFieldY + kFieldH);
    for (int y = 0; y <= kRows; ++y) SDL_RenderLine(r, kFieldX, kFieldY + static_cast<float>(y * kCell), kFieldX + kFieldW, kFieldY + static_cast<float>(y * kCell));

    const ww::Grid& g = app.machine.grid;
    for (int y = 0; y < kRows; ++y) {
        for (int x = 0; x < kCols; ++x) {
            const ww::Cell c = g.at(x, y);
            if (c == ww::Cell::Empty) continue;
            gd::fillRect(r, kFieldX + static_cast<float>(x * kCell) + 1.0F, kFieldY + static_cast<float>(y * kCell) + 1.0F, static_cast<float>(kCell - 1), static_cast<float>(kCell - 1), cellColor(c));
        }
    }

    if (app.circuit == nullptr) return;
    // Output probes: a ring that glows for a few steps after a head passed.
    const auto& probes = app.machine.ports.probes;
    for (std::size_t i = 0; i < probes.size(); ++i) {
        const float cx = kFieldX + static_cast<float>(probes[i].x * kCell) + 5.0F;
        const float cy = kFieldY + static_cast<float>(probes[i].y * kCell) + 5.0F;
        const bool lit = app.machine.lastPulse[i] >= 0 && app.machine.generation - app.machine.lastPulse[i] < kLampSteps;
        if (lit) gd::ring(r, cx, cy, 13.0F, gd::kOrange);
        gd::ring(r, cx, cy, 10.0F, lit ? gd::kOrange : gd::kMuted);
        labelCell(app, probes[i], app.circuit->outputNames[i], lit ? gd::kOrange : gd::kMuted);
    }
    // Inputs: the letter of the switch next to every cell the bench injects into.
    for (std::size_t i = 0; i < app.machine.ports.heads.size(); ++i) {
        const bool on = ((app.machine.bits >> i) & 1U) != 0U;
        for (const ww::Port& p : app.machine.ports.heads[i]) labelCell(app, p, app.circuit->inputs[i].name, on ? gd::kTealDark : gd::kMuted);
    }
}

void drawChip(App& app, const Chip& c) {
    SDL_Renderer* r = app.renderer;
    const bool on = active(app, c);
    const std::string label = chipLabel(app, c);
    gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
    app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, label, on ? gd::kWhite : gd::kInk,
                 gd::Hud::Style::Bold);
}

std::string bit(unsigned v, std::size_t i) { return ((v >> i) & 1U) != 0U ? "1" : "0"; }

void drawTable(App& app, float y) {
    SDL_Renderer* r = app.renderer;
    const ww::Circuit& c = *app.circuit;
    const float x0 = kPanelX + 12.0F;
    constexpr float kInW = 24.0F;
    // Header.
    float x = x0;
    for (const ww::InputDef& in : c.inputs) {
        app.hud.text(r, x, y, in.name, gd::kTealDark, gd::Hud::Style::Bold);
        x += kInW;
    }
    x += 10.0F;
    std::vector<float> outX;
    for (const std::string& name : c.outputNames) {
        outX.push_back(x);
        app.hud.text(r, x, y, name, gd::kMuted, gd::Hud::Style::Bold);
        x += app.hud.width(name, gd::Hud::Style::Bold) + 10.0F;
    }
    const float statusX = kPanelX + kPanelW - 48.0F;
    y += 26.0F;
    app.tableRows.clear();
    for (const ww::Row& row : app.table) {
        const bool current = row.in == app.machine.bits;
        const SDL_FRect rect{kPanelX + 4.0F, y - 2.0F, kPanelW - 8.0F, 24.0F};
        app.tableRows.push_back(rect);
        if (current) gd::fillRect(r, rect.x, rect.y, rect.w, rect.h, gd::kTealSoft);
        float cx = x0;
        for (std::size_t i = 0; i < c.inputs.size(); ++i) {
            app.hud.text(r, cx, y, bit(row.in, i), gd::kInk, gd::Hud::Style::Bold);
            cx += kInW;
        }
        for (std::size_t i = 0; i < outX.size(); ++i) {
            const bool right = ((row.out >> i) & 1U) == ((row.want >> i) & 1U);
            app.hud.text(r, outX[i], y, bit(row.out, i), right ? gd::kInk : gd::kOrange, gd::Hud::Style::Bold);
        }
        app.hud.text(r, statusX, y, row.ok() ? "ok" : "FAIL", row.ok() ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
        y += 24.0F;
    }
}

void drawPanel(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 44.0F, kPanelW, 382.0F, gd::kWhite, gd::kInk);
    // The three cell colors.
    float x = kPanelX + 12.0F;
    const std::array<std::pair<ww::Cell, const char*>, 3> legend{{{ww::Cell::Wire, "wire"}, {ww::Cell::Head, "head"}, {ww::Cell::Tail, "tail"}}};
    for (const auto& [cell, name] : legend) {
        gd::fillRect(r, x, 56.0F, 11.0F, 11.0F, cellColor(cell));
        app.hud.text(r, x + 16.0F, 50.0F, name, gd::kInk);
        x += 16.0F + app.hud.width(name) + 12.0F;
    }
    app.hud.text(r, kPanelX + 12.0F, 72.0F, "RULE (R)", gd::kMuted, gd::Hud::Style::Bold);

    // The rule chips are drawn with the other chips. Below them: what the circuit measures.
    float y = 200.0F;
    if (app.circuit != nullptr) {
        app.hud.text(r, kPanelX + 12.0F, y, app.circuit->title, gd::kInk, gd::Hud::Style::Bold);
        y += 26.0F;
        if (!app.table.empty()) {
            drawTable(app, y);
            y += 26.0F + 24.0F * static_cast<float>(app.table.size()) + 6.0F;
        } else if (app.circuit->expect == nullptr && !app.machine.ports.probes.empty()) {
            char line[48];
            if (app.clockPeriod > 0) {
                SDL_snprintf(line, sizeof line, "period %d steps", app.clockPeriod);
            } else {
                SDL_snprintf(line, sizeof line, "no steady pulse");
            }
            app.hud.text(r, kPanelX + 12.0F, y, line, app.clockPeriod > 0 ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
            y += 30.0F;
        }
        for (const std::string& note : app.circuit->notes) {
            app.hud.text(r, kPanelX + 12.0F, y, note, gd::kMuted);
            y += 20.0F;
        }
    } else {
        app.hud.text(r, kPanelX + 12.0F, y, "FREE DRAWING", gd::kInk, gd::Hud::Style::Bold);
        app.hud.text(r, kPanelX + 12.0F, y + 28.0F, "left mouse paints", gd::kMuted);
        app.hud.text(r, kPanelX + 12.0F, y + 48.0F, "right mouse erases", gd::kMuted);
        app.hud.text(r, kPanelX + 12.0F, y + 76.0F, "Head puts an", gd::kMuted);
        app.hud.text(r, kPanelX + 12.0F, y + 96.0F, "electron on a wire", gd::kMuted);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    const char* rule = app.machine.rule == ww::Rule::Wireworld ? "1-2 heads" : (app.machine.rule == ww::Rule::AnyHead ? ">=1 head" : "in place");
    if (app.machine.period > 0) {
        SDL_snprintf(right, sizeof right, "step %ld   inject in %d   rule: %s", app.machine.generation, app.machine.period - app.machine.phase, rule);
    } else {
        SDL_snprintf(right, sizeof right, "step %ld   rule: %s", app.machine.generation, rule);
    }
    app.hud.titleBar(r, kWidth, "[ WIREWORLD ]", right);
    drawField(app);
    drawPanel(app);
    for (const Chip& c : app.chips) drawChip(app, c);
}

void advance(App& app) {
    const Uint64 now = SDL_GetTicksNS();
    double dt = static_cast<double>(now - app.lastNs) * 1e-9;
    app.lastNs = now;
    dt = std::clamp(dt, 0.0, kMaxFrameTime);
    if (!app.running) return;
    app.accumulator += dt;
    const double stepTime = 1.0 / static_cast<double>(kSpeeds[static_cast<std::size_t>(app.speed)]);
    int steps = 0;
    while (app.accumulator >= stepTime && steps < kMaxStepsPerFrame) {
        app.machine.tick();
        app.accumulator -= stepTime;
        ++steps;
    }
    if (steps == kMaxStepsPerFrame) app.accumulator = 0.0;  // the machine is too slow for the speed: drop the backlog instead of bursting
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
    if (!SDL_CreateWindowAndRenderer("wireworld-gates", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    layoutChips(*app);
    loadPreset(*app, ww::kStartPreset);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            if (!app.painting) break;
            // Clamped to the field, so a stroke that leaves the field keeps its end on the border.
            const float px = std::clamp(event->motion.x, kFieldX, kFieldX + kFieldW - 1.0F);
            const float py = std::clamp(event->motion.y, kFieldY, kFieldY + kFieldH - 1.0F);
            int cx = 0;
            int cy = 0;
            if (fieldCell(px, py, cx, cy)) paintAt(app, cx, cy);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            if (event->button.button != SDL_BUTTON_LEFT && event->button.button != SDL_BUTTON_RIGHT) break;
            if (event->button.button == SDL_BUTTON_LEFT) {
                bool onChip = false;
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) {
                        apply(app, c.action, c.arg);
                        onChip = true;
                        break;
                    }
                }
                if (onChip) break;
                for (std::size_t i = 0; i < app.tableRows.size(); ++i) {
                    if (SDL_PointInRectFloat(&p, &app.tableRows[i]) && i < app.table.size()) {
                        setInputs(app, app.table[i].in);
                    }
                }
            }
            int cx = 0;
            int cy = 0;
            if (fieldCell(p.x, p.y, cx, cy)) {
                app.painting = true;
                app.stroke = event->button.button == SDL_BUTTON_RIGHT ? ww::Cell::Empty : app.brush;
                app.lastX = cx;
                app.lastY = cy;
                paintAt(app, cx, cy);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (app.painting) {
                app.painting = false;
                app.dirty = true;
            }
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            if (app.painting) {
                app.painting = false;
                app.dirty = true;
            }
            break;
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode k = event->key.key;
            if (k == SDLK_N) {  // stepping is the one key that may repeat
                apply(app, Action::Step, 0);
                break;
            }
            if (event->key.repeat) break;
            if (k == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            if (k >= SDLK_1 && k < SDLK_1 + static_cast<SDL_Keycode>(presets().size())) {
                apply(app, Action::Preset, static_cast<int>(k - SDLK_1));
                break;
            }
            switch (k) {
                case SDLK_C: apply(app, Action::Clear, 0); break;
                case SDLK_SPACE: apply(app, Action::RunPause, 0); break;
                case SDLK_W: apply(app, Action::Tool, static_cast<int>(ww::Cell::Wire)); break;
                case SDLK_H: apply(app, Action::Tool, static_cast<int>(ww::Cell::Head)); break;
                case SDLK_E: apply(app, Action::Tool, static_cast<int>(ww::Cell::Empty)); break;
                case SDLK_R: cycleRule(app); break;
                case SDLK_A: toggleInput(app, 0); break;
                case SDLK_B: toggleInput(app, 1); break;
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
    advance(app);
    if (app.dirty && !app.painting) refresh(app);
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
