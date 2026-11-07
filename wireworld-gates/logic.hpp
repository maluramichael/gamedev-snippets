// Wireworld: four cell states and one counting rule, no SDL in here so test.cpp can run it natively.
//
//   empty -> empty
//   head  -> tail
//   tail  -> wire
//   wire  -> head if exactly 1 or 2 of the 8 neighbours are heads, else wire
//
// Two ways to get it wrong are built in as rules so the demo can show them: "AnyHead" fires a wire on one or more
// head neighbours (nothing is ever blocked, so there are no diodes and no gates), "InPlace" updates the grid in one
// buffer so later cells see the new state of earlier ones (signals depend on the scan direction).
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace ww {

enum class Cell : std::uint8_t { Empty = 0, Wire = 1, Head = 2, Tail = 3 };
enum class Rule { Wireworld, AnyHead, InPlace };

inline Cell nextState(Cell s, int heads, Rule rule) {
    switch (s) {
        case Cell::Head: return Cell::Tail;
        case Cell::Tail: return Cell::Wire;
        case Cell::Wire: {
            const bool fires = rule == Rule::AnyHead ? heads >= 1 : (heads == 1 || heads == 2);
            return fires ? Cell::Head : Cell::Wire;
        }
        case Cell::Empty: break;
    }
    return Cell::Empty;
}

class Grid {
public:
    Grid(int w, int h) : w_(w), h_(h), cells_(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), Cell::Empty), scratch_(cells_) {}

    int width() const { return w_; }
    int height() const { return h_; }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
    Cell at(int x, int y) const { return inside(x, y) ? cells_[index(x, y)] : Cell::Empty; }
    void set(int x, int y, Cell c) {
        if (inside(x, y)) cells_[index(x, y)] = c;
    }

    int headNeighbors(int x, int y) const {
        int n = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if ((dx != 0 || dy != 0) && at(x + dx, y + dy) == Cell::Head) ++n;
            }
        }
        return n;
    }

    void step(Rule rule) {
        if (rule == Rule::InPlace) {
            // The bug on purpose: neighbours that were already visited this pass show their new state.
            for (int y = 0; y < h_; ++y) {
                for (int x = 0; x < w_; ++x) cells_[index(x, y)] = nextState(cells_[index(x, y)], headNeighbors(x, y), rule);
            }
            return;
        }
        for (int y = 0; y < h_; ++y) {
            for (int x = 0; x < w_; ++x) scratch_[index(x, y)] = nextState(cells_[index(x, y)], headNeighbors(x, y), rule);
        }
        cells_.swap(scratch_);
    }

    /// What the mouse does: wire and erase go anywhere, a head only goes onto something that conducts. A head in an empty
    /// cell would decay to tail and then to wire, i.e. draw wire out of nothing.
    void paint(int x, int y, Cell c) {
        if (c == Cell::Head && at(x, y) == Cell::Empty) return;
        set(x, y, c);
    }

    void clear() { std::fill(cells_.begin(), cells_.end(), Cell::Empty); }

    /// Heads and tails become plain wire again: the circuit without any electrons in it.
    void quench() {
        for (Cell& c : cells_) {
            if (c == Cell::Head || c == Cell::Tail) c = Cell::Wire;
        }
    }

    std::size_t count(Cell c) const { return static_cast<std::size_t>(std::count(cells_.begin(), cells_.end(), c)); }
    bool quiet() const { return count(Cell::Head) == 0 && count(Cell::Tail) == 0; }

    /// Straight line of one brush from (x0,y0) to (x1,y1), both ends included (Bresenham), so a fast mouse leaves no gaps.
    void line(int x0, int y0, int x1, int y1, Cell c) {
        const int dx = std::abs(x1 - x0);
        const int dy = -std::abs(y1 - y0);
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        for (;;) {
            paint(x0, y0, c);
            if (x0 == x1 && y0 == y1) break;
            const int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

private:
    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x); }

    int w_;
    int h_;
    std::vector<Cell> cells_;
    std::vector<Cell> scratch_;
};

// ---------------------------------------------------------------------------------------------------------------
// Circuits: ASCII art with ports. The demo is the "test bench": at the start of every period it injects one electron
// into each input cell whose bit is set, and it looks for electrons at the output cells.
//
//   ' ' or '.'   empty              '#'  wire              'H' / 'T'  head / tail already in the circuit
//   a..z         input cell: a wire where the bench injects a head (not h, t)
//   A..Z         tail partner of the input with the same letter, so the electron runs away from it (not H, T)
//   1..9         output probe: a wire where a head means "the circuit said 1"
// ---------------------------------------------------------------------------------------------------------------

struct Port {
    int x = 0;
    int y = 0;
};

struct Ports {
    std::vector<std::vector<Port>> heads;  // per logical input: where to put heads
    std::vector<std::vector<Port>> tails;  // per logical input: where to put the matching tails
    std::vector<Port> probes;              // per output
};

struct InputDef {
    std::string name;     // "A"
    std::string letters;  // art letters that belong to this input, e.g. "ac" when one bit feeds two cells
};

struct Circuit {
    std::string id;
    std::string label;                     // chip text
    std::string title;                     // shown above the table
    std::vector<std::string> notes;        // short lines under the table
    std::vector<std::string> art;
    std::vector<InputDef> inputs;
    std::vector<std::string> outputNames;  // probes '1', '2', ... in this order
    unsigned (*expect)(unsigned) = nullptr;  // input bits -> output bits, nullptr for circuits without a truth table
    int period = 0;                          // steps between two injections, 0 = nothing is injected
    unsigned startBits = 0;                  // which inputs are switched on when the preset loads
};

struct Placed {
    Grid grid;
    Ports ports;
};

inline int artWidth(const Circuit& c) {
    std::size_t w = 0;
    for (const std::string& row : c.art) w = std::max(w, row.size());
    return static_cast<int>(w);
}

inline int artHeight(const Circuit& c) { return static_cast<int>(c.art.size()); }

/// Draws the circuit centred (rounded down) into a fresh fieldW x fieldH grid and resolves the port letters.
inline Placed place(const Circuit& c, int fieldW, int fieldH) {
    Placed p{Grid(fieldW, fieldH), {}};
    const int ox = std::max(0, (fieldW - artWidth(c)) / 2);
    const int oy = std::max(0, (fieldH - artHeight(c)) / 2);
    p.ports.heads.resize(c.inputs.size());
    p.ports.tails.resize(c.inputs.size());
    p.ports.probes.resize(c.outputNames.size());
    for (int y = 0; y < artHeight(c); ++y) {
        const std::string& row = c.art[static_cast<std::size_t>(y)];
        for (int x = 0; x < static_cast<int>(row.size()); ++x) {
            const char ch = row[static_cast<std::size_t>(x)];
            const int gx = ox + x;
            const int gy = oy + y;
            if (ch == '#') {
                p.grid.set(gx, gy, Cell::Wire);
            } else if (ch == 'H') {
                p.grid.set(gx, gy, Cell::Head);
            } else if (ch == 'T') {
                p.grid.set(gx, gy, Cell::Tail);
            } else if (ch >= '1' && ch <= '9') {
                p.grid.set(gx, gy, Cell::Wire);
                const auto i = static_cast<std::size_t>(ch - '1');
                if (i < p.ports.probes.size()) p.ports.probes[i] = {gx, gy};
            } else if (ch >= 'a' && ch <= 'z') {
                p.grid.set(gx, gy, Cell::Wire);
                for (std::size_t i = 0; i < c.inputs.size(); ++i) {
                    if (c.inputs[i].letters.find(ch) != std::string::npos) p.ports.heads[i].push_back({gx, gy});
                }
            } else if (ch >= 'A' && ch <= 'Z') {
                p.grid.set(gx, gy, Cell::Wire);
                const char lower = static_cast<char>(ch - 'A' + 'a');
                for (std::size_t i = 0; i < c.inputs.size(); ++i) {
                    if (c.inputs[i].letters.find(lower) != std::string::npos) p.ports.tails[i].push_back({gx, gy});
                }
            }
        }
    }
    return p;
}

/// One electron into every input whose bit is set (bit i = input i). A port cell that is no longer wire (erased, say) is skipped.
inline void inject(Grid& g, const Ports& ports, unsigned bits) {
    for (std::size_t i = 0; i < ports.heads.size(); ++i) {
        if (((bits >> i) & 1U) == 0U) continue;
        for (const Port& p : ports.heads[i]) {
            if (g.at(p.x, p.y) == Cell::Wire) g.set(p.x, p.y, Cell::Head);
        }
        for (const Port& p : ports.tails[i]) {
            if (g.at(p.x, p.y) == Cell::Wire) g.set(p.x, p.y, Cell::Tail);
        }
    }
}

struct RunResult {
    std::vector<int> pulses;  // heads seen at each probe over the run
    bool quiet = true;        // no electron left anywhere at the end
};

/// Runs the circuit once from a clean state: all electrons removed, one injection at step 0, `steps` steps.
inline RunResult runInputs(const Grid& base, const Ports& ports, unsigned bits, int steps, Rule rule) {
    Grid g = base;
    g.quench();
    inject(g, ports, bits);
    RunResult r;
    r.pulses.assign(ports.probes.size(), 0);
    for (int t = 0; t < steps; ++t) {
        g.step(rule);
        for (std::size_t i = 0; i < ports.probes.size(); ++i) {
            if (g.at(ports.probes[i].x, ports.probes[i].y) == Cell::Head) ++r.pulses[i];
        }
    }
    r.quiet = g.quiet();
    return r;
}

struct Row {
    unsigned in = 0;
    unsigned out = 0;   // bit i = probe i saw a head
    unsigned want = 0;
    bool clean = true;  // no probe saw more than one head
    bool quiet = true;
    bool ok() const { return out == want && clean; }
};

/// The measured truth table: every input combination run on its own copy, compared with the expected function.
inline std::vector<Row> truthTable(const Grid& base, const Circuit& c, const Ports& ports, Rule rule) {
    std::vector<Row> rows;
    if (c.expect == nullptr || c.inputs.empty()) return rows;
    const unsigned combos = 1U << c.inputs.size();
    for (unsigned in = 0; in < combos; ++in) {
        const RunResult r = runInputs(base, ports, in, c.period, rule);
        Row row;
        row.in = in;
        row.want = c.expect(in);
        row.quiet = r.quiet;
        for (std::size_t i = 0; i < r.pulses.size(); ++i) {
            if (r.pulses[i] > 0) row.out |= 1U << i;
            if (r.pulses[i] > 1) row.clean = false;
        }
        rows.push_back(row);
    }
    return rows;
}

/// Steps between two heads at the probe, measured on a copy of the grid as it is now. 0 when the probe fires fewer than
/// three times or not at a steady rhythm within maxSteps.
inline int measurePeriod(const Grid& base, Port probe, int maxSteps, Rule rule) {
    Grid g = base;
    std::vector<int> hits;
    for (int t = 1; t <= maxSteps; ++t) {
        g.step(rule);
        if (g.at(probe.x, probe.y) == Cell::Head) hits.push_back(t);
    }
    if (hits.size() < 3) return 0;
    const int d1 = hits[hits.size() - 1] - hits[hits.size() - 2];
    const int d2 = hits[hits.size() - 2] - hits[hits.size() - 3];
    return d1 == d2 ? d1 : 0;
}

// ---------------------------------------------------------------------------------------------------------------
// The live machine: the grid plus the bench that injects every `period` steps.
// ---------------------------------------------------------------------------------------------------------------

struct Machine {
    Grid grid;
    Ports ports;
    Rule rule = Rule::Wireworld;
    unsigned bits = 0;     // the input switches
    int period = 0;        // 0 = no bench
    int phase = 0;         // steps since the last injection round
    long generation = 0;
    bool primed = false;  // the first round was injected by startRound, so tick() must not inject it again
    std::vector<long> lastPulse;  // generation of the last head at each probe, -1 = never

    Machine(Grid g, Ports p, int periodSteps) : grid(std::move(g)), ports(std::move(p)), period(periodSteps), lastPulse(ports.probes.size(), -1) {}

    void tick() {
        if (period > 0) {
            if (phase == 0) {
                if (primed) {
                    primed = false;
                } else {
                    inject(grid, ports, bits);
                }
            }
            phase = (phase + 1) % period;
        }
        grid.step(rule);
        ++generation;
        for (std::size_t i = 0; i < ports.probes.size(); ++i) {
            if (grid.at(ports.probes[i].x, ports.probes[i].y) == Cell::Head) lastPulse[i] = generation;
        }
    }

    /// Puts the first round's electrons in right away, so the very first frame already shows signals on the way.
    void startRound() {
        if (period <= 0) return;
        inject(grid, ports, bits);
        phase = 0;
        primed = true;
    }

    /// After the switches or the rule changed: wipe the electrons and the lamps and begin a fresh round with the next tick.
    void restartRound() {
        grid.quench();
        phase = 0;
        primed = false;
        std::fill(lastPulse.begin(), lastPulse.end(), -1L);
    }
};

}  // namespace ww
