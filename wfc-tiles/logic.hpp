// Wave function collapse (simple tiled model): the logic of the demo, no SDL, so test.cpp can check it.
//
// Every grid cell holds a SET of tiles that are still possible (a bit mask, at most 32 tiles). Two rules drive it:
//
//   1. Collapse: pick the undecided cell with the lowest entropy and reduce its set to one tile, chosen by weight.
//   2. Propagate: a tile may only stay in a cell if every neighbour still has a tile that fits next to it. Whenever a
//      set shrinks, its neighbours are checked again, until nothing changes.
//
// "Fits next to it" is a table built once from the edge labels of the tiles (compat[side][tile]). A cell whose set
// becomes empty is a contradiction. The solver then either throws the whole map away (restart) or undoes the last
// decision and bans the tile it picked (backtrack).
//
// The solver works in atomic steps (one propagation pop, one collapse, one recovery), so the demo can run it at
// any speed and the loop is bounded: restarts and backtracks are capped, see maxSteps().
#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

namespace wfc {

constexpr int kMaxTiles = 32;
constexpr int kSides = 4;  // north, east, south, west (clockwise)
enum Side : int { North = 0, East = 1, South = 2, West = 3 };
constexpr int opposite(int side) { return (side + 2) % kSides; }
inline constexpr std::array<int, kSides> kDx{0, 1, 0, -1};
inline constexpr std::array<int, kSides> kDy{-1, 0, 1, 0};

using Mask = std::uint32_t;
constexpr Mask bit(int i) { return Mask{1} << i; }
inline int count(Mask m) { return std::popcount(m); }
inline int lowest(Mask m) { return std::countr_zero(m); }

/// Edge labels of the land tileset. Two tiles may touch only if the labels on the touching edges are equal.
enum Label : int { Grass = 0, Water = 1, Road = 2 };

struct Tile {
    std::array<int, kSides> edge;  // label per side
    double weight;                 // how often this tile is picked among the tiles that are still possible
};

struct TileSet {
    std::vector<Tile> tiles;
    std::array<std::vector<Mask>, kSides> compat;  // compat[side][t] = tiles that may sit on that side of tile t
    std::vector<double> wlogw;                     // weight * ln(weight), for the entropy
    Mask all = 0;

    const Tile& tile(int t) const { return tiles[static_cast<std::size_t>(t)]; }
    /// The tiles that may sit on `side` of tile t.
    Mask fits(int side, int t) const { return compat[static_cast<std::size_t>(side)][static_cast<std::size_t>(t)]; }

    /// Fills compat, wlogw and all from tiles. False if the set is empty, too big or has a weight <= 0.
    bool build() {
        const int n = static_cast<int>(tiles.size());
        if (n == 0 || n > kMaxTiles) return false;
        for (const Tile& t : tiles) {
            if (!(t.weight > 0.0)) return false;
        }
        all = n == kMaxTiles ? ~Mask{0} : bit(n) - 1;
        wlogw.assign(tiles.size(), 0.0);
        for (int t = 0; t < n; ++t) wlogw[static_cast<std::size_t>(t)] = tile(t).weight * std::log(tile(t).weight);
        for (int s = 0; s < kSides; ++s) {
            compat[s].assign(tiles.size(), 0);
            for (int t = 0; t < n; ++t) {
                for (int u = 0; u < n; ++u) {
                    // u fits on side s of t if the label it shows towards t equals the label t shows towards u.
                    if (tile(t).edge[s] == tile(u).edge[opposite(s)]) compat[s][static_cast<std::size_t>(t)] |= bit(u);
                }
            }
        }
        return true;
    }
};

/// Shannon entropy (natural log) of the weighted choice among the tiles in `mask`: ln(S) - sum(w ln w) / S with
/// S = sum of the weights. 0 for one tile (and for none).
inline double entropy(const TileSet& set, Mask mask) {
    if (count(mask) < 2) return 0.0;
    double sum = 0.0;
    double wlw = 0.0;
    for (Mask m = mask; m != 0; m &= m - 1) {
        const int t = lowest(m);
        sum += set.tile(t).weight;
        wlw += set.wlogw[static_cast<std::size_t>(t)];
    }
    return std::max(0.0, std::log(sum) - wlw / sum);
}

/// The tile for a random number r in [0, 1): the weights lay side by side on a line of length S, r*S lands on one.
/// -1 for an empty mask. The last tile also catches the rounding at r close to 1.
inline int pickWeighted(const TileSet& set, Mask mask, double r) {
    if (mask == 0) return -1;
    double total = 0.0;
    for (Mask m = mask; m != 0; m &= m - 1) total += set.tile(lowest(m)).weight;
    const double target = std::clamp(r, 0.0, 1.0) * total;
    double acc = 0.0;
    int last = -1;
    for (Mask m = mask; m != 0; m &= m - 1) {
        last = lowest(m);
        acc += set.tile(last).weight;
        if (target < acc) return last;
    }
    return last;
}

/// splitmix64: tiny, the same numbers on every platform, so a seed is a map.
struct Rng {
    std::uint64_t state;
    explicit Rng(std::uint64_t seed = 1) : state(seed) {}
    std::uint64_t next() {
        std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    /// [0, 1) with 53 bits.
    double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
};

enum class Order { Entropy, Random, Scanline };
enum class Recovery { Restart, Backtrack };
enum class Status { Running, Contradiction, Done, GaveUp };

struct Limits {
    int maxRestarts = 40;    // after this many restarts the solver gives up instead of trying again
    int maxBacktracks = 200;  // per attempt; more than that and the attempt is restarted
};

struct Counters {
    long steps = 0;
    int contradictions = 0;
    int restarts = 0;
    int backtracks = 0;
};

class Solver {
public:
    const TileSet* set;
    int width;
    int height;
    Order order;
    Recovery recovery;
    Limits limits;
    bool trackTouched = false;  // the demo wants to know which cells changed; tests do not

    std::vector<Mask> domain;
    std::deque<int> queue;
    std::vector<char> queued;
    std::vector<double> noise;  // one random number per cell and attempt: tie-break and the random order
    Rng rng;
    Status status = Status::Running;
    int badCell = -1;  // the cell whose set ran empty
    Counters counters;
    int attemptBacktracks = 0;
    std::vector<int> touched;  // cells whose set changed, filled only when trackTouched

    struct Snapshot {
        std::vector<Mask> domain;
        std::deque<int> queue;
        int cell;
        int tile;
    };
    std::vector<Snapshot> stack;

    Solver(const TileSet& tileSet, int w, int h, std::uint64_t seed, Order o, Recovery r, Limits l = {})
        : set(&tileSet), width(w), height(h), order(o), recovery(r), limits(l), rng(seed) {
        reset(seed);
    }

    int cells() const { return width * height; }
    Mask& at(int cell) { return domain[static_cast<std::size_t>(cell)]; }
    Mask at(int cell) const { return domain[static_cast<std::size_t>(cell)]; }

    /// A new map from a seed. Counters start at 0.
    void reset(std::uint64_t seed) {
        rng = Rng(seed);
        counters = {};
        startAttempt();
    }

    bool finished() const { return status == Status::Done || status == Status::GaveUp; }

    int collapsedCount() const {
        int n = 0;
        for (Mask m : domain) n += count(m) == 1 ? 1 : 0;
        return n;
    }

    /// One atomic action: a recovery after a contradiction, one propagation pop, or one collapse.
    Status step() {
        if (finished()) return status;
        ++counters.steps;
        if (status == Status::Contradiction) {
            recover();
            return status;
        }
        if (!queue.empty()) {
            propagateOne();
            return status;
        }
        const int cell = pickCell();
        if (cell < 0) {
            status = Status::Done;
            return status;
        }
        collapse(cell, pickWeighted(*set, at(cell), rng.uniform()));
        return status;
    }

    /// Steps until the solver is finished or `maxSteps` steps are spent. Returns the steps taken.
    long run(long maxSteps) {
        long taken = 0;
        while (!finished() && taken < maxSteps) {
            step();
            ++taken;
        }
        return taken;
    }

    /// Upper bound of steps() for a solver that runs to Done or GaveUp. Every pop belongs to a change of a set (at most
    /// cells * tiles between two restores) or to an enqueue by a collapse, a backtrack or a restart.
    long maxSteps() const {
        const long n = cells();
        const long tiles = static_cast<long>(set->tiles.size());
        const long b = recovery == Recovery::Backtrack ? limits.maxBacktracks : 0;
        const long perAttempt = (b + 1) * (n * (tiles + 2) + 2) + b + 2;
        return (limits.maxRestarts + 1L) * perAttempt;
    }

    /// The cell the next collapse would take, -1 if every cell has one tile. Pure: same state, same answer.
    int pickCell() const {
        int best = -1;
        double bestKey = std::numeric_limits<double>::infinity();
        for (int i = 0; i < cells(); ++i) {
            const Mask m = at(i);
            if (count(m) < 2) continue;
            const double tiebreak = noise[static_cast<std::size_t>(i)];
            double key = 0.0;
            switch (order) {
                case Order::Entropy: key = entropy(*set, m) + tiebreak * 1e-6; break;
                case Order::Random: key = tiebreak; break;
                case Order::Scanline: key = static_cast<double>(i); break;
            }
            if (key < bestKey) {
                bestKey = key;
                best = i;
            }
        }
        return best;
    }

    /// A click: collapse this cell now. Only while running and only if the cell is still undecided.
    bool collapseCell(int cell) {
        if (status != Status::Running || cell < 0 || cell >= cells()) return false;
        const Mask m = at(cell);
        if (count(m) < 2) return false;
        collapse(cell, pickWeighted(*set, m, rng.uniform()));
        return true;
    }

    /// Every cell has exactly one tile and every pair of neighbours agrees on the touching edge.
    bool valid() const {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const Mask m = at(y * width + x);
                if (count(m) != 1) return false;
                const Tile& a = set->tile(lowest(m));
                for (int s = 1; s <= 2; ++s) {  // east and south, the other two sides are covered by the neighbours
                    const int nx = x + kDx[static_cast<std::size_t>(s)];
                    const int ny = y + kDy[static_cast<std::size_t>(s)];
                    if (nx >= width || ny >= height) continue;
                    const Mask nm = at(ny * width + nx);
                    if (count(nm) != 1) return false;
                    const Tile& b = set->tile(lowest(nm));
                    if (a.edge[static_cast<std::size_t>(s)] != b.edge[static_cast<std::size_t>(opposite(s))]) return false;
                }
            }
        }
        return true;
    }

private:
    void touch(int cell) {
        if (trackTouched) touched.push_back(cell);
    }

    void enqueue(int cell) {
        if (queued[static_cast<std::size_t>(cell)] == 0) {
            queued[static_cast<std::size_t>(cell)] = 1;
            queue.push_back(cell);
        }
    }

    void clearQueue() {
        queue.clear();
        std::fill(queued.begin(), queued.end(), char{0});
    }

    void contradict(int cell) {
        status = Status::Contradiction;
        badCell = cell;
        ++counters.contradictions;
        clearQueue();
    }

    /// New attempt: every cell can be every tile, then the initial propagation (a tile without any possible
    /// neighbour on a side can go right away). No step is counted for it, it is the same for every map.
    void startAttempt() {
        const auto n = static_cast<std::size_t>(cells());
        domain.assign(n, set->all);
        queued.assign(n, 0);
        queue.clear();
        stack.clear();
        attemptBacktracks = 0;
        status = Status::Running;
        badCell = -1;
        noise.resize(n);
        for (double& v : noise) v = rng.uniform();
        for (int i = 0; i < cells(); ++i) enqueue(i);
        while (!queue.empty() && status == Status::Running) propagateOne();
    }

    /// Takes one cell from the queue and makes its four neighbours consistent with it.
    void propagateOne() {
        const int c = queue.front();
        queue.pop_front();
        queued[static_cast<std::size_t>(c)] = 0;
        const int cx = c % width;
        const int cy = c / width;
        const Mask here = at(c);
        for (int s = 0; s < kSides; ++s) {
            const int nx = cx + kDx[static_cast<std::size_t>(s)];
            const int ny = cy + kDy[static_cast<std::size_t>(s)];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            Mask allowed = 0;  // everything the neighbour may be, given what this cell may still be
            for (Mask m = here; m != 0; m &= m - 1) allowed |= set->fits(s, lowest(m));
            const int n = ny * width + nx;
            const Mask next = at(n) & allowed;
            if (next == at(n)) continue;
            at(n) = next;
            touch(n);
            if (next == 0) {
                contradict(n);
                return;
            }
            enqueue(n);
        }
    }

    void collapse(int cell, int tile) {
        if (recovery == Recovery::Backtrack) stack.push_back({domain, queue, cell, tile});
        at(cell) = bit(tile);
        touch(cell);
        enqueue(cell);
    }

    /// After a contradiction: undo the last decision and ban its tile, or start over, or give up.
    void recover() {
        if (recovery == Recovery::Backtrack && !stack.empty() && attemptBacktracks < limits.maxBacktracks) {
            backtrackOnce();
            return;
        }
        if (counters.restarts >= limits.maxRestarts) {
            status = Status::GaveUp;
            return;
        }
        ++counters.restarts;
        startAttempt();
    }

    void backtrackOnce() {
        Snapshot s = std::move(stack.back());
        stack.pop_back();
        if (trackTouched) {
            for (std::size_t i = 0; i < domain.size(); ++i) {
                if (domain[i] != s.domain[i]) touched.push_back(static_cast<int>(i));
            }
        }
        domain = std::move(s.domain);
        queue = std::move(s.queue);
        std::fill(queued.begin(), queued.end(), char{0});
        for (int c : queue) queued[static_cast<std::size_t>(c)] = 1;
        ++counters.backtracks;
        ++attemptBacktracks;
        at(s.cell) &= ~bit(s.tile);
        status = Status::Running;
        badCell = -1;
        if (at(s.cell) == 0) {
            contradict(s.cell);  // no tile left for that decision: the one before it was wrong as well
            return;
        }
        touch(s.cell);
        enqueue(s.cell);
    }
};

struct Bench {
    int maps = 0;
    int contradictions = 0;
    int gaveUp = 0;
    long steps = 0;
};

/// `maps` maps with the restart strategy, each from its own seed. Counts the contradictions the order runs into.
inline Bench benchmark(const TileSet& set, int w, int h, Order order, int maps, std::uint64_t seed) {
    Bench b;
    for (int i = 0; i < maps; ++i) {
        Solver s(set, w, h, seed + 7919ULL * static_cast<std::uint64_t>(i), order, Recovery::Restart);
        s.run(s.maxSteps());
        ++b.maps;
        b.contradictions += s.counters.contradictions;
        b.steps += s.counters.steps;
        if (s.status == Status::GaveUp) ++b.gaveUp;
    }
    return b;
}

/// The land tileset, 29 tiles. Edge labels: grass, water, road.
///   16 ground tiles: every combination of water and grass on the four edges (all grass ... all water).
///   11 road tiles: roads on 2, 3 or 4 edges (no dead ends, so the road has to go somewhere).
///    2 bridges: a road across a water edge pair.
/// `tuned` weights make grass common, water rarer and roads rare. Without it all 29 tiles are equally likely.
inline TileSet landTileSet(bool tuned) {
    TileSet set;
    auto add = [&](std::array<int, kSides> edges, double weight) { set.tiles.push_back({edges, tuned ? weight : 1.0}); };
    for (int m = 0; m < 16; ++m) {  // bit s set = water on side s
        std::array<int, kSides> e{};
        int water = 0;
        for (int s = 0; s < kSides; ++s) {
            e[static_cast<std::size_t>(s)] = (m >> s & 1) != 0 ? Water : Grass;
            water += m >> s & 1;
        }
        double w = 1.0;
        if (water == 0) w = 14.0;
        else if (water == 1) w = 0.8;
        else if (water == 2) w = (m == 5 || m == 10) ? 0.4 : 0.8;  // opposite sides = a river piece
        else if (water == 3) w = 1.0;
        else w = 4.0;
        add(e, w);
    }
    for (int m = 1; m < 16; ++m) {  // bit s set = road on side s
        const int roads = std::popcount(static_cast<unsigned>(m));
        if (roads < 2) continue;
        std::array<int, kSides> e{};
        for (int s = 0; s < kSides; ++s) e[static_cast<std::size_t>(s)] = (m >> s & 1) != 0 ? Road : Grass;
        double w = 0.12;
        if (roads == 2) w = (m == 5 || m == 10) ? 0.6 : 0.35;
        else if (roads == 3) w = 0.12;
        else w = 0.06;
        add(e, w);
    }
    add({Water, Road, Water, Road}, 0.3);  // bridge running east-west
    add({Road, Water, Road, Water}, 0.3);  // bridge running north-south
    set.build();
    return set;
}

}  // namespace wfc
