// Wave function collapse (simple tiled model) with three levels of propagation: the logic of the demo, no SDL, so
// test.cpp can check it.
//
// Every grid cell holds a SET of tiles that are still possible (a bit mask, at most 32 tiles). The solver picks an
// undecided cell, decides it (one tile, chosen by weight) and then has to tell the rest of the map about it. How much it
// tells is the whole subject of this demo:
//
//   Off        Nobody is told. The sets of the open cells stay "all tiles". A cell only looks at its decided neighbours
//              at the moment the solver gets to it, and if nothing fits it is a contradiction.
//   Neighbors  The four neighbours of a decided cell lose the tiles that do not fit it. Nothing travels further.
//   Full       A neighbour that lost tiles tells its own neighbours, and so on until nothing changes (arc consistency,
//              the AC-3 idea that Karth and Smith recognise in WFC).
//
// A cell whose set becomes empty (Neighbors, Full) or that has no tile left that fits its decided neighbours (Off)
// is a contradiction. The solver then starts over: the map is thrown away. The number of restarts is capped.
//
// The solver works in atomic steps (one propagation pop or one decision or one recovery), so the demo can run it at any
// speed and the loop is bounded: see maxSteps().
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

enum class Order { Entropy, Scanline, Random };
enum class Propagation { Off, Neighbors, Full };
enum class Status { Running, Contradiction, Done, GaveUp };

struct Limits {
    int maxRestarts = 40;  // after this many restarts the solver gives up instead of trying again
};

struct Counters {
    long steps = 0;
    long decisions = 0;     // cells decided by the solver or by a click
    long narrowed = 0;      // times propagation took tiles out of a set
    int lastCascade = 0;    // sets narrowed since the last decision
    int contradictions = 0;
    int restarts = 0;
};

class Solver {
public:
    const TileSet* set;
    int width;
    int height;
    Order order;
    Propagation level;
    Limits limits;
    bool trackTouched = false;  // the demo wants to know which cells changed; tests do not
    bool singletonsSpeak = false;  // test variant of Neighbors: a cell reduced to one tile passes its news on at once

    std::vector<Mask> domain;
    std::deque<int> queue;
    std::vector<char> queued;
    std::vector<double> noise;  // one random number per cell and attempt: tie-break and the random order
    Rng rng;
    Status status = Status::Running;
    int badCell = -1;  // the cell that ran out of tiles
    Counters counters;
    std::vector<int> touched;  // cells whose set changed, filled only when trackTouched

    Solver(const TileSet& tileSet, int w, int h, std::uint64_t seed, Order o, Propagation p, Limits l = {})
        : set(&tileSet), width(w), height(h), order(o), level(p), limits(l), rng(seed) {
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

    /// The tiles a cell could still take if somebody looked. With propagation that is its stored set. Without it the
    /// stored set says "all tiles", the truth is only found by comparing with the decided neighbours.
    Mask options(int cell) const {
        const Mask stored = at(cell);
        if (level != Propagation::Off || count(stored) == 1) return stored;
        Mask m = stored;
        const int cx = cell % width;
        const int cy = cell / width;
        for (int s = 0; s < kSides; ++s) {
            const int nx = cx + kDx[static_cast<std::size_t>(s)];
            const int ny = cy + kDy[static_cast<std::size_t>(s)];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            const Mask neighbour = at(ny * width + nx);
            // The neighbour on side s is decided: this cell has to be on its opposite side.
            if (count(neighbour) == 1) m &= set->fits(opposite(s), lowest(neighbour));
        }
        return m;
    }

    /// Open cells that cannot take any tile any more, although nothing has noticed yet. Always 0 with propagation.
    int doomedCount() const {
        int n = 0;
        for (int i = 0; i < cells(); ++i) {
            if (count(at(i)) >= 2 && options(i) == 0) ++n;
        }
        return n;
    }

    /// One atomic action: a recovery after a contradiction, one propagation pop, or one decision.
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
        stand(cell);
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

    /// Upper bound of steps for a solver that runs to Done or GaveUp. Every pop belongs to a change of a set (at most
    /// cells * tiles between two restarts) or to an enqueue by a decision, and every attempt adds its decisions and a
    /// recovery step.
    long maxSteps() const {
        const long n = cells();
        const long tiles = static_cast<long>(set->tiles.size());
        const long perAttempt = n * (tiles + 2) + 2;
        return (limits.maxRestarts + 1L) * perAttempt;
    }

    /// The cell the next decision would take, -1 if every cell has one tile. Pure: same state, same answer.
    /// The entropy is read from the stored sets: without propagation they are all equal, and the noise decides.
    int pickCell() const {
        int best = -1;
        double bestKey = std::numeric_limits<double>::infinity();
        for (int i = 0; i < cells(); ++i) {
            const Mask m = at(i);
            if (count(m) == 1) continue;
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

    /// A click: decide this cell now, the tile is drawn by weight. Only while running and only if the cell is open.
    /// A cell that has no tile left shows its contradiction.
    bool collapseCell(int cell) {
        if (status != Status::Running || cell < 0 || cell >= cells() || count(at(cell)) == 1) return false;
        stand(cell);
        return true;
    }

    /// Decide a cell for a given tile. False unless the cell is open and the tile is one of its options.
    bool assign(int cell, int tile) {
        if (status != Status::Running || cell < 0 || cell >= cells() || count(at(cell)) == 1) return false;
        if (tile < 0 || tile >= static_cast<int>(set->tiles.size()) || (options(cell) >> tile & 1U) == 0) return false;
        decide(cell, tile);
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

    /// Arc consistency: every tile in every set has a partner in the set of each neighbour.
    bool consistent() const {
        for (int c = 0; c < cells(); ++c) {
            const int cx = c % width;
            const int cy = c / width;
            for (int s = 0; s < kSides; ++s) {
                const int nx = cx + kDx[static_cast<std::size_t>(s)];
                const int ny = cy + kDy[static_cast<std::size_t>(s)];
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                const Mask other = at(ny * width + nx);
                for (Mask m = at(c); m != 0; m &= m - 1) {
                    if ((set->fits(s, lowest(m)) & other) == 0) return false;
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

    /// New attempt: every cell can be every tile. With the full cascade the initial propagation runs first (a tile
    /// without any possible neighbour on a side can go right away). No step is counted for it, it is the same for every map.
    void startAttempt() {
        const auto n = static_cast<std::size_t>(cells());
        domain.assign(n, set->all);
        queued.assign(n, 0);
        queue.clear();
        status = Status::Running;
        badCell = -1;
        noise.resize(n);
        for (double& v : noise) v = rng.uniform();
        if (level != Propagation::Full) return;
        for (int i = 0; i < cells(); ++i) enqueue(i);
        while (!queue.empty() && status == Status::Running) propagateOne();
    }

    /// The solver gets to a cell: nothing fits it (contradiction) or one of its options is drawn by weight.
    void stand(int cell) {
        const Mask opts = options(cell);
        if (opts == 0) {
            at(cell) = 0;
            touch(cell);
            contradict(cell);
            return;
        }
        decide(cell, pickWeighted(*set, opts, rng.uniform()));
    }

    void decide(int cell, int tile) {
        ++counters.decisions;
        counters.lastCascade = 0;
        at(cell) = bit(tile);
        touch(cell);
        if (level == Propagation::Full) {
            enqueue(cell);
        } else if (level == Propagation::Neighbors) {
            narrowNeighbors(cell, bit(tile), false);
        }
    }

    /// Takes one cell from the queue and makes its four neighbours consistent with it.
    void propagateOne() {
        const int c = queue.front();
        queue.pop_front();
        queued[static_cast<std::size_t>(c)] = 0;
        narrowNeighbors(c, at(c), true);
    }

    /// Takes from each neighbour of `cell` every tile that does not fit any tile of `here`. A neighbour that changed
    /// is queued when `cascade` is set. False if a neighbour ran empty.
    bool narrowNeighbors(int cell, Mask here, bool cascade) {
        const int cx = cell % width;
        const int cy = cell / width;
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
            ++counters.narrowed;
            ++counters.lastCascade;
            if (next == 0) {
                contradict(n);
                return false;
            }
            if (cascade) enqueue(n);
            else if (singletonsSpeak && count(next) == 1 && !narrowNeighbors(n, next, false)) return false;
        }
        return true;
    }

    /// After a contradiction: start over, or give up.
    void recover() {
        if (counters.restarts >= limits.maxRestarts) {
            status = Status::GaveUp;
            return;
        }
        ++counters.restarts;
        startAttempt();
    }
};

struct Bench {
    int maps = 0;
    int contradictions = 0;
    int gaveUp = 0;
    int clean = 0;  // maps that were finished without a single contradiction
    long steps = 0;
};

/// The seed of map `index` in a benchmark that starts at `base`: the demo (which runs the maps one by one over many frames)
/// and benchmark() must draw the same maps.
inline std::uint64_t benchmarkSeed(std::uint64_t base, int index) { return base + 7919ULL * static_cast<std::uint64_t>(index); }

/// One map of a benchmark: a solver from `seed`, run to the end, added to `b`.
inline void benchmarkMap(Bench& b, const TileSet& set, int w, int h, Order order, Propagation level, std::uint64_t seed, Limits limits = {}) {
    Solver s(set, w, h, seed, order, level, limits);
    s.run(s.maxSteps());
    ++b.maps;
    b.contradictions += s.counters.contradictions;
    b.steps += s.counters.steps;
    if (s.status == Status::GaveUp) ++b.gaveUp;
    if (s.status == Status::Done && s.counters.contradictions == 0) ++b.clean;
}

/// `maps` maps, each from its own seed, with the same order and level. Counts the contradictions they run into.
inline Bench benchmark(const TileSet& set, int w, int h, Order order, Propagation level, int maps, std::uint64_t seed, Limits limits = {}) {
    Bench b;
    for (int i = 0; i < maps; ++i) {
        benchmarkMap(b, set, w, h, order, level, benchmarkSeed(seed, i), limits);
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
