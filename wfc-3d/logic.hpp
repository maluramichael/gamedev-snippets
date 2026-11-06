// Wave function collapse in three dimensions: the logic of the demo, no SDL and no GL, so test.cpp can check it.
//
// The algorithm is the one from the 2D demo, only the neighbour list is longer. Every grid cell holds a SET of tiles that
// are still possible (a bit mask). Two steps drive it:
//
//   1. Collapse: pick the undecided cell with the lowest entropy and reduce its set to one tile, chosen by weight.
//   2. Propagate: a tile may only stay in a cell if every neighbour still has a tile that fits next to it. Whenever a
//      set shrinks, the neighbours of that cell are checked again, until nothing changes.
//
// What changes with the dimension is only `kOffsets`: four neighbours in 2D, six in 3D. "Fits next to it" is a table
// built once from a rule function, compat[dir][tile] = the tiles that may sit on that side of the tile. A cell whose
// set becomes empty is a contradiction; the solver then restarts the whole attempt or undoes the last decision.
//
// The solver works in atomic steps (one propagation pop, one collapse, one recovery), so the demo can run it at any
// speed and the loop is bounded, see Solver::maxSteps().
//
// This header also holds the small amount of camera and picking maths the demo needs, because a wrong ray or a wrong
// matrix is as invisible in the code as a wrong rule. Conventions: right-handed, y up, matrices row-major m[row][col]
// with column vectors (v' = M * v).
#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace w3 {

// ---------------------------------------------------------------------------------------------- directions

constexpr int kDirCount = 6;
enum Dir : int { PosX = 0, NegX = 1, PosY = 2, NegY = 3, PosZ = 4, NegZ = 5 };

struct Int3 {
    int x = 0, y = 0, z = 0;
    bool operator==(const Int3&) const = default;
};

/// The neighbour list. The 2D demo has four entries here, this one has six; nothing else in the solver knows the dimension.
inline constexpr std::array<Int3, kDirCount> kOffsets{{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};

/// Directions come in pairs (+x -x, +y -y, +z -z), so the opposite side is the same index with the lowest bit flipped.
constexpr int opposite(int dir) { return dir ^ 1; }
constexpr bool isHorizontal(int dir) { return dir != PosY && dir != NegY; }

// ---------------------------------------------------------------------------------------------- masks

using Mask = std::uint64_t;
constexpr int kMaxTiles = 64;
constexpr Mask bit(int i) { return Mask{1} << i; }
inline int count(Mask m) { return std::popcount(m); }
inline int lowest(Mask m) { return std::countr_zero(m); }

// ---------------------------------------------------------------------------------------------- tile set

/// The tiles as the solver sees them: a weight each, a table of what fits where, and what may touch the outside of the
/// grid. What a tile looks like is not its business.
struct TileSet {
    using FitFn = std::function<bool(int a, int dir, int b)>;  // may tile b sit on side `dir` of tile a?
    using EdgeFn = std::function<bool(int a, int dir)>;        // may tile a have the outside of the grid on side `dir`?

    std::vector<double> weight;                           // how often a tile is picked among the tiles still possible
    std::vector<char> solid;                              // 0 for air: only used to show how much is still to come
    std::array<std::vector<Mask>, kDirCount> compat;      // compat[dir][t] = tiles that may sit on that side of tile t
    std::array<Mask, kDirCount> edgeOk{};                 // edgeOk[dir] = tiles allowed in a cell with the outside on that side
    std::vector<double> wlogw;                            // weight * ln(weight), for the entropy
    Mask all = 0;

    int size() const { return static_cast<int>(weight.size()); }
    /// The tiles that may sit on `dir` of tile t.
    Mask fits(int dir, int t) const { return compat[static_cast<std::size_t>(dir)][static_cast<std::size_t>(t)]; }
    bool isSolid(int t) const { return solid[static_cast<std::size_t>(t)] != 0; }

    /// Fills everything from the weights and the two rule functions. False if the set is empty, too big or has a weight <= 0.
    bool build(std::vector<double> weights, const FitFn& fit, const EdgeFn& edge) {
        const int n = static_cast<int>(weights.size());
        if (n == 0 || n > kMaxTiles) return false;
        for (double w : weights) {
            if (!(w > 0.0)) return false;
        }
        weight = std::move(weights);
        solid.assign(weight.size(), 1);
        all = n == kMaxTiles ? ~Mask{0} : bit(n) - 1;
        wlogw.assign(weight.size(), 0.0);
        for (int t = 0; t < n; ++t) wlogw[static_cast<std::size_t>(t)] = weight[static_cast<std::size_t>(t)] * std::log(weight[static_cast<std::size_t>(t)]);
        for (int d = 0; d < kDirCount; ++d) {
            compat[static_cast<std::size_t>(d)].assign(weight.size(), 0);
            edgeOk[static_cast<std::size_t>(d)] = 0;
            for (int t = 0; t < n; ++t) {
                if (edge(t, d)) edgeOk[static_cast<std::size_t>(d)] |= bit(t);
                for (int u = 0; u < n; ++u) {
                    if (fit(t, d, u)) compat[static_cast<std::size_t>(d)][static_cast<std::size_t>(t)] |= bit(u);
                }
            }
        }
        return true;
    }

    /// u fits on side d of t exactly when t fits on the opposite side of u. The solver only looks forward, so a rule that
    /// breaks this would leave pairs that nobody checks from the other side.
    bool symmetric() const {
        for (int d = 0; d < kDirCount; ++d) {
            for (int t = 0; t < size(); ++t) {
                for (int u = 0; u < size(); ++u) {
                    const bool forward = (fits(d, t) >> u & 1) != 0;
                    const bool backward = (fits(opposite(d), u) >> t & 1) != 0;
                    if (forward != backward) return false;
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
        const auto t = static_cast<std::size_t>(lowest(m));
        sum += set.weight[t];
        wlw += set.wlogw[t];
    }
    return std::max(0.0, std::log(sum) - wlw / sum);
}

/// Share of the weight in `mask` that belongs to solid tiles (0 .. 1). The demo draws undecided cells the bigger, the
/// more likely something solid still appears there.
inline double solidShare(const TileSet& set, Mask mask) {
    double all = 0.0;
    double solid = 0.0;
    for (Mask m = mask; m != 0; m &= m - 1) {
        const int t = lowest(m);
        all += set.weight[static_cast<std::size_t>(t)];
        if (set.isSolid(t)) solid += set.weight[static_cast<std::size_t>(t)];
    }
    return all > 0.0 ? solid / all : 0.0;
}

/// The tile for a random number r in [0, 1): the weights lay side by side on a line of length S, r*S lands on one.
/// -1 for an empty mask. The last tile also catches the rounding at r close to 1.
inline int pickWeighted(const TileSet& set, Mask mask, double r) {
    if (mask == 0) return -1;
    double total = 0.0;
    for (Mask m = mask; m != 0; m &= m - 1) total += set.weight[static_cast<std::size_t>(lowest(m))];
    const double target = std::clamp(r, 0.0, 1.0) * total;
    double acc = 0.0;
    int last = -1;
    for (Mask m = mask; m != 0; m &= m - 1) {
        last = lowest(m);
        acc += set.weight[static_cast<std::size_t>(last)];
        if (target < acc) return last;
    }
    return last;
}

/// splitmix64: tiny, the same numbers on every platform, so a seed is a world.
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

// ---------------------------------------------------------------------------------------------- solver

/// Which undecided cell is collapsed next. Layers goes bottom up (lowest y first, entropy inside a layer): the world
/// grows floor by floor.
enum class Order { Entropy, Layers, Random };
enum class Recovery { Restart, Backtrack };
enum class Status { Running, Contradiction, Done, GaveUp };

struct Limits {
    int maxRestarts = 40;     // after this many restarts the solver gives up instead of trying again
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
    int width;   // x
    int height;  // y, up
    int depth;   // z
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
    int badCell = -1;        // the cell whose set ran empty
    int lastCollapsed = -1;  // the cell of the latest decision (for the flash in the demo)
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

    Solver(const TileSet& tileSet, int w, int h, int d, std::uint64_t seed, Order o, Recovery r, Limits l = {})
        : set(&tileSet), width(w), height(h), depth(d), order(o), recovery(r), limits(l), rng(seed) {
        reset(seed);
    }

    int cells() const { return width * height * depth; }
    /// Layers are contiguous: all cells of y = 0 first, then y = 1 and so on.
    int index(int x, int y, int z) const { return x + width * (z + depth * y); }
    Int3 coords(int cell) const { return {cell % width, cell / (width * depth), (cell / width) % depth}; }
    bool inside(int x, int y, int z) const { return x >= 0 && y >= 0 && z >= 0 && x < width && y < height && z < depth; }
    Mask& at(int cell) { return domain[static_cast<std::size_t>(cell)]; }
    Mask at(int cell) const { return domain[static_cast<std::size_t>(cell)]; }

    /// A new world from a seed. Counters start at 0.
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

    /// Upper bound of steps for a solver that runs to Done or GaveUp. Every pop belongs to a change of a set (at most
    /// cells * tiles between two restores) or to an enqueue by a collapse, a backtrack or a restart.
    long maxSteps() const {
        const long n = cells();
        const long tiles = set->size();
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
                case Order::Layers: key = static_cast<double>(coords(i).y) * 1000.0 + entropy(*set, m) + tiebreak * 1e-6; break;
                case Order::Random: key = tiebreak; break;
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

    /// Every cell has exactly one tile and every pair of neighbours is allowed by the table.
    bool valid() const {
        for (int i = 0; i < cells(); ++i) {
            const Mask m = at(i);
            if (count(m) != 1) return false;
            const Int3 c = coords(i);
            for (int d = 0; d < kDirCount; d += 2) {  // +x, +y, +z; the other three sides are covered by the neighbours
                const Int3 o = kOffsets[static_cast<std::size_t>(d)];
                const int nx = c.x + o.x, ny = c.y + o.y, nz = c.z + o.z;
                if (!inside(nx, ny, nz)) continue;
                const Mask nm = at(index(nx, ny, nz));
                if (count(nm) != 1 || (set->fits(d, lowest(m)) & nm) == 0) return false;
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

    /// New attempt: every cell can be every tile, except where the grid ends: a tile with an arm pointing out of the
    /// grid, or one that cannot stand on the ground, is banned there. Then the initial propagation. No step is counted
    /// for it, it is the same for every world.
    void startAttempt() {
        const auto n = static_cast<std::size_t>(cells());
        domain.assign(n, set->all);
        queued.assign(n, 0);
        queue.clear();
        stack.clear();
        attemptBacktracks = 0;
        status = Status::Running;
        badCell = -1;
        lastCollapsed = -1;
        noise.resize(n);
        for (double& v : noise) v = rng.uniform();
        int empty = -1;
        for (int i = 0; i < cells(); ++i) {
            const Int3 c = coords(i);
            for (int d = 0; d < kDirCount; ++d) {
                const Int3 o = kOffsets[static_cast<std::size_t>(d)];
                if (!inside(c.x + o.x, c.y + o.y, c.z + o.z)) at(i) &= set->edgeOk[static_cast<std::size_t>(d)];
            }
            if (at(i) == 0 && empty < 0) empty = i;
            enqueue(i);
        }
        if (empty >= 0) {
            contradict(empty);
            return;
        }
        while (!queue.empty() && status == Status::Running) propagateOne();
    }

    /// Takes one cell from the queue and makes its neighbours consistent with it.
    void propagateOne() {
        const int c = queue.front();
        queue.pop_front();
        queued[static_cast<std::size_t>(c)] = 0;
        const Int3 p = coords(c);
        const Mask here = at(c);
        for (int d = 0; d < kDirCount; ++d) {
            const Int3 o = kOffsets[static_cast<std::size_t>(d)];
            const int nx = p.x + o.x, ny = p.y + o.y, nz = p.z + o.z;
            if (!inside(nx, ny, nz)) continue;
            Mask allowed = 0;  // everything the neighbour may be, given what this cell may still be
            for (Mask m = here; m != 0; m &= m - 1) allowed |= set->fits(d, lowest(m));
            const int n = index(nx, ny, nz);
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
        lastCollapsed = cell;
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
    int worlds = 0;
    int contradictions = 0;
    int gaveUp = 0;
    int invalid = 0;  // finished worlds that fail Solver::valid(), must stay 0
    long steps = 0;
    long solidCells = 0;
    long cells = 0;
};

/// `worlds` worlds, each from its own seed. Counts what the order runs into.
inline Bench benchmark(const TileSet& set, int w, int h, int d, Order order, Recovery recovery, int worlds, std::uint64_t seed) {
    Bench b;
    for (int i = 0; i < worlds; ++i) {
        Solver s(set, w, h, d, seed + 7919ULL * static_cast<std::uint64_t>(i), order, recovery);
        s.run(s.maxSteps());
        ++b.worlds;
        b.contradictions += s.counters.contradictions;
        b.steps += s.counters.steps;
        if (s.status == Status::GaveUp) {
            ++b.gaveUp;
            continue;
        }
        if (!s.valid()) ++b.invalid;
        for (Mask m : s.domain) b.solidCells += set.isSolid(lowest(m)) ? 1 : 0;
        b.cells += s.cells();
    }
    return b;
}

// ---------------------------------------------------------------------------------------------- time and numbers

/// Steps for this frame from a rate in steps per second. The remainder stays in the accumulator, the frame time is
/// clamped (a stalled tab must not start a burst) and a frame never takes more than `cap` steps.
struct StepClock {
    static constexpr double kMaxFrame = 0.25;
    double acc = 0.0;

    int take(double frameSeconds, double rate, int cap) {
        if (!(frameSeconds > 0.0) || !(rate > 0.0) || cap <= 0) return 0;
        acc += std::min(frameSeconds, kMaxFrame) * rate;
        const double whole = std::floor(acc);
        const int n = whole >= static_cast<double>(cap) ? cap : static_cast<int>(whole);
        acc -= static_cast<double>(n);
        if (acc >= 1.0) acc -= std::floor(acc);  // steps this frame could not afford are dropped, not saved up
        return n;
    }
    void reset() { acc = 0.0; }
};

/// Exponential decay that does not depend on the frame rate: after one second the value is exp(-rate) of what it was.
inline float decay(float value, float rate, float seconds) { return value * std::exp(-rate * std::clamp(seconds, 0.0F, 0.25F)); }

/// 0 at t = 0, 1 at t = 1, overshoots to about 1.1 on the way (a tile pops in and settles).
inline float easeOutBack(float t) {
    const float x = std::clamp(t, 0.0F, 1.0F) - 1.0F;
    constexpr float c1 = 1.70158F;
    return 1.0F + (c1 + 1.0F) * x * x * x + c1 * x * x;
}

/// 12345 -> "12,345". Built by hand: snprintf with a locale prints a dot or a comma depending on the system.
inline std::string groupDigits(long v) {
    std::string digits = std::to_string(v < 0 ? -v : v);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i != 0 && (digits.size() - i) % 3 == 0) out.push_back(',');
        out.push_back(digits[i]);
    }
    return v < 0 ? "-" + out : out;
}

// ---------------------------------------------------------------------------------------------- vectors and matrices

struct Vec3 {
    float x = 0.0F, y = 0.0F, z = 0.0F;
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
/// Unit vector; the zero vector stays zero instead of becoming NaN.
inline Vec3 normalize(Vec3 a) {
    const float l = length(a);
    return l > 1e-12F ? a * (1.0F / l) : Vec3{};
}

struct Mat4 {
    float m[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float sum = 0.0F;
            for (int k = 0; k < 4; ++k) sum += a.m[i][k] * b.m[k][j];
            r.m[i][j] = sum;
        }
    }
    return r;
}

struct Vec4 {
    float x = 0.0F, y = 0.0F, z = 0.0F, w = 0.0F;
};

inline Vec4 transform(const Mat4& a, Vec3 v) {
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z + a.m[0][3], a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z + a.m[1][3],
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z + a.m[2][3], a.m[3][0] * v.x + a.m[3][1] * v.y + a.m[3][2] * v.z + a.m[3][3]};
}

inline Mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
    const float f = 1.0F / std::tan(fovY / 2.0F);
    Mat4 r;
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (zFar + zNear) / (zNear - zFar);
    r.m[2][3] = 2.0F * zFar * zNear / (zNear - zFar);
    r.m[3][2] = -1.0F;
    r.m[3][3] = 0.0F;
    return r;
}

/// Orthographic box, the sun's view of the world for the shadow map.
inline Mat4 ortho(float halfWidth, float halfHeight, float zNear, float zFar) {
    Mat4 r;
    r.m[0][0] = 1.0F / halfWidth;
    r.m[1][1] = 1.0F / halfHeight;
    r.m[2][2] = -2.0F / (zFar - zNear);
    r.m[2][3] = -(zFar + zNear) / (zFar - zNear);
    return r;
}

inline Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 f = normalize(target - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);
    Mat4 r;
    r.m[0][0] = s.x; r.m[0][1] = s.y; r.m[0][2] = s.z; r.m[0][3] = -dot(s, eye);
    r.m[1][0] = u.x; r.m[1][1] = u.y; r.m[1][2] = u.z; r.m[1][3] = -dot(u, eye);
    r.m[2][0] = -f.x; r.m[2][1] = -f.y; r.m[2][2] = -f.z; r.m[2][3] = dot(f, eye);
    return r;
}

// ---------------------------------------------------------------------------------------------- camera and picking

inline constexpr float kFovY = 0.62F;
inline constexpr float kNear = 0.5F;
inline constexpr float kFar = 120.0F;
inline constexpr float kMinDistance = 9.0F;
inline constexpr float kMaxDistance = 38.0F;
inline constexpr float kMinPitch = 0.08F;
inline constexpr float kMaxPitch = 1.42F;

struct Basis {
    Vec3 right, up, forward;
};

struct OrbitCamera {
    float yaw = 0.6F;
    float pitch = 0.55F;
    float distance = 30.0F;
    Vec3 target{0.0F, 1.0F, 0.0F};

    Vec3 eye() const { return target + Vec3{distance * std::cos(pitch) * std::sin(yaw), distance * std::sin(pitch), distance * std::cos(pitch) * std::cos(yaw)}; }
    Mat4 view() const { return lookAt(eye(), target, {0.0F, 1.0F, 0.0F}); }
    Basis basis() const {
        const Vec3 f = normalize(target - eye());
        const Vec3 r = normalize(cross(f, {0.0F, 1.0F, 0.0F}));
        return {r, cross(r, f), f};
    }
    void orbit(float dYaw, float dPitch) {
        yaw += dYaw;
        pitch = std::clamp(pitch + dPitch, kMinPitch, kMaxPitch);
    }
    /// One wheel notch moves the camera 10 % closer (negative notches move away).
    void zoom(float notches) { distance = std::clamp(distance * std::pow(0.9F, std::clamp(notches, -50.0F, 50.0F)), kMinDistance, kMaxDistance); }
};

struct Ray {
    Vec3 origin;
    Vec3 dir;  // unit length
};

/// The picture is pushed up by this much (in normalised device coordinates, 2 = the whole height) so that the island sits
/// in the free area above the chip panel instead of behind the middle of the canvas.
inline constexpr float kViewShift = 0.12F;

/// perspective() with the picture moved up by `shift`: the same transformation, only clip.y gets `shift * w` added.
inline Mat4 shiftedPerspective(float fovY, float aspect, float zNear, float zFar, float shift) {
    Mat4 r = perspective(fovY, aspect, zNear, zFar);
    r.m[1][2] = -shift;  // clip.y += -shift * z, and w = -z
    return r;
}

/// The ray through a point of the viewport: u and v run 0..1 from the top left corner, like the pixels of the canvas.
/// `shift` is the push of shiftedPerspective(): the ray goes through what that picture shows at (u, v).
inline Ray rayThrough(const OrbitCamera& cam, float u, float v, float aspect, float shift = 0.0F) {
    const Basis b = cam.basis();
    const float th = std::tan(kFovY / 2.0F);
    const float ndcX = u * 2.0F - 1.0F;
    const float ndcY = 1.0F - v * 2.0F - shift;
    return {cam.eye(), normalize(b.forward + b.right * (ndcX * th * aspect) + b.up * (ndcY * th))};
}

/// Slab test of a ray against an axis aligned box. True with the entry distance in `t` (0 if the origin is inside).
/// A ray parallel to a pair of faces is only inside the slab if its origin is between them, no division by zero.
inline bool rayBox(const Ray& r, Vec3 lo, Vec3 hi, float& t) {
    float tMin = 0.0F;
    float tMax = std::numeric_limits<float>::infinity();
    const float o[3] = {r.origin.x, r.origin.y, r.origin.z};
    const float d[3] = {r.dir.x, r.dir.y, r.dir.z};
    const float a[3] = {lo.x, lo.y, lo.z};
    const float b[3] = {hi.x, hi.y, hi.z};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(d[i]) < 1e-9F) {
            if (o[i] < a[i] || o[i] > b[i]) return false;
            continue;
        }
        float t1 = (a[i] - o[i]) / d[i];
        float t2 = (b[i] - o[i]) / d[i];
        if (t1 > t2) std::swap(t1, t2);
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax) return false;
    }
    t = tMin;
    return true;
}

// ---------------------------------------------------------------------------------------------- world space

/// Cells are one unit wide. The grid is centred on the origin in x and z, y = 0 is the ground.
inline Vec3 cellMin(int w, int d, int x, int y, int z) { return {static_cast<float>(x) - static_cast<float>(w) / 2.0F, static_cast<float>(y), static_cast<float>(z) - static_cast<float>(d) / 2.0F}; }
inline Vec3 cellBase(int w, int d, int x, int y, int z) { return cellMin(w, d, x, y, z) + Vec3{0.5F, 0.0F, 0.5F}; }  // middle of the bottom face

/// The undecided cell that the ray hits first, -1 if none. Decided cells are not pickable (nothing to collapse).
inline int pickUndecided(const Solver& s, const Ray& ray) {
    int best = -1;
    float bestT = std::numeric_limits<float>::infinity();
    for (int i = 0; i < s.cells(); ++i) {
        if (count(s.at(i)) < 2) continue;
        const Int3 c = s.coords(i);
        const Vec3 lo = cellMin(s.width, s.depth, c.x, c.y, c.z);
        float t = 0.0F;
        if (rayBox(ray, lo, lo + Vec3{1.0F, 1.0F, 1.0F}, t) && t < bestT) {
            bestT = t;
            best = i;
        }
    }
    return best;
}

// ---------------------------------------------------------------------------------------------- the chips

enum class Chip : int { New, Step, Auto, Slow, Medium, Fast, Max, Entropy, Layers, Random, Undo, Restart, SeedDown, SeedUp, Spin, Count };

struct ChipRect {
    Chip id;
    float x, y, w, h;
};

constexpr float kCharWidth = 10.0F;  // Hud regular and bold cells are 10 px wide
constexpr float kChipHeight = 26.0F;
constexpr float kChipGap = 6.0F;
constexpr float kChipPad = 9.0F;
constexpr float kCanvasWidth = 960.0F;
constexpr float kCanvasHeight = 540.0F;

inline float chipWidth(const char* label) { return static_cast<float>(std::char_traits<char>::length(label)) * kCharWidth + 2.0F * kChipPad; }

struct ChipSpec {
    Chip id;
    const char* label;
};

struct GroupLabel {
    const char* text;
    float x, y;
};

struct ChipLayout {
    std::vector<ChipRect> chips;
    std::vector<GroupLabel> labels;
    float panelTop = 0.0F;
    float panelHeight = 0.0F;
};

/// One row of chips: every group is a text label followed by its chips. Returns where the row ends.
inline float layoutRow(ChipLayout& out, const std::vector<std::pair<const char*, std::vector<ChipSpec>>>& groups, float x0, float y) {
    float x = x0;
    for (const auto& g : groups) {
        if (g.first != nullptr && g.first[0] != '\0') {
            out.labels.push_back({g.first, x, y + 4.0F});
            x += chipWidth(g.first) - 2.0F * kChipPad + 10.0F;
        }
        for (const ChipSpec& s : g.second) {
            const float w = chipWidth(s.label);
            out.chips.push_back({s.id, x, y, w, kChipHeight});
            x += w + kChipGap;
        }
        x += 12.0F;
    }
    return x;
}

/// The two chip rows in the panel at the bottom of the canvas (960x540).
inline ChipLayout chipLayout() {
    ChipLayout out;
    out.panelTop = 462.0F;
    out.panelHeight = kCanvasHeight - out.panelTop;
    layoutRow(out, {{"run", {{Chip::New, "new"}, {Chip::Step, "step"}, {Chip::Auto, "auto"}}}, {"speed", {{Chip::Slow, "slow"}, {Chip::Medium, "mid"}, {Chip::Fast, "fast"}, {Chip::Max, "max"}}}}, 14.0F, 470.0F);
    layoutRow(out,
              {{"pick", {{Chip::Entropy, "entropy"}, {Chip::Layers, "layers"}, {Chip::Random, "random"}}},
               {"on conflict", {{Chip::Undo, "undo"}, {Chip::Restart, "restart"}}},
               {"seed", {{Chip::SeedDown, "<"}, {Chip::SeedUp, ">"}}},
               {"", {{Chip::Spin, "spin"}}}},
              14.0F, 502.0F);
    return out;
}

inline const ChipRect* hitChip(const std::vector<ChipRect>& chips, float px, float py) {
    for (const ChipRect& c : chips) {
        if (px >= c.x && px < c.x + c.w && py >= c.y && py < c.y + c.h) return &c;
    }
    return nullptr;
}

}  // namespace w3
