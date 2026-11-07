// Possibility map: an unseen guard is not simulated. A map of "he could be here" cells grows instead, and only
// when the player's view (or hearing) touches a possible cell does the AI have to decide: the guard appears there,
// or the cell is crossed off. No SDL in this file, test.cpp covers it.
//
// After John Manslow, "Possibility Maps for Opportunistic AI and Believable Worlds", Game AI Pro 2, chapter 7.
//
// Each cell holds a small set of states. Without the medipack feature there is one state (kFree). With it, the map
// forks when the cloud reaches the pack: kFree = "has not taken it", kTaken = "has taken it".
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pm {

constexpr double kPi = 3.14159265358979323846;

struct Cell {
    int x = 0;
    int y = 0;
};

/// A map from text rows. '#' is a wall, everything else floor: '.' plain, ',' noisy, 'P' player start,
/// 'G' guard post, 'M' medipack. Everything outside the rows counts as wall.
class Grid {
public:
    Grid() = default;
    explicit Grid(std::vector<std::string> rows) : rows_(std::move(rows)) {
        height_ = static_cast<int>(rows_.size());
        for (const std::string& r : rows_) width_ = std::max(width_, static_cast<int>(r.size()));
    }

    int width() const { return width_; }
    int height() const { return height_; }
    int size() const { return width_ * height_; }
    int index(int x, int y) const { return y * width_ + x; }
    Cell cell(int i) const { return {i % width_, i / width_}; }

    char at(int x, int y) const {
        if (x < 0 || y < 0 || y >= height_) return '#';
        const std::string& row = rows_[static_cast<std::size_t>(y)];
        return x < static_cast<int>(row.size()) ? row[static_cast<std::size_t>(x)] : '#';
    }
    bool wall(int x, int y) const { return at(x, y) == '#'; }
    bool floor(int x, int y) const { return !wall(x, y); }
    bool noisy(int x, int y) const { return at(x, y) == ','; }

    /// Index of the first cell with this character, -1 if there is none.
    int find(char c) const {
        for (int i = 0; i < size(); ++i) {
            if (at(i % width_, i / width_) == c) return i;
        }
        return -1;
    }

private:
    std::vector<std::string> rows_;
    int width_ = 0;
    int height_ = 0;
};

inline constexpr std::array<Cell, 4> kDirs{{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

/// Steps through floor cells (four-way) from `source`, -1 where unreachable. A wall is no source.
inline std::vector<int> distancesFrom(const Grid& g, int source) {
    std::vector<int> dist(static_cast<std::size_t>(g.size()), -1);
    if (source < 0 || source >= g.size()) return dist;
    const Cell s = g.cell(source);
    if (g.wall(s.x, s.y)) return dist;
    std::vector<int> queue{source};
    dist[static_cast<std::size_t>(source)] = 0;
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const int at = queue[head];
        const Cell c = g.cell(at);
        for (const Cell d : kDirs) {
            const int nx = c.x + d.x;
            const int ny = c.y + d.y;
            if (g.wall(nx, ny)) continue;
            const int n = g.index(nx, ny);
            if (dist[static_cast<std::size_t>(n)] >= 0) continue;
            dist[static_cast<std::size_t>(n)] = dist[static_cast<std::size_t>(at)] + 1;
            queue.push_back(n);
        }
    }
    return dist;
}

/// The neighbor one step closer to the cell the distance field was computed from. Stays put when already there
/// or when the cell has no way to it.
inline int stepToward(const Grid& g, const std::vector<int>& dist, int from) {
    const int here = dist[static_cast<std::size_t>(from)];
    if (here <= 0) return from;
    const Cell c = g.cell(from);
    for (const Cell d : kDirs) {
        const int nx = c.x + d.x;
        const int ny = c.y + d.y;
        if (g.wall(nx, ny)) continue;
        const int n = g.index(nx, ny);
        if (dist[static_cast<std::size_t>(n)] == here - 1) return n;
    }
    return from;
}

inline Cell tryMove(const Grid& g, Cell from, int dx, int dy) {
    return g.floor(from.x + dx, from.y + dy) ? Cell{from.x + dx, from.y + dy} : from;
}

/// True when no wall cell lies strictly between the two cells on the Bresenham line.
inline bool lineClear(const Grid& g, Cell a, Cell b) {
    const int dx = std::abs(b.x - a.x);
    const int dy = -std::abs(b.y - a.y);
    const int sx = a.x < b.x ? 1 : -1;
    const int sy = a.y < b.y ? 1 : -1;
    int err = dx + dy;
    int x = a.x;
    int y = a.y;
    while (x != b.x || y != b.y) {
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
        if (x == b.x && y == b.y) break;
        if (g.wall(x, y)) return false;
    }
    return true;
}

/// Floor cells the player sees: inside the cone (half angle on each side of `facing`, edge included), within
/// `range` cells and with a clear line. The own cell is always seen. Angles are in screen coordinates (y down).
inline std::vector<char> visibleFrom(const Grid& g, Cell eye, double facing, double halfAngle, double range) {
    std::vector<char> seen(static_cast<std::size_t>(g.size()), 0);
    if (g.wall(eye.x, eye.y)) return seen;
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            if (g.wall(x, y)) continue;
            const int dx = x - eye.x;
            const int dy = y - eye.y;
            bool ok = dx == 0 && dy == 0;
            if (!ok) {
                const double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
                if (dist > range + 1e-9) continue;
                const double diff = std::fabs(std::remainder(std::atan2(static_cast<double>(dy), static_cast<double>(dx)) - facing, 2.0 * kPi));
                ok = diff <= halfAngle + 1e-9 && lineClear(g, eye, {x, y});
            }
            if (ok) seen[static_cast<std::size_t>(g.index(x, y))] = 1;
        }
    }
    return seen;
}

/// SplitMix64, deterministic across platforms. next() is in [0, 1).
class Rng {
public:
    explicit Rng(std::uint64_t seed = 1) : state_(seed) {}
    double next() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) * (1.0 / 9007199254740992.0);
    }

private:
    std::uint64_t state_;
};

constexpr std::uint8_t kFree = 1;   // possible, the medipack was not taken
constexpr std::uint8_t kTaken = 2;  // possible, the medipack was taken

struct Guard {
    int cell = 0;
    int waypoint = 0;  // the patrol point he walks to
};

enum class Outcome { None, Appeared, Cleared };
enum class PackOutcome { None, Taken, Kept };

/// What one settle() decided, for the readout.
struct Observation {
    Outcome outcome = Outcome::None;
    int seen = 0;        // possible (cell, state) pairs the player observed
    int total = 0;       // all possible pairs before the decision
    double chance = 0.0; // seen / total, the odds the guard was among them
    int spawned = 0;     // guards created (naive mode creates one per observed cell)
    int cell = -1;       // where the guard appeared
    PackOutcome pack = PackOutcome::None;
    double packChance = 0.0;  // taken states / all states, the odds the pack was gone
};

/// The AI's roll for "he is among the seen states": seen out of total, the fair odds. Written as roll * total < seen
/// so that seen == total is always a yes (the only possibility left, the AI has no choice) and seen == 0 never.
inline bool appears(int seen, int total, double roll) {
    return seen > 0 && total > 0 && roll * static_cast<double>(total) < static_cast<double>(seen);
}

class Sim {
public:
    Sim() = default;
    Sim(Grid grid, std::vector<int> patrol, std::uint64_t seed, bool honest = true)
        : grid_(std::move(grid)), possible_(static_cast<std::size_t>(grid_.size()), 0), visible_(possible_.size(), 0), patrol_(std::move(patrol)), rng_(seed), honest_(honest) {
        for (const int w : patrol_) patrolDist_.push_back(distancesFrom(grid_, w));
    }

    const Grid& grid() const { return grid_; }
    const std::vector<std::uint8_t>& possible() const { return possible_; }
    const std::vector<Guard>& guards() const { return guards_; }
    bool honest() const { return honest_; }
    int medipack() const { return medipack_; }
    bool medipackGone() const { return medipackGone_; }
    bool noise() const { return noise_; }
    int earshot() const { return earshot_; }

    /// Where the player stands. Nobody else can stand there: the cloud never enters the cell and guards do not
    /// walk into it, so the cloud cannot slip past the player in a one-cell corridor either.
    void setPlayer(int cell) { player_ = cell; }
    void setNoise(bool on) { noise_ = on; }
    void setEarshot(int steps) { earshot_ = steps; }

    /// Turns the pack on at `cell` (not taken yet).
    void setMedipack(int cell) {
        medipack_ = cell;
        medipackGone_ = false;
    }

    /// Turns the pack off: every branch collapses into the plain "possible" state.
    void clearMedipack() {
        medipack_ = -1;
        medipackGone_ = false;
        for (std::uint8_t& m : possible_) m = m != 0 ? kFree : 0;
    }

    /// Marks every cell within `radius` steps of `cell` as possible.
    void seed(int cell, int radius) {
        const std::vector<int> dist = distancesFrom(grid_, cell);
        for (std::size_t i = 0; i < dist.size(); ++i) {
            if (dist[i] >= 0 && dist[i] <= radius) possible_[i] |= guardState();
        }
    }

    /// Puts a real guard on `cell` without touching the map.
    void addGuard(int cell) { guards_.push_back({cell, nearestWaypoint(cell)}); }

    int cells() const {
        int n = 0;
        for (const std::uint8_t m : possible_) n += m != 0 ? 1 : 0;
        return n;
    }

    int states() const {
        int n = 0;
        for (const std::uint8_t m : possible_) n += (m & kFree ? 1 : 0) + (m & kTaken ? 1 : 0);
        return n;
    }

    /// Noisy cells the player can hear from `playerCell`: within `earshot` steps along the floor.
    std::vector<char> heard(int playerCell) const {
        std::vector<char> out(possible_.size(), 0);
        if (!noise_) return out;
        const std::vector<int> dist = distancesFrom(grid_, playerCell);
        for (std::size_t i = 0; i < dist.size(); ++i) {
            const Cell c = grid_.cell(static_cast<int>(i));
            if (grid_.noisy(c.x, c.y) && dist[i] >= 0 && dist[i] <= earshot_) out[i] = 1;
        }
        return out;
    }

    /// One step of the world: the cloud grows by one cell in every direction, real guards walk one cell.
    void tick() {
        spread();
        stepGuards();
    }

    /// Reconciles the map with what the player observes right now (`visible` by sight, `heard` by ear). Every possible
    /// cell that is observed forces a decision, so afterwards none of them is possible any more. Guards that are no
    /// longer observed go back into the map.
    Observation settle(const std::vector<char>& visible, const std::vector<char>& heard) {
        Observation obs;
        visible_ = visible;
        if (player_ >= 0) possible_[static_cast<std::size_t>(player_)] = 0;
        std::vector<char> seen(possible_.size(), 0);
        for (std::size_t i = 0; i < seen.size(); ++i) seen[i] = (visible[i] != 0 || heard[i] != 0) ? 1 : 0;

        decidePack(obs);
        decideCells(obs, seen);

        for (std::size_t g = 0; g < guards_.size();) {
            if (seen[static_cast<std::size_t>(guards_[g].cell)] == 0) {
                possible_[static_cast<std::size_t>(guards_[g].cell)] |= guardState();
                guards_.erase(guards_.begin() + static_cast<std::ptrdiff_t>(g));
            } else {
                ++g;
            }
        }
        return obs;
    }

private:
    Grid grid_;
    std::vector<std::uint8_t> possible_;
    std::vector<char> visible_;  // what the last settle() saw, so a pack in view cannot be taken silently
    std::vector<int> patrol_;
    std::vector<std::vector<int>> patrolDist_;
    std::vector<Guard> guards_;
    Rng rng_;
    bool honest_ = true;
    int medipack_ = -1;
    bool medipackGone_ = false;
    bool noise_ = false;
    int earshot_ = 8;
    int player_ = -1;

    std::uint8_t guardState() const { return medipackGone_ ? kTaken : kFree; }

    int nearestWaypoint(int cell) const {
        int best = 0;
        int bestDist = -1;
        for (std::size_t w = 0; w < patrolDist_.size(); ++w) {
            const int d = patrolDist_[w][static_cast<std::size_t>(cell)];
            if (d >= 0 && (bestDist < 0 || d < bestDist)) {
                best = static_cast<int>(w);
                bestDist = d;
            }
        }
        return best;
    }

    void markGone() {
        medipackGone_ = true;
        for (std::uint8_t& m : possible_) {
            if ((m & kFree) != 0) m = static_cast<std::uint8_t>((m & ~kFree) | kTaken);
        }
    }

    void pickup(const Guard& g) {
        if (medipack_ >= 0 && !medipackGone_ && g.cell == medipack_) markGone();
    }

    void spread() {
        std::vector<std::uint8_t> next(possible_.size(), 0);
        for (std::size_t i = 0; i < possible_.size(); ++i) {
            const std::uint8_t m = possible_[i];
            if (m == 0) continue;
            next[i] |= m;
            const Cell c = grid_.cell(static_cast<int>(i));
            for (const Cell d : kDirs) {
                if (grid_.wall(c.x + d.x, c.y + d.y)) continue;
                const int n = grid_.index(c.x + d.x, c.y + d.y);
                if (n != player_) next[static_cast<std::size_t>(n)] |= m;
            }
        }
        // The fork: whoever reaches the pack unseen may have taken it. Not while it is in view.
        if (medipack_ >= 0 && !medipackGone_ && visible_[static_cast<std::size_t>(medipack_)] == 0 && (next[static_cast<std::size_t>(medipack_)] & kFree) != 0) {
            next[static_cast<std::size_t>(medipack_)] |= kTaken;
        }
        possible_.swap(next);
    }

    void stepGuards() {
        if (patrol_.empty()) return;
        for (Guard& g : guards_) {
            if (g.cell == patrol_[static_cast<std::size_t>(g.waypoint)]) g.waypoint = (g.waypoint + 1) % static_cast<int>(patrol_.size());
            const int to = stepToward(grid_, patrolDist_[static_cast<std::size_t>(g.waypoint)], g.cell);
            if (to != player_) g.cell = to;  // he waits instead of walking through the player
            pickup(g);
        }
    }

    /// The player looks at the pack. If the cloud holds states where it was taken, the AI decides which world this is.
    void decidePack(Observation& obs) {
        if (medipack_ < 0 || medipackGone_ || visible_[static_cast<std::size_t>(medipack_)] == 0) return;
        int taken = 0;
        int free = 0;
        for (const std::uint8_t m : possible_) {
            taken += (m & kTaken) != 0 ? 1 : 0;
            free += (m & kFree) != 0 ? 1 : 0;
        }
        if (taken == 0) return;
        obs.packChance = static_cast<double>(taken) / static_cast<double>(taken + free);
        if (appears(taken, taken + free, rng_.next())) {
            obs.pack = PackOutcome::Taken;
            for (std::uint8_t& m : possible_) m = static_cast<std::uint8_t>(m & ~kFree);
            medipackGone_ = true;
        } else {
            obs.pack = PackOutcome::Kept;
            for (std::uint8_t& m : possible_) m = static_cast<std::uint8_t>(m & ~kTaken);
        }
    }

    void decideCells(Observation& obs, const std::vector<char>& seen) {
        std::vector<std::pair<int, std::uint8_t>> hits;
        int total = 0;
        for (std::size_t i = 0; i < possible_.size(); ++i) {
            for (const std::uint8_t bit : {kFree, kTaken}) {
                if ((possible_[i] & bit) == 0) continue;
                ++total;
                if (seen[i] != 0) hits.emplace_back(static_cast<int>(i), bit);
            }
        }
        if (hits.empty()) return;
        obs.seen = static_cast<int>(hits.size());
        obs.total = total;
        obs.chance = static_cast<double>(obs.seen) / static_cast<double>(total);

        if (!honest_) {
            // The bug: every observed possibility becomes a guard. One real guard, many instances.
            int last = -1;
            for (const auto& [cell, bit] : hits) {
                possible_[static_cast<std::size_t>(cell)] = 0;
                if (cell == last) continue;
                last = cell;
                guards_.push_back({cell, nearestWaypoint(cell)});
                ++obs.spawned;
                if (bit == kTaken && medipack_ >= 0) markGone();
                pickup(guards_.back());
            }
            obs.outcome = Outcome::Appeared;
            obs.cell = hits.front().first;
            return;
        }

        if (appears(obs.seen, total, rng_.next())) {
            const auto pick = std::min(hits.size() - 1, static_cast<std::size_t>(rng_.next() * static_cast<double>(hits.size())));
            const auto [cell, bit] = hits[pick];
            std::fill(possible_.begin(), possible_.end(), static_cast<std::uint8_t>(0));  // he exists now: nowhere else is possible
            guards_.push_back({cell, nearestWaypoint(cell)});
            if (bit == kTaken && medipack_ >= 0) markGone();
            pickup(guards_.back());
            obs.outcome = Outcome::Appeared;
            obs.spawned = 1;
            obs.cell = cell;
        } else {
            for (const auto& hit : hits) possible_[static_cast<std::size_t>(hit.first)] = 0;
            obs.outcome = Outcome::Cleared;
        }
    }
};

/// The slider: x position to a whole speed and back.
inline int speedFromX(double x, double x0, double x1, int lo, int hi) {
    const double t = std::clamp((x - x0) / (x1 - x0), 0.0, 1.0);
    return lo + static_cast<int>(std::llround(t * static_cast<double>(hi - lo)));
}

inline double xFromSpeed(int speed, double x0, double x1, int lo, int hi) {
    return x0 + (x1 - x0) * static_cast<double>(speed - lo) / static_cast<double>(hi - lo);
}

/// 30 x 13 cells, one cell is one square of the paper grid. Three rooms on top, a two-row hall, four rooms below.
/// The guard's room (top right) has one door, and it is the noisy one.
inline std::vector<std::string> levelRows() {
    return {
        "##############################",
        "#.....#..........#...........#",
        "#.....#..........#......G....#",
        "#.....#..........#...........#",
        "#.....#..........#..,........#",
        "###.######.#########,#########",
        "#............................#",
        "#.....P.........M............#",
        "####.#######.#####.#####.#####",
        "#........#.......#...#.......#",
        "#........#.......#...#.......#",
        "#........#.......#...#.......#",
        "##############################",
    };
}

/// The guard's rounds: his post, the medipack, two lower rooms, then the two upper rooms on the left.
inline std::vector<int> levelPatrol(const Grid& g) {
    return {g.index(24, 2), g.index(16, 7), g.index(12, 10), g.index(4, 10), g.index(3, 3), g.index(10, 3)};
}

constexpr int kSeedRadius = 9;

/// The start of the demo: nobody has seen the guard for a while, so the cloud already fills his room and pokes
/// out of the door.
inline Sim makeLevelSim(std::uint64_t seed, bool honest) {
    const Grid g(levelRows());
    Sim sim(g, levelPatrol(g), seed, honest);
    sim.seed(g.find('G'), kSeedRadius);
    return sim;
}

}  // namespace pm
