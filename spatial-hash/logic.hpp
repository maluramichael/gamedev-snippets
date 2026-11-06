// Spatial partitioning for circles: brute force, uniform hash grid, quadtree. No SDL, so test.cpp can run it headless.
//
// All three answer the same question: which pairs of circles are close enough to be worth a circle-against-circle test?
// Every engine counts the pairs it hands to the narrow phase (Counters::tests). That number does not depend on the
// machine, which is why the demo shows it first and the milliseconds second.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace sh {

struct Rect {
    float x0 = 0.0F;
    float y0 = 0.0F;
    float x1 = 0.0F;
    float y1 = 0.0F;
};

struct Body {
    float x = 0.0F;
    float y = 0.0F;
    float vx = 0.0F;
    float vy = 0.0F;
    float r = 0.0F;
    float invMass = 0.0F;
};

struct Pair {
    int a = 0;
    int b = 0;
};

struct Counters {
    long tests = 0;    // pairs handed to the narrow phase
    long skipped = 0;  // grid only: duplicates rejected by the first-common-cell rule
};

enum class Mode { Brute, Grid, Quad };

/// The demo's playing field in canvas pixels, and the seed that makes every start scene the same.
inline Rect defaultField() { return Rect{10.0F, 36.0F, 950.0F, 370.0F}; }
inline constexpr std::uint32_t kSeed = 1;
inline constexpr float kSmallRadius = 3.5F;
/// Radius above which a body counts as "big" (drawn with an outline, mass grows with r squared).
inline constexpr float kBigRadius = 6.0F;

inline Rect boundsOf(const Body& b) { return {b.x - b.r, b.y - b.r, b.x + b.r, b.y + b.r}; }
inline bool touches(const Rect& a, const Rect& b) { return a.x0 <= b.x1 && b.x0 <= a.x1 && a.y0 <= b.y1 && b.y0 <= a.y1; }
inline bool contains(const Rect& outer, const Rect& in) { return in.x0 >= outer.x0 && in.y0 >= outer.y0 && in.x1 <= outer.x1 && in.y1 <= outer.y1; }

/// Narrow phase. Touching exactly is not an overlap.
inline bool overlaps(const Body& a, const Body& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float rr = a.r + b.r;
    return dx * dx + dy * dy < rr * rr;
}

/// n * (n - 1) / 2, what testing every pair costs.
inline long brutePairs(long n) { return n < 2 ? 0 : n * (n - 1) / 2; }

// ---- uniform grid ----

struct CellRange {
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
};

/// A uniform grid over a bounded world. The cell index cy * cols + cx is the "hash"; for an endless world you would put
/// (cx, cy) through a real hash function into a fixed table, everything else stays the same.
/// A body is listed in every cell its bounding box touches. Built by counting sort, so a rebuild allocates nothing
/// once the vectors have their size.
class Grid {
public:
    void build(const std::vector<Body>& bodies, const Rect& world, float cellSize) {
        world_ = world;
        cell_ = std::max(cellSize, 1.0F);
        cols_ = std::max(1, static_cast<int>(std::ceil((world.x1 - world.x0) / cell_)));
        rows_ = std::max(1, static_cast<int>(std::ceil((world.y1 - world.y0) / cell_)));
        const std::size_t cells = static_cast<std::size_t>(cols_) * static_cast<std::size_t>(rows_);
        start_.assign(cells + 1, 0);
        ranges_.resize(bodies.size());
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const CellRange r = rangeOf(bodies[i]);
            ranges_[i] = r;
            for (int y = r.y0; y <= r.y1; ++y) {
                for (int x = r.x0; x <= r.x1; ++x) ++start_[index(x, y) + 1];
            }
        }
        for (std::size_t c = 0; c < cells; ++c) start_[c + 1] += start_[c];
        items_.resize(static_cast<std::size_t>(start_[cells]));
        cursor_.assign(start_.begin(), start_.end() - 1);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const CellRange r = ranges_[i];
            for (int y = r.y0; y <= r.y1; ++y) {
                for (int x = r.x0; x <= r.x1; ++x) items_[static_cast<std::size_t>(cursor_[index(x, y)]++)] = static_cast<int>(i);
            }
        }
    }

    /// Every pair that shares a cell. A pair that shares several cells is seen once per cell; with `dedup` only the
    /// first common cell (the one with the larger of the two lower x indices and the larger of the two lower y indices)
    /// keeps it and the others count as `skipped`.
    template <class F>
    void pairs(bool dedup, Counters& counters, F&& visit) const {
        for (int cy = 0; cy < rows_; ++cy) {
            for (int cx = 0; cx < cols_; ++cx) {
                const std::size_t cell = index(cx, cy);
                const auto first = static_cast<std::size_t>(start_[cell]);
                const auto last = static_cast<std::size_t>(start_[cell + 1]);
                for (std::size_t i = first; i < last; ++i) {
                    for (std::size_t j = i + 1; j < last; ++j) {
                        const int a = items_[i];
                        const int b = items_[j];
                        if (dedup && !isFirstCommonCell(a, b, cx, cy)) {
                            ++counters.skipped;
                            continue;
                        }
                        ++counters.tests;
                        visit(a, b);
                    }
                }
            }
        }
    }

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    float cellSize() const { return cell_; }
    std::size_t entries() const { return items_.size(); }
    Rect cellRect(int cx, int cy) const {
        const float x = world_.x0 + static_cast<float>(cx) * cell_;
        const float y = world_.y0 + static_cast<float>(cy) * cell_;
        return Rect{x, y, std::min(x + cell_, world_.x1), std::min(y + cell_, world_.y1)};
    }

    std::span<const int> cellItems(int cx, int cy) const {
        if (cx < 0 || cy < 0 || cx >= cols_ || cy >= rows_) return {};
        const std::size_t cell = index(cx, cy);
        return std::span<const int>(items_).subspan(static_cast<std::size_t>(start_[cell]), static_cast<std::size_t>(start_[cell + 1] - start_[cell]));
    }

    /// The cell under a point, false when the point lies outside the world.
    bool cellAt(float x, float y, int& cx, int& cy) const {
        if (!(x >= world_.x0 && x < world_.x1 && y >= world_.y0 && y < world_.y1)) return false;
        cx = clampCell(std::floor((x - world_.x0) / cell_), cols_);
        cy = clampCell(std::floor((y - world_.y0) / cell_), rows_);
        return true;
    }

private:
    Rect world_;
    float cell_ = 16.0F;
    int cols_ = 1;
    int rows_ = 1;
    std::vector<int> start_;
    std::vector<int> items_;
    std::vector<int> cursor_;
    std::vector<CellRange> ranges_;

    std::size_t index(int cx, int cy) const { return static_cast<std::size_t>(cy) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(cx); }

    /// `v` is already floored. NaN and negative values land in 0, too large ones in the last cell, so the cast is safe.
    static int clampCell(float v, int count) {
        if (!(v > 0.0F)) return 0;
        if (v >= static_cast<float>(count - 1)) return count - 1;
        return static_cast<int>(v);
    }

    CellRange rangeOf(const Body& b) const {
        const Rect r = boundsOf(b);
        return CellRange{clampCell(std::floor((r.x0 - world_.x0) / cell_), cols_), clampCell(std::floor((r.y0 - world_.y0) / cell_), rows_),
                         clampCell(std::floor((r.x1 - world_.x0) / cell_), cols_), clampCell(std::floor((r.y1 - world_.y0) / cell_), rows_)};
    }

    bool isFirstCommonCell(int a, int b, int cx, int cy) const {
        const CellRange& ra = ranges_[static_cast<std::size_t>(a)];
        const CellRange& rb = ranges_[static_cast<std::size_t>(b)];
        return std::max(ra.x0, rb.x0) == cx && std::max(ra.y0, rb.y0) == cy;
    }
};

// ---- quadtree ----

/// Rebuilt every step. A body goes into the deepest node whose rectangle contains its whole bounding box, so bodies that
/// straddle a split line stay in the parent. A leaf splits when it holds more than `capacity` bodies and is above `maxDepth`.
class Quadtree {
public:
    struct Node {
        Rect rect;
        int child = -1;  // index of the first of four consecutive children: top-left, top-right, bottom-left, bottom-right
        int head = -1;   // first body of this node's list (linked through next_)
        int count = 0;
        int depth = 0;
    };

    void build(const std::vector<Body>& bodies, const Rect& world, int capacity, int maxDepth) {
        capacity_ = std::max(capacity, 1);
        maxDepth_ = std::max(maxDepth, 0);
        deepest_ = 0;
        nodes_.clear();
        nodes_.push_back(Node{world, -1, -1, 0, 0});
        next_.assign(bodies.size(), -1);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            // A body that is not completely inside the world fits no child and lives in the root.
            if (contains(world, boundsOf(bodies[i]))) {
                insertAt(bodies, 0, static_cast<int>(i));
            } else {
                push(0, static_cast<int>(i));
            }
        }
    }

    /// Every candidate pair: bodies of one node against each other, and each body against every body below it whose
    /// rectangle its bounding box touches. Bodies in different branches cannot overlap and are never paired.
    template <class F>
    void pairs(const std::vector<Body>& bodies, Counters& counters, F&& visit) const {
        for (const Node& node : nodes_) {
            for (int a = node.head; a >= 0; a = next_[static_cast<std::size_t>(a)]) {
                for (int b = next_[static_cast<std::size_t>(a)]; b >= 0; b = next_[static_cast<std::size_t>(b)]) {
                    ++counters.tests;
                    visit(a, b);
                }
                if (node.child >= 0) {
                    const Rect box = boundsOf(bodies[static_cast<std::size_t>(a)]);
                    for (int q = 0; q < 4; ++q) {
                        if (touches(nodes_[static_cast<std::size_t>(node.child + q)].rect, box)) against(node.child + q, a, box, counters, visit);
                    }
                }
            }
        }
    }

    const Node& node(std::size_t i) const { return nodes_[i]; }
    std::size_t nodeCount() const { return nodes_.size(); }
    int deepest() const { return deepest_; }

    /// The deepest node whose rectangle holds the point, -1 when the point lies outside the root.
    int nodeAt(float x, float y) const {
        if (nodes_.empty() || !(x >= nodes_[0].rect.x0 && x <= nodes_[0].rect.x1 && y >= nodes_[0].rect.y0 && y <= nodes_[0].rect.y1)) return -1;
        int n = 0;
        while (nodes_[static_cast<std::size_t>(n)].child >= 0) {
            const Rect& r = nodes_[static_cast<std::size_t>(n)].rect;
            const int qx = x < (r.x0 + r.x1) * 0.5F ? 0 : 1;
            const int qy = y < (r.y0 + r.y1) * 0.5F ? 0 : 1;
            n = nodes_[static_cast<std::size_t>(n)].child + qy * 2 + qx;
        }
        return n;
    }

    /// The bodies stored directly in node `n`, written into `out`.
    void itemsOf(int n, std::vector<int>& out) const {
        out.clear();
        if (n < 0 || static_cast<std::size_t>(n) >= nodes_.size()) return;
        for (int b = nodes_[static_cast<std::size_t>(n)].head; b >= 0; b = next_[static_cast<std::size_t>(b)]) out.push_back(b);
    }

private:
    std::vector<Node> nodes_;
    std::vector<int> next_;
    int capacity_ = 8;
    int maxDepth_ = 8;
    int deepest_ = 0;

    static Rect childRect(const Rect& r, int q) {
        const float mx = (r.x0 + r.x1) * 0.5F;
        const float my = (r.y0 + r.y1) * 0.5F;
        const bool right = (q & 1) != 0;
        const bool bottom = (q & 2) != 0;
        return Rect{right ? mx : r.x0, bottom ? my : r.y0, right ? r.x1 : mx, bottom ? r.y1 : my};
    }

    /// The child quadrant that contains the whole box, -1 when the box crosses a split line or leaves the node.
    int fittingChild(const Node& n, const Rect& box) const {
        const float mx = (n.rect.x0 + n.rect.x1) * 0.5F;
        const float my = (n.rect.y0 + n.rect.y1) * 0.5F;
        int qx = -1;
        int qy = -1;
        if (box.x1 <= mx) {
            qx = 0;
        } else if (box.x0 >= mx) {
            qx = 1;
        }
        if (box.y1 <= my) {
            qy = 0;
        } else if (box.y0 >= my) {
            qy = 1;
        }
        if (qx < 0 || qy < 0) return -1;
        const int q = qy * 2 + qx;
        return contains(childRect(n.rect, q), box) ? n.child + q : -1;
    }

    void push(int n, int body) {
        Node& node = nodes_[static_cast<std::size_t>(n)];
        next_[static_cast<std::size_t>(body)] = node.head;
        node.head = body;
        ++node.count;
    }

    void insertAt(const std::vector<Body>& bodies, int n, int body) {
        const Rect box = boundsOf(bodies[static_cast<std::size_t>(body)]);
        if (nodes_[static_cast<std::size_t>(n)].child >= 0) {
            const int c = fittingChild(nodes_[static_cast<std::size_t>(n)], box);
            if (c >= 0) {
                insertAt(bodies, c, body);
            } else {
                push(n, body);
            }
            return;
        }
        push(n, body);
        const Node& node = nodes_[static_cast<std::size_t>(n)];
        if (node.count > capacity_ && node.depth < maxDepth_) split(bodies, n);
    }

    void split(const std::vector<Body>& bodies, int n) {
        const Rect rect = nodes_[static_cast<std::size_t>(n)].rect;
        const int depth = nodes_[static_cast<std::size_t>(n)].depth + 1;
        const int first = static_cast<int>(nodes_.size());
        for (int q = 0; q < 4; ++q) nodes_.push_back(Node{childRect(rect, q), -1, -1, 0, depth});
        deepest_ = std::max(deepest_, depth);
        nodes_[static_cast<std::size_t>(n)].child = first;
        int item = nodes_[static_cast<std::size_t>(n)].head;
        nodes_[static_cast<std::size_t>(n)].head = -1;
        nodes_[static_cast<std::size_t>(n)].count = 0;
        while (item >= 0) {
            const int following = next_[static_cast<std::size_t>(item)];
            const int c = fittingChild(nodes_[static_cast<std::size_t>(n)], boundsOf(bodies[static_cast<std::size_t>(item)]));
            if (c >= 0) {
                insertAt(bodies, c, item);
            } else {
                push(n, item);
            }
            item = following;
        }
    }

    template <class F>
    void against(int n, int a, const Rect& box, Counters& counters, F& visit) const {
        const Node& node = nodes_[static_cast<std::size_t>(n)];
        for (int b = node.head; b >= 0; b = next_[static_cast<std::size_t>(b)]) {
            ++counters.tests;
            visit(a, b);
        }
        if (node.child < 0) return;
        for (int q = 0; q < 4; ++q) {
            if (touches(nodes_[static_cast<std::size_t>(node.child + q)].rect, box)) against(node.child + q, a, box, counters, visit);
        }
    }
};

// ---- brute force ----

template <class F>
void bruteForce(std::size_t n, Counters& counters, F&& visit) {
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            ++counters.tests;
            visit(static_cast<int>(i), static_cast<int>(j));
        }
    }
}

// ---- physics ----

/// Pushes two overlapping circles apart (share by inverse mass) and, if they approach, swaps the normal velocity
/// elastically. False when they do not overlap (any more) or neither can move.
inline bool resolve(Body& a, Body& b) {
    const float inv = a.invMass + b.invMass;
    if (!(inv > 0.0F)) return false;
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float rr = a.r + b.r;
    const float d2 = dx * dx + dy * dy;
    if (!(d2 < rr * rr)) return false;
    const float d = std::sqrt(d2);
    float nx = 1.0F;
    float ny = 0.0F;
    if (d > 1e-6F) {
        nx = dx / d;
        ny = dy / d;
    }
    const float push = (rr - d) / inv;
    a.x -= nx * push * a.invMass;
    a.y -= ny * push * a.invMass;
    b.x += nx * push * b.invMass;
    b.y += ny * push * b.invMass;
    const float vn = (b.vx - a.vx) * nx + (b.vy - a.vy) * ny;
    if (vn < 0.0F) {
        const float j = -2.0F * vn / inv;  // restitution 1: the normal velocity is mirrored
        a.vx -= j * a.invMass * nx;
        a.vy -= j * a.invMass * ny;
        b.vx += j * b.invMass * nx;
        b.vy += j * b.invMass * ny;
    }
    return true;
}

/// xorshift32, so a seed gives the same scene on every platform (std distributions differ between standard libraries).
class Rng {
public:
    explicit Rng(std::uint32_t seed) : state_(seed != 0 ? seed : 1U) {}
    std::uint32_t next() {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return state_;
    }
    /// Uniform in [lo, hi).
    float range(float lo, float hi) { return lo + (hi - lo) * (static_cast<float>(next() >> 8) * (1.0F / 16777216.0F)); }

private:
    std::uint32_t state_;
};

class World {
public:
    Rect bounds = defaultField();
    std::vector<Body> bodies;
    int quadCapacity = 8;
    int quadDepth = 8;

    /// n bodies, the last n / 80 of them big (radius 12 to 20) if `big`, the rest radius 3.5. Same seed, same scene.
    void populate(int n, bool big, std::uint32_t seed) {
        bodies.clear();
        n = std::max(n, 0);
        const int bigCount = big ? std::min(n, std::max(1, n / 80)) : 0;
        Rng rng(seed);
        for (int i = 0; i < n; ++i) {
            const bool isBig = i >= n - bigCount;
            Body b;
            b.r = isBig ? rng.range(12.0F, 20.0F) : kSmallRadius;
            b.x = rng.range(bounds.x0 + b.r, bounds.x1 - b.r);
            b.y = rng.range(bounds.y0 + b.r, bounds.y1 - b.r);
            const float angle = rng.range(0.0F, 6.2831853F);
            const float speed = isBig ? rng.range(30.0F, 60.0F) : rng.range(60.0F, 110.0F);
            b.vx = std::cos(angle) * speed;
            b.vy = std::sin(angle) * speed;
            b.invMass = 1.0F / (b.r * b.r);
            bodies.push_back(b);
        }
        hits_.clear();
    }

    /// Moves everything and bounces it off the four walls.
    void integrate(float dt) {
        for (Body& b : bodies) {
            b.x += b.vx * dt;
            b.y += b.vy * dt;
            if (b.x - b.r < bounds.x0) {
                b.x = bounds.x0 + b.r;
                if (b.vx < 0.0F) b.vx = -b.vx;
            } else if (b.x + b.r > bounds.x1) {
                b.x = bounds.x1 - b.r;
                if (b.vx > 0.0F) b.vx = -b.vx;
            }
            if (b.y - b.r < bounds.y0) {
                b.y = bounds.y0 + b.r;
                if (b.vy < 0.0F) b.vy = -b.vy;
            } else if (b.y + b.r > bounds.y1) {
                b.y = bounds.y1 - b.r;
                if (b.vy > 0.0F) b.vy = -b.vy;
            }
        }
    }

    /// Broad phase plus narrow phase. Fills hits() with the overlapping pairs and moves nothing.
    Counters findContacts(Mode mode, int cellSize, bool dedup) {
        Counters counters;
        hits_.clear();
        auto visit = [this](int a, int b) {
            if (overlaps(bodies[static_cast<std::size_t>(a)], bodies[static_cast<std::size_t>(b)])) hits_.push_back(Pair{a, b});
        };
        switch (mode) {
            case Mode::Brute: bruteForce(bodies.size(), counters, visit); break;
            case Mode::Grid:
                grid_.build(bodies, bounds, static_cast<float>(cellSize));
                grid_.pairs(dedup, counters, visit);
                break;
            case Mode::Quad:
                tree_.build(bodies, bounds, quadCapacity, quadDepth);
                tree_.pairs(bodies, counters, visit);
                break;
        }
        return counters;
    }

    /// One fixed step: move, find the contacts, resolve them. The pairs of a step stay readable in hits().
    Counters step(float dt, Mode mode, int cellSize, bool dedup) {
        integrate(dt);
        const Counters counters = findContacts(mode, cellSize, dedup);
        for (const Pair& p : hits_) resolve(bodies[static_cast<std::size_t>(p.a)], bodies[static_cast<std::size_t>(p.b)]);
        return counters;
    }

    const std::vector<Pair>& hits() const { return hits_; }
    const Grid& grid() const { return grid_; }
    const Quadtree& tree() const { return tree_; }

private:
    std::vector<Pair> hits_;
    Grid grid_;
    Quadtree tree_;
};

// ---- numbers on screen ----

/// 1234567 -> "1,234,567". Built by hand, snprintf would follow the locale.
inline std::string formatInt(long v) {
    const bool negative = v < 0;
    const unsigned long magnitude = negative ? 0UL - static_cast<unsigned long>(v) : static_cast<unsigned long>(v);
    std::string digits = std::to_string(magnitude);
    for (int i = static_cast<int>(digits.size()) - 3; i > 0; i -= 3) digits.insert(static_cast<std::size_t>(i), ",");
    return negative ? "-" + digits : digits;
}

/// Two decimals with a dot, whatever the locale: 0.375 -> "0.38".
inline std::string formatFixed2(double v) {
    const long long hundredths = std::llround(v * 100.0);
    const long long magnitude = hundredths < 0 ? -hundredths : hundredths;
    const long long frac = magnitude % 100;
    std::string out = hundredths < 0 ? "-" : "";
    out += std::to_string(magnitude / 100);
    out += frac < 10 ? ".0" : ".";
    out += std::to_string(frac);
    return out;
}

// ---- cell size slider: 4 to 128 px on a log scale, because the interesting part is the small end ----

inline constexpr int kMinCell = 4;
inline constexpr int kMaxCell = 128;

inline int cellFromT(float t) {
    if (!(t > 0.0F)) return kMinCell;
    if (t >= 1.0F) return kMaxCell;
    const auto v = static_cast<int>(std::lround(static_cast<double>(kMinCell) * std::pow(static_cast<double>(kMaxCell) / kMinCell, static_cast<double>(t))));
    return std::clamp(v, kMinCell, kMaxCell);
}

inline float tFromCell(int cell) {
    const int c = std::clamp(cell, kMinCell, kMaxCell);
    return static_cast<float>(std::log(static_cast<double>(c) / kMinCell) / std::log(static_cast<double>(kMaxCell) / kMinCell));
}

/// One keyboard step, about 12 percent, at least 1 px.
inline int stepCell(int cell, int dir) {
    const int c = std::clamp(cell, kMinCell, kMaxCell);
    const int next = dir > 0 ? std::max(c + 1, static_cast<int>(std::lround(c * 1.12))) : std::min(c - 1, static_cast<int>(std::lround(c / 1.12)));
    return std::clamp(next, kMinCell, kMaxCell);
}

// ---- contact glow: a contact lasts one or two 60 Hz steps, so a circle that is either lit or not would flicker ----

inline constexpr float kHeatRiseTau = 0.1F;  // seconds to cover 63 percent of the way up while touching
inline constexpr float kHeatTau = 0.35F;      // seconds for the glow to fall to 37 percent without contact

/// New glow of a circle after `dt` seconds: it rises toward 1 while the circle touches and decays toward 0 otherwise,
/// both smoothly, so no step changes the color by much. Always in [0, 1]; a non-positive or NaN `dt` changes nothing.
inline float stepHeat(float heat, bool touching, float dt) {
    heat = std::clamp(std::isnan(heat) ? 0.0F : heat, 0.0F, 1.0F);
    if (!(dt > 0.0F)) return heat;
    if (touching) return 1.0F - (1.0F - heat) * std::exp(-dt / kHeatRiseTau);
    return heat * std::exp(-dt / kHeatTau);
}

}  // namespace sh
