// The abelian sandpile on a finite grid, no SDL.
//
// A cell with 4 or more grains topples: it gives one grain to each of its four neighbors for every 4 it holds. A neighbor
// outside the grid is the sink, the grain is gone (a corner cell loses 2 of its 4 grains per toppling, an edge cell 1).
// The point of the model: the picture you end up with does not depend on the order in which the unstable cells are
// toppled, and neither does the number of topplings. The order only changes the intermediate pictures.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sp {

constexpr int kLimit = 4;  // a cell with this many grains or more topples

/// The order in which the unstable cells of one pass are visited.
enum class Order { Rows, Random, Reverse };

/// splitmix64: tiny, fast, and the same numbers on every platform.
struct Rng {
    explicit Rng(std::uint64_t seed) : state(seed) {}

    std::uint64_t next() {
        state += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    /// A number in [0, n), 0 for n == 0. Multiply-shift instead of a modulo, the bias is below 2^-32.
    std::uint32_t below(std::uint32_t n) { return static_cast<std::uint32_t>((next() >> 32) * n >> 32); }

    std::uint64_t state;
};

struct Pile {
    Pile(int width, int height) : w(width), h(height), cells(static_cast<std::size_t>(width * height), 0), queued(cells.size(), 0) {}

    int idx(int x, int y) const { return y * w + x; }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }

    int w;
    int h;
    std::vector<int> cells;
    std::vector<int> pending;            // cells with 4 or more grains that still wait for their toppling, each at most once
    std::vector<unsigned char> queued;   // 1 while a cell is in pending (or in the batch of the pass that is running)
    std::vector<int> batch;              // scratch for pass(), kept to avoid allocating every frame
    long long total = 0;                 // grains put on the grid since the last reset
    long long lost = 0;                  // grains that fell off the edge
    long long topplings = 0;             // single topplings (4 grains each) since the last reset
};

inline bool stable(const Pile& p) { return p.pending.empty(); }

inline long long grains(const Pile& p) {
    long long sum = 0;
    for (const int c : p.cells) sum += c;
    return sum;
}

inline void enqueue(Pile& p, int i) {
    const auto k = static_cast<std::size_t>(i);
    if (p.cells[k] >= kLimit && p.queued[k] == 0) {
        p.queued[k] = 1;
        p.pending.push_back(i);
    }
}

/// Every cell gets the same number of grains (0 empties the grid). Counters start again.
inline void reset(Pile& p, int value) {
    std::fill(p.cells.begin(), p.cells.end(), value);
    std::fill(p.queued.begin(), p.queued.end(), static_cast<unsigned char>(0));
    p.pending.clear();
    p.total = static_cast<long long>(value) * static_cast<long long>(p.cells.size());
    p.lost = 0;
    p.topplings = 0;
    if (value >= kLimit) {
        for (int i = 0; i < p.w * p.h; ++i) enqueue(p, i);
    }
}

/// Puts n grains on a cell. Drops outside the grid are ignored. The cell is not toppled yet, see pass() and settle().
inline void add(Pile& p, int x, int y, int n) {
    if (!p.inside(x, y) || n <= 0) return;
    p.cells[static_cast<std::size_t>(p.idx(x, y))] += n;
    p.total += n;
    enqueue(p, p.idx(x, y));
}

inline void orderCells(std::vector<int>& v, Order order, Rng& rng) {
    switch (order) {
        case Order::Rows: std::sort(v.begin(), v.end()); break;
        case Order::Reverse: std::sort(v.begin(), v.end(), [](int a, int b) { return a > b; }); break;
        case Order::Random:
            // Fisher-Yates with our own generator, std::shuffle would give different results per standard library.
            for (std::size_t i = v.size(); i > 1; --i) std::swap(v[i - 1], v[rng.below(static_cast<std::uint32_t>(i))]);
            break;
    }
}

/// One pass: topple every cell that is waiting, once each, in the given order. A cell topples as often as it has groups of 4
/// (k times in one go, which is k single topplings in a row). Cells that become unstable during the pass wait for the next
/// one, except cells that are still to come in this pass: they have not toppled yet and take the new grains along.
/// Returns the number of single topplings.
inline long long pass(Pile& p, Order order, Rng& rng) {
    p.batch.clear();
    p.batch.swap(p.pending);
    orderCells(p.batch, order, rng);
    long long units = 0;
    for (const int i : p.batch) {
        const auto k = static_cast<std::size_t>(i);
        p.queued[k] = 0;
        const int times = p.cells[k] / kLimit;
        if (times == 0) continue;
        p.cells[k] -= times * kLimit;
        units += times;
        const int x = i % p.w;
        const int y = i / p.w;
        const int nx[4] = {x - 1, x + 1, x, x};
        const int ny[4] = {y, y, y - 1, y + 1};
        for (int d = 0; d < 4; ++d) {
            if (!p.inside(nx[d], ny[d])) {
                p.lost += times;
                continue;
            }
            const int n = p.idx(nx[d], ny[d]);
            p.cells[static_cast<std::size_t>(n)] += times;
            enqueue(p, n);
        }
    }
    p.topplings += units;
    return units;
}

struct Avalanche {
    long long topplings = 0;
    int passes = 0;
};

/// Passes until nothing is unstable. Terminates because every grain eventually reaches the sink.
inline Avalanche settle(Pile& p, Order order, Rng& rng) {
    Avalanche a;
    while (!stable(p)) {
        a.topplings += pass(p, order, rng);
        ++a.passes;
    }
    return a;
}

/// FNV-1a over the cells, four little-endian bytes each. A fingerprint to compare two piles at a glance.
inline std::uint32_t hash(const Pile& p) {
    std::uint32_t h = 2166136261U;
    for (const int c : p.cells) {
        const auto v = static_cast<std::uint32_t>(c);
        for (int shift = 0; shift < 32; shift += 8) {
            h ^= (v >> shift) & 0xFFU;
            h *= 16777619U;
        }
    }
    return h;
}

inline std::string hex8(std::uint32_t v) {
    std::string s(8, '0');
    for (int i = 7; i >= 0; --i) {
        s[static_cast<std::size_t>(i)] = "0123456789ABCDEF"[v & 0xFU];
        v >>= 4;
    }
    return s;
}

/// 1234567 -> "1,234,567", built by hand so the output does not depend on the locale.
inline std::string grouped(long long v) {
    const bool negative = v < 0;
    std::string digits = std::to_string(negative ? -v : v);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return negative ? "-" + out : out;
}

constexpr int kBuckets = 18;  // sizes 0, 1, 2-3, 4-7, ... up to 32768-65535, and 65536 or more

/// Bucket 0 holds the drops that toppled nothing, bucket b >= 1 the sizes from 2^(b-1) to 2^b - 1, the last one everything above.
inline int bucketOf(long long size) {
    if (size <= 0) return 0;
    int b = 0;
    while (size > 0 && b < kBuckets - 1) {
        size >>= 1;
        ++b;
    }
    return b;
}

struct Histogram {
    void record(long long size) {
        ++count[static_cast<std::size_t>(bucketOf(size))];
        ++drops;
        largest = std::max(largest, size);
    }

    long long peak() const { return *std::max_element(count.begin(), count.end()); }

    void clear() { *this = Histogram{}; }

    std::array<long long, kBuckets> count{};
    long long drops = 0;
    long long largest = 0;
};

}  // namespace sp
