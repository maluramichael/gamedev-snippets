// Rule 184 as a traffic model, no SDL, covered by test.cpp.
//
// The road is a ring of cells, 1 is a car, 0 is free. Every step all cells look at the previous row at once: a cell
// that holds a car takes its next value from the cell ahead (a car with a free cell ahead leaves, a car with a car
// ahead stays), an empty cell takes it from the cell behind (a car there drives in). That is Wolfram's rule 184.
// stepInPlace is the common mistake: one array, one sweep, so a car is looked at again after it has moved.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

namespace r184 {

/// One byte per cell, cars drive towards higher indices and the last cell is followed by the first.
using Road = std::vector<std::uint8_t>;

/// splitmix64, small and the same on every platform, so a seed always gives the same road.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : state_(seed) {}

    std::uint64_t next() {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31U);
    }

    /// 0 .. n-1 (the modulo bias is below 2^-50 for the sizes used here). n = 0 gives 0.
    std::size_t below(std::size_t n) { return n == 0 ? 0 : static_cast<std::size_t>(next() % n); }

private:
    std::uint64_t state_;
};

inline int carCount(const Road& road) {
    return static_cast<int>(std::count_if(road.begin(), road.end(), [](std::uint8_t c) { return c != 0; }));
}

/// Cars that have a free cell ahead, so cars that will move in the next parallel step.
inline int movingCars(const Road& road) {
    const std::size_t n = road.size();
    int moving = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (road[i] != 0 && road[(i + 1) % n] == 0) ++moving;
    }
    return moving;
}

/// One step of rule 184 with all cells updated from the old row. Returns how many cars moved.
inline int stepParallel(Road& road) {
    const std::size_t n = road.size();
    if (n == 0) return 0;
    Road next(n, 0);
    int hops = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint8_t behind = road[(i + n - 1) % n];
        const std::uint8_t ahead = road[(i + 1) % n];
        next[i] = road[i] != 0 ? ahead : behind;
        if (road[i] != 0 && ahead == 0) ++hops;
    }
    road.swap(next);
    return hops;
}

/// The mistake: sweep from cell 0 upwards and move cars inside the one array. A car that has just moved is met
/// again in the next cell and moves again. Returns the number of single-cell hops, which can exceed the car count.
inline int stepInPlace(Road& road) {
    const std::size_t n = road.size();
    int hops = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        if (road[i] != 0 && road[j] == 0) {
            road[i] = 0;
            road[j] = 1;
            ++hops;
        }
    }
    return hops;
}

/// A road of n cells with exactly `cars` cars (clamped to n) at random places.
inline Road randomRoad(std::size_t n, std::size_t cars, Rng& rng) {
    Road road(n, 0);
    std::vector<std::size_t> cells(n);
    for (std::size_t i = 0; i < n; ++i) cells[i] = i;
    const std::size_t k = std::min(cars, n);
    for (std::size_t i = 0; i < k; ++i) {
        std::swap(cells[i], cells[i + rng.below(n - i)]);
        road[cells[i]] = 1;
    }
    return road;
}

/// All cars in one queue of `cars` cells starting at `start`, wrapping around the end of the ring.
inline Road blockRoad(std::size_t n, std::size_t cars, std::size_t start) {
    Road road(n, 0);
    for (std::size_t k = 0; k < std::min(cars, n); ++k) road[(start + k) % n] = 1;
    return road;
}

/// Adds or removes random cars until there are `target` (clamped to the road length). Cars that stay are untouched.
inline void setCarCount(Road& road, std::size_t target, Rng& rng) {
    const std::size_t want = std::min(target, road.size());
    while (static_cast<std::size_t>(carCount(road)) != want) {
        const bool remove = static_cast<std::size_t>(carCount(road)) > want;
        std::vector<std::size_t> candidates;
        for (std::size_t i = 0; i < road.size(); ++i) {
            if ((road[i] != 0) == remove) candidates.push_back(i);
        }
        road[candidates[rng.below(candidates.size())]] = remove ? 0 : 1;
    }
}

/// Speed in cells per car per step once all jams have dissolved: min(1, (1 - d) / d) for the car density d.
/// Below one half every car moves. Above it the gaps move (one cell backwards per step) and every gap releases exactly
/// one car, so (N - K) of the K cars move per step.
inline double theorySpeed(double density) {
    if (density <= 0.5) return 1.0;
    if (density >= 1.0) return 0.0;
    return (1.0 - density) / density;
}

/// Average speed over the last `capacity` steps, the displayed measurement. Clear it whenever the cars change.
class SpeedWindow {
public:
    explicit SpeedWindow(std::size_t capacity) : capacity_(capacity) {}

    void push(long hops, long cars) {
        if (capacity_ == 0) return;
        if (entries_.size() == capacity_) {
            hops_ -= entries_.front().first;
            cars_ -= entries_.front().second;
            entries_.pop_front();
        }
        entries_.emplace_back(hops, cars);
        hops_ += hops;
        cars_ += cars;
    }

    void clear() {
        entries_.clear();
        hops_ = 0;
        cars_ = 0;
    }

    std::size_t count() const { return entries_.size(); }
    bool ready(std::size_t minimum) const { return entries_.size() >= minimum; }

    /// Hops per car per step, 0 when there were no cars.
    double mean() const { return cars_ > 0 ? static_cast<double>(hops_) / static_cast<double>(cars_) : 0.0; }

private:
    std::size_t capacity_;
    std::deque<std::pair<long, long>> entries_;
    long hops_ = 0;
    long cars_ = 0;
};

/// Random start with `cars` cars, `relax` steps to let the jams settle, then the mean speed over `measure` steps.
inline double measureSpeed(std::size_t n, std::size_t cars, std::uint64_t seed, int relax, int measure) {
    Rng rng(seed);
    Road road = randomRoad(n, cars, rng);
    const long k = carCount(road);
    if (k == 0 || measure <= 0) return 0.0;
    for (int t = 0; t < relax; ++t) stepParallel(road);
    long hops = 0;
    for (int t = 0; t < measure; ++t) hops += stepParallel(road);
    return static_cast<double>(hops) / (static_cast<double>(k) * static_cast<double>(measure));
}

}  // namespace r184
