// The 2D wave equation on a height field, no SDL and no GL so it can be tested natively.
//
// Two buffers, `cur` and `prev`. The new height of a cell is the average of its four neighbours, doubled,
// minus its own height one step ago, then damped:
//
//     next = (left + right + up + down) / 2 - prev;   next *= damping;
//
// That is the textbook leapfrog update u' = 2u - prev + c2 * (sum4 - 4u) with c2 = 0.5 written out, and 0.5 is
// exactly the stability limit of the scheme in 2D (c*dt/dx <= 1/sqrt(2)). The border cells are never written,
// so they stay 0 and reflect the waves.
#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>

namespace gd::water {

inline constexpr float kCourant2 = 0.5F;  // (c * dt / dx)^2 at the stability limit

class Field {
public:
    Field(int width, int height)
        : width_(std::max(width, 1)), height_(std::max(height, 1)), cur_(static_cast<std::size_t>(width_ * height_), 0.0F), prev_(cur_) {}

    int width() const { return width_; }
    int height() const { return height_; }
    float at(int x, int y) const { return cur_[index(x, y)]; }
    float previous(int x, int y) const { return prev_[index(x, y)]; }
    const std::vector<float>& heights() const { return cur_; }

    /// Adds a raised-cosine bump (radius in cells) around (cx, cy). Cells outside the interior are skipped.
    void drop(float cx, float cy, float radius, float strength) {
        if (!(radius > 0.0F) || width_ < 3 || height_ < 3) return;
        const int x0 = static_cast<int>(std::max(1.0F, std::ceil(std::min(cx - radius, 1.0e6F))));
        const int x1 = static_cast<int>(std::min(static_cast<float>(width_ - 2), std::floor(std::max(cx + radius, -1.0e6F))));
        const int y0 = static_cast<int>(std::max(1.0F, std::ceil(std::min(cy - radius, 1.0e6F))));
        const int y1 = static_cast<int>(std::min(static_cast<float>(height_ - 2), std::floor(std::max(cy + radius, -1.0e6F))));
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const float d = std::hypot(static_cast<float>(x) - cx, static_cast<float>(y) - cy);
                if (d < radius) cur_[index(x, y)] += strength * 0.5F * (1.0F + std::cos(3.14159265F * d / radius));
            }
        }
    }

    /// Adds `value` to one interior cell (used by the tests for hand-computed cases).
    void impulse(int x, int y, float value) {
        if (x > 0 && y > 0 && x < width_ - 1 && y < height_ - 1) cur_[index(x, y)] += value;
    }

    /// One fixed simulation step. `courant2` stays at its default except in the test that shows what happens above 0.5.
    void step(float damping, float courant2 = kCourant2) {
        for (int y = 1; y < height_ - 1; ++y) {
            const float* up = &cur_[index(0, y - 1)];
            const float* mid = &cur_[index(0, y)];
            const float* down = &cur_[index(0, y + 1)];
            float* old = &prev_[index(0, y)];
            for (int x = 1; x < width_ - 1; ++x) {
                const float laplace = up[x] + down[x] + mid[x - 1] + mid[x + 1] - 4.0F * mid[x];
                old[x] = (2.0F * mid[x] - old[x] + courant2 * laplace) * damping;  // written over prev, which is not needed anymore
            }
        }
        std::swap(cur_, prev_);
    }

    /// Conserved by the undamped scheme: |cur - prev|^2 plus the gradient term c2 * <grad cur, grad prev>.
    double energy(float courant2 = kCourant2) const {
        double sum = 0.0;
        for (std::size_t i = 0; i < cur_.size(); ++i) {
            const double v = static_cast<double>(cur_[i]) - static_cast<double>(prev_[i]);
            sum += v * v;
        }
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (x + 1 < width_) sum += courant2 * edge(index(x, y), index(x + 1, y));
                if (y + 1 < height_) sum += courant2 * edge(index(x, y), index(x, y + 1));
            }
        }
        return sum;
    }

    float maxAbs() const {
        float m = 0.0F;
        for (const float v : cur_) m = std::max(m, std::fabs(v));
        return m;
    }

private:
    int width_;
    int height_;
    std::vector<float> cur_;
    std::vector<float> prev_;

    std::size_t index(int x, int y) const {
        assert(x >= 0 && y >= 0 && x < width_ && y < height_);
        return static_cast<std::size_t>(y * width_ + x);
    }

    double edge(std::size_t a, std::size_t b) const {
        return (static_cast<double>(cur_[a]) - static_cast<double>(cur_[b])) * (static_cast<double>(prev_[a]) - static_cast<double>(prev_[b]));
    }
};

}  // namespace gd::water
