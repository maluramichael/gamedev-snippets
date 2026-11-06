// Tests for the easing functions and the Tween. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "logic.hpp"

using namespace ease;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

constexpr double kEps = 1e-12;

template <class F>
void forEveryEasing(F&& body) {
    for (int f = 0; f < kFamilyCount; ++f) {
        for (int v = 0; v < 3; ++v) body(static_cast<Family>(f), static_cast<Variant>(v), lookup(static_cast<Family>(f), static_cast<Variant>(v)));
    }
}

bool smoothFamily(Family f) { return f != Family::Back && f != Family::Elastic && f != Family::Bounce; }

// ---------------------------------------------------------------------------------------------------------------
// The functions
// ---------------------------------------------------------------------------------------------------------------

void endpoints() {
    forEveryEasing([](Family f, Variant, Fn e) {
        // Polynomials, expo and circ hit 0 and 1 exactly. The ones with sin/cos or a spring only to rounding.
        const bool exact = f == Family::Linear || f == Family::Quad || f == Family::Cubic || f == Family::Quart || f == Family::Quint || f == Family::Expo || f == Family::Circ;
        if (exact) {
            assert(e(0.0) == 0.0);
            assert(e(1.0) == 1.0);
        } else {
            assert(near(e(0.0), 0.0));
            assert(near(e(1.0), 1.0));
        }
    });
}

void handComputedValues() {
    assert(near(quadIn(0.5), 0.25, kEps));
    assert(near(quadOut(0.5), 0.75, kEps));
    assert(near(quadInOut(0.25), 0.125, kEps));  // 2 * 0.25^2
    assert(near(cubicOut(0.5), 0.875, kEps));    // 1 - 0.5^3
    assert(near(cubicIn(0.5), 0.125, kEps));
    assert(near(quartIn(0.5), 0.0625, kEps));
    assert(near(quintIn(0.5), 0.03125, kEps));
    assert(near(sineInOut(0.5), 0.5, kEps));
    assert(near(sineIn(0.5), 1.0 - std::sqrt(0.5), kEps));  // 1 - cos(pi/4)
    assert(near(sineOut(0.5), std::sqrt(0.5), kEps));
    assert(near(expoIn(0.5), 0.03125, kEps));   // 2^-5
    assert(near(expoOut(0.5), 0.96875, kEps));  // 1 - 2^-5
    assert(near(circIn(0.6), 0.2, kEps));       // 1 - sqrt(1 - 0.36) = 1 - 0.8
    assert(near(circOut(0.8), std::sqrt(0.96), kEps));
    // back: c1 = 1.70158, c3 = c1 + 1. backIn(0.5) = c3 * 0.125 - c1 * 0.25
    assert(near(backIn(0.5), -0.0876975, 1e-9));
    assert(near(backOut(0.5), 1.0876975, 1e-9));
    // bounce: 7.5625 * 0.25^2 and the second parabola 7.5625 * (0.5 - 1.5/2.75)^2 + 0.75
    assert(near(bounceOut(0.25), 0.47265625, kEps));
    assert(near(bounceOut(0.5), 0.765625, kEps));
    assert(near(bounceOut(1.0 / 2.75), 1.0, kEps));  // the first bounce touches 1 exactly
    // elastic: the sine factor is 0 at 10t - 0.75 = 0, so the curve crosses 1 at t = 0.075.
    assert(near(elasticOut(0.075), 1.0, kEps));
    assert(near(elasticIn(0.925), 0.0, kEps));
    // expo is special cased at the ends: 2^(10t-10) alone would be 0.000977 at t = 0.
    assert(expoIn(0.0) == 0.0);
    assert(expoOut(1.0) == 1.0);
    assert(expoInOut(0.0) == 0.0);
    assert(expoInOut(1.0) == 1.0);
    assert(expoIn(1e-6) < 0.001);
}

void outIsMirroredIn() {
    // easeOut(t) = 1 - easeIn(1 - t), for every family, including the springs.
    for (int f = 0; f < kFamilyCount; ++f) {
        const Fn in = lookup(static_cast<Family>(f), Variant::In);
        const Fn out = lookup(static_cast<Family>(f), Variant::Out);
        for (int i = 0; i <= 200; ++i) {
            const double t = i / 200.0;
            assert(near(out(t), 1.0 - in(1.0 - t)));
        }
    }
}

void inOutIsPointSymmetric() {
    // The two halves are mirror images around (0.5, 0.5): inOut(1 - t) = 1 - inOut(t). Also true for back and elastic.
    forEveryEasing([](Family, Variant v, Fn e) {
        if (v != Variant::InOut) return;
        assert(near(e(0.5), 0.5));
        for (int i = 0; i <= 200; ++i) {
            const double t = i / 200.0;
            assert(near(e(1.0 - t), 1.0 - e(t)));
        }
    });
}

void inOutIsComposition() {
    // For these families inOut is "in at double speed" for the first half and the mirror of it for the second.
    // Back and elastic use their own constants (c2, c5) for inOut and are left out on purpose.
    for (Family f : {Family::Sine, Family::Quad, Family::Cubic, Family::Quart, Family::Quint, Family::Expo, Family::Circ, Family::Bounce}) {
        const Fn in = lookup(f, Variant::In);
        const Fn out = lookup(f, Variant::Out);
        const Fn inOut = lookup(f, Variant::InOut);
        for (int i = 0; i <= 200; ++i) {
            const double t = i / 200.0;
            if (t < 0.5) {
                assert(near(inOut(t), in(2.0 * t) / 2.0));
            } else {
                assert(near(inOut(t), (1.0 + out(2.0 * t - 1.0)) / 2.0));
            }
        }
    }
}

void smoothFamiliesAreMonotone() {
    forEveryEasing([](Family f, Variant, Fn e) {
        if (!smoothFamily(f)) return;
        double previous = e(0.0);
        for (int i = 1; i <= 2000; ++i) {
            const double value = e(i / 2000.0);
            assert(value >= previous - kEps);
            previous = value;
        }
    });
}

void springsAreNotMonotone() {
    // Not monotone is the point of back, elastic and bounce: somewhere the value goes down while t goes up.
    for (Family f : {Family::Back, Family::Elastic, Family::Bounce}) {
        for (int v = 0; v < 3; ++v) {
            const Fn e = lookup(f, static_cast<Variant>(v));
            bool decreases = false;
            for (int i = 1; i <= 2000; ++i) decreases = decreases || e(i / 2000.0) < e((i - 1) / 2000.0) - 1e-9;
            assert(decreases);
        }
    }
}

Range sampled(Fn e) { return sampleRange(e, 4000); }

void overshoot() {
    // back leaves [0,1] by about 10 percent (that is what 1.70158 is for), in on the low side, out on the high side.
    const Range bi = sampled(backIn);
    const Range bo = sampled(backOut);
    const Range bio = sampled(backInOut);
    assert(bi.min < -0.09 && bi.min > -0.11 && near(bi.max, 1.0, 1e-9));
    assert(bo.max > 1.09 && bo.max < 1.11 && near(bo.min, 0.0, 1e-9));
    assert(bio.min < 0.0 && bio.max > 1.0);
    // elastic swings further: about 0.37 over the target (in) or under the start (out is above, in below).
    const Range ei = sampled(elasticIn);
    const Range eo = sampled(elasticOut);
    assert(ei.min < -0.3 && ei.min > -0.4);
    assert(eo.max > 1.3 && eo.max < 1.4);
    const Range eio = sampled(elasticInOut);
    assert(eio.min < 0.0 && eio.max > 1.0);
    // the plain families never leave [0,1]
    forEveryEasing([](Family f, Variant, Fn e) {
        if (!smoothFamily(f)) return;
        const Range r = sampled(e);
        assert(r.min >= -kEps && r.max <= 1.0 + kEps);
    });
}

void bounceStaysInRange() {
    for (int v = 0; v < 3; ++v) {
        const Range r = sampled(lookup(Family::Bounce, static_cast<Variant>(v)));
        assert(r.min >= -kEps && r.max <= 1.0 + kEps);
    }
}

void lookupAndNames() {
    assert(lookup(Family::Quad, Variant::In) == quadIn);
    assert(lookup(Family::Bounce, Variant::InOut) == bounceInOut);
    // linear has no variants, every variant is the identity
    for (int v = 0; v < 3; ++v) assert(near(lookup(Family::Linear, static_cast<Variant>(v))(0.3), 0.3, kEps));
    assert(std::string(name(Family::Elastic)) == "elastic");
    assert(std::string(name(Variant::InOut)) == "inOut");
    assert(near(apply(Family::Cubic, Variant::Out, 0.5), 0.875, kEps));
}

// ---------------------------------------------------------------------------------------------------------------
// Tween
// ---------------------------------------------------------------------------------------------------------------

void tweenValue() {
    Tween<double> t{10.0, 20.0, 2.0, linear};
    assert(near(t.value(), 10.0));  // before the first update: the start value
    t.update(1.0);
    assert(near(t.progress(), 0.5));
    assert(near(t.value(), 15.0));
    Tween<double> q{0.0, 8.0, 1.0, quadIn};
    q.update(0.5);
    assert(near(q.value(), 2.0));  // 8 * 0.25
    // Vec2 and Rgb mix component by component
    Tween<Vec2> p{{0, 0}, {10, 20}, 1.0, linear};
    p.update(0.5);
    assert(near(p.value().x, 5.0) && near(p.value().y, 10.0));
    Tween<Rgb> c{{0, 100, 200}, {100, 100, 0}, 1.0, linear};
    c.update(0.25);
    assert(near(c.value().r, 25.0) && near(c.value().g, 100.0) && near(c.value().b, 150.0));
    // The eased value may leave the range of the endpoints (back goes past the target).
    Tween<double> b{0.0, 100.0, 1.0, backOut};
    b.update(0.5);
    assert(b.value() > 100.0);
}

void tweenIsFrameRateIndependent() {
    // 1000 steps of 1 ms, 60 steps of 1/60 s and a single step of 1 s end up at the same value.
    for (Fn e : {linear, quadIn, cubicInOut, backOut, elasticOut, bounceOut, expoInOut}) {
        Tween<double> a{0.0, 10.0, 2.0, e};
        Tween<double> b{0.0, 10.0, 2.0, e};
        Tween<double> c{0.0, 10.0, 2.0, e};
        for (int i = 0; i < 1000; ++i) a.update(0.001);
        for (int i = 0; i < 60; ++i) b.update(1.0 / 60.0);
        c.update(1.0);
        assert(near(a.value(), c.value()));
        assert(near(b.value(), c.value()));
        assert(near(a.progress(), 0.5) && near(b.progress(), 0.5));
    }
}

void tweenClampsAndReturnsRest() {
    Tween<double> t{0.0, 10.0, 1.0, quadIn};
    assert(!t.finished());
    assert(t.update(0.75) == 0.0);
    assert(!t.finished());
    const double rest = t.update(0.5);  // 0.25 s of this step are left over
    assert(near(rest, 0.25));
    assert(t.finished());
    assert(t.progress() == 1.0);
    assert(t.value() == 10.0);  // exactly the end value, no rounding left
    // once finished, every further step is handed back whole and the value stays put
    assert(near(t.update(0.3), 0.3));
    assert(t.value() == 10.0);
    // restart goes back to the start
    t.restart();
    assert(!t.finished());
    assert(near(t.value(), 0.0));
    assert(near(t.progress(), 0.0));
}

void tweenIgnoresBackwardsTime() {
    Tween<double> t{0.0, 1.0, 1.0, linear};
    t.update(0.5);
    assert(t.update(-1.0) == 0.0);
    assert(near(t.progress(), 0.5));
}

void tweenLoop() {
    Tween<double> t{0.0, 1.0, 1.0, linear, Mode::Loop};
    assert(t.update(2.25) == 0.0);
    assert(!t.finished());
    assert(near(t.progress(), 0.25));
    t.update(0.75);  // lands exactly on the wrap, back at the start
    assert(near(t.progress(), 0.0));
}

void tweenPingPong() {
    Tween<double> t{0.0, 1.0, 1.0, linear, Mode::PingPong};
    t.update(0.25);
    assert(near(t.progress(), 0.25));
    t.update(1.0);  // 1.25 s: 0.25 s into the way back
    assert(near(t.progress(), 0.75));
    t.update(0.5);  // 1.75 s
    assert(near(t.progress(), 0.25));
    t.update(0.5);  // 2.25 s: the second forward run has started
    assert(near(t.progress(), 0.25));
    t.update(0.75);  // 3.0 s: turn around at the end
    assert(near(t.progress(), 1.0));
    t.update(0.25);
    assert(near(t.progress(), 0.75));
    // many small steps give the same result as one big one
    Tween<double> a{0.0, 1.0, 0.7, linear, Mode::PingPong};
    Tween<double> b{0.0, 1.0, 0.7, linear, Mode::PingPong};
    for (int i = 0; i < 500; ++i) a.update(0.01);
    b.update(5.0);
    assert(near(a.progress(), b.progress()));
}

void tweenZeroDuration() {
    Tween<double> t{3.0, 7.0, 0.0, quadIn};
    assert(t.progress() == 1.0);  // no division by zero, no NaN
    assert(t.value() == 7.0);
    assert(near(t.update(0.5), 0.5));  // takes no time at all, the whole step is left over
    assert(t.finished());
    Tween<double> l{3.0, 7.0, 0.0, linear, Mode::Loop};
    assert(l.update(1.0) == 0.0);  // must not hang or divide by zero
    assert(l.value() == 7.0);
    Tween<double> p{3.0, 7.0, 0.0, linear, Mode::PingPong};
    assert(p.update(1.0) == 0.0);
    assert(p.value() == 7.0);
    Tween<double> neg{3.0, 7.0, -1.0, linear};
    assert(neg.value() == 7.0);
}

void chainPassesRestOn() {
    Tween<double> a{0.0, 1.0, 1.0, linear};
    Tween<double> b{0.0, 1.0, 1.0, linear};
    Tween<double> c{0.0, 1.0, 1.0, linear};
    assert(updateChain(0.5, a, b, c) == 0.0);
    assert(near(a.progress(), 0.5) && near(b.progress(), 0.0) && near(c.progress(), 0.0));
    assert(!b.finished());
    // a needs 0.5 s more, the remaining 0.5 s of this step already belong to b
    assert(updateChain(1.0, a, b, c) == 0.0);
    assert(a.finished() && near(b.progress(), 0.5) && near(c.progress(), 0.0));
    // b needs 0.5, c gets 1.5 and uses 1.0, so 0.5 s come out at the end
    assert(near(updateChain(2.0, a, b, c), 0.5));
    assert(a.finished() && b.finished() && c.finished());
    assert(allFinished(a, b, c));
}

void chainIsFrameRateIndependent() {
    Tween<Vec2> move{{0, 0}, {100, 50}, 0.8, backOut};
    Tween<double> grow{1.0, 2.0, 0.6, elasticOut};
    Tween<Rgb> tint{{0, 0, 0}, {255, 128, 0}, 0.6, quadInOut};
    Tween<Vec2> move2 = move;
    Tween<double> grow2 = grow;
    Tween<Rgb> tint2 = tint;
    for (int i = 0; i < 100; ++i) updateChain(0.01, move, grow, tint);  // 1.0 s of 2.0 total
    updateChain(1.0, move2, grow2, tint2);
    assert(near(move.value().x, move2.value().x) && near(move.value().y, move2.value().y));
    assert(near(grow.progress(), grow2.progress()));
    assert(near(grow.value(), grow2.value()));
    assert(near(tint.progress(), tint2.progress()));
    // 1.0 s in: move (0.8) is done, grow (0.6) is 0.2 s in
    assert(move.finished() && near(grow.progress(), 0.2 / 0.6) && near(tint.progress(), 0.0));
}

void chainWithZeroDurationWaitsForItsTurn() {
    Tween<double> a{0.0, 1.0, 1.0, linear};
    Tween<double> jump{0.0, 5.0, 0.0, linear};  // instant
    Tween<double> c{0.0, 1.0, 1.0, linear};
    updateChain(0.5, a, jump, c);
    assert(!jump.finished());  // a is still running, the jump must not fire early
    assert(jump.progress() == 1.0);  // by definition a zero length tween is at its end, but it is not marked finished yet
    assert(near(c.progress(), 0.0));
    updateChain(0.75, a, jump, c);
    assert(a.finished() && jump.finished() && near(c.progress(), 0.25));
}

void chainWithNothingToDo() {
    Tween<double> a{0.0, 1.0, 1.0, linear};
    a.update(5.0);
    assert(near(updateChain(0.4, a), 0.4));  // all finished: the whole step comes back
    assert(updateChain(0.0, a) == 0.0);
}

// Fixed low frame rate for the chain: time is cut into equal chunks and the rest waits in the accumulator. That rest
// belongs to the old run, so a restart or a change of the chunk size has to throw it away.
void chunkClockCutsEqualChunks() {
    ChunkClock c;
    assert(c.advance(0.15, 0.2) == 0);  // not enough for one chunk yet
    assert(near(c.accumulator, 0.15));
    assert(c.advance(0.05, 0.2) == 1);  // 0.15 + 0.05 = exactly one chunk
    assert(near(c.accumulator, 0.0));
    assert(c.advance(0.5, 0.2) == 2);  // two chunks, 0.1 stays behind
    assert(near(c.accumulator, 0.1));
}

void chunkClockResetDropsTheRest() {
    ChunkClock c;
    c.advance(0.15, 0.2);  // 0.15 s of a run that is being restarted
    c.reset();
    assert(near(c.accumulator, 0.0));
    // Without the reset the first chunk would come after 0.05 s instead of after a full chunk.
    assert(c.advance(0.1, 0.2) == 0);
    assert(c.advance(0.1, 0.2) == 1);
}

void chunkClockIgnoresNonsense() {
    ChunkClock c;
    assert(c.advance(-1.0, 0.2) == 0);  // backwards time adds nothing
    assert(near(c.accumulator, 0.0));
    assert(c.advance(1.0, 0.0) == 0);  // a chunk of 0 would never drain
    assert(c.advance(1.0, -0.2) == 0);
}

}  // namespace

int main() {
    endpoints();
    handComputedValues();
    outIsMirroredIn();
    inOutIsPointSymmetric();
    inOutIsComposition();
    smoothFamiliesAreMonotone();
    springsAreNotMonotone();
    overshoot();
    bounceStaysInRange();
    lookupAndNames();
    tweenValue();
    tweenIsFrameRateIndependent();
    tweenClampsAndReturnsRest();
    tweenIgnoresBackwardsTime();
    tweenLoop();
    tweenPingPong();
    tweenZeroDuration();
    chainPassesRestOn();
    chainIsFrameRateIndependent();
    chainWithZeroDurationWaitsForItsTurn();
    chainWithNothingToDo();
    chunkClockCutsEqualChunks();
    chunkClockResetDropsTheRest();
    chunkClockIgnoresNonsense();
    std::puts("easing logic: all tests passed");
    return EXIT_SUCCESS;
}
