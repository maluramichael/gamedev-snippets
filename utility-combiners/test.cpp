// Tests for the utility AI logic. Plain asserts on hand-computed cases, no framework, no SDL.
// The last block runs the whole villager for a while and prints what it measured, those numbers go into the post.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace ua;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

Curve linear() { return Curve{CurveType::Linear, 0.5}; }
Curve quadratic() { return Curve{CurveType::Quadratic, 0.5}; }
Curve logistic(double mid) { return Curve{CurveType::Logistic, mid}; }

Settings with(Combiner c, double inertia = 0.0) {
    Settings s;
    s.combiner = c;
    s.inertia = inertia;
    return s;
}

Evaluation scores(double eat, double sleep, double work, double wander) {
    Evaluation e{};
    e[0].score = eat;
    e[1].score = sleep;
    e[2].score = work;
    e[3].score = wander;
    return e;
}

Action pick(const State& s, Combiner c, Action current = Action::Wander, double inertia = 0.0) {
    const Settings settings = with(c, inertia);
    return choose(evaluate(s, settings), current, inertia);
}

void curves() {
    assert(near(respond(linear(), 0.3), 0.3));
    assert(near(respond(linear(), -1.0), 0.0));  // outside the range is clamped
    assert(near(respond(linear(), 2.0), 1.0));
    assert(near(respond(linear(), std::nan("")), 0.0));  // NaN counts as 0

    assert(near(respond(quadratic(), 0.5), 0.25));
    assert(near(respond(quadratic(), 0.0), 0.0));
    assert(near(respond(quadratic(), 1.0), 1.0));
    assert(respond(quadratic(), 0.5) < respond(linear(), 0.5));

    // Logistic with the steep part in the middle: starts at 0, ends at 1, 0.5 in the middle, symmetric.
    assert(near(respond(logistic(0.5), 0.0), 0.0));
    assert(near(respond(logistic(0.5), 1.0), 1.0));
    assert(near(respond(logistic(0.5), 0.5), 0.5));
    assert(near(respond(logistic(0.5), 0.6), 0.73419, 1e-4));  // (s(1) - s(-5)) / (s(5) - s(-5)), computed by hand
    assert(near(respond(logistic(0.5), 0.3) + respond(logistic(0.5), 0.7), 1.0));
    assert(respond(logistic(0.5), 0.4) < 0.4 && respond(logistic(0.5), 0.6) > 0.6);  // steeper than the line there

    // The end points hold for every midpoint, and the curve never goes down.
    for (const double mid : {0.1, 0.5, 0.9}) {
        assert(near(respond(logistic(mid), 0.0), 0.0));
        assert(near(respond(logistic(mid), 1.0), 1.0));
        double last = 0.0;
        for (int i = 0; i <= 100; ++i) {
            const double y = respond(logistic(mid), i / 100.0);
            assert(y >= last - 1e-12 && y <= 1.0);
            last = y;
        }
    }
    // Moving the midpoint right makes the same input score lower.
    assert(respond(logistic(0.8), 0.5) < 0.5 && respond(logistic(0.2), 0.5) > 0.5);
    // A midpoint outside 0.05..0.95 behaves like the nearest allowed one.
    assert(near(respond(logistic(5.0), 0.7), respond(logistic(0.95), 0.7)));
    assert(near(respond(logistic(-1.0), 0.7), respond(logistic(0.05), 0.7)));
}

void combiners() {
    const double f[2] = {0.9, 0.0};
    assert(near(combine(Combiner::Mean, f, 2), 0.45));  // a zero factor only halves the score
    assert(near(combine(Combiner::Product, f, 2), 0.0));  // and here it kills it
    assert(near(combine(Combiner::Bands, f, 2), 0.0));
    const double g[2] = {1.0, 0.5};
    assert(near(combine(Combiner::Mean, g, 2), 0.75));
    assert(near(combine(Combiner::Product, g, 2), 0.5));
    const double h[2] = {0.5, 0.5};
    assert(near(combine(Combiner::Product, h, 2), 0.25));
    const double over[2] = {1.5, -2.0};  // out of range inputs are clamped first
    assert(near(combine(Combiner::Mean, over, 2), 0.5));
    assert(near(combine(Combiner::Product, over, 2), 0.0));
    assert(near(combine(Combiner::Mean, f, 0), 0.0));  // no factors, no score
    assert(near(combine(Combiner::Product, f, 0), 0.0));
    // More factors pull a product down even when each one is good: 0.8 * 0.8 * 0.8.
    const double three[3] = {0.8, 0.8, 0.8};
    assert(near(combine(Combiner::Product, three, 3), 0.512));
    assert(near(combine(Combiner::Mean, three, 3), 0.8));
}

void daytime() {
    assert(near(daylight(0.0), 0.5));
    assert(near(daylight(10.0), 1.0));  // noon: 0.5 + 0.8 is clamped to 1
    assert(near(daylight(30.0), 0.0));  // midnight: 0.5 - 0.8 is clamped to 0
    assert(near(daylight(5.0), 1.0));
    assert(near(daylight(10.0 + kDayLength), daylight(10.0)));
    assert(near(daylight(-10.0), daylight(30.0)));  // negative clocks wrap the same way
    assert(near(daylight(25.0), 0.0));  // sin(1.25 pi) = -0.7071, 0.5 - 0.5657 is below 0 and clamped
    assert(near(daylight(2.5), 0.80615, 1e-4));  // 0.5 + 0.8 * sin(pi / 8) = 0.5 + 0.8 * 0.38268
}

void evaluation() {
    // Night, hungry, rested, larder 0.1, drive 1. Hand computed:
    //   mean:    eat (1 + 0.1) / 2 = 0.55, sleep (0 + 1) / 2 = 0.5, work (1 + 0) / 2 = 0.5, wander 0.15
    //   product: eat 0.1, sleep 0, work 0, wander 0.15
    const State night = emptyLarderAtNight();
    const Evaluation mean = evaluate(night, with(Combiner::Mean));
    assert(near(mean[0].score, 0.55));
    assert(near(mean[1].score, 0.5));
    assert(near(mean[2].score, 0.5));
    assert(near(mean[3].score, 0.15));
    const Evaluation prod = evaluate(night, with(Combiner::Product));
    assert(near(prod[0].score, 0.1));
    assert(near(prod[1].score, 0.0));
    assert(near(prod[2].score, 0.0));
    assert(near(prod[3].score, 0.15));

    assert(pick(night, Combiner::Mean) == Action::Eat);  // the mean sends the villager to an empty larder
    assert(pick(night, Combiner::Product) == Action::Wander);  // every veto holds
    assert(pick(night, Combiner::Bands) == Action::Wander);  // 0.1 is far below the band edge

    // Noon, starving, drive 1: Eat 0.95 * 0.9 = 0.855 (mean 0.925) loses to work 1.0 unless bands step in.
    const State eager = eagerButStarving();
    assert(near(evaluate(eager, with(Combiner::Product))[0].score, 0.855));
    assert(near(evaluate(eager, with(Combiner::Mean))[0].score, 0.925));
    assert(near(evaluate(eager, with(Combiner::Product))[2].score, 1.0));
    assert(pick(eager, Combiner::Mean) == Action::Work);
    assert(pick(eager, Combiner::Product) == Action::Work);
    assert(pick(eager, Combiner::Bands) == Action::Eat);

    // Sleep needs darkness: at noon even an exhausted villager scores 0 for it under the product.
    const State tired{0.0, 1.0, 0.0, 1.0, 10.0};
    assert(near(evaluate(tired, with(Combiner::Product))[1].score, 0.0));
    assert(near(evaluate(tired, with(Combiner::Mean))[1].score, 0.5));

    // Wander has one factor and the same score under every combiner.
    for (const Combiner c : {Combiner::Mean, Combiner::Product, Combiner::Bands}) {
        assert(near(evaluate(night, with(c))[3].score, kWander));
    }
    // The curves feed the scores: quadratic hunger 0.5 -> 0.25, times a larder of 1.
    Settings q = with(Combiner::Product);
    q.hunger = quadratic();
    assert(near(evaluate(State{0.5, 0.0, 0.0, 1.0, 10.0}, q)[0].score, 0.25));
}

void bands() {
    // Hunger 1.0 and larder 0.6 give exactly 0.6, the edge counts as urgent. Just below does not.
    Settings s = with(Combiner::Bands);
    assert(evaluate(State{1.0, 0.0, 0.0, 0.6, 10.0}, s)[0].band == 1);
    assert(evaluate(State{1.0, 0.0, 0.0, 0.59, 10.0}, s)[0].band == 0);
    // Only survival actions get a band, work never does, however high its score.
    assert(evaluate(State{0.0, 0.0, 1.0, 1.0, 10.0}, s)[2].band == 0);
    assert(near(evaluate(State{0.0, 0.0, 1.0, 1.0, 10.0}, s)[2].score, 1.0));
    // Sleep is in the band at night when exhausted: 1.0 * 1.0.
    assert(evaluate(State{0.0, 1.0, 0.0, 0.0, 30.0}, s)[1].band == 1);
    // The other combiners never set a band.
    for (const Combiner c : {Combiner::Mean, Combiner::Product}) {
        assert(evaluate(eagerButStarving(), with(c))[0].band == 0);
    }
    // Both survival actions in the band: the higher score wins.
    Evaluation e = scores(0.7, 0.9, 1.0, 0.15);
    e[0].band = 1;
    e[1].band = 1;
    assert(choose(e, Action::Wander, 0.0) == Action::Sleep);
    // A band beats any score of a lower band, with or without inertia on the loser.
    Evaluation f = scores(0.6, 0.0, 1.0, 0.15);
    f[0].band = 1;
    assert(choose(f, Action::Work, 0.0) == Action::Eat);
    assert(choose(f, Action::Work, 0.15) == Action::Eat);
}

void inertiaAndTies() {
    const Evaluation e = scores(0.5, 0.0, 0.75, 0.0);  // binary fractions, so the sums below are exact
    assert(choose(e, Action::Eat, 0.0) == Action::Work);  // no bonus, the better score takes over
    assert(choose(e, Action::Eat, 0.5) == Action::Eat);  // 1.0 beats 0.75
    assert(choose(e, Action::Eat, 0.125) == Action::Work);  // 0.625 does not
    assert(choose(e, Action::Eat, 0.25) == Action::Eat);  // 0.75 ties with 0.75 and the running action wins ties
    assert(choose(e, Action::Work, 0.0) == Action::Work);
    // Without a bonus, equal scores keep the running action, and the first index wins when nothing runs yet.
    assert(choose(scores(0.5, 0.5, 0.5, 0.5), Action::Sleep, 0.0) == Action::Sleep);
    assert(choose(scores(0.0, 0.0, 0.7, 0.0), Action::Wander, 0.0) == Action::Work);
    assert(choose(scores(0.7, 0.0, 0.7, 0.0), Action::Wander, 0.0) == Action::Eat);

    // The bonus is added to the score, so it also lifts a vetoed action. Night, nothing needed, drive 1:
    // every score is 0 except wander 0.15. A bonus of 0.15 lets the working villager keep working in the dark.
    const State dark{0.0, 0.0, 1.0, 1.0, 30.0};
    assert(pick(dark, Combiner::Product, Action::Work, 0.0) == Action::Wander);
    assert(pick(dark, Combiner::Product, Action::Work, 0.10) == Action::Wander);
    assert(pick(dark, Combiner::Product, Action::Work, 0.15) == Action::Work);
}

void needs() {
    // One second from (hunger 0.5, fatigue 0.2, drive 0.6, larder 0.5), per action, computed by hand.
    const State base{0.5, 0.2, 0.6, 0.5, 5.0};
    State s = base;
    advance(s, Action::Eat, 1.0);
    assert(near(s.hunger, 0.35) && near(s.fatigue, 0.212) && near(s.drive, 0.63) && near(s.larder, 0.44));
    s = base;
    advance(s, Action::Work, 1.0);
    assert(near(s.hunger, 0.55) && near(s.fatigue, 0.237) && near(s.drive, 0.55) && near(s.larder, 0.55));
    s = base;
    advance(s, Action::Sleep, 1.0);
    assert(near(s.hunger, 0.515) && near(s.fatigue, 0.14) && near(s.drive, 0.63) && near(s.larder, 0.5));
    s = base;
    advance(s, Action::Wander, 1.0);
    assert(near(s.hunger, 0.53) && near(s.fatigue, 0.212) && near(s.drive, 0.63) && near(s.larder, 0.5));
    assert(near(s.clock, 6.0));

    // The clock wraps and every need stays in 0..1 however long one action runs.
    State w{0.0, 0.0, 0.0, 0.0, 39.95};
    advance(w, Action::Wander, 0.1);
    assert(near(w.clock, 0.05));
    for (const Action a : {Action::Eat, Action::Sleep, Action::Work, Action::Wander}) {
        State long1{0.5, 0.5, 0.5, 0.5, 0.0};
        for (int i = 0; i < 5000; ++i) {
            advance(long1, a, kStep);
            assert(long1.hunger >= 0.0 && long1.hunger <= 1.0);
            assert(long1.fatigue >= 0.0 && long1.fatigue <= 1.0);
            assert(long1.drive >= 0.0 && long1.drive <= 1.0);
            assert(long1.larder >= 0.0 && long1.larder <= 1.0);
            assert(long1.clock >= 0.0 && long1.clock < kDayLength);
        }
    }
}

void history() {
    History h;
    assert(h.size() == 0 && h.switches() == 0);
    for (const Action a : {Action::Eat, Action::Eat, Action::Work, Action::Work, Action::Eat}) {
        Sample s;
        s.chosen = a;
        h.push(s);
    }
    assert(h.size() == 5);
    assert(h.switches() == 2);
    assert(h.at(0).chosen == Action::Eat && h.at(2).chosen == Action::Work);

    // Past the capacity the oldest decision falls out, the order stays oldest first.
    History big;
    for (int i = 0; i < 700; ++i) {
        Sample s;
        s.chosen = i % 2 == 0 ? Action::Eat : Action::Work;
        s.score[0] = static_cast<float>(i);
        big.push(s);
    }
    assert(big.size() == kHistory);
    assert(near(static_cast<double>(big.at(0).score[0]), 100.0));
    assert(near(static_cast<double>(big.at(kHistory - 1).score[0]), 699.0));
    assert(big.switches() == static_cast<int>(kHistory) - 1);
}

void stepping() {
    Sim sim;
    sim.update(0.05);
    assert(sim.history.size() == 0);  // half a step is kept, not run
    sim.update(0.05);
    assert(sim.history.size() == 1);
    assert(near(sim.pending, 0.0));
    sim.update(10.0);  // a huge frame is clamped to 0.25 s: two steps, 0.05 s left
    assert(sim.history.size() == 3);
    assert(near(sim.pending, 0.05, 1e-9));
    sim.update(-1.0);  // time never runs backwards
    assert(sim.history.size() == 3);
    assert(near(sim.pending, 0.05, 1e-9));
}

struct Measured {
    int switches = 0;
    int sleptRested = 0;  // asleep with fatigue below 0.05
    int workedInDark = 0;  // working with daylight below 0.1
    int wandered = 0;
};

Measured measure(Settings settings, int ticks) {
    Sim sim;
    sim.settings = settings;
    Measured m;
    Action last = sim.current;
    for (int i = 0; i < ticks; ++i) {
        const State before = sim.state;
        sim.tick();
        m.switches += i > 0 && sim.current != last ? 1 : 0;
        last = sim.current;
        m.sleptRested += sim.current == Action::Sleep && before.fatigue < 0.05 ? 1 : 0;
        m.workedInDark += sim.current == Action::Work && daylight(before.clock) < 0.1 ? 1 : 0;
        m.wandered += sim.current == Action::Wander ? 1 : 0;
    }
    return m;
}

void simulation() {
    // Whatever the combiner, curve and bonus: needs stay in range, scores finite and in 0..1, the clock wraps.
    const Curve curveSets[5] = {linear(), quadratic(), logistic(0.5), logistic(0.1), logistic(0.9)};
    for (const Combiner c : {Combiner::Mean, Combiner::Product, Combiner::Bands}) {
        for (const Curve& curve : curveSets) {
            for (const double inertia : {0.0, 0.15}) {
                Sim sim;
                sim.settings = Settings{c, curve, curve, curve, inertia};
                for (int i = 0; i < 3000; ++i) {
                    sim.tick();
                    const State& s = sim.state;
                    assert(s.hunger >= 0.0 && s.hunger <= 1.0 && s.fatigue >= 0.0 && s.fatigue <= 1.0);
                    assert(s.drive >= 0.0 && s.drive <= 1.0 && s.larder >= 0.0 && s.larder <= 1.0);
                    assert(s.clock >= 0.0 && s.clock < kDayLength);
                    const Sample& last = sim.history.at(sim.history.size() - 1);
                    for (const float score : last.score) assert(std::isfinite(score) && score >= 0.0F && score <= 1.0F);
                }
                assert(sim.history.size() == kHistory);
            }
        }
    }

    // Measured over 120 s from the default start, printed for the post.
    const int ticks = 1200;
    for (const Combiner c : {Combiner::Mean, Combiner::Product, Combiner::Bands}) {
        const Measured plain = measure(with(c), ticks);
        const Measured bonus = measure(with(c, 0.15), ticks);
        std::printf("combiner %d: switches %d -> %d with bonus 0.15 | slept rested %d -> %d | worked in dark %d -> %d | wandered %d -> %d\n",
                    static_cast<int>(c), plain.switches, bonus.switches, plain.sleptRested, bonus.sleptRested, plain.workedInDark,
                    bonus.workedInDark, plain.wandered, bonus.wandered);
        assert(plain.switches > 100);  // without a bonus the villager flickers constantly
        assert(bonus.switches * 4 < plain.switches);  // with it, a fraction
    }
    // The product keeps every veto, the mean does not: at a zero score a villager can only keep doing something
    // by wandering, so product never sleeps while rested and never works in the dark.
    const Measured mean = measure(with(Combiner::Mean), ticks);
    const Measured product = measure(with(Combiner::Product), ticks);
    assert(product.sleptRested == 0 && product.workedInDark == 0);
    assert(mean.sleptRested > 0);
}

}  // namespace

int main() {
    curves();
    combiners();
    daytime();
    evaluation();
    bands();
    inertiaAndTies();
    needs();
    history();
    stepping();
    simulation();
    std::puts("utility-combiners logic: all tests passed");
    return 0;
}
