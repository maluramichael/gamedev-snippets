// Tests for the Gerstner wave math in logic.hpp. The shader runs the same formulas, so these cases are the proof that
// the CPU side (buoy, floating objects) sees the same water as the GPU. Hand-computed cases first, then properties.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                   \
        }                                                                 \
    } while (0)

bool near(double a, double b, double eps = 1e-4) { return std::fabs(a - b) <= eps; }
bool near(const gw::Vec3& a, const gw::Vec3& b, double eps = 1e-4) { return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps); }

constexpr double kPi = 3.14159265358979323846;

using gw::Vec3;
using gw::Wave;
using gw::Waves;

// One wave along +x: wavelength 10, amplitude 0.5, steepness q.
Wave alongX(float q) { return Wave{1.0F, 0.0F, 10.0F, 0.5F, q, 0.0F}; }

// Deterministic points for the property tests.
struct Lcg {
    std::uint32_t s = 2463534242U;
    float next(float lo, float hi) {
        s = s * 1664525U + 1013904223U;
        return lo + (hi - lo) * (static_cast<float>(s >> 8) / 16777216.0F);
    }
};

// ------------------------------------------------------------------------------------------ dispersion

// omega = sqrt(g k). With k = 1/g it is exactly 1, which gives T = 2 pi and c = omega / k = g.
void dispersionHandComputed() {
    const float k = 1.0F / gw::kGravity;
    CHECK(near(gw::dispersion(k), 1.0));
    CHECK(near(gw::dispersion(1.0F), std::sqrt(9.81), 1e-5));
    CHECK(near(gw::dispersion(0.0F), 0.0));
    CHECK(near(gw::dispersion(-3.0F), 0.0));  // no negative wave numbers

    Wave w;
    w.wavelength = 2.0F * static_cast<float>(kPi) * gw::kGravity;  // k = 1/g
    CHECK(near(gw::wavenumber(w), 1.0 / 9.81, 1e-6));
    CHECK(near(gw::omega(w), 1.0, 1e-5));
    CHECK(near(gw::period(w), 2.0 * kPi, 1e-4));    // T = 2 pi / omega
    CHECK(near(gw::phaseSpeed(w), 9.81, 1e-3));     // c = omega / k = sqrt(g / k)
}

// A 22 m swell: T = sqrt(2 pi L / g) = 3.754 s, c = L / T = 5.86 m/s. Long waves run faster (c grows with sqrt(L)).
void longWavesRunFaster() {
    Wave w = alongX(0.0F);
    w.wavelength = 22.0F;
    CHECK(near(gw::period(w), std::sqrt(2.0 * kPi * 22.0 / 9.81), 1e-4));
    CHECK(near(gw::phaseSpeed(w), 22.0 / gw::period(w), 1e-4));
    Wave a = w, b = w;
    a.wavelength = 100.0F;
    b.wavelength = 10.0F;
    CHECK(gw::phaseSpeed(a) > gw::phaseSpeed(b));
    CHECK(near(gw::phaseSpeed(a) / gw::phaseSpeed(b), std::sqrt(10.0), 1e-4));
}

// ------------------------------------------------------------------------------------------ single wave by hand

// Wave along +x, L = 10, A = 0.5, Q = 0.4, t = 0, no phase offset: theta = k x.
//   x = 0:       theta = 0       -> y = 0,     horizontal shift Q A cos 0 = 0.2 in +x
//   x = L/4:     theta = pi/2    -> y = +A,    shift 0 (crest)
//   x = L/2:     theta = pi      -> y = 0,     shift -0.2
//   x = 3 L / 4: theta = 3 pi/2  -> y = -A,    shift 0 (trough)
void singleWaveByHand() {
    const Waves w{alongX(0.4F)};
    CHECK(near(gw::displace(w, 0.0F, 0.0F, 0.0), Vec3{0.2F, 0.0F, 0.0F}));
    CHECK(near(gw::displace(w, 2.5F, 0.0F, 0.0), Vec3{2.5F, 0.5F, 0.0F}));
    CHECK(near(gw::displace(w, 5.0F, 0.0F, 0.0), Vec3{4.8F, 0.0F, 0.0F}));
    CHECK(near(gw::displace(w, 7.5F, 0.0F, 0.0), Vec3{7.5F, -0.5F, 0.0F}));
    // z does not matter for a wave along x
    CHECK(near(gw::displace(w, 2.5F, 33.0F, 0.0), Vec3{2.5F, 0.5F, 33.0F}));
}

// Crest +A, trough -A, and the sum of all waves never exceeds the sum of the amplitudes.
void heightRange() {
    const Waves w{alongX(0.7F)};
    float lo = 0.0F, hi = 0.0F;
    for (int i = 0; i < 1000; ++i) {
        const float y = gw::displace(w, static_cast<float>(i) * 0.01F, 0.0F, 0.0).y;
        lo = std::min(lo, y);
        hi = std::max(hi, y);
    }
    CHECK(near(hi, 0.5, 1e-3));
    CHECK(near(lo, -0.5, 1e-3));

    const Waves set = gw::makeWaves(gw::kStorm);
    const float limit = gw::maxHeight(set);
    Lcg rng;
    float seen = 0.0F;
    for (int i = 0; i < 4000; ++i) {
        const float y = gw::displace(set, rng.next(-50.0F, 50.0F), rng.next(-50.0F, 50.0F), static_cast<double>(rng.next(0.0F, 60.0F))).y;
        CHECK(std::fabs(y) <= limit + 1e-4F);
        seen = std::max(seen, std::fabs(y));
    }
    CHECK(seen > 0.5F * limit);  // and the waves really do get tall
}

// The crest travels with c = omega / k: after t seconds it sits at x = L/4 + c t.
void crestTravelsAtPhaseSpeed() {
    const Waves w{alongX(0.0F)};
    const double c = gw::phaseSpeed(w[0]);
    for (const double t : {0.0, 0.5, 1.0, 2.7}) {
        const float xCrest = static_cast<float>(2.5 + c * t);
        CHECK(near(gw::displace(w, xCrest, 0.0F, t).y, 0.5, 1e-3));
    }
    // A direction that is not an axis: the crest line moves along D.
    Wave d = alongX(0.0F);
    d.dirX = 0.6F;
    d.dirZ = 0.8F;
    const Waves wd{d};
    const double cd = gw::phaseSpeed(d);
    const double s = 2.5 + cd * 1.5;  // distance of the crest from the origin along D at t = 1.5
    CHECK(near(gw::displace(wd, static_cast<float>(0.6 * s), static_cast<float>(0.8 * s), 1.5).y, 0.5, 1e-3));
}

// ------------------------------------------------------------------------------------------ periodicity

void periodicInSpaceAndTime() {
    Wave d = alongX(0.5F);
    d.dirX = 0.6F;
    d.dirZ = 0.8F;
    const Waves w{d};
    const float L = d.wavelength;
    const double T = gw::period(d);
    Lcg rng;
    for (int i = 0; i < 200; ++i) {
        const float x = rng.next(-20.0F, 20.0F), z = rng.next(-20.0F, 20.0F);
        const double t = rng.next(0.0F, 10.0F);
        const Vec3 base = gw::displace(w, x, z, t);
        // one wavelength further along D: same height, same offset from the rest position
        const Vec3 shifted = gw::displace(w, x + 0.6F * L, z + 0.8F * L, t);
        CHECK(near(shifted.y, base.y, 2e-4));
        CHECK(near(shifted.x - (x + 0.6F * L), base.x - x, 2e-4));
        CHECK(near(shifted.z - (z + 0.8F * L), base.z - z, 2e-4));
        // sideways along the crest: nothing changes
        CHECK(near(gw::displace(w, x - 0.8F * 7.0F, z + 0.6F * 7.0F, t).y, base.y, 2e-4));
        // one period later
        CHECK(near(gw::displace(w, x, z, t + T), base, 2e-4));
    }
}

// The time handed to the shader is reduced to one turn on the CPU, in double. After a day it is still precise.
void timePhaseStaysPrecise() {
    const Wave w = alongX(0.0F);
    CHECK(near(gw::timePhase(w, 0.0), 0.0));
    CHECK(near(gw::timePhase(w, gw::period(w)), 0.0, 1e-4) || near(gw::timePhase(w, gw::period(w)), 2.0 * kPi, 1e-4));
    const double day = 86400.0;
    const double ph = gw::timePhase(w, day);
    CHECK(ph >= 0.0 && ph < 2.0 * kPi);
    CHECK(near(ph, std::fmod(static_cast<double>(gw::omega(w)) * day, 2.0 * kPi), 1e-9));
}

// Displacements add up: the set is the sum of its waves.
void superposition() {
    const Wave a = alongX(0.3F);
    Wave b = alongX(0.2F);
    b.dirX = 0.0F;
    b.dirZ = 1.0F;
    b.wavelength = 7.0F;
    b.amplitude = 0.2F;
    b.phase = 1.1F;
    Lcg rng;
    for (int i = 0; i < 50; ++i) {
        const float x = rng.next(-9.0F, 9.0F), z = rng.next(-9.0F, 9.0F);
        const double t = rng.next(0.0F, 5.0F);
        const Vec3 pa = gw::displace({a}, x, z, t);
        const Vec3 pb = gw::displace({b}, x, z, t);
        const Vec3 both = gw::displace({a, b}, x, z, t);
        CHECK(near(both.x, pa.x + pb.x - x));
        CHECK(near(both.y, pa.y + pb.y));
        CHECK(near(both.z, pa.z + pb.z - z));
    }
    CHECK(near(gw::displace({}, 3.0F, 4.0F, 1.0), Vec3{3.0F, 0.0F, 4.0F}));  // no waves, flat water
}

// ------------------------------------------------------------------------------------------ normals

Vec3 numericNormal(const Waves& w, float x, float z, double t) {
    constexpr float h = 0.02F;
    const Vec3 tx = (gw::displace(w, x + h, z, t) - gw::displace(w, x - h, z, t)) * (1.0F / (2.0F * h));
    const Vec3 tz = (gw::displace(w, x, z + h, t) - gw::displace(w, x, z - h, t)) * (1.0F / (2.0F * h));
    return gw::normalize(gw::cross(tz, tx));
}

// A hand case: single wave along x, theta = 0 (x = 0, t = 0). Tangent along x is (1, kA, 0), along z (0, 0, 1),
// so the normal is (-kA, 1, 0) / sqrt(1 + (kA)^2).
void normalByHand() {
    const Waves w{alongX(0.4F)};
    const float kA = gw::wavenumber(w[0]) * 0.5F;  // 0.31416
    const float len = std::sqrt(1.0F + kA * kA);
    CHECK(near(gw::normal(w, 0.0F, 0.0F, 0.0), Vec3{-kA / len, 1.0F / len, 0.0F}));
    CHECK(near(gw::normal(w, 2.5F, 0.0F, 0.0), Vec3{0.0F, 1.0F, 0.0F}));  // crest: flat for a moment
    CHECK(near(gw::normal(w, 7.5F, 0.0F, 0.0), Vec3{0.0F, 1.0F, 0.0F}));  // trough as well
    CHECK(near(gw::normal(w, 5.0F, 0.0F, 0.0), Vec3{kA / len, 1.0F / len, 0.0F}));  // falling flank leans the other way
}

// The analytic normal equals the central difference of displace(), for every preset, everywhere, at any time.
void analyticNormalMatchesNumeric() {
    for (const gw::Preset& p : {gw::kCalm, gw::kBreeze, gw::kStorm}) {
        const Waves w = gw::makeWaves(p);
        Lcg rng;
        for (int i = 0; i < 300; ++i) {
            const float x = rng.next(-30.0F, 30.0F), z = rng.next(-30.0F, 30.0F);
            const double t = rng.next(0.0F, 40.0F);
            const Vec3 a = gw::normal(w, x, z, t);
            const Vec3 n = numericNormal(w, x, z, t);
            CHECK(near(gw::length(a), 1.0, 1e-5));
            CHECK(near(a, n, 4e-3));
            CHECK(a.y > 0.0F);
        }
    }
}

// The y component of the unnormalised normal IS the Jacobian of the horizontal map. That is why foam can use it.
void normalYIsTheJacobian() {
    const Waves w = gw::makeWaves(gw::kStorm);
    Lcg rng;
    for (int i = 0; i < 100; ++i) {
        const float x = rng.next(-30.0F, 30.0F), z = rng.next(-30.0F, 30.0F);
        const double t = rng.next(0.0F, 40.0F);
        const gw::Frame f = gw::frame(w, x, z, t);
        CHECK(near(f.jacobian * 1.0F, f.rawNormal.y, 1e-5));
        CHECK(near(gw::normalize(f.rawNormal), f.normal, 1e-5));
    }
}

// ------------------------------------------------------------------------------------------ steepness limit

// Single wave: J = 1 - Q k A sin(theta). At the crest (theta = pi/2) that is 1 - S exactly.
void jacobianByHand() {
    const float k = gw::wavenumber(alongX(0.0F));
    const float q = 0.4F;
    const Waves w{alongX(q)};
    const float s = q * k * 0.5F;  // Q k A
    CHECK(near(gw::sharpness(w[0]), s, 1e-6));
    CHECK(near(gw::jacobian(w, 2.5F, 0.0F, 0.0), 1.0 - s, 1e-5));   // crest
    CHECK(near(gw::jacobian(w, 7.5F, 0.0F, 0.0), 1.0 + s, 1e-5));   // trough: the surface is stretched
    CHECK(near(gw::jacobian(w, 0.0F, 0.0F, 0.0), 1.0, 1e-5));       // zero crossing
}

void steepnessBudget() {
    const float k = gw::wavenumber(alongX(0.0F));
    const float qExactlyOne = 1.0F / (k * 0.5F);  // Q k A = 1
    CHECK(gw::steepnessOk({alongX(0.0F)}));
    CHECK(gw::steepnessOk({alongX(0.5F * qExactlyOne)}));
    CHECK(gw::steepnessOk({alongX(qExactlyOne * 0.999F)}));
    CHECK(!gw::steepnessOk({alongX(qExactlyOne * 1.2F)}));
    CHECK(near(gw::steepnessSum({alongX(0.5F * qExactlyOne), alongX(0.25F * qExactlyOne)}), 0.75, 1e-5));
    CHECK(gw::steepnessOk({}));
    for (const gw::Preset& p : {gw::kCalm, gw::kBreeze, gw::kStorm}) {
        CHECK(gw::steepnessOk(gw::makeWaves(p)));
    }
}

// Smallest Jacobian and a check for folded rows over a scan of space and time.
float minJacobian(const Waves& w) {
    float lowest = 1.0e9F;
    for (int ti = 0; ti < 6; ++ti) {
        for (int zi = -60; zi < 60; ++zi) {
            for (int xi = -60; xi < 60; ++xi) {
                lowest = std::min(lowest, gw::jacobian(w, static_cast<float>(xi) * 0.4F, static_cast<float>(zi) * 0.4F, ti * 1.7));
            }
        }
    }
    return lowest;
}

// Within the budget the surface never folds: J stays >= 0 for all presets. Above it, loops appear.
void overTheLimitTheSurfaceFolds() {
    for (const gw::Preset& p : {gw::kCalm, gw::kBreeze, gw::kStorm}) {
        CHECK(minJacobian(gw::makeWaves(p)) >= -1e-5F);
    }
    CHECK(minJacobian(gw::makeWaves(gw::kStorm)) < 0.35F);  // the storm is squeezed hard (about 0.26 in this scan)

    // Single wave, S = 0.9: x -> X(x) is strictly increasing. S = 1.5: it runs backwards, rows cross each other.
    const float k = gw::wavenumber(alongX(0.0F));
    const float qPerS = 1.0F / (k * 0.5F);
    for (const float s : {0.9F, 1.5F}) {
        const Waves w{alongX(s * qPerS)};
        bool monotone = true;
        float negativeJ = 1.0F;
        float previous = gw::displace(w, 0.0F, 0.0F, 0.0).x;
        for (int i = 1; i <= 2000; ++i) {
            const float x = static_cast<float>(i) * 0.005F;
            const float now = gw::displace(w, x, 0.0F, 0.0).x;
            monotone = monotone && now > previous;
            previous = now;
            negativeJ = std::min(negativeJ, gw::jacobian(w, x, 0.0F, 0.0));
        }
        if (s < 1.0F) {
            CHECK(monotone);
            CHECK(near(negativeJ, 1.0 - s, 1e-3));
        } else {
            CHECK(!monotone);
            CHECK(near(negativeJ, 1.0 - s, 1e-3));  // 1 - 1.5 = -0.5 at the crest
            CHECK(gw::normal(w, 2.5F, 0.0F, 0.0).y < 0.0F);  // the normal flips over at the fold
        }
    }

    // The storm preset with twice the steepness (sum 1.91) folds somewhere, too. The budget is a sufficient condition,
    // not a tight one: with six waves from different directions the first loops show up at about 1.9, not at 1.0.
    gw::Preset over = gw::kStorm;
    over.steepness *= 2.0F;
    const Waves w = gw::makeWaves(over);
    CHECK(!gw::steepnessOk(w));
    CHECK(minJacobian(w) < 0.0F);
}

// ------------------------------------------------------------------------------------------ floating objects

// Newton on the horizontal map: find the rest position whose displaced position is a given world position.
void restPointInvertsTheDisplacement() {
    for (const gw::Preset& p : {gw::kCalm, gw::kBreeze, gw::kStorm}) {
        const Waves w = gw::makeWaves(p);
        Lcg rng;
        for (int i = 0; i < 300; ++i) {
            const float wx = rng.next(-20.0F, 20.0F), wz = rng.next(-20.0F, 20.0F);
            const double t = rng.next(0.0F, 30.0F);
            const gw::Rest r = gw::restPoint(w, wx, wz, t);
            const Vec3 q = gw::displace(w, r.x, r.z, t);
            CHECK(near(q.x, wx, 2e-3));
            CHECK(near(q.z, wz, 2e-3));
        }
    }
    CHECK(near(gw::restPoint({}, 3.0F, -2.0F, 0.0).x, 3.0));  // flat water: rest point = world point
}

void buoyOnFlatWater() {
    const gw::BuoyPose pose = gw::buoy({}, 2.0F, -3.0F, 5.0);
    CHECK(near(pose.position, Vec3{2.0F, 0.0F, -3.0F}));
    CHECK(near(pose.up, Vec3{0.0F, 1.0F, 0.0F}));
}

// Single wave along x, no steepness: the buoy at x = 0 sits at theta = -omega t. At t = 0 it is on the rising flank,
// its up vector leans back by atan(kA); a quarter period later the crest passes under it and it stands upright at +A.
void buoyByHand() {
    const Waves w{alongX(0.0F)};
    const float kA = gw::wavenumber(w[0]) * 0.5F;
    const float len = std::sqrt(1.0F + kA * kA);
    const gw::BuoyPose p0 = gw::buoy(w, 0.0F, 0.0F, 0.0);
    CHECK(near(p0.position, Vec3{0.0F, 0.0F, 0.0F}));
    CHECK(near(p0.up, Vec3{-kA / len, 1.0F / len, 0.0F}));

    const double quarter = gw::period(w[0]) / 4.0;
    // theta = -pi/2 at t = T/4 for x = 0: that is a trough. At t = 3T/4 theta = -3 pi/2 = +pi/2: crest.
    CHECK(near(gw::buoy(w, 0.0F, 0.0F, quarter).position.y, -0.5, 1e-3));
    CHECK(near(gw::buoy(w, 0.0F, 0.0F, 3.0 * quarter).position.y, 0.5, 1e-3));
    CHECK(near(gw::buoy(w, 0.0F, 0.0F, 3.0 * quarter).up, Vec3{0.0F, 1.0F, 0.0F}, 1e-3));
}

// On a real wave set the buoy is exactly the surface: position = displace(rest point), up = analytic normal,
// and it is always where it is moored (x and z do not drift).
void buoyFollowsTheSurface() {
    const Waves w = gw::makeWaves(gw::kBreeze);
    for (int i = 0; i < 100; ++i) {
        const double t = 0.37 * i;
        const gw::BuoyPose p = gw::buoy(w, -1.5F, 4.0F, t);
        const gw::Rest r = gw::restPoint(w, -1.5F, 4.0F, t);
        CHECK(near(p.position, gw::displace(w, r.x, r.z, t), 1e-4));
        CHECK(near(p.up, gw::normal(w, r.x, r.z, t), 1e-4));
        CHECK(near(p.position.x, -1.5, 2e-3));
        CHECK(near(p.position.z, 4.0, 2e-3));
        CHECK(std::fabs(p.position.y) <= gw::maxHeight(w) + 1e-3F);
    }
}

// ------------------------------------------------------------------------------------------ presets and clock

void presets() {
    const Waves calm = gw::makeWaves(gw::kCalm), breeze = gw::makeWaves(gw::kBreeze), storm = gw::makeWaves(gw::kStorm);
    CHECK(calm.size() == breeze.size());
    CHECK(breeze.size() == storm.size());
    CHECK(breeze.size() >= 4 && breeze.size() <= static_cast<std::size_t>(gw::kMaxWaves));
    CHECK(gw::maxHeight(calm) < gw::maxHeight(breeze));
    CHECK(gw::maxHeight(breeze) < gw::maxHeight(storm));
    CHECK(gw::steepnessSum(calm) < gw::steepnessSum(breeze));
    CHECK(gw::steepnessSum(breeze) < gw::steepnessSum(storm));
    CHECK(gw::steepnessSum(storm) <= 1.0F);
    for (const Wave& w : storm) CHECK(near(w.dirX * w.dirX + w.dirZ * w.dirZ, 1.0, 1e-5));  // unit directions
    // blending between presets stays inside the budget, too
    for (int i = 0; i <= 10; ++i) {
        const gw::Preset m = gw::mix(gw::kBreeze, gw::kStorm, static_cast<float>(i) / 10.0F);
        CHECK(gw::steepnessOk(gw::makeWaves(m)));
    }
    CHECK(near(gw::mix(gw::kCalm, gw::kStorm, 0.0F).amplitude, gw::kCalm.amplitude));
    CHECK(near(gw::mix(gw::kCalm, gw::kStorm, 1.0F).amplitude, gw::kStorm.amplitude));
    CHECK(near(gw::mix(gw::kCalm, gw::kStorm, 0.5F).steepness, 0.5 * (gw::kCalm.steepness + gw::kStorm.steepness)));
}

// Accumulated frame time: a stall of two seconds must not throw the water two seconds forward, pause freezes it.
void clock() {
    gw::WaveClock c;
    CHECK(near(c.advance(0.016, false), 0.016, 1e-12));
    CHECK(near(c.time, 0.016, 1e-12));
    c.advance(2.0, false);
    CHECK(near(c.time, 0.016 + gw::WaveClock::kMaxFrame, 1e-12));
    const double before = c.time;
    CHECK(c.advance(0.5, true) == 0.0);
    CHECK(c.time == before);
    c.advance(-1.0, false);  // a negative frame time (clock hiccup) changes nothing
    CHECK(c.time == before);
    for (int i = 0; i < 360000; ++i) c.advance(1.0 / 60.0, false);  // 100 minutes at 60 fps
    CHECK(near(c.time, before + 6000.0, 1e-3));
}

// ------------------------------------------------------------------------------------------ meshes

void gridMesh() {
    const int n = 5;
    const std::vector<float> pos = gw::gridPositions(n, 8.0F);
    CHECK(pos.size() == 50U);
    CHECK(near(pos[0], -4.0) && near(pos[1], -4.0));   // first vertex: corner
    CHECK(near(pos[48], 4.0) && near(pos[49], 4.0));   // last vertex: opposite corner
    CHECK(near(pos[2], -2.0) && near(pos[3], -4.0));   // spacing 8 / 4 = 2, x runs first
    const std::vector<std::uint16_t> tri = gw::gridTriangles(n);
    CHECK(tri.size() == 3U * 2U * 4U * 4U);            // (n-1)^2 quads, 2 triangles each
    CHECK(*std::max_element(tri.begin(), tri.end()) == n * n - 1);
    // Every triangle faces up (counter clockwise seen from +y): the y part of its normal is positive.
    bool allUp = true;
    for (std::size_t i = 0; i < tri.size(); i += 3) {
        const auto at = [&](std::uint16_t v) { return Vec3{pos[2U * v], 0.0F, pos[2U * v + 1U]}; };
        const Vec3 a = at(tri[i]), b = at(tri[i + 1]), c = at(tri[i + 2]);
        allUp = allUp && gw::cross(b - a, c - a).y > 0.0F;
    }
    CHECK(allUp);
    const std::vector<std::uint16_t> lines = gw::gridLines(n);
    CHECK(lines.size() == 2U * (2U * 5U * 4U));        // 2 * n * (n-1) edges, 2 indices each
    CHECK(*std::max_element(lines.begin(), lines.end()) == n * n - 1);

    // The demo size: 201 x 201 vertices fit 16 bit indices.
    CHECK(gw::gridPositions(201, 100.0F).size() == 201U * 201U * 2U);
    CHECK(gw::gridTriangles(201).size() == 6U * 200U * 200U);
    CHECK(gw::gridPositions(1, 10.0F).size() == 2U);   // degenerate sizes do not crash
    CHECK(gw::gridTriangles(1).empty());
    CHECK(gw::gridLines(0).empty());
}

void buoyMesh() {
    const gw::Mesh m = gw::buoyMesh(12);
    CHECK(!m.v.empty());
    CHECK(m.idx.size() % 3U == 0U);
    CHECK(!m.idx.empty());
    CHECK(*std::max_element(m.idx.begin(), m.idx.end()) < m.v.size());
    float lo = 1e9F, hi = -1e9F, wide = 0.0F;
    for (const Vec3& v : m.v) {
        lo = std::min(lo, v.y);
        hi = std::max(hi, v.y);
        wide = std::max(wide, std::hypot(v.x, v.z));
    }
    CHECK(lo < 0.0F && hi > 1.0F);  // hull below the waterline, mast above
    CHECK(wide > 0.3F && wide < 0.7F);
    // Triangles that have an area face away from the axis.
    bool outward = true;
    int counted = 0;
    for (std::size_t i = 0; i < m.idx.size(); i += 3) {
        const Vec3 a = m.v[m.idx[i]], b = m.v[m.idx[i + 1]], c = m.v[m.idx[i + 2]];
        const Vec3 n = gw::cross(b - a, c - a);
        if (gw::length(n) < 1e-6F) continue;
        const Vec3 centre = (a + b + c) * (1.0F / 3.0F);
        const Vec3 away{centre.x, 0.0F, centre.z};
        // hull sides must point away from the axis; the flat caps have no radial part and are skipped
        if (gw::length(away) > 0.02F && std::fabs(n.y) < 0.9F * gw::length(n)) {
            outward = outward && gw::dot(n, away) > 0.0F;
            ++counted;
        }
    }
    CHECK(outward);
    CHECK(counted > 20);
}

// ------------------------------------------------------------------------------------------ camera and matrices

void camera() {
    gw::OrbitCamera cam;
    CHECK(near(gw::length(cam.eye() - cam.target), cam.distance, 1e-3));
    // never below the minimum height above the water, whatever the user drags
    for (const float distance : {8.0F, 15.0F, 40.0F}) {
        cam.distance = distance;
        cam.pitch = -3.0F;
        cam.orbit(0.0F, -5.0F);
        CHECK(cam.eye().y >= gw::kMinEyeHeight - 1e-3F);
        cam.orbit(0.3F, 9.0F);
        CHECK(cam.pitch <= gw::kMaxPitch + 1e-6F);
        CHECK(cam.eye().y > cam.target.y);
    }
    cam.zoom(1000.0F);
    CHECK(near(cam.distance, gw::kMinDistance, 1e-4));
    cam.zoom(-1000.0F);
    CHECK(near(cam.distance, gw::kMaxDistance, 1e-4));
}

void matrices() {
    gw::OrbitCamera cam;
    const gw::Mat4 viewProj = gw::perspective(0.8F, 16.0F / 9.0F, 0.1F, 200.0F) * cam.view();
    // The look-at target lands in the middle of the screen, in front of the camera.
    const gw::Vec4 c = viewProj * gw::Vec4{cam.target.x, cam.target.y, cam.target.z, 1.0F};
    CHECK(c.w > 0.0F);
    CHECK(near(c.x / c.w, 0.0, 1e-4) && near(c.y / c.w, 0.0, 1e-4));
    CHECK(c.z / c.w > -1.0F && c.z / c.w < 1.0F);
    // Camera basis is orthonormal and looks at the target.
    const gw::Basis b = cam.basis();
    CHECK(near(gw::dot(b.right, b.up), 0.0, 1e-5) && near(gw::dot(b.right, b.forward), 0.0, 1e-5) && near(gw::dot(b.up, b.forward), 0.0, 1e-5));
    CHECK(near(gw::length(b.right), 1.0, 1e-5) && near(gw::length(b.up), 1.0, 1e-5) && near(gw::length(b.forward), 1.0, 1e-5));
    CHECK(near(b.forward, gw::normalize(cam.target - cam.eye()), 1e-5));

    // alignUp turns +y into the given direction (and stays a rotation).
    for (const Vec3 up : {Vec3{0.0F, 1.0F, 0.0F}, gw::normalize(Vec3{0.3F, 1.0F, -0.2F}), Vec3{1.0F, 0.0F, 0.0F}, Vec3{0.0F, -1.0F, 0.0F}}) {
        const gw::Mat4 r = gw::alignUp(up);
        const gw::Vec4 y = r * gw::Vec4{0.0F, 1.0F, 0.0F, 0.0F};
        CHECK(near(Vec3{y.x, y.y, y.z}, up, 1e-5));
        const gw::Vec4 x = r * gw::Vec4{1.0F, 0.0F, 0.0F, 0.0F};
        CHECK(near(std::sqrt(x.x * x.x + x.y * x.y + x.z * x.z), 1.0, 1e-5));
    }
    const gw::Mat4 model = gw::translate({1.0F, 2.0F, 3.0F}) * gw::alignUp({0.0F, 1.0F, 0.0F});
    const gw::Vec4 o = model * gw::Vec4{0.0F, 0.0F, 0.0F, 1.0F};
    CHECK(near(o.x, 1.0) && near(o.y, 2.0) && near(o.z, 3.0));
}

}  // namespace

int main() {
    dispersionHandComputed();
    longWavesRunFaster();
    singleWaveByHand();
    heightRange();
    crestTravelsAtPhaseSpeed();
    periodicInSpaceAndTime();
    timePhaseStaysPrecise();
    superposition();
    normalByHand();
    analyticNormalMatchesNumeric();
    normalYIsTheJacobian();
    jacobianByHand();
    steepnessBudget();
    overTheLimitTheSurfaceFolds();
    restPointInvertsTheDisplacement();
    buoyOnFlatWater();
    buoyByHand();
    buoyFollowsTheSurface();
    presets();
    clock();
    gridMesh();
    buoyMesh();
    camera();
    matrices();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("all checks passed\n");
    return EXIT_SUCCESS;
}
