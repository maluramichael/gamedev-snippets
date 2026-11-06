// Tests for the biome and hillshade logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace bio;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

double distance(Rgb a, Rgb b) { return std::sqrt(double(a.r - b.r) * (a.r - b.r) + double(a.g - b.g) * (a.g - b.g) + double(a.b - b.b) * (a.b - b.b)); }

void smoothAndFade() {
    assert(near(smoothstep(0.0), 0.0));
    assert(near(smoothstep(1.0), 1.0));
    assert(near(smoothstep(0.5), 0.5));
    assert(near(smoothstep(0.25), 0.15625));  // 3/16 - 2/64
    assert(near(smoothstep(-3.0), 0.0));      // clamped
    assert(near(smoothstep(9.0), 1.0));
    assert(near(fade(0.0), 0.0));
    assert(near(fade(1.0), 1.0));
    assert(near(fade(0.5), 0.5));
    assert(near(fade(0.25), 0.103515625));  // 6/1024 - 15/256 + 10/64
}

void noiseProperties() {
    // Gradient noise is exactly zero on every lattice point.
    for (int ix = -4; ix <= 4; ++ix) {
        for (int iy = -4; iy <= 4; ++iy) assert(noise2(ix, iy, 7) == 0.0);
    }
    // Deterministic, and another seed gives another field.
    assert(noise2(1.37, 2.61, 7) == noise2(1.37, 2.61, 7));
    int differs = 0;
    double lo = 1.0;
    double hi = -1.0;
    for (int i = 0; i < 40; ++i) {
        for (int j = 0; j < 40; ++j) {
            const double x = 0.37 + i * 0.173;
            const double y = 0.11 + j * 0.211;
            const double a = noise2(x, y, 7);
            assert(a >= -1.0 && a <= 1.0);
            lo = std::min(lo, a);
            hi = std::max(hi, a);
            if (a != noise2(x, y, 8)) ++differs;
        }
    }
    assert(differs > 1000);
    assert(lo < -0.4 && hi > 0.4);  // it really swings, not a flat field
    // Continuous: a tiny step moves the value by a tiny amount, also across a lattice line.
    for (int i = 0; i < 200; ++i) {
        const double x = 0.999995 + i * 0.37;
        const double y = 0.31 + i * 0.13;
        assert(std::fabs(noise2(x + 1e-5, y, 7) - noise2(x, y, 7)) < 1e-4);
    }
}

void fbmProperties() {
    double lo = 1.0;
    double hi = 0.0;
    for (int i = 0; i < 60; ++i) {
        for (int j = 0; j < 60; ++j) {
            const double v = fbm(i * 0.113, j * 0.127, 3, 5);
            assert(v >= 0.0 && v <= 1.0);
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
    }
    assert(hi - lo > 0.3);
    // One octave is the plain noise mapped from [-1, 1] to [0, 1].
    assert(near(fbm(1.5, 2.5, 3, 1), 0.5 + 0.5 * noise2(1.5, 2.5, 3)));
    assert(fbm(2.0, 2.0, 3, 5) == 0.5);  // lattice point: every octave is zero
}

void bandBlendCurve() {
    // Width 0: hard step at the middle between two cell centers.
    assert(bandBlend(0.49, 0.0) == 0.0);
    assert(bandBlend(0.5, 0.0) == 1.0);
    assert(bandBlend(0.9, 0.0) == 1.0);
    // Width 1: smoothstep across the whole way.
    assert(near(bandBlend(0.5, 1.0), 0.5));
    assert(near(bandBlend(0.25, 1.0), 0.15625));
    // Width 0.5: flat until 0.25, flat from 0.75, smoothstep in between.
    assert(near(bandBlend(0.25, 0.5), 0.0));
    assert(near(bandBlend(0.75, 0.5), 1.0));
    assert(near(bandBlend(0.5, 0.5), 0.5));
    assert(near(bandBlend(0.375, 0.5), 0.15625));
    assert(near(bandBlend(0.1, 0.5), 0.0));
}

void tableLayout() {
    const auto& t = table();
    assert(t.size() == 16);
    for (const Biome& b : t) assert(b.name != nullptr && b.name[0] != '\0');
    // Row 0 is cold, row 3 is hot, column 0 is dry, column 3 is wet.
    assert(std::string(t[0].name) == "Snow");
    assert(std::string(t[12].name) == "Desert");
    assert(std::string(t[15].name) == "Rainforest");
    // No two cells share a color, otherwise the table would hide a border.
    for (std::size_t i = 0; i < t.size(); ++i) {
        for (std::size_t j = i + 1; j < t.size(); ++j) assert(distance(t[i].color, t[j].color) > 0.04);
    }
}

void nearestCellIndex() {
    assert(nearestCell(0.0, 0.0) == 0);
    assert(nearestCell(0.24, 0.0) == 0);
    assert(nearestCell(0.26, 0.0) == 1);  // columns are quarters of [0, 1]
    assert(nearestCell(0.99, 0.0) == 3);
    assert(nearestCell(0.0, 0.26) == 4);  // one row down is +4
    assert(nearestCell(1.0, 1.0) == 15);
    assert(nearestCell(0.6, 0.6) == 2 * 4 + 2);
    assert(nearestCell(-5.0, 7.0) == 12);  // out of range clamps to the corner
}

void colorAtCellCentersIsTheCell() {
    const auto& t = table();
    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            const double m = (col + 0.5) / kCols;
            const double tt = (row + 0.5) / kRows;
            for (double w : {0.0, 0.4, 1.0}) {
                assert(distance(biomeColor(m, tt, w), t[static_cast<std::size_t>(row * kCols + col)].color) < 1e-6);
            }
        }
    }
}

void colorBetweenCells() {
    const auto& t = table();
    // Exactly on the border between column 1 and 2 (m = 0.5) at the row center of row 2.
    const Rgb a = t[2 * 4 + 1].color;
    const Rgb b = t[2 * 4 + 2].color;
    const Rgb mid = biomeColor(0.5, 2.5 / 4.0, 1.0);
    assert(distance(mid, lerp(a, b, 0.5F)) < 1e-6);
    // Hard: just left of the border is the left cell, just right is the right cell.
    assert(distance(biomeColor(0.499, 2.5 / 4.0, 0.0), a) < 1e-6);
    assert(distance(biomeColor(0.501, 2.5 / 4.0, 0.0), b) < 1e-6);
    // Soft with width 1: the same two points are almost the same color (no edge), hard: a visible jump.
    assert(distance(biomeColor(0.499, 2.5 / 4.0, 1.0), biomeColor(0.501, 2.5 / 4.0, 1.0)) < 0.01);
    assert(distance(biomeColor(0.499, 2.5 / 4.0, 0.0), biomeColor(0.501, 2.5 / 4.0, 0.0)) > 0.1);
    // Beyond the first and last center the color stays at the outer cell (no extrapolation).
    assert(distance(biomeColor(0.0, 0.5 / 4.0, 1.0), t[0].color) < 1e-6);
    assert(distance(biomeColor(1.0, 3.5 / 4.0, 1.0), t[15].color) < 1e-6);
    // Bilinear: the middle of four cells is their mean.
    const Rgb c00 = t[1 * 4 + 1].color;
    const Rgb c10 = t[1 * 4 + 2].color;
    const Rgb c01 = t[2 * 4 + 1].color;
    const Rgb c11 = t[2 * 4 + 2].color;
    const Rgb centre = biomeColor(0.5, 0.5, 1.0);
    assert(near(centre.r, (c00.r + c10.r + c01.r + c11.r) / 4.0F, 1e-6));
    assert(near(centre.g, (c00.g + c10.g + c01.g + c11.g) / 4.0F, 1e-6));
    assert(near(centre.b, (c00.b + c10.b + c01.b + c11.b) / 4.0F, 1e-6));
}

void hardEqualsNearestCell() {
    const auto& t = table();
    for (int i = 0; i <= 40; ++i) {
        for (int j = 0; j <= 40; ++j) {
            const double m = i / 40.0 + 0.0003;  // off the exact borders
            const double tt = j / 40.0 + 0.0003;
            const double mm = std::min(m, 1.0);
            const double tc = std::min(tt, 1.0);
            const int cell = nearestCell(mm, tc);
            assert(distance(biomeColor(mm, tc, 0.0), t[static_cast<std::size_t>(cell)].color) < 1e-6);
        }
    }
}

void temperatureFallsWithHeight() {
    assert(near(temperature(kSeaLevel, 0.8), 0.8));
    assert(near(temperature(1.0, 1.5), 1.5 - kLapse));  // 0.3, the peak is kLapse colder than the shore
    assert(near(temperature(1.0, 1.0), 0.0));          // 1 - 1.2 clamps
    assert(near(temperature(kSeaLevel + (1.0 - kSeaLevel) / 2.0, 0.8), 0.8 - kLapse / 2.0));
    assert(near(temperature(0.1, 0.8), 0.8));  // under water it does not get warmer than at the shore
    assert(near(temperature(1.0, 0.2), 0.0));  // clamped
    assert(near(temperature(kSeaLevel, 7.0), 1.0));
}

void hornGradient() {
    // Plane z = x (rows are north to south, columns west to east): rises 1 per cell to the east.
    const double east[9] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
    const Gradient g = horn(east, 1.0);
    assert(near(g.dzdx, 1.0) && near(g.dzdy, 0.0));
    // Plane z = y: rises to the south.
    const double south[9] = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    const Gradient s = horn(south, 1.0);
    assert(near(s.dzdx, 0.0) && near(s.dzdy, 1.0));
    // A bigger cell size flattens the slope.
    assert(near(horn(east, 2.0).dzdx, 0.5));
    // 45 degree slope, facing west (math angle 180) resp. north (90).
    assert(near(slopeOf(g, 1.0), kPi / 4.0));
    assert(near(std::fabs(aspectOf(g)), kPi));
    assert(near(aspectOf(s), kPi / 2.0));
    // The z factor scales the height: slope atan(0.5 * 2) = 45 degrees again.
    assert(near(slopeOf(Gradient{0.5, 0.0}, 2.0), kPi / 4.0));
    assert(near(slopeOf(Gradient{0.0, 0.0}, 50.0), 0.0));
}

void hillshadeValues() {
    const double c45 = std::cos(kPi / 4.0);
    // Flat ground: shade is sin(altitude), whatever the azimuth.
    assert(near(flatShade(45.0), c45));
    assert(near(flatShade(90.0), 1.0));
    assert(near(flatShade(30.0), 0.5));
    assert(near(shade(0.0, 0.0, 123.0, 45.0), c45));
    assert(near(shade(0.0, 2.0, 321.0, 30.0), 0.5));
    // 45 degree slope facing west. Sun in the west at 45 degrees: the ray hits it head-on.
    assert(near(shade(kPi / 4.0, kPi, 270.0, 45.0), 1.0));
    // Sun in the east: the ray grazes the slope, cos(zenith + slope) = cos 90 = 0.
    assert(near(shade(kPi / 4.0, kPi, 90.0, 45.0), 0.0, 1e-12));
    // Sun in the south or north: cos(zenith) * cos(slope) = 0.5.
    assert(near(shade(kPi / 4.0, kPi, 180.0, 45.0), 0.5));
    assert(near(shade(kPi / 4.0, kPi, 0.0, 45.0), 0.5));
    // Slope facing north, sun in the north.
    assert(near(shade(kPi / 4.0, kPi / 2.0, 0.0, 45.0), 1.0));
    assert(near(shade(kPi / 4.0, kPi / 2.0, 180.0, 45.0), 0.0, 1e-12));
    // A steep slope turned away from a low sun is clamped to 0, not negative.
    assert(shade(1.2, kPi, 90.0, 20.0) == 0.0);
    // Sun straight overhead: only the slope matters.
    assert(near(shade(kPi / 3.0, kPi, 270.0, 90.0), 0.5));
}

void shadeFactorIsNeutralOnFlatGround() {
    assert(near(shadeFactor(flatShade(45.0), 45.0), 1.0));
    assert(near(shadeFactor(flatShade(20.0), 20.0), 1.0));
    assert(shadeFactor(1.0, 45.0) > 1.0);   // lit slope is brighter than flat ground
    assert(shadeFactor(0.0, 45.0) < 1.0);   // shaded slope is darker
    assert(shadeFactor(0.0, 45.0) >= kShadeMin);
    assert(shadeFactor(10.0, 45.0) <= kShadeMax);
}

World planeWorld(int w, int h, double base, double perCell) {
    World world;
    world.w = w;
    world.h = h;
    world.height.resize(static_cast<std::size_t>(w * h));
    world.moisture.assign(static_cast<std::size_t>(w * h), 0.5F);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) world.height[static_cast<std::size_t>(y * w + x)] = static_cast<float>(base + perCell * x);
    }
    return world;
}

void reliefOfAPlane() {
    World world = planeWorld(5, 5, kSeaLevel + 0.1, 0.02);  // rises to the east, all land
    computeRelief(world);
    const std::size_t centre = 2 * 5 + 2;
    assert(near(world.slope[centre], std::atan(kZFactor * 0.02), 1e-4));
    assert(near(std::fabs(world.aspect[centre]), kPi, 1e-4));
    // At the left edge the missing neighbor is the edge cell itself, so only half of the step is seen.
    assert(near(world.slope[2 * 5 + 0], std::atan(kZFactor * 0.01), 1e-4));
    // Everything is lit the same on a plane: same shade in the middle and in the interior rows.
    assert(near(world.slope[1 * 5 + 2], world.slope[3 * 5 + 2], 1e-9));
}

void seaIsFlat() {
    // A slope that ends in the sea: under water the height counts as sea level, so the ground is flat there.
    World world = planeWorld(5, 5, kSeaLevel - 0.2, 0.02);  // cells 0..4 are 0.22 .. 0.30, all below 0.42
    computeRelief(world);
    for (std::size_t i = 0; i < world.slope.size(); ++i) assert(world.slope[i] == 0.0F);
}

void generatedWorld() {
    World world;
    generate(world, 64, 36, 11);
    assert(world.height.size() == 64U * 36U);
    assert(world.moisture.size() == world.height.size());
    assert(world.slope.size() == world.height.size());
    int water = 0;
    for (std::size_t i = 0; i < world.height.size(); ++i) {
        assert(world.height[i] >= 0.0F && world.height[i] <= 1.0F);
        assert(world.moisture[i] >= 0.0F && world.moisture[i] <= 1.0F);
        assert(world.slope[i] >= 0.0F);
        if (world.height[i] < kSeaLevel) ++water;
    }
    const double share = water / double(world.height.size());
    assert(share > 0.15 && share < 0.75);  // a map with both land and sea
    World same;
    generate(same, 64, 36, 11);
    assert(same.height == world.height && same.moisture == world.moisture);
    World other;
    generate(other, 64, 36, 12);
    assert(other.height != world.height);
    // The two maps are independent: moisture does not simply copy the height.
    assert(other.moisture != world.moisture);
    int equal = 0;
    for (std::size_t i = 0; i < world.height.size(); ++i) {
        if (std::fabs(world.height[i] - world.moisture[i]) < 1e-6F) ++equal;
    }
    assert(equal < 20);
}

void classification() {
    Params heightOnly;
    heightOnly.mode = Classify::HeightOnly;
    Params both;
    both.mode = Classify::HeightMoisture;
    both.tempBase = 1.0;
    heightOnly.tempBase = 1.0;
    // Water has no biome.
    assert(cellIndex(0.1, 0.5, both) == -1);
    assert(cellIndex(kSeaLevel - 0.001, 0.5, both) == -1);
    // On the shore (h = sea) with base temperature 1 it is hot: row 3.
    assert(cellIndex(kSeaLevel, 0.1, both) == 3 * 4 + 0);
    assert(cellIndex(kSeaLevel, 0.9, both) == 3 * 4 + 3);
    // Height only ignores the moisture map: always column 2 (m = 0.5), the row follows the height.
    assert(cellIndex(kSeaLevel, 0.1, heightOnly) == 3 * 4 + 2);
    assert(cellIndex(kSeaLevel, 0.9, heightOnly) == 3 * 4 + 2);
    assert(cellIndex(1.0, 0.1, heightOnly) == 0 * 4 + 2);  // top: t = 1 - 0.9 = 0.1, cold
    assert(cellIndex(1.0, 0.1, both) == 0 * 4 + 0);
    // The moisture offset moves every cell: 0.12 + 0.4 = 0.52 is column 2.
    both.moistureOffset = 0.4;
    assert(cellIndex(kSeaLevel, 0.12, both) == 3 * 4 + 2);
    both.moistureOffset = -0.7;  // clamped at 0 (and m + offset below 0)
    assert(cellIndex(kSeaLevel, 0.3, both) == 3 * 4 + 0);
    // The temperature slider moves the rows: with base 0.3 the shore is cool (row 1).
    both.moistureOffset = 0.0;
    both.tempBase = 0.3;
    assert(cellIndex(kSeaLevel, 0.1, both) == 1 * 4 + 0);
}

void colorsOfCells() {
    Params p;
    p.tempBase = 1.0;
    // Water: depth darkens it.
    const Rgb shallow = cellColor(kSeaLevel - 0.01, 0.5, p);
    const Rgb deep = cellColor(0.0, 0.5, p);
    assert(deep.b < shallow.b || deep.g < shallow.g);
    assert(shallow.b > shallow.r);  // it is blue
    // The shore is sand (hard mode: an exact band), above it the biome color takes over.
    p.soft = false;
    const Rgb sand = cellColor(kSeaLevel + 0.001, 0.5, p);
    assert(distance(sand, kSand) < 1e-6);
    const double inland = kSeaLevel + kShoreWidth + 0.01;
    assert(distance(cellColor(inland, 0.5, p), kSand) > 0.05);
    // Soft: the sand fades out continuously with the height.
    p.soft = true;
    p.width = 1.0;
    const Rgb a = cellColor(kSeaLevel + kShoreWidth - 0.0005, 0.5, p);
    const Rgb b = cellColor(kSeaLevel + kShoreWidth + 0.0005, 0.5, p);
    assert(distance(a, b) < 0.02);
}

void edgeColorJumpsOnlyWhenHard() {
    // Land well above the shore, at a row center (so only the moisture border matters).
    const double h = kSeaLevel + 0.1;
    Params p;
    p.tempBase = 2.5 / 4.0 + kLapse * ((h - kSeaLevel) / (1.0 - kSeaLevel));
    p.soft = false;
    const Rgb hardA = cellColor(h, 0.499, p);
    const Rgb hardB = cellColor(h, 0.501, p);
    assert(distance(hardA, hardB) > 0.1);
    p.soft = true;
    p.width = 1.0;
    assert(distance(cellColor(h, 0.499, p), cellColor(h, 0.501, p)) < 0.01);
    // Soft mode with width 0 is the same as hard.
    p.width = 0.0;
    assert(distance(cellColor(h, 0.499, p), hardA) < 1e-6);
    assert(distance(cellColor(h, 0.501, p), hardB) < 1e-6);
}

void countingBiomes() {
    World world;
    world.w = 4;
    world.h = 1;
    // Land at sea level (hot), land at the top (cold), water, and another hot shore cell with another moisture.
    world.height = {static_cast<float>(kSeaLevel), 1.0F, 0.1F, static_cast<float>(kSeaLevel)};
    world.moisture = {0.1F, 0.9F, 0.5F, 0.9F};
    Params p;
    p.tempBase = 1.0;
    p.mode = Classify::HeightOnly;
    assert(countBiomes(world, p) == 2);  // cells 0 and 3 land in the same biome, cell 2 is water
    p.mode = Classify::HeightMoisture;
    assert(countBiomes(world, p) == 3);
    // Only water: none.
    world.height = {0.1F, 0.2F, 0.3F, 0.4F};
    assert(countBiomes(world, p) == 0);
}

void probeStartsOnLand() {
    World world;
    world.w = 5;
    world.h = 3;
    world.height.assign(15, 0.1F);  // all water
    world.moisture.assign(15, 0.5F);
    // Fallback: no land, the requested cell itself.
    assert(nearestLandCell(world, 2, 1, kSeaLevel) == 1 * 5 + 2);
    world.height[0 * 5 + 4] = 0.9F;  // land in the top right corner
    world.height[2 * 5 + 0] = 0.9F;  // and in the bottom left corner
    world.height[1 * 5 + 3] = 0.45F;  // above the sea but below the 0.5 the caller asks for
    // Distance from (2,1): (4,0) is 5, (0,2) is 5, (3,1) is 1 but too low. Nearer to the right: tie broken by scan order.
    assert(nearestLandCell(world, 2, 1, 0.5) == 0 * 5 + 4);
    assert(nearestLandCell(world, 3, 1, 0.4) == 1 * 5 + 3);  // the cell itself when it is high enough
    assert(nearestLandCell(world, 0, 2, 0.5) == 2 * 5 + 0);
}

void popcountHelper() {
    assert(popcount16(0) == 0);
    assert(popcount16(0xFFFF) == 16);
    assert(popcount16(0b1010'0000'0000'0101) == 4);
}

}  // namespace

int main() {
    smoothAndFade();
    noiseProperties();
    fbmProperties();
    bandBlendCurve();
    tableLayout();
    nearestCellIndex();
    colorAtCellCentersIsTheCell();
    colorBetweenCells();
    hardEqualsNearestCell();
    temperatureFallsWithHeight();
    hornGradient();
    hillshadeValues();
    shadeFactorIsNeutralOnFlatGround();
    reliefOfAPlane();
    seaIsFlat();
    generatedWorld();
    classification();
    colorsOfCells();
    edgeColorJumpsOnlyWhenHard();
    countingBiomes();
    probeStartsOnLand();
    popcountHelper();
    std::puts("terrain-biomes logic: all asserts passed");
    return 0;
}
