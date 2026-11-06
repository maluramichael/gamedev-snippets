// The tile set of the demo: a small walled town. 34 tiles, no pictures, only rules.
//
// Two kinds of rules, both written as plain functions over (tile, direction, tile):
//
//   Sideways (north, east, south, west): wall arms. A pillar or tower tile has an arm, a stretch of wall, on each of its
//   four sides or not. Two neighbouring tiles must agree: arm on one side means arm on the other side, no arm means
//   no arm. Everything else (houses, trees, air) has no arms, so it can stand next to anything that has none on that side.
//
//   Up and down: what may stand on what, see canStack(). A pillar may carry a pillar, a tower a tower or a roof, a house
//   has to carry a house or a roof, and above air there is only air. Air comes in three kinds (over the ground, over
//   something built, over air) so the weights can say "mostly empty lawns, but a wall that has started goes on".
//
// A cell on the border of the grid counts as having open air on the sides and above and the ground below. That bans
// arms that point out of the grid, houses without room for a roof, and everything that cannot stand on the ground
// from the lowest layer.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "logic.hpp"

namespace w3 {

enum class Kind : int { AirGround, AirBuilt, AirSky, Tree, Pine, House, HouseUpper, RoofX, RoofZ, Post, Tower, TowerRoof };

/// Wall arms: bit i of `arms` is an arm towards kArmDirs[i].
inline constexpr std::array<int, 4> kArmDirs{PosX, NegX, PosZ, NegZ};

struct CastleTile {
    Kind kind = Kind::AirSky;
    int arms = 0;  // 0..15, only for Post and Tower
};

constexpr bool hasArms(Kind k) { return k == Kind::Post || k == Kind::Tower; }
constexpr bool isAir(Kind k) { return k == Kind::AirGround || k == Kind::AirBuilt || k == Kind::AirSky; }

/// Is there an arm on side `dir` of this tile? Up and down never have arms.
inline bool armTowards(const CastleTile& t, int dir) {
    for (int i = 0; i < 4; ++i) {
        if (kArmDirs[static_cast<std::size_t>(i)] == dir) return (t.arms >> i & 1) != 0;
    }
    return false;
}

/// May a tile of kind `upper` sit directly on top of a tile of kind `lower`?
inline bool canStack(Kind lower, Kind upper) {
    switch (lower) {
        case Kind::AirGround:
        case Kind::AirBuilt:
        case Kind::AirSky: return upper == Kind::AirSky;
        case Kind::Tree:
        case Kind::Pine:
        case Kind::RoofX:
        case Kind::RoofZ:
        case Kind::TowerRoof: return upper == Kind::AirBuilt;
        case Kind::House:
        case Kind::HouseUpper: return upper == Kind::HouseUpper || upper == Kind::RoofX || upper == Kind::RoofZ;
        case Kind::Post: return upper == Kind::AirBuilt || upper == Kind::Post;
        case Kind::Tower: return upper == Kind::AirBuilt || upper == Kind::Tower || upper == Kind::TowerRoof;
    }
    return false;
}

/// Can this kind stand on the ground (the lowest layer)? Air over the ground counts: that is the lawn.
constexpr bool standsOnGround(Kind k) {
    return k == Kind::AirGround || k == Kind::Tree || k == Kind::Pine || k == Kind::House || k == Kind::Post || k == Kind::Tower;
}

/// Is open air allowed right above this kind? (the top of the grid)
inline bool allowsAirAbove(Kind k) { return canStack(k, Kind::AirBuilt) || canStack(k, Kind::AirSky); }

/// The rule between two tiles: b on side `dir` of a.
inline bool castleFits(const CastleTile& a, int dir, const CastleTile& b) {
    if (dir == PosY) return canStack(a.kind, b.kind);
    if (dir == NegY) return canStack(b.kind, a.kind);
    return armTowards(a, dir) == armTowards(b, opposite(dir));
}

/// The rule at the border of the grid.
inline bool castleEdge(const CastleTile& a, int dir) {
    if (dir == PosY) return allowsAirAbove(a.kind);
    if (dir == NegY) return standsOnGround(a.kind);
    return !armTowards(a, dir);
}

inline int armCount(int arms) { return std::popcount(static_cast<unsigned>(arms)); }
/// Arms on opposite sides only (east and west, or north and south).
inline bool straightArms(int arms) { return arms == 0b0011 || arms == 0b1100; }

/// The weights of the tile set in one place. Air over the ground is the heavy one: it decides how empty the lawns are.
struct Weights {
    double airGround = 18.0;
    double airBuilt = 0.5;
    double airSky = 1.0;
    double tree = 1.0;
    double pine = 1.0;
    double house = 5.0;       // a house on the ground
    double houseUpper = 1.0;  // another floor on a house, against the two roofs
    double roof = 2.0;       // each of the two roofs
    double towerRoof = 3.0;
    // A wall that just stops in the middle of nowhere. Off: every wall has to lead somewhere, which is what makes the
    // solver run into contradictions.
    bool deadEnds = false;
    // Pillar by number of arms: none, one (a dead end), two straight, two bent, three, four.
    double post[6] = {0.18, 0.32, 2.10, 0.97, 0.21, 0.09};
    // Towers like to sit where walls meet.
    double tower[6] = {0.72, 0.43, 0.22, 0.86, 1.30, 1.30};
};

inline int armShape(int arms) {
    switch (armCount(arms)) {
        case 0: return 0;
        case 1: return 1;
        case 2: return straightArms(arms) ? 2 : 3;
        case 3: return 4;
        default: return 5;
    }
}

struct CastleSet {
    std::vector<CastleTile> tiles;
    TileSet set;

    int find(Kind kind, int arms = 0) const {
        for (std::size_t i = 0; i < tiles.size(); ++i) {
            if (tiles[i].kind == kind && tiles[i].arms == arms) return static_cast<int>(i);
        }
        return -1;
    }
    const CastleTile& tile(int t) const { return tiles[static_cast<std::size_t>(t)]; }
};

/// 34 tiles: 3 air, 2 trees, 2 houses (ground floor, upper floor), 2 roofs, 12 pillars, 12 towers (no dead ends), 1 tower roof.
inline CastleSet makeCastle(const Weights& w = {}) {
    CastleSet c;
    std::vector<double> weights;
    auto add = [&](Kind kind, int arms, double weight) {
        c.tiles.push_back({kind, arms});
        weights.push_back(weight);
    };
    add(Kind::AirGround, 0, w.airGround);
    add(Kind::AirBuilt, 0, w.airBuilt);
    add(Kind::AirSky, 0, w.airSky);
    add(Kind::Tree, 0, w.tree);
    add(Kind::Pine, 0, w.pine);
    add(Kind::House, 0, w.house);
    add(Kind::HouseUpper, 0, w.houseUpper);
    add(Kind::RoofX, 0, w.roof);
    add(Kind::RoofZ, 0, w.roof);
    for (int m = 0; m < 16; ++m) {
        if (w.deadEnds || armCount(m) != 1) add(Kind::Post, m, w.post[armShape(m)]);
    }
    for (int m = 0; m < 16; ++m) {
        if (w.deadEnds || armCount(m) != 1) add(Kind::Tower, m, w.tower[armShape(m)]);
    }
    add(Kind::TowerRoof, 0, w.towerRoof);

    const auto& tiles = c.tiles;
    const bool ok = c.set.build(
        weights, [&tiles](int a, int dir, int b) { return castleFits(tiles[static_cast<std::size_t>(a)], dir, tiles[static_cast<std::size_t>(b)]); },
        [&tiles](int a, int dir) { return castleEdge(tiles[static_cast<std::size_t>(a)], dir); });
    (void)ok;
    for (std::size_t i = 0; i < tiles.size(); ++i) c.set.solid[i] = isAir(tiles[i].kind) ? 0 : 1;
    return c;
}

}  // namespace w3
