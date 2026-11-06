// Tests for the affix rolling logic. Plain asserts on hand-computed cases, no framework, no SDL.
// The pool in logic.hpp is the demo's own example data. The numbers below are worked out from that table by hand:
// "S" groups weigh 40/100/250/500/800/1000 from tier 1 to tier 6, "F" is 1000 for every tier.
#undef NDEBUG
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <set>
#include <utility>
#include <vector>

#include "logic.hpp"

using namespace ar;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

/// A random source that hands out prepared values, so a test can force exact branches. at() throws when it runs dry.
struct Script {
    std::vector<double> values;
    std::size_t next_ = 0;
    double unit() { return values.at(next_++); }
};

Item itemOf(int base, int level, Rarity rarity, std::vector<Mod> mods = {}) {
    Item it;
    it.base = base;
    it.level = level;
    it.rarity = rarity;
    it.mods = std::move(mods);
    return it;
}

std::vector<Entry> poolFor(int base, int level, Rarity limits, const std::vector<Mod>& mods = {}, Rules rules = {}) {
    return buildPool(base, level, limits, mods, rules);
}

int countGroup(const std::vector<Entry>& pool, int group) {
    int n = 0;
    for (const Entry& e : pool) n += e.group == group ? 1 : 0;
    return n;
}

long weightOf(const std::vector<Entry>& pool) { return totalWeight(pool); }

void rngAndHelpers() {
    // Reference values of SplitMix64 started at 0, worked out independently of this code.
    Rng a(0);
    assert(a.next() == 0xe220a8397b1dcdafULL);
    assert(a.next() == 0x6e789e6aa1b965f4ULL);
    assert(a.next() == 0x06c45d188009454fULL);
    Rng u(0);
    assert(near(u.unit(), 0.8833108082136426, 1e-12));  // top 53 bits of the first output over 2^53
    // Same seed, same stream. Another seed differs.
    Rng b(123);
    Rng c(123);
    Rng d(124);
    bool differs = false;
    for (int i = 0; i < 20; ++i) {
        const std::uint64_t x = b.next();
        assert(x == c.next());
        differs = differs || x != d.next();
    }
    assert(differs);
    // unit() stays in [0, 1) and is centered.
    Rng e(99);
    double sum = 0.0;
    for (int i = 0; i < 100000; ++i) {
        const double v = e.unit();
        assert(v >= 0.0 && v < 1.0);
        sum += v;
    }
    assert(std::fabs(sum / 100000.0 - 0.5) < 0.01);
}

void weightedPick() {
    const std::vector<Entry> pool{{0, 0, 1}, {0, 1, 1}, {0, 2, 2}};  // total 4: cut points at 0.25 and 0.5
    assert(pickIndex(pool, 0.0) == 0);
    assert(pickIndex(pool, 0.2499) == 0);
    assert(pickIndex(pool, 0.25) == 1);  // the cut point belongs to the next entry
    assert(pickIndex(pool, 0.4999) == 1);
    assert(pickIndex(pool, 0.5) == 2);
    assert(pickIndex(pool, 0.75) == 2);
    assert(pickIndex(pool, 0.99999) == 2);
    assert(pickIndex(pool, 1.0) == 2);   // out of range inputs are clamped, never past the end
    assert(pickIndex(pool, -3.0) == 0);
    // An entry with weight 0 is never picked, an empty pool and an all-zero pool pick nothing.
    const std::vector<Entry> zeros{{0, 0, 0}, {0, 1, 5}, {0, 2, 0}};
    for (const double r : {0.0, 0.3, 0.5, 0.9, 0.99999, 1.0}) assert(pickIndex(zeros, r) == 1);
    assert(pickIndex({}, 0.5) == -1);
    assert(pickIndex(std::vector<Entry>{{0, 0, 0}}, 0.5) == -1);
    // Over many draws the shares follow the weights 1:1:2.
    Rng rng(5);
    std::array<int, 3> hits{};
    for (int i = 0; i < 40000; ++i) ++hits[static_cast<std::size_t>(pickIndex(pool, rng.unit()))];
    assert(std::abs(hits[0] - 10000) < 600 && std::abs(hits[1] - 10000) < 600 && std::abs(hits[2] - 20000) < 800);
}

void valueRolls() {
    const Tier t{1, 1000, 10, 20};  // 11 values
    assert(rollValue(t, 0.0) == 10);
    assert(rollValue(t, 0.5) == 15);   // 10 + floor(0.5 * 11)
    assert(rollValue(t, 0.09) == 10);  // the first value owns the first 1/11 of [0, 1)
    assert(rollValue(t, 0.1) == 11);
    assert(rollValue(t, 0.9999999) == 20);
    assert(rollValue(t, 1.0) == 20);   // clamped
    assert(rollValue(t, -1.0) == 10);
    assert(rollValue(Tier{1, 1, 7, 7}, 0.7) == 7);
    assert(rollValue(Tier{1, 1, 9, 3}, 0.7) == 9);  // a broken range does not read out of bounds
    // Every value in the range turns up about equally often, and none outside it.
    Rng rng(11);
    std::array<int, 11> count{};
    for (int i = 0; i < 11000; ++i) {
        const int v = rollValue(t, rng.unit());
        assert(v >= 10 && v <= 20);
        ++count[static_cast<std::size_t>(v - 10)];
    }
    for (const int n : count) assert(std::abs(n - 1000) < 250);
}

void limitsAndPreview() {
    assert(limitsFor(Rarity::Normal).prefixes == 0 && limitsFor(Rarity::Normal).suffixes == 0);
    assert(limitsFor(Rarity::Magic).prefixes == 1 && limitsFor(Rarity::Magic).suffixes == 1);
    assert(limitsFor(Rarity::Rare).prefixes == 3 && limitsFor(Rarity::Rare).suffixes == 3);
    // The pool shown for the next mod: a normal item is looked at as magic, a full magic item as rare.
    assert(previewRarity(itemOf(Amulet, 10, Rarity::Normal)) == Rarity::Magic);
    assert(previewRarity(itemOf(Amulet, 10, Rarity::Magic)) == Rarity::Magic);
    assert(previewRarity(itemOf(Amulet, 10, Rarity::Magic, {{Life, 5, 15}})) == Rarity::Magic);
    assert(previewRarity(itemOf(Amulet, 10, Rarity::Magic, {{Life, 5, 15}, {FireRes, 5, 6}})) == Rarity::Rare);
    assert(previewRarity(itemOf(Amulet, 10, Rarity::Rare)) == Rarity::Rare);
}

void poolByLevel() {
    // Amulet at item level 1, magic limits: only the lowest tier of every group that has one at level 1.
    // Life, Spell, Mana, FireRes, ColdRes, Crit, Strength. Chaos resistance starts at 16 and gem level at 20.
    const auto low = poolFor(Amulet, 1, Rarity::Magic);
    assert(low.size() == 7);
    assert(weightOf(low) == 7000);  // every lowest tier weighs 1000
    assert(low[0].group == Life && low[0].tier == 5 && low[0].weight == 1000);
    assert(low[1].group == Spell && low[2].group == Mana && low[3].group == FireRes);
    assert(low[6].group == Strength);
    assert(countGroup(low, ChaosRes) == 0 && countGroup(low, GemLevel) == 0);

    // Item level 80 opens everything: 7 groups with 6 tiers, chaos resistance with 4, gem level with 3.
    const auto high = poolFor(Amulet, 80, Rarity::Rare);
    assert(high.size() == 49);
    // Five S groups 2690 each, crit 3875, strength 6000 (flat), chaos 4 x 250, gem 100 + 250 + 500.
    assert(weightOf(high) == 5 * 2690 + 3875 + 6000 + 1000 + 850);
    assert(weightOf(high) == 25175);

    // A tier opens exactly at its level: life tier 3 needs 44.
    assert(countGroup(poolFor(Amulet, 43, Rarity::Rare), Life) == 3);
    assert(countGroup(poolFor(Amulet, 44, Rarity::Rare), Life) == 4);
    // Gem level: tier 3 at 20, tier 2 at 45, tier 1 at 75.
    assert(countGroup(poolFor(Amulet, 19, Rarity::Rare), GemLevel) == 0);
    assert(countGroup(poolFor(Amulet, 20, Rarity::Rare), GemLevel) == 1);
    assert(countGroup(poolFor(Amulet, 44, Rarity::Rare), GemLevel) == 1);
    assert(countGroup(poolFor(Amulet, 45, Rarity::Rare), GemLevel) == 2);
    assert(countGroup(poolFor(Amulet, 74, Rarity::Rare), GemLevel) == 2);
    assert(countGroup(poolFor(Amulet, 75, Rarity::Rare), GemLevel) == 3);
    // The best tier of life is the first entry of its group and needs 76.
    const auto top = poolFor(Amulet, 76, Rarity::Rare);
    assert(top[0].group == Life && top[0].tier == 0 && top[0].weight == 40);
    assert(poolFor(Amulet, 75, Rarity::Rare)[0].tier == 1);  // at 75 the first life entry is tier 2
}

void poolByTagsAndSlots() {
    // Tags: a greatsword never rolls life, an amulet never rolls physical damage.
    const auto sword = poolFor(Greatsword, 80, Rarity::Rare);
    assert(countGroup(sword, Life) == 0 && countGroup(sword, Armour) == 0 && countGroup(sword, Spell) == 0);
    assert(countGroup(sword, Phys) == 6 && countGroup(sword, AtkSpeed) == 6);
    assert(sword.size() == 36);
    // Phys, Fire, AtkSpeed 2690 each, Accuracy 6 x 400, Crit 3875, Strength 6000.
    assert(weightOf(sword) == 3 * 2690 + 2400 + 3875 + 6000);
    assert(countGroup(poolFor(Amulet, 80, Rarity::Rare), Phys) == 0);

    // A group on the item is not in the pool again. With the rule switched off it is.
    const std::vector<Mod> withLife{{Life, 0, 110}};
    const auto unique = poolFor(Amulet, 80, Rarity::Rare, withLife);
    assert(countGroup(unique, Life) == 0 && unique.size() == 43 && weightOf(unique) == 25175 - 2690);
    Rules naive;
    naive.uniqueGroups = false;
    const auto dup = poolFor(Amulet, 80, Rarity::Rare, withLife, naive);
    assert(countGroup(dup, Life) == 6 && dup.size() == 49);

    // Slots: three prefixes on a rare item leave only suffixes, and the same for one prefix on a magic item.
    const std::vector<Mod> threePrefixes{{Life, 0, 110}, {Spell, 0, 75}, {Mana, 0, 100}};
    const auto onlySuffix = poolFor(Amulet, 80, Rarity::Rare, threePrefixes);
    assert(onlySuffix.size() == 31);  // FireRes 6, ColdRes 6, Crit 6, Strength 6, ChaosRes 4, GemLevel 3
    assert(weightOf(onlySuffix) == 2690 + 2690 + 3875 + 6000 + 1000 + 850);
    for (const Entry& e : onlySuffix) assert(kGroups[static_cast<std::size_t>(e.group)].slot == Slot::Suffix);
    const auto magicFull = poolFor(Amulet, 80, Rarity::Magic, {{Life, 0, 110}});
    assert(magicFull.size() == 31 && weightOf(magicFull) == 17105);
    // A normal item has no slot at all.
    assert(poolFor(Amulet, 80, Rarity::Normal).empty());
}

void flatWeightsMistake() {
    Rules flat;
    flat.realWeights = false;
    const auto pool = poolFor(Amulet, 80, Rarity::Rare, {}, flat);
    assert(pool.size() == 49 && weightOf(pool) == 49);  // every tier of every group is one ticket
    for (const Entry& e : pool) assert(e.weight == 1);
    const auto odds = oddsByGroup(pool);
    // Life has 6 of the 49 tickets, gem level only 3: item level opens tiers, the tier count decides the chance.
    for (const GroupOdds& g : odds) {
        if (g.group == Life) assert(near(g.chance, 6.0 / 49.0));
        if (g.group == GemLevel) assert(near(g.chance, 3.0 / 49.0));
    }
}

void odds() {
    const auto pool = poolFor(Amulet, 80, Rarity::Rare);
    const auto groups = oddsByGroup(pool);
    assert(groups.size() == 9);
    double sum = 0.0;
    for (const GroupOdds& g : groups) {
        double inner = 0.0;
        for (const double t : g.tier) inner += t;
        assert(near(inner, g.chance));
        sum += g.chance;
    }
    assert(near(sum, 1.0));
    for (const GroupOdds& g : groups) {
        if (g.group == Life) {
            assert(near(g.chance, 2690.0 / 25175.0));
            assert(near(g.tier[0], 40.0 / 25175.0));    // tier 1 is rare although it is open
            assert(near(g.tier[5], 1000.0 / 25175.0));  // tier 6 is common
            assert(g.open == 6);
        }
        if (g.group == Crit) assert(near(g.tier[0], 125.0 / 25175.0));
        if (g.group == GemLevel) {
            assert(g.open == 3 && near(g.tier[0], 100.0 / 25175.0) && near(g.tier[3], 0.0));
        }
    }
    assert(oddsByGroup({}).empty());
    assert(weightOf({}) == 0);
}

void groupStatuses() {
    const Rules rules;
    Rules naive;
    naive.uniqueGroups = false;
    const Item low = itemOf(Amulet, 1, Rarity::Magic);
    assert(groupStatus(low, Life, Rarity::Magic, rules) == GroupStatus::Open);
    assert(groupStatus(low, GemLevel, Rarity::Magic, rules) == GroupStatus::LevelTooLow);
    assert(groupStatus(low, ChaosRes, Rarity::Magic, rules) == GroupStatus::LevelTooLow);
    assert(lowestLevel(GemLevel) == 20 && lowestLevel(ChaosRes) == 16 && lowestLevel(Life) == 1);
    const Item has = itemOf(Amulet, 80, Rarity::Magic, {{Life, 0, 110}});
    assert(groupStatus(has, Life, Rarity::Magic, rules) == GroupStatus::OnItem);
    assert(groupStatus(has, Life, Rarity::Magic, naive) == GroupStatus::SlotFull);  // would be open, but the prefix is used
    assert(groupStatus(has, Spell, Rarity::Magic, rules) == GroupStatus::SlotFull);  // magic: one prefix only
    assert(groupStatus(has, Spell, Rarity::Rare, rules) == GroupStatus::Open);
    assert(groupStatus(has, FireRes, Rarity::Magic, rules) == GroupStatus::Open);
    const Item rare = itemOf(Amulet, 80, Rarity::Rare, {{Life, 0, 110}, {Spell, 0, 75}, {Mana, 0, 100}});
    assert(groupStatus(rare, Life, Rarity::Rare, naive) == GroupStatus::SlotFull);
    assert(groupStatus(rare, FireRes, Rarity::Rare, rules) == GroupStatus::Open);
    assert(groupOnBase(Life, Amulet) && !groupOnBase(Life, Greatsword) && groupOnBase(Strength, Greatsword));
    assert(groupOnBase(Crit, Wand) && !groupOnBase(Phys, Wand));
}

void formatting() {
    assert(modText(Mod{Life, 0, 100}) == "+100 maximum Life");
    assert(modText(Mod{Crit, 5, 5}) == "5% Critical Chance");
    assert(modText(Mod{GemLevel, 0, 3}) == "+3 Level of all Skills");
    assert(tierLabel(0) == "T1" && tierLabel(5) == "T6");
    assert(!hasDuplicateGroup(itemOf(Amulet, 1, Rarity::Rare, {{Life, 5, 15}, {Spell, 5, 10}})));
    assert(hasDuplicateGroup(itemOf(Amulet, 1, Rarity::Rare, {{Life, 5, 15}, {Spell, 5, 10}, {Life, 2, 80}})));
    assert(!hasDuplicateGroup(itemOf(Amulet, 1, Rarity::Normal)));
}

void scriptedOrbs() {
    const Rules rules;
    // Transmute on a normal amulet at level 1. The pool is Life, Spell, Mana, FireRes, ColdRes, Crit, Strength at 1000 each.
    {
        Item it = itemOf(Amulet, 1, Rarity::Normal);
        Script s{{0.0, 0.0}};
        const Result r = applyOrb(Orb::Transmute, it, rules, s);
        assert(r.ok && r.fail == Fail::None && r.added == 1);
        assert(it.rarity == Rarity::Magic && it.mods.size() == 1);
        assert(it.mods[0].group == Life && it.mods[0].tier == 5 && it.mods[0].value == 15);  // tier 6 of life is 15 to 24
        assert(r.pickWeight == 1000 && r.pickTotal == 7000);
    }
    {
        Item it = itemOf(Amulet, 1, Rarity::Normal);
        Script s{{0.99, 0.99}};  // 0.99 * 7000 = 6930, the last entry
        applyOrb(Orb::Transmute, it, rules, s);
        assert(it.mods[0].group == Strength && it.mods[0].value == 13);  // strength tier 6 is 6 to 13
    }
    {
        Item it = itemOf(Amulet, 1, Rarity::Normal);
        Script s{{0.2, 0.5}};  // 1400 falls into the second entry
        applyOrb(Orb::Transmute, it, rules, s);
        assert(it.mods[0].group == Spell && it.mods[0].value == 16);  // spell tier 6 is 10 to 21: 10 + floor(0.5 * 12)
    }
    // Augment adds the other slot: the item has a prefix, so the new mod is a suffix.
    {
        Item it = itemOf(Amulet, 1, Rarity::Magic, {{Life, 5, 15}});
        Script s{{0.0, 0.0}};
        const Result r = applyOrb(Orb::Augment, it, rules, s);
        assert(r.ok && it.mods.size() == 2 && it.rarity == Rarity::Magic);
        assert(it.mods[1].group == FireRes && it.mods[1].value == 6);  // first suffix of the pool, lowest value
    }
    // Regal makes it rare and adds one mod, the old one stays and its group is out of the pool.
    {
        Item it = itemOf(Amulet, 1, Rarity::Magic, {{Life, 5, 15}});
        Script s{{0.0, 0.0}};
        const Result r = applyOrb(Orb::Regal, it, rules, s);
        assert(r.ok && it.rarity == Rarity::Rare && it.mods.size() == 2);
        assert(it.mods[0].group == Life && it.mods[1].group == Spell && it.mods[1].value == 10);
        assert(r.pickTotal == 6000);  // life is gone from the pool
    }
    // Exalt on a rare item with a free slot.
    {
        Item it = itemOf(Amulet, 1, Rarity::Rare, {{Life, 5, 15}, {Spell, 5, 10}, {FireRes, 5, 6}, {ColdRes, 5, 6}});
        Script s{{0.0, 0.0}};
        const Result r = applyOrb(Orb::Exalt, it, rules, s);
        assert(r.ok && it.mods.size() == 5 && it.mods[4].group == Mana);  // first prefix left: mana
    }
    // Chaos removes one mod and adds one. Remove index 0 (life), then life is back in the pool as its first entry.
    {
        Item it = itemOf(Amulet, 1, Rarity::Rare, {{Life, 5, 15}, {Spell, 5, 10}, {Mana, 5, 10}, {FireRes, 5, 6}});
        Script s{{0.0, 0.0, 0.0}};
        const Result r = applyOrb(Orb::Chaos, it, rules, s);
        assert(r.ok && it.mods.size() == 4);
        assert(it.mods[0].group == Spell && it.mods[1].group == Mana && it.mods[2].group == FireRes);
        assert(it.mods[3].group == Life && it.mods[3].value == 15);
    }
    // Chaos removing the last mod (r = 0.99 of 4): the three prefixes stay, so only suffixes can come in.
    {
        Item it = itemOf(Amulet, 1, Rarity::Rare, {{Life, 5, 15}, {Spell, 5, 10}, {Mana, 5, 10}, {FireRes, 5, 6}});
        Script s{{0.99, 0.0, 0.99}};
        applyOrb(Orb::Chaos, it, rules, s);
        assert(it.mods.size() == 4 && it.mods[3].group == FireRes && it.mods[3].value == 12);  // 6 + floor(0.99 * 7)
    }
    // Reset throws the mods away.
    {
        Item it = itemOf(Amulet, 1, Rarity::Rare, {{Life, 5, 15}});
        Script s{{}};
        const Result r = applyOrb(Orb::Reset, it, rules, s);
        assert(r.ok && it.rarity == Rarity::Normal && it.mods.empty());
    }
}

void prechecks() {
    const Item normal = itemOf(Amulet, 50, Rarity::Normal);
    const Item magic1 = itemOf(Amulet, 50, Rarity::Magic, {{Life, 5, 15}});
    const Item magic2 = itemOf(Amulet, 50, Rarity::Magic, {{Life, 5, 15}, {FireRes, 5, 6}});
    const Item rare = itemOf(Amulet, 50, Rarity::Rare, {{Life, 5, 15}});
    const Item rare6 = itemOf(Amulet, 80, Rarity::Rare, {{Life, 0, 110}, {Spell, 0, 75}, {Mana, 0, 100}, {FireRes, 0, 43}, {ColdRes, 0, 43}, {Crit, 0, 40}});
    assert(orbPrecheck(Orb::Transmute, normal) == Fail::None && orbPrecheck(Orb::Transmute, magic1) == Fail::WrongRarity);
    assert(orbPrecheck(Orb::Augment, magic1) == Fail::None && orbPrecheck(Orb::Augment, magic2) == Fail::Full);
    assert(orbPrecheck(Orb::Augment, normal) == Fail::WrongRarity && orbPrecheck(Orb::Augment, rare) == Fail::WrongRarity);
    assert(orbPrecheck(Orb::Regal, magic2) == Fail::None && orbPrecheck(Orb::Regal, rare) == Fail::WrongRarity);
    assert(orbPrecheck(Orb::Alchemy, normal) == Fail::None && orbPrecheck(Orb::Alchemy, magic2) == Fail::None);
    assert(orbPrecheck(Orb::Alchemy, rare) == Fail::WrongRarity);
    assert(orbPrecheck(Orb::Exalt, rare) == Fail::None && orbPrecheck(Orb::Exalt, rare6) == Fail::Full);
    assert(orbPrecheck(Orb::Exalt, magic1) == Fail::WrongRarity);
    assert(orbPrecheck(Orb::Chaos, rare) == Fail::None && orbPrecheck(Orb::Chaos, magic1) == Fail::WrongRarity);
    assert(orbPrecheck(Orb::Chaos, itemOf(Amulet, 50, Rarity::Rare)) == Fail::NoCandidates);
    for (const Item* it : {&normal, &magic1, &magic2, &rare, &rare6}) assert(orbPrecheck(Orb::Reset, *it) == Fail::None);
}

void orbRefusals() {
    const Rules rules;
    Script none{{}};  // a refused orb must not draw anything
    Item magic = itemOf(Amulet, 1, Rarity::Magic, {{Life, 5, 15}});
    Result r = applyOrb(Orb::Transmute, magic, rules, none);
    assert(!r.ok && r.fail == Fail::WrongRarity && magic.mods.size() == 1 && magic.rarity == Rarity::Magic);
    Item normal = itemOf(Amulet, 1, Rarity::Normal);
    assert(applyOrb(Orb::Augment, normal, rules, none).fail == Fail::WrongRarity);
    assert(applyOrb(Orb::Regal, normal, rules, none).fail == Fail::WrongRarity);
    assert(applyOrb(Orb::Exalt, normal, rules, none).fail == Fail::WrongRarity);
    assert(applyOrb(Orb::Chaos, normal, rules, none).fail == Fail::WrongRarity);
    assert(normal.rarity == Rarity::Normal && normal.mods.empty());
    Item rareMagic = itemOf(Amulet, 1, Rarity::Rare, {{Life, 5, 15}});
    assert(applyOrb(Orb::Augment, rareMagic, rules, none).fail == Fail::WrongRarity);
    assert(applyOrb(Orb::Regal, rareMagic, rules, none).fail == Fail::WrongRarity);

    Item fullMagic = itemOf(Amulet, 1, Rarity::Magic, {{Life, 5, 15}, {FireRes, 5, 6}});
    assert(applyOrb(Orb::Augment, fullMagic, rules, none).fail == Fail::Full);
    Item fullRare = itemOf(Amulet, 80, Rarity::Rare,
                           {{Life, 0, 110}, {Spell, 0, 75}, {Mana, 0, 100}, {FireRes, 0, 43}, {ColdRes, 0, 43}, {Crit, 0, 40}});
    assert(applyOrb(Orb::Exalt, fullRare, rules, none).fail == Fail::Full);
    assert(fullRare.mods.size() == 6);

    // Free slot but nothing can roll: a wand at level 1 with three prefixes and castspeed + crit has only gem level
    // left, which needs level 20.
    Item stuck = itemOf(Wand, 1, Rarity::Rare, {{Fire, 5, 3}, {Accuracy, 5, 20}, {Spell, 5, 10}, {CastSpeed, 5, 5}, {Crit, 5, 5}});
    r = applyOrb(Orb::Exalt, stuck, rules, none);
    assert(!r.ok && r.fail == Fail::NoCandidates && stuck.mods.size() == 5);
    // A refused orb leaves rarity and mods as they were, also when the pool is empty from the start.
    Item bare = itemOf(Wand, 1, Rarity::Magic, {{Fire, 5, 3}, {CastSpeed, 5, 5}, {Spell, 5, 10}});
    r = applyOrb(Orb::Augment, bare, rules, none);
    assert(!r.ok && bare.rarity == Rarity::Magic && bare.mods.size() == 3);
}

bool validItem(const Item& it, const Rules& rules) {
    int prefixes = 0;
    int suffixes = 0;
    std::set<int> groups;
    for (const Mod& m : it.mods) {
        const Group& g = kGroups[static_cast<std::size_t>(m.group)];
        if (m.tier < 0 || m.tier >= g.tierCount) return false;
        const Tier& t = g.tiers[static_cast<std::size_t>(m.tier)];
        if (t.minLevel > it.level || m.value < t.lo || m.value > t.hi) return false;
        if (!groupOnBase(m.group, it.base)) return false;
        (g.slot == Slot::Prefix ? prefixes : suffixes)++;
        groups.insert(m.group);
    }
    const Limits lim = limitsFor(it.rarity);
    if (prefixes > lim.prefixes || suffixes > lim.suffixes) return false;
    return !rules.uniqueGroups || groups.size() == it.mods.size();
}

void alchemyEverywhere() {
    Rules rules;
    Rules naive;
    naive.uniqueGroups = false;
    int duplicates = 0;
    for (int base = 0; base < 4; ++base) {
        for (const int level : {1, 25, 50, 80}) {
            for (std::uint64_t seed = 0; seed < 60; ++seed) {
                Rng rng(seed * 7919 + static_cast<std::uint64_t>(level));
                Item it = itemOf(base, level, Rarity::Normal);
                const Result r = applyOrb(Orb::Alchemy, it, rules, rng);
                assert(r.ok && it.rarity == Rarity::Rare && it.mods.size() == 4);
                assert(validItem(it, rules));
                assert(!hasDuplicateGroup(it));
                // The same orb on a magic item does not keep its old mod: four new ones, not five.
                Item magic = itemOf(base, level, Rarity::Magic, {it.mods[0]});
                const Result m = applyOrb(Orb::Alchemy, magic, rules, rng);
                assert(m.ok && magic.mods.size() == 4 && magic.rarity == Rarity::Rare && validItem(magic, rules));
                // With the group rule off the slots still hold, only the duplicates appear.
                Rng rng2(seed + 1000);
                Item bad = itemOf(base, level, Rarity::Normal);
                assert(applyOrb(Orb::Alchemy, bad, naive, rng2).ok);
                assert(bad.mods.size() == 4 && validItem(bad, naive));
                duplicates += hasDuplicateGroup(bad) ? 1 : 0;
            }
        }
    }
    assert(duplicates > 50);  // 960 naive items, a repeated group is common
}

void chainOfOrbs() {
    // Transmute, Augment, Regal and a pile of Exalts never break the rules, the item fills up and then refuses.
    const Rules rules;
    for (std::uint64_t seed = 0; seed < 100; ++seed) {
        Rng rng(seed);
        Item it = itemOf(Amulet, 80, Rarity::Normal);
        assert(applyOrb(Orb::Transmute, it, rules, rng).ok);
        assert(applyOrb(Orb::Augment, it, rules, rng).ok);
        assert(it.rarity == Rarity::Magic && it.mods.size() == 2 && validItem(it, rules));
        assert(applyOrb(Orb::Regal, it, rules, rng).ok);
        for (int i = 0; i < 3; ++i) assert(applyOrb(Orb::Exalt, it, rules, rng).ok && validItem(it, rules));
        assert(it.mods.size() == 6);
        assert(applyOrb(Orb::Exalt, it, rules, rng).fail == Fail::Full);
        for (int i = 0; i < 20; ++i) {
            assert(applyOrb(Orb::Chaos, it, rules, rng).ok);
            assert(it.mods.size() == 6 && validItem(it, rules));
        }
    }
}

void firstPickShares() {
    // 14000 transmutes at level 1: seven groups, every one about 2000 times.
    const Rules rules;
    std::array<int, kGroupCount> count{};
    Rng rng(3);
    for (int i = 0; i < 14000; ++i) {
        Item it = itemOf(Amulet, 1, Rarity::Normal);
        applyOrb(Orb::Transmute, it, rules, rng);
        ++count[static_cast<std::size_t>(it.mods[0].group)];
    }
    for (const int g : {Life, Spell, Mana, FireRes, ColdRes, Crit, Strength}) assert(std::abs(count[static_cast<std::size_t>(g)] - 2000) < 300);
    for (const int g : {Armour, Phys, ChaosRes, GemLevel}) assert(count[static_cast<std::size_t>(g)] == 0);
}

void simulation() {
    const Rules real;
    Rules flat;
    flat.realWeights = false;
    Rules naive;
    naive.uniqueGroups = false;

    const Tally a = simulateAlchemy(Amulet, 80, real, 4000, 7);
    assert(a.items == 4000 && a.mods == 16000);
    int sum = 0;
    for (const int n : a.perTier) sum += n;
    assert(sum == a.mods);
    assert(a.itemsWithDuplicate == 0);
    const Tally again = simulateAlchemy(Amulet, 80, real, 4000, 7);
    assert(again.perTier == a.perTier && again.mods == a.mods);  // same seed, same numbers

    // Real weights: tier 1 is about 6 percent of the mods, tier 6 about 28 percent. Flat weights treat every tier
    // alike and push tier 1 up to about 19 percent.
    const Tally f = simulateAlchemy(Amulet, 80, flat, 4000, 7);
    const double t1Real = static_cast<double>(a.perTier[0]) / a.mods;
    const double t1Flat = static_cast<double>(f.perTier[0]) / f.mods;
    const double t6Real = static_cast<double>(a.perTier[5]) / a.mods;
    const double t6Flat = static_cast<double>(f.perTier[5]) / f.mods;
    assert(t1Real < 0.10 && t1Flat > 0.14);
    assert(t6Real > 0.20 && t6Flat < 0.20);

    // Item level 1 has nothing but tier 6 to give, whatever the weights are.
    const Tally low = simulateAlchemy(Amulet, 1, real, 500, 1);
    assert(low.perTier[5] == low.mods);

    // The group rule off: repeated groups show up in a large share of the items.
    const Tally d = simulateAlchemy(Amulet, 80, naive, 4000, 7);
    assert(d.itemsWithDuplicate > 1200);
    assert(simulateAlchemy(Amulet, 80, real, 0, 7).mods == 0);
}

void slider() {
    assert(levelFromX(100.0, 100.0, 300.0) == kMinLevel);
    assert(levelFromX(300.0, 100.0, 300.0) == kMaxLevel);
    assert(levelFromX(200.0, 100.0, 300.0) == 41);  // 1 + 0.5 * 79 = 40.5 rounds up
    assert(levelFromX(-50.0, 100.0, 300.0) == kMinLevel);
    assert(levelFromX(900.0, 100.0, 300.0) == kMaxLevel);
    assert(levelFromX(150.0, 300.0, 100.0) == kMinLevel);  // a reversed or empty track does not divide by zero
    assert(levelFromX(150.0, 100.0, 100.0) == kMinLevel);
    assert(near(xFromLevel(kMinLevel, 100.0, 300.0), 100.0));
    assert(near(xFromLevel(kMaxLevel, 100.0, 300.0), 300.0));
    for (int level = kMinLevel; level <= kMaxLevel; ++level) assert(levelFromX(xFromLevel(level, 100.0, 300.0), 100.0, 300.0) == level);
    assert(clampLevel(0) == kMinLevel && clampLevel(500) == kMaxLevel && clampLevel(33) == 33);
}

}  // namespace

int main() {
    rngAndHelpers();
    weightedPick();
    valueRolls();
    limitsAndPreview();
    poolByLevel();
    poolByTagsAndSlots();
    flatWeightsMistake();
    odds();
    groupStatuses();
    formatting();
    scriptedOrbs();
    prechecks();
    orbRefusals();
    alchemyEverywhere();
    chainOfOrbs();
    firstPickShares();
    simulation();
    slider();
    std::puts("affix-rolling logic: all checks passed");
    return 0;
}
