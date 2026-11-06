// Affix rolling: the logic of the demo, no SDL, so test.cpp can check it.
//
// An item with random bonus lines (affixes, "mods") is built in four steps, and every step is a filter or a draw:
//   1. Pool    Which (group, tier) pairs may be added right now? A group has to fit the base item (tags), its slot must
//              be free (prefix or suffix, how many depends on the rarity), the group must not be on the item yet and the
//              tier must be open at the item level.
//   2. Pick    One pool entry is drawn, weighted: the chance of an entry is its weight over the total weight.
//   3. Value   The number on the line is drawn uniformly inside the tier's range.
//   4. Orb     A crafting orb is one of these steps, or a few of them: Transmute makes a normal item magic with one mod,
//              Alchemy makes it rare with four, Chaos removes one mod and adds one, and so on.
//
// The two classic mistakes are switches here (Rules): ignoring the weights (every tier is one ticket) and ignoring the
// group (the same line can come twice). The pool below is the demo's own example data, not a table of any game.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace ar {

constexpr int kMaxTiers = 6;
constexpr int kMinLevel = 1;
constexpr int kMaxLevel = 80;
constexpr int kMaxMods = 6;
constexpr int kGroupCount = 16;
constexpr int kBaseCount = 4;

enum class Slot { Prefix, Suffix };
enum class Rarity { Normal, Magic, Rare };
enum class Orb { Transmute, Augment, Regal, Alchemy, Exalt, Chaos, Reset };
enum class Fail { None, WrongRarity, Full, NoCandidates };
enum class GroupStatus { Open, OnItem, SlotFull, LevelTooLow };

/// Tag bits: a group may roll on a base item when they share one.
enum Tag : unsigned { kWeapon = 1U, kMelee = 2U, kCaster = 4U, kArmour = 8U, kJewellery = 16U };

enum BaseId { Greatsword = 0, Wand, Plate, Amulet };

enum GroupId {
    Life = 0, Armour, ArmourPct, Phys, Fire, Accuracy, Spell, Mana,                       // prefixes
    FireRes, ColdRes, AtkSpeed, CastSpeed, Crit, Strength, ChaosRes, GemLevel              // suffixes
};

/// One tier of a group: the item level that opens it, its weight in the pool and the range of the number.
struct Tier {
    int minLevel;
    int weight;
    int lo;
    int hi;
};

struct Group {
    const char* name;  // short, for the pool table
    const char* text;  // one '#', replaced by the rolled number
    Slot slot;
    unsigned tags;
    int tierCount;
    std::array<Tier, kMaxTiers> tiers;  // index 0 is tier 1, the best
};

struct Base {
    const char* name;
    unsigned tags;
};

inline constexpr std::array<Base, kBaseCount> kBases{{
    {"Greatsword", kWeapon | kMelee},
    {"Wand", kWeapon | kCaster},
    {"Plate Vest", kArmour},
    {"Amulet", kJewellery},
}};

// Weights from tier 1 to tier 6. "Steep" makes the best tier the rarest. Real tables are often flatter, see the post.
#define AR_STEEP 40, 100, 250, 500, 800, 1000
#define AR_FLAT 1000, 1000, 1000, 1000, 1000, 1000
#define AR_LEVELS 76, 60, 44, 28, 12, 1

inline constexpr std::array<Group, kGroupCount> kGroups{{
    {"Life", "+# maximum Life", Slot::Prefix, kArmour | kJewellery, 6,
     {{{76, 40, 100, 119}, {60, 100, 80, 99}, {44, 250, 60, 79}, {28, 500, 40, 59}, {12, 800, 25, 39}, {1, 1000, 15, 24}}}},
    {"Armour", "+# Armour", Slot::Prefix, kArmour, 6,
     {{{76, 40, 300, 359}, {60, 100, 240, 299}, {44, 250, 180, 239}, {28, 500, 120, 179}, {12, 800, 70, 119}, {1, 1000, 30, 69}}}},
    {"Armour %", "#% increased Armour", Slot::Prefix, kArmour, 6,
     {{{76, 40, 60, 69}, {60, 100, 50, 59}, {44, 250, 40, 49}, {28, 500, 30, 39}, {12, 800, 20, 29}, {1, 1000, 10, 19}}}},
    {"Phys dmg", "#% Physical Damage", Slot::Prefix, kMelee, 6,
     {{{76, 40, 130, 149}, {60, 100, 110, 129}, {44, 250, 90, 109}, {28, 500, 70, 89}, {12, 800, 50, 69}, {1, 1000, 30, 49}}}},
    {"Fire dmg", "+# Fire Damage", Slot::Prefix, kWeapon, 6,
     {{{76, 40, 40, 49}, {60, 100, 32, 39}, {44, 250, 24, 31}, {28, 500, 16, 23}, {12, 800, 9, 15}, {1, 1000, 3, 8}}}},
    {"Accuracy", "+# Accuracy Rating", Slot::Prefix, kWeapon, 6,
     {{{76, 400, 300, 399}, {60, 400, 240, 299}, {44, 400, 180, 239}, {28, 400, 120, 179}, {12, 400, 70, 119}, {1, 400, 20, 69}}}},
    {"Spell dmg", "#% Spell Damage", Slot::Prefix, kCaster | kJewellery, 6,
     {{{76, 40, 70, 79}, {60, 100, 58, 69}, {44, 250, 46, 57}, {28, 500, 34, 45}, {12, 800, 22, 33}, {1, 1000, 10, 21}}}},
    {"Mana", "+# maximum Mana", Slot::Prefix, kCaster | kJewellery, 6,
     {{{76, 40, 90, 109}, {60, 100, 70, 89}, {44, 250, 55, 69}, {28, 500, 40, 54}, {12, 800, 25, 39}, {1, 1000, 10, 24}}}},
    {"Fire res", "+#% Fire Resistance", Slot::Suffix, kArmour | kJewellery, 6,
     {{{76, 40, 41, 45}, {60, 100, 35, 40}, {44, 250, 29, 34}, {28, 500, 21, 28}, {12, 800, 13, 20}, {1, 1000, 6, 12}}}},
    {"Cold res", "+#% Cold Resistance", Slot::Suffix, kArmour | kJewellery, 6,
     {{{76, 40, 41, 45}, {60, 100, 35, 40}, {44, 250, 29, 34}, {28, 500, 21, 28}, {12, 800, 13, 20}, {1, 1000, 6, 12}}}},
    {"Atk speed", "#% Attack Speed", Slot::Suffix, kMelee, 6,
     {{{76, 40, 20, 24}, {60, 100, 17, 19}, {44, 250, 14, 16}, {28, 500, 11, 13}, {12, 800, 8, 10}, {1, 1000, 5, 7}}}},
    {"Cast speed", "#% Cast Speed", Slot::Suffix, kCaster, 6,
     {{{76, 40, 20, 24}, {60, 100, 17, 19}, {44, 250, 14, 16}, {28, 500, 11, 13}, {12, 800, 8, 10}, {1, 1000, 5, 7}}}},
    {"Crit", "#% Critical Chance", Slot::Suffix, kWeapon | kJewellery, 6,
     {{{76, 125, 38, 45}, {60, 250, 31, 37}, {44, 500, 24, 30}, {28, 1000, 17, 23}, {12, 1000, 10, 16}, {1, 1000, 5, 9}}}},
    {"Strength", "+# Strength", Slot::Suffix, kArmour | kMelee | kJewellery, 6,
     {{{76, 1000, 46, 55}, {60, 1000, 38, 45}, {44, 1000, 30, 37}, {28, 1000, 22, 29}, {12, 1000, 14, 21}, {1, 1000, 6, 13}}}},
    {"Chaos res", "+#% Chaos Resistance", Slot::Suffix, kArmour | kJewellery, 4,
     {{{70, 250, 24, 28}, {52, 250, 19, 23}, {34, 250, 14, 18}, {16, 250, 9, 13}, {0, 0, 0, 0}, {0, 0, 0, 0}}}},
    {"Gem level", "+# Level of all Skills", Slot::Suffix, kCaster | kJewellery, 3,
     {{{75, 100, 3, 3}, {45, 250, 2, 2}, {20, 500, 1, 1}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}}},
}};

#undef AR_STEEP
#undef AR_FLAT
#undef AR_LEVELS

struct Mod {
    int group;
    int tier;  // 0 is tier 1
    int value;
};

struct Item {
    int base = Amulet;
    int level = 60;
    Rarity rarity = Rarity::Normal;
    std::vector<Mod> mods;
};

/// The two classic mistakes, as switches. Both true is the correct way.
struct Rules {
    bool realWeights = true;   // false: every tier is one ticket, whatever its weight is
    bool uniqueGroups = true;  // false: a group already on the item may be drawn again
};

struct Entry {
    int group;
    int tier;
    int weight;
};

struct Limits {
    int prefixes;
    int suffixes;
};

struct GroupOdds {
    int group = 0;
    double chance = 0.0;
    std::array<double, kMaxTiers> tier{};
    int open = 0;
};

struct Result {
    bool ok = false;
    Fail fail = Fail::None;
    int added = 0;
    int pickWeight = 0;  // of the last pick: its weight and the total it was drawn from
    int pickTotal = 0;
};

struct Tally {
    int items = 0;
    int mods = 0;
    std::array<int, kMaxTiers> perTier{};
    int itemsWithDuplicate = 0;
};

/// SplitMix64: tiny, fast and good enough for a demo. The same seed always gives the same stream.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : state_(seed) {}

    std::uint64_t next() {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    /// Uniform in [0, 1): the top 53 bits of a draw, which is exactly what a double can hold.
    double unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

private:
    std::uint64_t state_;
};

/// Slots by rarity. A normal item has none, magic has one prefix and one suffix, rare three of each.
inline Limits limitsFor(Rarity rarity) {
    switch (rarity) {
        case Rarity::Normal: return {0, 0};
        case Rarity::Magic: return {1, 1};
        case Rarity::Rare: return {3, 3};
    }
    return {0, 0};
}

/// The rarity whose slots the "next mod" is drawn under: a normal item is looked at as magic (Transmute), a magic item
/// with a free slot stays magic (Augment) and a full one is looked at as rare (Regal).
inline Rarity previewRarity(const Item& item) {
    if (item.rarity == Rarity::Normal) return Rarity::Magic;
    if (item.rarity == Rarity::Magic && item.mods.size() < 2) return Rarity::Magic;
    return item.rarity == Rarity::Magic ? Rarity::Rare : item.rarity;
}

inline const Group& group(int index) { return kGroups[static_cast<std::size_t>(index)]; }

inline bool groupOnBase(int groupIndex, int base) {
    if (groupIndex < 0 || groupIndex >= kGroupCount || base < 0 || base >= kBaseCount) return false;
    return (group(groupIndex).tags & kBases[static_cast<std::size_t>(base)].tags) != 0U;
}

/// The item level at which a group rolls at all: the lowest minLevel of its tiers.
inline int lowestLevel(int groupIndex) {
    const Group& g = group(groupIndex);
    int lowest = g.tiers[0].minLevel;
    for (int t = 1; t < g.tierCount; ++t) lowest = std::min(lowest, g.tiers[static_cast<std::size_t>(t)].minLevel);
    return lowest;
}

inline int clampLevel(int level) { return std::clamp(level, kMinLevel, kMaxLevel); }

inline int slotsUsed(const std::vector<Mod>& mods, Slot slot) {
    int n = 0;
    for (const Mod& m : mods) n += group(m.group).slot == slot ? 1 : 0;
    return n;
}

inline bool groupPresent(const std::vector<Mod>& mods, int groupIndex) {
    return std::any_of(mods.begin(), mods.end(), [groupIndex](const Mod& m) { return m.group == groupIndex; });
}

/// Every (group, tier) that may be added: the group fits the base, its slot is free under `limits`, the group is not
/// on the item (unless the rule is off) and the tier is open at `level`. Order: groups as in the table, tier 1 first.
/// With realWeights off every entry weighs 1.
inline std::vector<Entry> buildPool(int base, int level, Rarity limits, const std::vector<Mod>& mods, const Rules& rules) {
    std::vector<Entry> pool;
    const Limits lim = limitsFor(limits);
    const int freePrefix = lim.prefixes - slotsUsed(mods, Slot::Prefix);
    const int freeSuffix = lim.suffixes - slotsUsed(mods, Slot::Suffix);
    for (int gi = 0; gi < kGroupCount; ++gi) {
        const Group& g = group(gi);
        if (!groupOnBase(gi, base)) continue;
        if ((g.slot == Slot::Prefix ? freePrefix : freeSuffix) <= 0) continue;
        if (rules.uniqueGroups && groupPresent(mods, gi)) continue;
        for (int t = 0; t < g.tierCount; ++t) {
            const Tier& tier = g.tiers[static_cast<std::size_t>(t)];
            if (tier.minLevel <= level) pool.push_back({gi, t, rules.realWeights ? tier.weight : 1});
        }
    }
    return pool;
}

inline long totalWeight(const std::vector<Entry>& pool) {
    long total = 0;
    for (const Entry& e : pool) total += std::max(0, e.weight);
    return total;
}

/// The entry that the unit number `r` (0 to 1) lands on, with every entry owning a stretch as long as its weight.
/// A cut point belongs to the next entry. Entries with weight 0 are never picked. -1 for an empty pool.
inline int pickIndex(const std::vector<Entry>& pool, double r) {
    const long total = totalWeight(pool);
    if (total <= 0) return -1;
    const double target = std::clamp(r, 0.0, 1.0) * static_cast<double>(total);
    double acc = 0.0;
    int last = -1;
    for (std::size_t i = 0; i < pool.size(); ++i) {
        if (pool[i].weight <= 0) continue;
        acc += static_cast<double>(pool[i].weight);
        last = static_cast<int>(i);
        if (target < acc) return last;
    }
    return last;  // r = 1 or rounding: the last entry that has weight
}

/// A uniform integer in [lo, hi] from the unit number `r`. A broken range (hi < lo) gives lo.
inline int rollValue(const Tier& tier, double r) {
    if (tier.hi <= tier.lo) return tier.lo;
    const int span = tier.hi - tier.lo + 1;
    const int step = static_cast<int>(std::floor(std::clamp(r, 0.0, 1.0) * span));
    return tier.lo + std::min(step, span - 1);
}

/// The pool folded per group: chance of the group, of each of its tiers, and how many tiers are open.
inline std::vector<GroupOdds> oddsByGroup(const std::vector<Entry>& pool) {
    std::vector<GroupOdds> out;
    const double total = static_cast<double>(totalWeight(pool));
    if (total <= 0.0) return out;
    for (const Entry& e : pool) {
        auto it = std::find_if(out.begin(), out.end(), [&e](const GroupOdds& g) { return g.group == e.group; });
        if (it == out.end()) {
            out.emplace_back();
            out.back().group = e.group;
            it = out.end() - 1;
        }
        const double p = static_cast<double>(std::max(0, e.weight)) / total;
        it->chance += p;
        it->tier[static_cast<std::size_t>(e.tier)] += p;
        ++it->open;
    }
    return out;
}

/// Why a group is or is not in the pool of the next mod. First reason wins: on the item, slot full, level too low.
inline GroupStatus groupStatus(const Item& item, int groupIndex, Rarity limits, const Rules& rules) {
    if (rules.uniqueGroups && groupPresent(item.mods, groupIndex)) return GroupStatus::OnItem;
    const Limits lim = limitsFor(limits);
    const Slot slot = group(groupIndex).slot;
    const int free = (slot == Slot::Prefix ? lim.prefixes : lim.suffixes) - slotsUsed(item.mods, slot);
    if (free <= 0) return GroupStatus::SlotFull;
    if (lowestLevel(groupIndex) > item.level) return GroupStatus::LevelTooLow;
    return GroupStatus::Open;
}

inline bool hasDuplicateGroup(const Item& item) {
    for (std::size_t i = 0; i < item.mods.size(); ++i) {
        for (std::size_t j = i + 1; j < item.mods.size(); ++j) {
            if (item.mods[i].group == item.mods[j].group) return true;
        }
    }
    return false;
}

inline std::string tierLabel(int tier) { return "T" + std::to_string(tier + 1); }

/// The line as the item shows it, the '#' of the group's text replaced by the rolled number.
inline std::string modText(const Mod& mod) {
    std::string text = group(mod.group).text;
    const std::size_t at = text.find('#');
    if (at != std::string::npos) text.replace(at, 1, std::to_string(mod.value));
    return text;
}

namespace detail {

/// Draws one mod from the pool under `limits` and puts it on the item. False when the pool is empty.
template <class R>
bool addOne(Item& item, Rarity limits, const Rules& rules, R& rng, Result& result) {
    const std::vector<Entry> pool = buildPool(item.base, item.level, limits, item.mods, rules);
    if (totalWeight(pool) <= 0) return false;  // nothing to draw, so nothing is consumed from the generator
    const int index = pickIndex(pool, rng.unit());
    if (index < 0) return false;
    const Entry& e = pool[static_cast<std::size_t>(index)];
    const Tier& tier = group(e.group).tiers[static_cast<std::size_t>(e.tier)];
    item.mods.push_back({e.group, e.tier, rollValue(tier, rng.unit())});
    result.pickWeight = e.weight;
    result.pickTotal = static_cast<int>(totalWeight(pool));
    ++result.added;
    return true;
}

inline Result refuse(Fail why) {
    Result r;
    r.fail = why;
    return r;
}

}  // namespace detail

/// Whether an orb can be used on the item at all, judged by rarity and mod count only (the pool may still turn out
/// empty). Fail::None means the orb may be tried.
inline Fail orbPrecheck(Orb orb, const Item& item) {
    const std::size_t count = item.mods.size();
    switch (orb) {
        case Orb::Reset: return Fail::None;
        case Orb::Transmute: return item.rarity == Rarity::Normal ? Fail::None : Fail::WrongRarity;
        case Orb::Augment:
            if (item.rarity != Rarity::Magic) return Fail::WrongRarity;
            return count >= 2 ? Fail::Full : Fail::None;
        case Orb::Regal: return item.rarity == Rarity::Magic ? Fail::None : Fail::WrongRarity;
        case Orb::Alchemy: return item.rarity == Rarity::Rare ? Fail::WrongRarity : Fail::None;
        case Orb::Exalt:
            if (item.rarity != Rarity::Rare) return Fail::WrongRarity;
            return count >= static_cast<std::size_t>(kMaxMods) ? Fail::Full : Fail::None;
        case Orb::Chaos:
            if (item.rarity != Rarity::Rare) return Fail::WrongRarity;
            return count == 0 ? Fail::NoCandidates : Fail::None;
    }
    return Fail::WrongRarity;
}

/// Applies a crafting orb. An orb that cannot work is refused before it draws anything and leaves the item as it was.
///   Transmute  normal -> magic, one mod            Augment  magic with a free slot, one more mod
///   Regal      magic -> rare, one more mod         Alchemy  normal or magic -> rare, the old mods go, four new ones
///   Exalt      rare with a free slot, one more     Chaos    rare: one random mod goes, one new one comes
///   Reset      back to a normal item
/// `rng` needs a `double unit()` in [0, 1).
template <class R>
Result applyOrb(Orb orb, Item& item, const Rules& rules, R& rng) {
    Result result;
    if (const Fail refused = orbPrecheck(orb, item); refused != Fail::None) return detail::refuse(refused);
    switch (orb) {
        case Orb::Reset:
            item.rarity = Rarity::Normal;
            item.mods.clear();
            break;
        case Orb::Transmute:
            if (!detail::addOne(item, Rarity::Magic, rules, rng, result)) return detail::refuse(Fail::NoCandidates);
            item.rarity = Rarity::Magic;
            break;
        case Orb::Augment:
            if (!detail::addOne(item, Rarity::Magic, rules, rng, result)) return detail::refuse(Fail::NoCandidates);
            break;
        case Orb::Regal:
            if (!detail::addOne(item, Rarity::Rare, rules, rng, result)) return detail::refuse(Fail::NoCandidates);
            item.rarity = Rarity::Rare;
            break;
        case Orb::Exalt:
            if (!detail::addOne(item, Rarity::Rare, rules, rng, result)) return detail::refuse(Fail::NoCandidates);
            break;
        case Orb::Chaos: {
            const std::vector<Mod> before = item.mods;
            const double r = std::clamp(rng.unit(), 0.0, 1.0);
            const auto count = item.mods.size();
            const auto removed = std::min(count - 1, static_cast<std::size_t>(r * static_cast<double>(count)));
            item.mods.erase(item.mods.begin() + static_cast<std::ptrdiff_t>(removed));
            if (!detail::addOne(item, Rarity::Rare, rules, rng, result)) {
                item.mods = before;
                return detail::refuse(Fail::NoCandidates);
            }
            break;
        }
        case Orb::Alchemy: {
            const std::vector<Mod> before = item.mods;
            item.mods.clear();
            for (int i = 0; i < 4; ++i) {
                if (!detail::addOne(item, Rarity::Rare, rules, rng, result)) break;
            }
            if (item.mods.empty()) {
                item.mods = before;
                return detail::refuse(Fail::NoCandidates);
            }
            item.rarity = Rarity::Rare;
            break;
        }
    }
    result.ok = true;
    return result;
}

/// `items` fresh items of the given base and level, each hit by one Alchemy orb. Counts the tiers of all rolled mods and
/// the items that carry the same group twice. Deterministic for a seed.
inline Tally simulateAlchemy(int base, int level, const Rules& rules, int items, std::uint64_t seed) {
    Tally tally;
    Rng rng(seed);
    for (int i = 0; i < std::max(0, items); ++i) {
        Item item;
        item.base = base;
        item.level = level;
        if (!applyOrb(Orb::Alchemy, item, rules, rng).ok) continue;
        ++tally.items;
        tally.mods += static_cast<int>(item.mods.size());
        for (const Mod& m : item.mods) ++tally.perTier[static_cast<std::size_t>(m.tier)];
        if (hasDuplicateGroup(item)) ++tally.itemsWithDuplicate;
    }
    return tally;
}

/// Item level for a mouse x on a slider track from x0 to x1. Out of range clamps, an empty track gives the minimum.
inline int levelFromX(double x, double x0, double x1) {
    if (!(x1 > x0)) return kMinLevel;
    const double t = std::clamp((x - x0) / (x1 - x0), 0.0, 1.0);
    return clampLevel(static_cast<int>(std::lround(kMinLevel + t * (kMaxLevel - kMinLevel))));
}

inline double xFromLevel(int level, double x0, double x1) {
    return x0 + (x1 - x0) * static_cast<double>(clampLevel(level) - kMinLevel) / static_cast<double>(kMaxLevel - kMinLevel);
}

}  // namespace ar
