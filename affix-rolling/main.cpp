// Rolling affixes with SDL3 main callbacks.
//
// An item gets random lines (affixes) from a pool. The middle panel shows that pool for the next line: every group that
// fits the item, how many of its tiers the item level has opened (the dots), and the chance of each tier (the bar). The
// right panel rolls 2000 fresh items with one Alchemy orb and counts which tier the lines came in.
//
// Two things to see. First, item level only opens tiers; with the example weights the best tier stays a thin slice of
// the bar even when it is open. Second, a group is on the item at most once. The two switches below the orbs turn on the
// classic mistakes (orange): treating every tier as one ticket, and drawing a group that is already on the item. Watch
// the tier counts and the "repeated group" counter move. The pool, the draws and the orbs live in logic.hpp and are
// covered by test.cpp. The pool is the demo's own example data.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kRow1Y = 40.0F;
constexpr float kChipH = 28.0F;
constexpr float kPanelY = 80.0F;
constexpr float kPanelH = 272.0F;
constexpr float kItemX = 24.0F;
constexpr float kItemW = 290.0F;
constexpr float kPoolX = 326.0F;
constexpr float kPoolW = 340.0F;
constexpr float kStatsX = 678.0F;
constexpr float kStatsW = 258.0F;
constexpr float kOrbY = 362.0F;
constexpr float kRuleY = 398.0F;
constexpr float kHintY = 436.0F;
constexpr float kStatusY = 466.0F;
constexpr float kRow = 24.0F;

constexpr float kSliderX0 = 592.0F;
constexpr float kSliderX1 = 852.0F;
constexpr float kSliderY = 54.0F;

constexpr int kSimItems = 2000;
constexpr std::uint64_t kSimSeed = 7;
constexpr std::uint64_t kStartSeed = 2027;
constexpr int kFlashFrames = 70;

constexpr gd::Color kMagic{37, 99, 235};
constexpr gd::Color kRare{180, 120, 10};

enum class Action { Base, Orb, Weights, Groups };

struct Chip {
    Action action;
    int arg;
    std::string label;
    std::string altLabel;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    ar::Item item;
    ar::Rules rules;
    ar::Rng rng{kStartSeed};
    ar::Tally tally;
    bool statsDirty = true;
    bool dragging = false;
    std::string status1;
    std::string status2;
    bool statusIsRefusal = false;
    int flashFrom = 0;
    int flashLeft = 0;
    long maxFrames = -1;
    long frames = 0;
};

const char* orbName(ar::Orb orb) {
    switch (orb) {
        case ar::Orb::Transmute: return "Transmute";
        case ar::Orb::Augment: return "Augment";
        case ar::Orb::Regal: return "Regal";
        case ar::Orb::Alchemy: return "Alchemy";
        case ar::Orb::Exalt: return "Exalt";
        case ar::Orb::Chaos: return "Chaos";
        case ar::Orb::Reset: return "Reset";
    }
    return "";
}

gd::Color tierColor(int tier) {
    const float t = static_cast<float>(std::clamp(tier, 0, 5)) / 5.0F;
    const auto mix = [t](Uint8 a, Uint8 b) { return static_cast<Uint8>(static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * t); };
    return gd::Color{mix(gd::kTealDark.r, gd::kTealSoft.r), mix(gd::kTealDark.g, gd::kTealSoft.g), mix(gd::kTealDark.b, gd::kTealSoft.b)};
}

gd::Color tierInk(int tier) { return tier <= 2 ? gd::kWhite : gd::kInk; }

gd::Color rarityColor(ar::Rarity rarity) {
    switch (rarity) {
        case ar::Rarity::Normal: return gd::kMuted;
        case ar::Rarity::Magic: return kMagic;
        case ar::Rarity::Rare: return kRare;
    }
    return gd::kMuted;
}

const char* rarityName(ar::Rarity rarity) {
    switch (rarity) {
        case ar::Rarity::Normal: return "NORMAL";
        case ar::Rarity::Magic: return "MAGIC";
        case ar::Rarity::Rare: return "RARE";
    }
    return "";
}

std::string percent(double fraction) {
    char buf[24];
    SDL_snprintf(buf, sizeof buf, "%.1f%%", fraction * 100.0);
    return buf;
}

std::string chipLabel(const App& app, const Chip& c) {
    if (c.action == Action::Weights) return app.rules.realWeights ? c.label : c.altLabel;
    if (c.action == Action::Groups) return app.rules.uniqueGroups ? c.label : c.altLabel;
    return c.label;
}

bool chipOn(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Base: return app.item.base == c.arg;
        case Action::Orb: return false;
        case Action::Weights: return !app.rules.realWeights;
        case Action::Groups: return !app.rules.uniqueGroups;
    }
    return false;
}

void layoutChips(App& app) {
    float x = 24.0F;
    const auto place = [&](Action action, int arg, const std::string& label, const std::string& alt, float y, float gap) {
        x += gap;
        const float w = std::max(app.hud.width(label, gd::Hud::Style::Bold), app.hud.width(alt, gd::Hud::Style::Bold)) + 24.0F;
        app.chips.push_back({action, arg, label, alt, SDL_FRect{x, y, w, kChipH}});
        x += w;
    };
    for (int b = 0; b < ar::kBaseCount; ++b) {
        place(Action::Base, b, std::to_string(b + 1) + " " + ar::kBases[static_cast<std::size_t>(b)].name, "", kRow1Y, b == 0 ? 0.0F : 8.0F);
    }
    x = 24.0F;
    const std::array<std::pair<ar::Orb, const char*>, 7> orbs{{{ar::Orb::Transmute, "T Transmute"},
                                                                 {ar::Orb::Augment, "U Augment"},
                                                                 {ar::Orb::Regal, "R Regal"},
                                                                 {ar::Orb::Alchemy, "A Alchemy"},
                                                                 {ar::Orb::Exalt, "E Exalt"},
                                                                 {ar::Orb::Chaos, "C Chaos"},
                                                                 {ar::Orb::Reset, "X Reset"}}};
    bool first = true;
    for (const auto& [orb, label] : orbs) {
        place(Action::Orb, static_cast<int>(orb), label, "", kOrbY, first ? 0.0F : 8.0F);
        first = false;
    }
    x = 24.0F;
    place(Action::Weights, 0, "W Weights: real", "W Weights: flat", kRuleY, 0.0F);
    place(Action::Groups, 0, "G Groups: unique", "G Groups: repeat", kRuleY, 8.0F);
}

void markStats(App& app) { app.statsDirty = true; }

void setStatus(App& app, std::string line1, std::string line2, bool refusal) {
    app.status1 = std::move(line1);
    app.status2 = std::move(line2);
    app.statusIsRefusal = refusal;
}

std::string refusalReason(ar::Orb orb, ar::Fail fail) {
    if (fail == ar::Fail::Full) return "no free mod slot";
    if (fail == ar::Fail::NoCandidates) return "nothing in the pool can roll";
    switch (orb) {
        case ar::Orb::Transmute: return "needs a normal item";
        case ar::Orb::Augment:
        case ar::Orb::Regal: return "needs a magic item";
        case ar::Orb::Alchemy: return "the item is already rare";
        case ar::Orb::Exalt:
        case ar::Orb::Chaos: return "needs a rare item";
        case ar::Orb::Reset: break;
    }
    return "";
}

void craft(App& app, ar::Orb orb) {
    const ar::Result r = ar::applyOrb(orb, app.item, app.rules, app.rng);
    if (!r.ok) {
        setStatus(app, std::string(orbName(orb)) + " refused: " + refusalReason(orb, r.fail), "The item is unchanged.", true);
        return;
    }
    markStats(app);
    if (orb == ar::Orb::Reset) {
        app.flashLeft = 0;
        setStatus(app, "Reset: a normal item again", "No mods, no slots.", false);
        return;
    }
    app.flashFrom = orb == ar::Orb::Alchemy ? 0 : static_cast<int>(app.item.mods.size()) - 1;
    app.flashLeft = kFlashFrames;
    const ar::Mod& last = app.item.mods.back();
    std::string line1;
    if (orb == ar::Orb::Alchemy) {
        line1 = "Alchemy: a rare item with " + std::to_string(app.item.mods.size()) + " mods";
    } else {
        line1 = std::string(orbName(orb)) + ": " + ar::tierLabel(last.tier) + " " + ar::modText(last);
    }
    std::string line2 = std::string(orb == ar::Orb::Alchemy ? "last pick: " : "this pick: ") + "weight " + std::to_string(r.pickWeight) + " of " +
                        std::to_string(r.pickTotal) + " = " + percent(static_cast<double>(r.pickWeight) / static_cast<double>(std::max(1, r.pickTotal)));
    if (!app.rules.realWeights) line2 += "  (weights ignored: one ticket per tier)";
    if (!app.rules.uniqueGroups && ar::hasDuplicateGroup(app.item)) line2 += "  (a group is on the item twice)";
    setStatus(app, std::move(line1), std::move(line2), false);
}

/// A fresh item of the given base and level, then one Alchemy orb so the card is never empty.
void newItem(App& app, int base, int level, bool roll) {
    app.item = ar::Item{};
    app.item.base = std::clamp(base, 0, ar::kBaseCount - 1);
    app.item.level = ar::clampLevel(level);
    app.flashLeft = 0;
    markStats(app);
    if (roll) {
        craft(app, ar::Orb::Alchemy);
    } else {
        setStatus(app, "New " + std::string(ar::kBases[static_cast<std::size_t>(app.item.base)].name) + ", item level " + std::to_string(app.item.level),
                  "Let go of the slider and an Alchemy orb is used.", false);
    }
}

void setLevelFromMouse(App& app, float x) {
    const int level = ar::levelFromX(static_cast<double>(x), static_cast<double>(kSliderX0), static_cast<double>(kSliderX1));
    if (level != app.item.level) newItem(app, app.item.base, level, false);
}

void endDrag(App& app) {
    if (!app.dragging) return;
    app.dragging = false;
    if (app.item.rarity == ar::Rarity::Normal) craft(app, ar::Orb::Alchemy);
}

void apply(App& app, const Chip& c) {
    switch (c.action) {
        case Action::Base: newItem(app, c.arg, app.item.level, true); break;
        case Action::Orb: craft(app, static_cast<ar::Orb>(c.arg)); break;
        case Action::Weights:
            app.rules.realWeights = !app.rules.realWeights;
            markStats(app);
            setStatus(app, app.rules.realWeights ? "Weights: real, a tier counts as often as its weight says" : "Weights: flat, every tier is one ticket",
                      app.rules.realWeights ? "Tier 1 is open at a high item level, and still rare." : "Every open tier has the same chance now. Look at the pool and the counts.",
                      false);
            break;
        case Action::Groups:
            app.rules.uniqueGroups = !app.rules.uniqueGroups;
            markStats(app);
            setStatus(app, app.rules.uniqueGroups ? "Groups: unique, a group is on the item once" : "Groups: repeat, a group on the item can be drawn again",
                      app.rules.uniqueGroups ? "Used groups drop out of the pool." : "Watch the counter on the right: items with the same line twice.", false);
            break;
    }
}

void applyKey(App& app, Action action, int arg) {
    for (const Chip& c : app.chips) {
        if (c.action == action && c.arg == arg) {
            apply(app, c);
            return;
        }
    }
}

// ---------------------------------------------------------------- drawing

void centerText(App& app, float cx, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Bold) {
    app.hud.text(app.renderer, cx - app.hud.width(s, style) / 2.0F, y, s, c, style);
}

void rightText(App& app, float right, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, right - app.hud.width(s, style), y, s, c, style);
}

void drawChips(App& app) {
    for (const Chip& c : app.chips) {
        const bool on = chipOn(app, c);
        const bool orb = c.action == Action::Orb;
        const bool toggle = c.action == Action::Weights || c.action == Action::Groups;
        const bool refused = orb && ar::orbPrecheck(static_cast<ar::Orb>(c.arg), app.item) != ar::Fail::None;
        gd::Color fill = gd::kWhite;
        gd::Color ink = gd::kInk;
        if (on) {
            fill = toggle ? gd::kOrange : gd::kTeal;
            ink = gd::kWhite;
        } else if (toggle) {
            fill = gd::kTealSoft;
        } else if (refused) {
            fill = gd::kPaper;
            ink = gd::kMuted;
        }
        gd::card(app.renderer, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        centerText(app, c.rect.x + c.rect.w / 2.0F, c.rect.y + 3.0F, chipLabel(app, c), ink);
    }
}

void drawSlider(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, 540.0F, kSliderY - 11.0F, "ilvl", gd::kInk, gd::Hud::Style::Bold);
    gd::fillRect(r, kSliderX0, kSliderY - 2.0F, kSliderX1 - kSliderX0, 4.0F, gd::kGrid);
    gd::strokeRect(r, kSliderX0, kSliderY - 2.0F, kSliderX1 - kSliderX0, 4.0F, gd::kInk);
    const auto x = static_cast<float>(ar::xFromLevel(app.item.level, static_cast<double>(kSliderX0), static_cast<double>(kSliderX1)));
    gd::fillRect(r, kSliderX0, kSliderY - 2.0F, x - kSliderX0, 4.0F, gd::kTeal);
    gd::card(r, x - 6.0F, kSliderY - 10.0F, 12.0F, 20.0F, app.dragging ? gd::kOrange : gd::kWhite, gd::kInk);
    app.hud.text(r, kSliderX1 + 14.0F, kSliderY - 11.0F, std::to_string(app.item.level), gd::kInk, gd::Hud::Style::Bold);
}

void drawItem(App& app) {
    SDL_Renderer* r = app.renderer;
    const ar::Item& it = app.item;
    const float x = kItemX;
    const float y = kPanelY;
    gd::card(r, x, y, kItemW, kPanelH, gd::kWhite, gd::kInk);
    gd::fillRect(r, x, y, kItemW, 48.0F, rarityColor(it.rarity));
    gd::strokeRect(r, x, y, kItemW, 48.0F, gd::kInk);
    app.hud.text(r, x + 12.0F, y + 4.0F, std::string(rarityName(it.rarity)) + "  " + ar::kBases[static_cast<std::size_t>(it.base)].name, gd::kWhite, gd::Hud::Style::Bold);
    app.hud.text(r, x + 12.0F, y + 24.0F, "item level " + std::to_string(it.level), gd::kWhite);

    // Prefixes first, then suffixes, like the in-game order.
    std::vector<std::size_t> order;
    for (const ar::Slot slot : {ar::Slot::Prefix, ar::Slot::Suffix}) {
        for (std::size_t i = 0; i < it.mods.size(); ++i) {
            if (ar::group(it.mods[i].group).slot == slot) order.push_back(i);
        }
    }
    float ly = y + 58.0F;
    if (it.mods.empty()) app.hud.text(r, x + 14.0F, ly, "no mods yet", gd::kMuted);
    const float flash = static_cast<float>(app.flashLeft) / static_cast<float>(kFlashFrames);
    for (const std::size_t i : order) {
        const ar::Mod& m = it.mods[i];
        const ar::Group& g = ar::group(m.group);
        const int dupes = static_cast<int>(std::count_if(it.mods.begin(), it.mods.end(), [&m](const ar::Mod& o) { return o.group == m.group; }));
        if (app.flashLeft > 0 && static_cast<int>(i) >= app.flashFrom) {
            gd::fillRect(r, x + 4.0F, ly - 1.0F, kItemW - 8.0F, kRow - 2.0F, gd::Color{234, 88, 12, static_cast<Uint8>(110.0F * flash)});
        }
        gd::fillRect(r, x + 10.0F, ly + 3.0F, 4.0F, 16.0F, g.slot == ar::Slot::Prefix ? gd::kTeal : gd::kInk);
        app.hud.text(r, x + 22.0F, ly, ar::modText(m), dupes > 1 ? gd::kOrange : gd::kInk);
        const float cw = 30.0F;
        const float cx = x + kItemW - 10.0F - cw;
        gd::fillRect(r, cx, ly + 1.0F, cw, 20.0F, tierColor(m.tier));
        gd::strokeRect(r, cx, ly + 1.0F, cw, 20.0F, gd::kInk);
        centerText(app, cx + cw / 2.0F, ly, ar::tierLabel(m.tier), tierInk(m.tier));
        ly += kRow;
    }
    // Legend of the tier colors: T1 is the best tier of its group, with its own item level requirement.
    app.hud.text(r, x + 12.0F, y + kPanelH - 64.0F, "tier", gd::kMuted);
    for (int t = 0; t < ar::kMaxTiers; ++t) {
        const float lx = x + 62.0F + 36.0F * static_cast<float>(t);
        gd::fillRect(r, lx, y + kPanelH - 64.0F, 32.0F, 20.0F, tierColor(t));
        gd::strokeRect(r, lx, y + kPanelH - 64.0F, 32.0F, 20.0F, gd::kInk);
        centerText(app, lx + 16.0F, y + kPanelH - 65.0F, ar::tierLabel(t), tierInk(t));
    }
    const ar::Limits lim = ar::limitsFor(it.rarity);
    gd::fillRect(r, x + 10.0F, y + kPanelH - 36.0F, kItemW - 20.0F, 1.0F, gd::kGrid);
    char slots[64];
    SDL_snprintf(slots, sizeof slots, "prefix %d/%d   suffix %d/%d", ar::slotsUsed(it.mods, ar::Slot::Prefix), lim.prefixes,
                 ar::slotsUsed(it.mods, ar::Slot::Suffix), lim.suffixes);
    app.hud.text(r, x + 12.0F, y + kPanelH - 30.0F, slots, gd::kMuted);
}

void drawPool(App& app) {
    SDL_Renderer* r = app.renderer;
    const ar::Item& it = app.item;
    const float x = kPoolX;
    const float y = kPanelY;
    gd::card(r, x, y, kPoolW, kPanelH, gd::kWhite, gd::kInk);

    const ar::Rarity preview = ar::previewRarity(it);
    const std::vector<ar::Entry> pool = ar::buildPool(it.base, it.level, preview, it.mods, app.rules);
    const std::vector<ar::GroupOdds> odds = ar::oddsByGroup(pool);
    double top = 0.0;
    double maxChance = 0.0;
    for (const ar::GroupOdds& g : odds) {
        top += g.tier[0];
        maxChance = std::max(maxChance, g.chance);
    }
    app.hud.text(r, x + 12.0F, y + 6.0F, "POOL FOR THE NEXT MOD", gd::kInk, gd::Hud::Style::Bold);
    rightText(app, x + kPoolW - 10.0F, y + 6.0F, "T1 " + percent(top), !app.rules.realWeights ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);
    char sub[96];
    SDL_snprintf(sub, sizeof sub, "%d entries, weight %ld", static_cast<int>(pool.size()), ar::totalWeight(pool));
    app.hud.text(r, x + 12.0F, y + 28.0F, sub, !app.rules.realWeights ? gd::kOrange : gd::kMuted);

    float ry = y + 54.0F;
    for (int gi = 0; gi < ar::kGroupCount; ++gi) {
        if (!ar::groupOnBase(gi, it.base)) continue;
        const ar::Group& g = ar::group(gi);
        const ar::GroupStatus st = ar::groupStatus(it, gi, preview, app.rules);
        const bool open = st == ar::GroupStatus::Open;
        const gd::Color text = open ? gd::kInk : gd::kMuted;
        app.hud.text(r, x + 10.0F, ry, g.slot == ar::Slot::Prefix ? "P" : "S", gd::kMuted);
        app.hud.text(r, x + 24.0F, ry, g.name, text, open ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        // Tier dots: filled when the item level has opened the tier.
        for (int t = 0; t < g.tierCount; ++t) {
            const float dx = x + 134.0F + 9.0F * static_cast<float>(t);
            const bool tierOpen = g.tiers[static_cast<std::size_t>(t)].minLevel <= it.level;
            if (tierOpen) {
                gd::disc(r, dx, ry + 11.0F, 3.5F, tierColor(t), 12);
                gd::ring(r, dx, ry + 11.0F, 3.5F, gd::kInk, 12);
            } else {
                gd::ring(r, dx, ry + 11.0F, 3.0F, gd::kMuted, 12);
            }
        }
        const float bx = x + 190.0F;
        const float bw = 76.0F;
        if (open) {
            const auto it2 = std::find_if(odds.begin(), odds.end(), [gi](const ar::GroupOdds& o) { return o.group == gi; });
            if (it2 != odds.end() && maxChance > 0.0) {
                const auto scale = static_cast<float>(static_cast<double>(bw) / maxChance);
                float cx = bx;
                for (int t = 0; t < ar::kMaxTiers; ++t) {
                    const double p = it2->tier[static_cast<std::size_t>(t)];
                    if (p <= 0.0) continue;
                    const float w = std::max(1.0F, static_cast<float>(p) * scale);
                    gd::fillRect(r, cx, ry + 4.0F, w, 14.0F, tierColor(t));
                    cx += w;
                }
                gd::strokeRect(r, bx, ry + 4.0F, cx - bx, 14.0F, gd::kInk);
                const bool dup = !app.rules.uniqueGroups && ar::groupPresent(it.mods, gi);
                rightText(app, x + kPoolW - 8.0F, ry, percent(it2->chance), dup ? gd::kOrange : gd::kInk);
            }
        } else {
            std::string why;
            if (st == ar::GroupStatus::OnItem) why = "on the item";
            if (st == ar::GroupStatus::SlotFull) why = "slots full";
            if (st == ar::GroupStatus::LevelTooLow) why = "needs level " + std::to_string(ar::lowestLevel(gi));
            app.hud.text(r, bx, ry, why, gd::kMuted);
        }
        ry += kRow;
    }
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const float x = kStatsX;
    const float y = kPanelY;
    gd::card(r, x, y, kStatsW, kPanelH, gd::kWhite, gd::kInk);
    app.hud.text(r, x + 12.0F, y + 6.0F, std::to_string(kSimItems) + " ALCHEMY ORBS", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, x + 12.0F, y + 28.0F, std::string("new ") + ar::kBases[static_cast<std::size_t>(app.item.base)].name + ", level " + std::to_string(app.item.level), gd::kMuted);
    const ar::Tally& t = app.tally;
    float ry = y + 54.0F;
    for (int tier = 0; tier < ar::kMaxTiers; ++tier) {
        const double share = t.mods > 0 ? static_cast<double>(t.perTier[static_cast<std::size_t>(tier)]) / t.mods : 0.0;
        const float w = std::clamp(static_cast<float>(share / 0.5) * 130.0F, 0.0F, 130.0F);
        centerText(app, x + 24.0F, ry, ar::tierLabel(tier), gd::kInk);
        gd::fillRect(r, x + 42.0F, ry + 3.0F, 130.0F, 16.0F, gd::kPaper);
        if (w > 0.0F) gd::fillRect(r, x + 42.0F, ry + 3.0F, std::max(1.0F, w), 16.0F, tierColor(tier));
        gd::strokeRect(r, x + 42.0F, ry + 3.0F, 130.0F, 16.0F, gd::kInk);
        rightText(app, x + kStatsW - 10.0F, ry, percent(share), gd::kInk);
        ry += kRow;
    }
    gd::fillRect(r, x + 10.0F, ry + 4.0F, kStatsW - 20.0F, 1.0F, gd::kGrid);
    const bool dupes = t.itemsWithDuplicate > 0;
    app.hud.text(r, x + 12.0F, ry + 10.0F, "same group twice:", gd::kMuted);
    app.hud.text(r, x + 12.0F, ry + 32.0F, std::to_string(t.itemsWithDuplicate) + " of " + std::to_string(t.items) + " items", dupes ? gd::kOrange : gd::kTeal, gd::Hud::Style::Bold);
}

void drawStatus(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 24.0F, kStatusY, 912.0F, 58.0F, gd::kWhite, gd::kInk);
    gd::fillRect(r, 24.0F, kStatusY, 6.0F, 58.0F, app.statusIsRefusal ? gd::kOrange : gd::kTeal);
    app.hud.text(r, 42.0F, kStatusY + 7.0F, app.status1, app.statusIsRefusal ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, 42.0F, kStatusY + 30.0F, app.status2, gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ ROLLING AFFIXES ]", "example pool, not a game's table");
    drawChips(app);
    drawSlider(app);
    drawItem(app);
    drawPool(app);
    drawStats(app);
    app.hud.text(r, 24.0F, kHintY, "keys are on the chips   arrows: item level, new item + Alchemy   1-4: base item", gd::kMuted);
    app.hud.text(r, 400.0F, kRuleY + 3.0F, "orange = the classic mistake", gd::kOrange);
    drawStatus(app);
}

}  // namespace

SDL_AppResult SDL_AppInit(void** appstate, int argc, char** argv) {
    auto* app = new App();
    *appstate = app;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) app->maxFrames = std::atol(argv[i + 1]);
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("affix-rolling", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    // Start with a rare amulet that still has room: Transmute, Augment, Regal. The slots left show the pool at work.
    newItem(*app, ar::Amulet, 78, false);
    for (const ar::Orb orb : {ar::Orb::Transmute, ar::Orb::Augment, ar::Orb::Regal}) craft(*app, orb);
    app->flashLeft = 0;
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.dragging) setLevelFromMouse(app, event->motion.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c);
                    return SDL_APP_CONTINUE;
                }
            }
            const SDL_FRect track{kSliderX0 - 14.0F, kRow1Y, kSliderX1 - kSliderX0 + 28.0F, kChipH};
            if (SDL_PointInRectFloat(&p, &track)) {
                app.dragging = true;
                setLevelFromMouse(app, p.x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) endDrag(app);
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: endDrag(app); break;
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode key = event->key.key;
            if (key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            // Scrubbing the level may repeat; everything else is one action per press.
            if (key == SDLK_LEFT || key == SDLK_RIGHT || key == SDLK_DOWN || key == SDLK_UP) {
                const int step = (key == SDLK_LEFT || key == SDLK_RIGHT) ? 1 : 10;
                const int dir = (key == SDLK_RIGHT || key == SDLK_UP) ? 1 : -1;
                const int level = ar::clampLevel(app.item.level + dir * step);
                if (level != app.item.level) newItem(app, app.item.base, level, true);
                break;
            }
            if (event->key.repeat) break;
            switch (key) {
                case SDLK_1: applyKey(app, Action::Base, 0); break;
                case SDLK_2: applyKey(app, Action::Base, 1); break;
                case SDLK_3: applyKey(app, Action::Base, 2); break;
                case SDLK_4: applyKey(app, Action::Base, 3); break;
                case SDLK_T: applyKey(app, Action::Orb, static_cast<int>(ar::Orb::Transmute)); break;
                case SDLK_U: applyKey(app, Action::Orb, static_cast<int>(ar::Orb::Augment)); break;
                case SDLK_R: applyKey(app, Action::Orb, static_cast<int>(ar::Orb::Regal)); break;
                case SDLK_A: applyKey(app, Action::Orb, static_cast<int>(ar::Orb::Alchemy)); break;
                case SDLK_E: applyKey(app, Action::Orb, static_cast<int>(ar::Orb::Exalt)); break;
                case SDLK_C: applyKey(app, Action::Orb, static_cast<int>(ar::Orb::Chaos)); break;
                case SDLK_X: applyKey(app, Action::Orb, static_cast<int>(ar::Orb::Reset)); break;
                case SDLK_W: applyKey(app, Action::Weights, 0); break;
                case SDLK_G: applyKey(app, Action::Groups, 0); break;
                default: break;
            }
            break;
        }
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    if (app.statsDirty) {
        app.tally = ar::simulateAlchemy(app.item.base, app.item.level, app.rules, kSimItems, kSimSeed);
        app.statsDirty = false;
    }
    if (app.flashLeft > 0) --app.flashLeft;
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
