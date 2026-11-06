import test from "node:test";
import assert from "node:assert/strict";
import { CATEGORIES, categoryCounts, filterByCategory, categoryFromHash, categoryLabel } from "../categories.js";

const demos = [
  { name: "a", category: "terrain" },
  { name: "b", category: "terrain" },
  { name: "c", category: "ai" },
  { name: "d" },
  { name: "e", category: "not-a-category" },
];

test("every category has a key and a label in both languages, keys are unique", () => {
  const keys = new Set();
  for (const c of CATEGORIES) {
    assert.match(c.key, /^[a-z]+$/);
    assert.ok(c.de.length > 0 && c.en.length > 0);
    keys.add(c.key);
  }
  assert.equal(keys.size, CATEGORIES.length);
});

test("categoryCounts counts per category in list order and skips empty and unknown ones", () => {
  const counts = categoryCounts(demos);
  assert.deepEqual(counts.map((c) => [c.key, c.count]), [["terrain", 2], ["ai", 1]].sort((x, y) => CATEGORIES.findIndex((c) => c.key === x[0]) - CATEGORIES.findIndex((c) => c.key === y[0])));
  assert.deepEqual(categoryCounts([]), []);
});

test("filterByCategory keeps only the category and returns everything for all or an unknown key", () => {
  assert.deepEqual(filterByCategory(demos, "terrain").map((d) => d.name), ["a", "b"]);
  assert.deepEqual(filterByCategory(demos, "ai").map((d) => d.name), ["c"]);
  assert.equal(filterByCategory(demos, "all").length, 5);
  assert.equal(filterByCategory(demos, "nope").length, 5);
  assert.equal(filterByCategory(demos, undefined).length, 5);
});

test("categoryFromHash accepts a category that has demos and falls back to all", () => {
  assert.equal(categoryFromHash("#terrain", demos), "terrain");
  assert.equal(categoryFromHash("#physics", demos), "all");
  assert.equal(categoryFromHash("#bogus", demos), "all");
  assert.equal(categoryFromHash("", demos), "all");
  assert.equal(categoryFromHash("#all", demos), "all");
});

test("categoryLabel falls back to English for an unknown language and to the key for an unknown category", () => {
  assert.equal(categoryLabel("terrain", "de"), CATEGORIES.find((c) => c.key === "terrain").de);
  assert.equal(categoryLabel("terrain", "fr"), CATEGORIES.find((c) => c.key === "terrain").en);
  assert.equal(categoryLabel("zzz", "de"), "zzz");
});
