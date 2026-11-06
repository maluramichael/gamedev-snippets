import test from "node:test";
import assert from "node:assert/strict";
import { isPostLive, visibleDemos, barModel } from "../index.js";

const now = new Date("2026-11-10T12:00:00Z");

test("isPostLive is true without publishAt", () => {
  assert.equal(isPostLive(undefined, now), true);
});

test("isPostLive is false before and true after publishAt", () => {
  assert.equal(isPostLive("2026-11-13T08:00:00+01:00", now), false);
  assert.equal(isPostLive("2026-11-01T08:00:00+01:00", now), true);
});

test("visibleDemos hides demos whose post is not live yet", () => {
  const demos = [
    { name: "past", publishAt: "2026-11-01T08:00:00+01:00" },
    { name: "future", publishAt: "2026-11-13T08:00:00+01:00" },
    { name: "undated" },
  ];
  assert.deepEqual(visibleDemos(demos, now), ["past", "undated"]);
});

test("visibleDemos names the demo with an invalid publishAt", () => {
  assert.throws(() => visibleDemos([{ name: "broken", publishAt: "nope" }], now), /broken/);
});

const demo = {
  post: { de: "update-loop", en: "update-loop-en" },
  repo: "update-loop",
  publishAt: "2026-11-13T08:00:00+01:00",
};

test("barModel shows a notice instead of the post link before publishAt", () => {
  const model = barModel(demo, "de", now);
  assert.equal(model.postUrl, null);
  assert.match(model.notice, /13\.11\.2026/);
});

test("barModel links the post of the active language after publishAt", () => {
  const later = new Date("2026-11-14T00:00:00Z");
  assert.equal(barModel(demo, "de", later).postUrl, "https://malura.de/blog/update-loop");
  assert.equal(barModel(demo, "en", later).postUrl, "https://malura.de/en/blog/update-loop-en");
});

test("barModel always links the repo folder and Ko-fi", () => {
  const model = barModel(demo, "en", now);
  assert.equal(model.repoUrl, "https://github.com/maluramichael/gamedev-snippets/tree/master/update-loop");
  assert.equal(model.kofiUrl, "https://ko-fi.com/malura");
});
