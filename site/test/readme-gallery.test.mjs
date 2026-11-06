import test from "node:test";
import assert from "node:assert/strict";
import { renderGallery, replaceBetweenMarkers } from "../../scripts/readme-gallery.mjs";

const demos = [
  { folder: "alpha", title: "Alpha demo", category: "Graphics", publishAt: "2026-07-24T08:00:00+02:00" },
  { folder: "beta", title: "Beta | pipe", category: "Space", publishAt: "2026-07-31T08:00:00+02:00" },
  { folder: "gamma", title: "Gamma", category: "Time & Motion", publishAt: "2026-08-07T08:00:00+02:00" },
];

test("renderGallery builds a two column table with thumbnail, title and links per cell", () => {
  const md = renderGallery(demos);
  const lines = md.trim().split("\n");
  assert.equal(lines[0], "| | |");
  assert.equal(lines[1], "|:--|:--|");
  assert.equal(lines.length, 4, "header, separator, two rows for three demos");
  assert.match(lines[2], /<img src="alpha\/thumb\.png"/);
  assert.match(lines[2], /<img src="beta\/thumb\.png"/);
  assert.match(lines[3], /<img src="gamma\/thumb\.png"/);
  assert.match(lines[2], /https:\/\/demos\.malura\.de\/alpha\//);
});

test("renderGallery fills the last row of an odd list with an empty cell", () => {
  const row = renderGallery(demos).trim().split("\n")[3];
  assert.equal(row.split("|").length, 4, "leading pipe, two cells, trailing pipe");
  assert.match(row, /\| *\|$/);
});

test("renderGallery orders by publishAt and does not let a title break the table", () => {
  const md = renderGallery([demos[2], demos[1], demos[0]]);
  assert.ok(md.indexOf("alpha/") < md.indexOf("beta/") && md.indexOf("beta/") < md.indexOf("gamma/"));
  assert.ok(!/\|\s*Beta \| pipe/.test(md), "a raw pipe in a title would split the cell");
  assert.match(md, /Beta \\\| pipe/);
});

test("renderGallery with no demos returns an empty string", () => {
  assert.equal(renderGallery([]), "");
});

test("replaceBetweenMarkers swaps only the marked block and keeps the rest", () => {
  const src = "before\n<!-- demos:start -->\nold\n<!-- demos:end -->\nafter\n";
  assert.equal(replaceBetweenMarkers(src, "NEW\n"), "before\n<!-- demos:start -->\nNEW\n<!-- demos:end -->\nafter\n");
});

test("replaceBetweenMarkers throws when a marker is missing", () => {
  assert.throws(() => replaceBetweenMarkers("no markers here", "x"), /marker/i);
});
