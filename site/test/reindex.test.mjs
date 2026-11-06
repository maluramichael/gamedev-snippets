import test from "node:test";
import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { mkdtempSync, mkdirSync, writeFileSync, readFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";

const script = join(dirname(fileURLToPath(import.meta.url)), "..", "reindex.sh");

function demoRoot(folders) {
  const root = mkdtempSync(join(tmpdir(), "demos-"));
  for (const [name, withJson] of Object.entries(folders)) {
    mkdirSync(join(root, name));
    if (withJson) writeFileSync(join(root, name, "demo.json"), "{}");
  }
  return root;
}

function reindex(root) {
  execFileSync("sh", [script, root]);
  return JSON.parse(readFileSync(join(root, "demos.json"), "utf8"));
}

test("lists only folders with a demo.json, sorted", () => {
  const root = demoRoot({ "b-demo": true, "a-demo": true, "no-json": false });
  assert.deepEqual(reindex(root), ["a-demo", "b-demo"]);
});

test("ignores shared and hidden folders and invalid names", () => {
  const root = demoRoot({ _shared: true, ".git": true, "Bad Name": true, "ok-1": true });
  assert.deepEqual(reindex(root), ["ok-1"]);
});

test("writes an empty list for an empty root", () => {
  assert.deepEqual(reindex(demoRoot({})), []);
});
