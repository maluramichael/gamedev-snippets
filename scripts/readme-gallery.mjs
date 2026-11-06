// Rewrites the demo gallery in README.md: a two column table with the thumbnail of every committed demo.
// Usage: node scripts/readme-gallery.mjs    (run after adding or renaming a demo, then commit README.md)
import { execFileSync } from "node:child_process";
import { existsSync, readFileSync, writeFileSync } from "node:fs";
import { pathToFileURL } from "node:url";
import { categoryLabel } from "../site/categories.js";

const START = "<!-- demos:start -->";
const END = "<!-- demos:end -->";
const LIVE = "https://demos.malura.de";

const cell = (d) => {
  const title = d.title.replaceAll("|", "\\|");
  return (
    `<a href="${d.folder}/"><img src="${d.folder}/thumb.png" width="440" alt="${title.replaceAll('"', "&quot;")}"></a><br>` +
    `**${title}**<br>${d.category} · [live](${LIVE}/${d.folder}/) · [code](${d.folder}/)`
  );
};

/** Markdown table, two demos per row, ordered by publishAt (missing dates last, then by folder). */
export function renderGallery(demos) {
  if (demos.length === 0) return "";
  const sorted = [...demos].sort((a, b) => (a.publishAt ?? "9").localeCompare(b.publishAt ?? "9") || a.folder.localeCompare(b.folder));
  const rows = [];
  for (let i = 0; i < sorted.length; i += 2) {
    rows.push(`| ${cell(sorted[i])} | ${sorted[i + 1] ? cell(sorted[i + 1]) : ""} |`);
  }
  return ["| | |", "|:--|:--|", ...rows].join("\n") + "\n";
}

export function replaceBetweenMarkers(source, block) {
  const start = source.indexOf(START);
  const end = source.indexOf(END);
  if (start < 0 || end < start) throw new Error(`marker ${START} / ${END} missing in README.md`);
  return source.slice(0, start + START.length) + "\n" + block + source.slice(end);
}

function committedDemos(root) {
  const files = execFileSync("git", ["ls-files", "*/demo.json"], { cwd: root, encoding: "utf8" }).split("\n").filter(Boolean);
  const demos = [];
  for (const file of files) {
    const folder = file.split("/")[0];
    if (!existsSync(`${root}/${folder}/thumb.png`)) continue;
    const json = JSON.parse(readFileSync(`${root}/${file}`, "utf8"));
    demos.push({ folder, title: json.title.en, category: categoryLabel(json.category, "en"), publishAt: json.publishAt });
  }
  return demos;
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  const root = new URL("..", import.meta.url).pathname.replace(/\/$/, "");
  const readme = `${root}/README.md`;
  const demos = committedDemos(root);
  writeFileSync(readme, replaceBetweenMarkers(readFileSync(readme, "utf8"), renderGallery(demos)));
  console.log(`README gallery: ${demos.length} demos`);
}
