// Categories of the gallery. A demo names one of these keys in the "category" field of its demo.json.
export const CATEGORIES = [
  { key: "terrain", de: "Terrain & Prozedural", en: "Terrain & Procedural" },
  { key: "grid", de: "Hex & Raster", en: "Hex & Grids" },
  { key: "ai", de: "KI & Wegfindung", en: "AI & Pathfinding" },
  { key: "physics", de: "Physik & Kollision", en: "Physics & Collision" },
  { key: "graphics", de: "Grafik", en: "Graphics" },
  { key: "gameplay", de: "Gameplay & Balance", en: "Gameplay & Balance" },
  { key: "time", de: "Zeit & Bewegung", en: "Time & Motion" },
  { key: "space", de: "Weltraum", en: "Space" },
];

/** Categories that have at least one demo, in CATEGORIES order, with their count. Unknown keys are ignored. */
export function categoryCounts(demos) {
  return CATEGORIES.map((c) => ({ key: c.key, count: demos.filter((d) => d.category === c.key).length })).filter((c) => c.count > 0);
}

/** Demos of one category. "all", undefined or an unknown key returns every demo. */
export function filterByCategory(demos, key) {
  if (!CATEGORIES.some((c) => c.key === key)) return demos;
  return demos.filter((d) => d.category === key);
}

/** The category a URL hash like "#terrain" selects, "all" when the hash is empty, unknown or the category has no demo. */
export function categoryFromHash(hash, demos) {
  const key = String(hash ?? "").replace(/^#/, "");
  return categoryCounts(demos).some((c) => c.key === key) ? key : "all";
}

export function categoryLabel(key, lang) {
  const category = CATEGORIES.find((c) => c.key === key);
  if (!category) return key;
  return lang === "de" ? category.de : category.en;
}
