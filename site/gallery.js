import { isPostLive, listedDemos, KOFI_URL } from "/_shared/index.js";

const lang = (navigator.language || "en").startsWith("de") ? "de" : "en";
const cards = document.getElementById("cards");

async function loadDemo(name) {
  try {
    const response = await fetch(`/${name}/demo.json`, { cache: "no-cache" });
    return response.ok ? { ...(await response.json()), name } : null;
  } catch {
    return null;
  }
}

function card(demo) {
  const link = document.createElement("a");
  link.className = "card";
  link.href = `/${demo.name}/`;

  const shot = document.createElement("div");
  shot.className = "shot";
  shot.style.backgroundImage = `url("/${demo.name}/cover.webp")`;
  const img = document.createElement("img");
  img.loading = "lazy";
  img.alt = "";
  img.src = `/${demo.name}/thumb.png`;
  img.addEventListener("error", () => img.remove());
  shot.append(img);

  const text = document.createElement("div");
  text.className = "text";
  const title = document.createElement("h2");
  title.textContent = demo.title?.[lang] ?? demo.name;
  const hint = document.createElement("p");
  hint.textContent = demo.hint?.[lang] ?? "";
  text.append(title, hint);

  if (demo.publishAt && !isPostLive(demo.publishAt)) {
    const badge = document.createElement("span");
    badge.className = "badge";
    const date = new Date(demo.publishAt).toLocaleDateString(lang === "de" ? "de-DE" : "en-GB", { timeZone: "Europe/Berlin" });
    badge.textContent = lang === "de" ? `Post folgt am ${date}` : `post on ${date}`;
    shot.append(badge);
  }

  link.append(shot, text);
  return link;
}

try {
  const names = await (await fetch("/demos.json", { cache: "no-cache" })).json();
  const loaded = (await Promise.all(names.map(loadDemo))).filter(Boolean);
  const valid = loaded.filter((demo) => {
    try {
      isPostLive(demo.publishAt);
      return true;
    } catch (error) {
      console.error(`${demo.name}: ${error.message}`);
      return false;
    }
  });
  const listed = new Set(listedDemos(valid));
  const shown = valid
    .filter((demo) => listed.has(demo.name))
    .sort((a, b) => Date.parse(a.publishAt ?? 0) - Date.parse(b.publishAt ?? 0));
  if (shown.length === 0) {
    cards.innerHTML = "";
    const empty = document.createElement("p");
    empty.className = "empty";
    empty.textContent = lang === "de" ? "Noch keine Demos vorhanden." : "No demos yet.";
    cards.replaceWith(empty);
  } else {
    cards.replaceChildren(...shown.map(card));
  }
} catch (error) {
  cards.textContent = lang === "de" ? "Die Demo-Liste konnte nicht geladen werden." : "The demo list could not be loaded.";
  console.error(error);
}
document.querySelector('a[data-kofi]').href = KOFI_URL;
