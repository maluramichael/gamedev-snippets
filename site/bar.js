// Link bar shown on every demo page unless it is embedded on malura.de (?embed=1).
import { barModel } from "/_shared/index.js";

const params = new URLSearchParams(location.search);
if (params.get("embed") !== "1") {
  try {
    const response = await fetch("demo.json", { cache: "no-cache" });
    if (response.ok) {
      const demo = await response.json();
      const lang = (params.get("lang") || navigator.language || "en").startsWith("de") ? "de" : "en";
      const model = barModel(demo, lang);
      const label = lang === "de"
        ? { post: "Zum Blogpost", source: "Quellcode", all: "Alle Demos" }
        : { post: "Read the post", source: "Source code", all: "All demos" };

      const bar = document.createElement("nav");
      bar.className = "demo-bar";
      const add = (text, href) => {
        const a = document.createElement("a");
        a.textContent = text;
        a.href = href;
        a.target = "_top";
        bar.append(a);
      };
      add("demos.malura.de", "/");
      add(label.all, "/");
      if (model.postUrl) add(label.post, model.postUrl);
      else if (model.notice) {
        const span = document.createElement("span");
        span.textContent = model.notice;
        bar.append(span);
      }
      add(label.source, model.repoUrl);
      add("Ko-fi", model.kofiUrl);
      add("malura.de", model.homeUrl);
      document.body.append(bar);
    }
  } catch (error) {
    console.warn("link bar unavailable", error);
  }
}
