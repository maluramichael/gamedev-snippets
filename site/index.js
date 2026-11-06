// Shared logic for demos.malura.de: gallery validation and the link bar.
// Plain ES module, no dependencies, tested with `node --test site/test/index.test.mjs`.

const BLOG = "https://malura.de";
const REPO = "https://github.com/maluramichael/gamedev-snippets";
export const KOFI_URL = "https://ko-fi.com/malura";

/** True when the blog post of a demo is public. Demos without publishAt count as live. */
export function isPostLive(publishAt, now = new Date()) {
  if (publishAt === undefined || publishAt === null) return true;
  const time = Date.parse(publishAt);
  if (Number.isNaN(time)) throw new Error(`invalid publishAt: ${publishAt}`);
  return time <= now.getTime();
}

/**
 * Names of the demos that belong in the gallery: all of them. Demos go live as soon as they are pushed,
 * publishAt only decides when the link to the blog post appears. Throws with the demo name on a bad publishAt.
 */
export function listedDemos(demos) {
  return demos.map((demo) => {
    try {
      isPostLive(demo.publishAt);
    } catch (error) {
      throw new Error(`${demo.name}: ${error.message}`);
    }
    return demo.name;
  });
}

/** What the link bar shows for one demo, in language "de" or "en". The post link appears once the post is live, before that the bar says nothing about it. */
export function barModel(demo, lang, now = new Date()) {
  const language = lang === "de" ? "de" : "en";
  const slug = demo.post?.[language];
  const live = isPostLive(demo.publishAt, now);
  const postUrl = live && slug ? (language === "de" ? `${BLOG}/blog/${slug}` : `${BLOG}/en/blog/${slug}`) : null;
  return {
    postUrl,
    repoUrl: `${REPO}/tree/master/${demo.repo}`,
    kofiUrl: KOFI_URL,
    homeUrl: BLOG,
  };
}
