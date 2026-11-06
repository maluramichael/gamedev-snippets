# Adding a demo

Every demo is one folder. CMake finds it by its `demo.json`, there is no list to edit.

```
<name>/
  main.cpp        SDL3 callbacks app (or main.py for a Pyodide demo)
  logic.hpp       optional: the testable logic, no SDL (any extra *.cpp/*.hpp next to main.cpp is fine)
  test.cpp        optional, native only: plain asserts on logic.hpp, run by ctest as <name>-logic
  requires-gl     optional empty marker file: GPU demo (OpenGL ES 3 / WebGL2), built for the web only
  demo.json       title/hint (de, en), post slugs, repo folder, publishAt
  thumb.png       screenshot of the canvas, 1280x720
  README.md       one paragraph, no link to the post (the demo page links it by itself once the post is live)
```

## Look (same on every demo)

* Canvas is **960x540**. Call `SDL_SetRenderLogicalPresentation(renderer, 960, 540, SDL_LOGICAL_PRESENTATION_LETTERBOX)` right
  after the renderer exists and `SDL_ConvertEventToRenderCoordinates(renderer, event)` at the top of `SDL_AppEvent`, so the
  demo scales with the page and the mouse maps correctly.
* Use `common/hud.hpp` (`#include "hud.hpp"`): JetBrains Mono from embedded bitmap atlases, teal and orange on paper with a grid.

  ```cpp
  gd::Hud hud;  if (!hud.load(renderer)) return SDL_APP_FAILURE;   // once
  gd::backdrop(r, 960, 540);                                       // every frame, first
  hud.titleBar(r, 960, "[ TITLE ]", "right text");                 // 30 px teal strip
  hud.text(r, x, y, "text", gd::kInk, gd::Hud::Style::Bold);       // Regular | Bold | Large (cells 10x22, 10x22, 19x43)
  gd::card(r, x, y, w, h, fill, edge);                             // rectangle with hard offset shadow, buttons and chips
  gd::fillRect / strokeRect / gradient / disc / ring               // primitives
  hud.destroy();                                                   // in SDL_AppQuit
  ```

  Colors: `kTeal kTealDark kTealSoft kOrange kInk kMuted kPaper kGrid kWhite`. Teal is the "good/correct" side, orange the
  "naive/problem" side. Text charset: ASCII plus ä ö ü Ä Ö Ü ß ° · × ± … § €. Anything else is skipped.
* Interaction should be discoverable: clickable chips (see `delta-time/main.cpp`, `fixed-timestep/main.cpp`), keys as a bonus,
  the keys named on screen. The first frame must already show something interesting without input.
* Text on screen is English. Keep it short, labels not sentences.

## Rules for main.cpp

* `--frames N` exits with code 0 after N rendered frames (ctest runs 120 frames with `SDL_VIDEODRIVER=dummy`).
* `hud.load` failure and any SDL failure return `SDL_APP_FAILURE` with a log line, no silent fallbacks.
* Logic that can be wrong (geometry, math, simulation) goes into a header without SDL and gets a `test.cpp` with asserts
  on exact, hand-computed cases and edge cases. Write the test first and watch it fail.
* Comment the *why* at the top of `main.cpp` in a few lines. No em-dashes anywhere (U+2014), English in code and comments.
* GPU demos: add the empty file `requires-gl`, create the window with `SDL_WINDOW_OPENGL` and an OpenGL ES 3.0 context
  (`SDL_GL_CONTEXT_PROFILE_ES`, major 3, minor 0). Shaders start with `#version 300 es` and `precision highp float;`.
  The web build links WebGL2 automatically (`-sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2`). A headless container has no GL,
  so `--frames` must still exit 0 when the context cannot be created ONLY in the native build, which does not exist for GL demos.
  Check GPU demos in the browser instead.

## demo.json

```json
{
  "title": { "de": "Punkt im Polygon", "en": "Point in polygon" },
  "hint": { "de": "Ein Satz, was man in der Demo tun kann.", "en": "One sentence on what to do in the demo." },
  "post": { "de": "<de-slug>", "en": "<en-slug>" },
  "repo": "<name>",
  "category": "physics",
  "publishAt": "2026-12-04T08:00:00+01:00"
}
```

`category` is one key of `site/categories.js` (`terrain`, `grid`, `ai`, `physics`, `graphics`, `gameplay`, `time`, `space`). The gallery shows one filter
chip per category that has demos. Pick the topic the demo teaches, not the technique it happens to use.

The en slug is the slug the English title gets on malura.de (lowercase, hyphens, no umlauts, no special characters).

## Build and check (on vault, never on the Mac)

```
sh scripts/preview.sh <name> native     # configure, build, ctest on vault, prints the ctest summary
sh scripts/preview.sh <name> web        # Emscripten build, fetched to build/web-<name>
uv run python -m http.server <free port> -d build/web-<name>    # then http://localhost:<port>/<name>/?embed=1
```

Look at it with `playwright-cli -s=<name> ...` (screenshot the canvas: `screenshot canvas --filename <file>`; clicks with
`run-code "async page => { await page.mouse.click(x, y); }"`), then read the screenshot. Check the browser console for errors.
Save the thumb as `<name>/thumb.png`.
