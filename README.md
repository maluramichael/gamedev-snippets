# gamedev-snippets

<!-- links:start -->
[![Support me on Ko-fi](https://malura.de/assets/badge/kofi.svg)](https://malura.de/go/kofi?ref=gamedev-snippets)
[![Check out my page](https://malura.de/assets/badge/malura.svg)](https://malura.de/go/site?ref=gamedev-snippets)
<!-- links:end -->

Small, self-contained game development demos in C++20 with SDL3. Every folder is one topic, one
demo, one blog post on [malura.de](https://malura.de). The same code builds natively and runs in the
browser through WebAssembly.

Live demos: <https://demos.malura.de>

## Build

Native (needs CMake 3.21+, a C++20 compiler and Git for fetching SDL3):

```sh
cmake --preset native
cmake --build --preset native
ctest --preset native
```

Web (needs the Emscripten SDK, `EMSDK` must point to it):

```sh
cmake --preset web
cmake --build --preset web      # output: build/web/<demo>/index.html
```

`scripts/build.sh native|web` runs the same steps in the pinned `emscripten/emsdk` Docker image.
Serve `build/web/<demo>/` with any static file server, WebAssembly needs the right MIME type.

## Demos

| Folder | Topic | Blog post |
|--------|-------|-----------|
| `update-loop/` | Fixed timestep with an accumulator and interpolation | (published 2026-11-13) |

## Adding a demo

1. Create `<name>/main.cpp` and `<name>/demo.json` (see `update-loop/demo.json` for the fields).
2. Add `add_demo(<name> SOURCES <name>/main.cpp)` to the top-level `CMakeLists.txt`.
3. Every demo accepts `--frames N` and exits with code 0 after N frames, so `ctest` can run it headless.

## License

Apache-2.0, see `LICENSE` and `NOTICE`.
