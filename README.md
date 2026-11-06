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

| Folder | Topic | Category |
|--------|-------|----------|
| `update-loop/` | Update loop with a fixed timestep | Time & Motion |
| `update-loop-py/` | Update loop with a fixed timestep (Python) | Time & Motion |
| `delta-time/` | Delta time: motion independent of the frame rate | Time & Motion |
| `fixed-timestep/` | Fixed timestep: drawing with interpolation | Time & Motion |
| `point-in-polygon/` | Point in polygon | Physics & Collision |
| `picking-3d/` | Picking in 3D space | Physics & Collision |
| `raytracing/` | Raytracing | Graphics |
| `image-loading/` | Loading and rendering an image | Graphics |
| `rigid-bodies/` | Rigid bodies | Physics & Collision |
| `pixel-water/` | Pixel water | Graphics |
| `water-ripples-2d/` | 2D water ripples | Graphics |
| `water-waves-3d/` | 3D water waves | Graphics |
| `ray-marching/` | Ray marching | Graphics |
| `voronoi/` | Voronoi diagrams | Terrain & Procedural |
| `xp-curves/` | XP curves | Gameplay & Balance |
| `easing/` | Easing and tweening | Time & Motion |
| `ca-caves/` | Caves with Cellular Automata | Terrain & Procedural |
| `terrain-mesh/` | Terrain in 3D | Terrain & Procedural |
| `poisson-disc/` | Poisson Disc Sampling | Terrain & Procedural |
| `curved-paths/` | Curves: Bezier and Arcs | Graphics |

Every demo runs on <https://demos.malura.de>, where the gallery can be filtered by category.

## Adding a demo

See [docs/adding-a-demo.md](docs/adding-a-demo.md): one folder with a `demo.json`, CMake finds it by itself. Every
demo accepts `--frames N` and exits with code 0 after N frames, so `ctest` can run it headless. The logic of a demo
lives in a header without SDL and has its own test.

## License

Apache-2.0, see `LICENSE` and `NOTICE`.
