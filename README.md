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
| `update-loop/` | Update loop with a fixed timestep | [update-loop](https://malura.de/en/blog/update-loop) (from 2026-11-13) |
| `update-loop-py/` | Update loop with a fixed timestep (Python) | [update-loop](https://malura.de/en/blog/update-loop) (from 2026-11-13) |
| `delta-time/` | Delta time: motion independent of the frame rate | [delta-time-motion-independent-of-the-frame-rate](https://malura.de/en/blog/delta-time-motion-independent-of-the-frame-rate) (from 2026-11-20) |
| `fixed-timestep/` | Fixed timestep: drawing with interpolation | [fixed-timestep-drawing-with-interpolation](https://malura.de/en/blog/fixed-timestep-drawing-with-interpolation) (from 2026-11-27) |
| `point-in-polygon/` | Point in polygon | [point-in-polygon](https://malura.de/en/blog/point-in-polygon) (from 2026-12-04) |
| `picking-3d/` | Picking in 3D space | [picking-in-3d-space](https://malura.de/en/blog/picking-in-3d-space) (from 2026-12-11) |
| `raytracing/` | Raytracing | [raytracing](https://malura.de/en/blog/raytracing) (from 2026-12-18) |
| `image-loading/` | Loading and rendering an image | [loading-and-rendering-an-image](https://malura.de/en/blog/loading-and-rendering-an-image) (from 2026-12-25) |
| `rigid-bodies/` | Rigid bodies | [rigid-bodies](https://malura.de/en/blog/rigid-bodies) (from 2027-01-01) |
| `pixel-water/` | Pixel water | [pixel-water](https://malura.de/en/blog/pixel-water) (from 2027-01-08) |
| `water-ripples-2d/` | 2D water ripples | [2d-water-ripples](https://malura.de/en/blog/2d-water-ripples) (from 2027-01-15) |
| `water-waves-3d/` | 3D water waves | [3d-water-waves](https://malura.de/en/blog/3d-water-waves) (from 2027-01-22) |
| `ray-marching/` | Ray marching | [ray-marching](https://malura.de/en/blog/ray-marching) (from 2027-01-29) |
| `voronoi/` | Voronoi diagrams | [voronoi-diagrams](https://malura.de/en/blog/voronoi-diagrams) (from 2027-02-05) |
| `xp-curves/` | XP curves | [xp-curves](https://malura.de/en/blog/xp-curves) (from 2027-02-12) |
| `easing/` | Easing and tweening | [easing-and-tweening](https://malura.de/en/blog/easing-and-tweening) (from 2027-02-19) |

Every demo is on <https://demos.malura.de>, the blog post links appear on the posts' publication dates.

## Adding a demo

See [docs/adding-a-demo.md](docs/adding-a-demo.md): one folder with a `demo.json`, CMake finds it by itself. Every
demo accepts `--frames N` and exits with code 0 after N frames, so `ctest` can run it headless. The logic of a demo
lives in a header without SDL and has its own test.

## License

Apache-2.0, see `LICENSE` and `NOTICE`.
