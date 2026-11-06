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

<!-- demos:start -->
| | |
|:--|:--|
| <a href="update-loop/"><img src="update-loop/thumb.png" width="440" alt="Update loop with a fixed timestep"></a><br>**Update loop with a fixed timestep**<br>Time & Motion · [live](https://demos.malura.de/update-loop/) · [code](update-loop/) | <a href="update-loop-py/"><img src="update-loop-py/thumb.png" width="440" alt="Update loop with a fixed timestep (Python)"></a><br>**Update loop with a fixed timestep (Python)**<br>Time & Motion · [live](https://demos.malura.de/update-loop-py/) · [code](update-loop-py/) |
| <a href="delta-time/"><img src="delta-time/thumb.png" width="440" alt="Delta time: motion independent of the frame rate"></a><br>**Delta time: motion independent of the frame rate**<br>Time & Motion · [live](https://demos.malura.de/delta-time/) · [code](delta-time/) | <a href="fixed-timestep/"><img src="fixed-timestep/thumb.png" width="440" alt="Fixed timestep: drawing with interpolation"></a><br>**Fixed timestep: drawing with interpolation**<br>Time & Motion · [live](https://demos.malura.de/fixed-timestep/) · [code](fixed-timestep/) |
| <a href="point-in-polygon/"><img src="point-in-polygon/thumb.png" width="440" alt="Point in polygon"></a><br>**Point in polygon**<br>Physics & Collision · [live](https://demos.malura.de/point-in-polygon/) · [code](point-in-polygon/) | <a href="picking-3d/"><img src="picking-3d/thumb.png" width="440" alt="Picking in 3D space"></a><br>**Picking in 3D space**<br>Physics & Collision · [live](https://demos.malura.de/picking-3d/) · [code](picking-3d/) |
| <a href="raytracing/"><img src="raytracing/thumb.png" width="440" alt="Raytracing"></a><br>**Raytracing**<br>Graphics · [live](https://demos.malura.de/raytracing/) · [code](raytracing/) | <a href="image-loading/"><img src="image-loading/thumb.png" width="440" alt="Loading and rendering an image"></a><br>**Loading and rendering an image**<br>Graphics · [live](https://demos.malura.de/image-loading/) · [code](image-loading/) |
| <a href="rigid-bodies/"><img src="rigid-bodies/thumb.png" width="440" alt="Rigid bodies"></a><br>**Rigid bodies**<br>Physics & Collision · [live](https://demos.malura.de/rigid-bodies/) · [code](rigid-bodies/) | <a href="pixel-water/"><img src="pixel-water/thumb.png" width="440" alt="Pixel water"></a><br>**Pixel water**<br>Graphics · [live](https://demos.malura.de/pixel-water/) · [code](pixel-water/) |
| <a href="water-ripples-2d/"><img src="water-ripples-2d/thumb.png" width="440" alt="2D water ripples"></a><br>**2D water ripples**<br>Graphics · [live](https://demos.malura.de/water-ripples-2d/) · [code](water-ripples-2d/) | <a href="water-waves-3d/"><img src="water-waves-3d/thumb.png" width="440" alt="3D water waves"></a><br>**3D water waves**<br>Graphics · [live](https://demos.malura.de/water-waves-3d/) · [code](water-waves-3d/) |
| <a href="ray-marching/"><img src="ray-marching/thumb.png" width="440" alt="Ray marching"></a><br>**Ray marching**<br>Graphics · [live](https://demos.malura.de/ray-marching/) · [code](ray-marching/) | <a href="voronoi/"><img src="voronoi/thumb.png" width="440" alt="Voronoi diagrams"></a><br>**Voronoi diagrams**<br>Terrain & Procedural · [live](https://demos.malura.de/voronoi/) · [code](voronoi/) |
| <a href="xp-curves/"><img src="xp-curves/thumb.png" width="440" alt="XP curves"></a><br>**XP curves**<br>Gameplay & Balance · [live](https://demos.malura.de/xp-curves/) · [code](xp-curves/) | <a href="easing/"><img src="easing/thumb.png" width="440" alt="Easing and tweening"></a><br>**Easing and tweening**<br>Time & Motion · [live](https://demos.malura.de/easing/) · [code](easing/) |
| <a href="terrain-shaping/"><img src="terrain-shaping/thumb.png" width="440" alt="Height Curves and Terraces"></a><br>**Height Curves and Terraces**<br>Terrain & Procedural · [live](https://demos.malura.de/terrain-shaping/) · [code](terrain-shaping/) | <a href="steering-behaviors/"><img src="steering-behaviors/thumb.png" width="440" alt="Steering Behaviors"></a><br>**Steering Behaviors**<br>AI & Pathfinding · [live](https://demos.malura.de/steering-behaviors/) · [code](steering-behaviors/) |
| <a href="terrain-mesh/"><img src="terrain-mesh/thumb.png" width="440" alt="Terrain in 3D"></a><br>**Terrain in 3D**<br>Terrain & Procedural · [live](https://demos.malura.de/terrain-mesh/) · [code](terrain-mesh/) | <a href="ca-caves/"><img src="ca-caves/thumb.png" width="440" alt="Caves with Cellular Automata"></a><br>**Caves with Cellular Automata**<br>Terrain & Procedural · [live](https://demos.malura.de/ca-caves/) · [code](ca-caves/) |
| <a href="wfc-tiles/"><img src="wfc-tiles/thumb.png" width="440" alt="Wave Function Collapse"></a><br>**Wave Function Collapse**<br>Terrain & Procedural · [live](https://demos.malura.de/wfc-tiles/) · [code](wfc-tiles/) | <a href="poisson-disc/"><img src="poisson-disc/thumb.png" width="440" alt="Poisson Disc Sampling"></a><br>**Poisson Disc Sampling**<br>Terrain & Procedural · [live](https://demos.malura.de/poisson-disc/) · [code](poisson-disc/) |
| <a href="curved-paths/"><img src="curved-paths/thumb.png" width="440" alt="Curves: Bezier and Arcs"></a><br>**Curves: Bezier and Arcs**<br>Graphics · [live](https://demos.malura.de/curved-paths/) · [code](curved-paths/) | <a href="turing-patterns/"><img src="turing-patterns/thumb.png" width="440" alt="Turing Patterns"></a><br>**Turing Patterns**<br>Terrain & Procedural · [live](https://demos.malura.de/turing-patterns/) · [code](turing-patterns/) |
| <a href="line-circle-drawing/"><img src="line-circle-drawing/thumb.png" width="440" alt="Lines and Circles on a Grid"></a><br>**Lines and Circles on a Grid**<br>Hex & Grids · [live](https://demos.malura.de/line-circle-drawing/) · [code](line-circle-drawing/) | <a href="wfc-3d/"><img src="wfc-3d/thumb.png" width="440" alt="Wave Function Collapse in 3D"></a><br>**Wave Function Collapse in 3D**<br>Terrain & Procedural · [live](https://demos.malura.de/wfc-3d/) · [code](wfc-3d/) |
| <a href="integration-methods/"><img src="integration-methods/thumb.png" width="440" alt="Integrators Compared"></a><br>**Integrators Compared**<br>Physics & Collision · [live](https://demos.malura.de/integration-methods/) · [code](integration-methods/) | <a href="orbit-mechanics/"><img src="orbit-mechanics/thumb.png" width="440" alt="Orbits"></a><br>**Orbits**<br>Space · [live](https://demos.malura.de/orbit-mechanics/) · [code](orbit-mechanics/) |
| <a href="uniform-sampling/"><img src="uniform-sampling/thumb.png" width="440" alt="Uniform Sampling in a Disc and a Triangle"></a><br>**Uniform Sampling in a Disc and a Triangle**<br>Gameplay & Balance · [live](https://demos.malura.de/uniform-sampling/) · [code](uniform-sampling/) | <a href="fizzle-fade/"><img src="fizzle-fade/thumb.png" width="440" alt="FizzleFade: Every Pixel Exactly Once"></a><br>**FizzleFade: Every Pixel Exactly Once**<br>Graphics · [live](https://demos.malura.de/fizzle-fade/) · [code](fizzle-fade/) |
| <a href="n-body-gravity/"><img src="n-body-gravity/thumb.png" width="440" alt="Gravity Between Many Bodies"></a><br>**Gravity Between Many Bodies**<br>Space · [live](https://demos.malura.de/n-body-gravity/) · [code](n-body-gravity/) | <a href="lcg-lattice/"><img src="lcg-lattice/thumb.png" width="440" alt="Bad Random: The Fingerprint of an LCG"></a><br>**Bad Random: The Fingerprint of an LCG**<br>Gameplay & Balance · [live](https://demos.malura.de/lcg-lattice/) · [code](lcg-lattice/) |
| <a href="triangle-rasterizer/"><img src="triangle-rasterizer/thumb.png" width="440" alt="Filling Triangles Without Seams"></a><br>**Filling Triangles Without Seams**<br>Graphics · [live](https://demos.malura.de/triangle-rasterizer/) · [code](triangle-rasterizer/) |  |
<!-- demos:end -->

Every demo runs on <https://demos.malura.de>, where the gallery can be filtered by category.

## Adding a demo

See [docs/adding-a-demo.md](docs/adding-a-demo.md): one folder with a `demo.json`, CMake finds it by itself. Every
demo accepts `--frames N` and exits with code 0 after N frames, so `ctest` can run it headless. The logic of a demo
lives in a header without SDL and has its own test.

## License

Apache-2.0, see `LICENSE` and `NOTICE`.
