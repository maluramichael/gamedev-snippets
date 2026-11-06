# update-loop

Fixed timestep update loop with an accumulator and interpolated rendering, using the SDL3 main
callbacks (`SDL_AppIterate`). Logic runs in steps of 1/60 s, rendering runs at the display rate.
A frame time clamp of 0.25 s avoids the spiral of death after a stall.

Measured web build (Emscripten 6.0.13, Release): `index.wasm` 948 KB (278 KB gzipped),
`index.js` 162 KB (41 KB gzipped).
