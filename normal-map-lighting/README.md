# normal-map-lighting

A flat picture lit as if it had depth. The sprite is a stone tile painted in flat colours (the albedo). Its relief
exists only in the normal map, a second picture whose colours are the direction the surface faces at each texel. Every
frame the CPU shades 208 x 208 pixels: unit vector from the pixel to the light, dot product with the normal, plus a
Blinn-Phong highlight. The mouse is the light, the wheel or the slider changes its height (a low light makes the relief
pop). The chips switch between the lit picture, the diffuse term alone and the specular term alone, and let you make
the typical mistakes: no normal map (flat normals, the tile looks printed), a flipped green channel (DirectX-style map in
an OpenGL-style shader, bumps become dents) and interpolated normals that are not renormalized (visible in the
specular view with the small 52 and 26 texel maps). The panel shows the normal, the light vector and the terms of the
pixel under the cursor. Space orbits the light, M cycles the map size, Up and Down change the height.
The map is baked from a height function at start and whenever the size changes, the maths and the sprite are in
`logic.hpp` and covered by `test.cpp`. Shading works directly on 8 bit colours, there is no gamma correction.
