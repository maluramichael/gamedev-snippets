# picking-3d

Which 3D object is under the mouse? A small software-rendered scene (own 4x4 matrices, `SDL_RenderGeometry`) with
seven spheres and boxes. The mouse pixel is turned into a world-space ray (NDC, inverse of projection * view), the ray
is tested against the real shapes (quadratic for spheres, slab method for boxes, a plane for the floor) and the nearest
hit wins. Hover for an orange outline, click to select, drag to orbit, wheel to zoom. The inset shows the same ray from
above. The math has no SDL in it and is covered by `test.cpp`. Write-up: https://malura.de/en/blog/picking-in-3d-space
