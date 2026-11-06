# gamma-linear

Why 50 % gray is not half the light. Two bars blend the same two colors: one interpolates the stored sRGB values, the
other interpolates the light and encodes the result, and a panel prints what each one really emits. Below, four spheres
are lit by the same light: the correct pipeline (decode the texture, light in linear space, encode once at the end) next
to three common mistakes (no curve at all, texture not decoded, output encoded twice). Drag on the bars to mix, move the
mouse over the spheres to move the light, switch the color pair and a colored or plain gray sphere with the chips (keys 1
to 5). The sRGB curve is the exact piecewise formula, not `pow(x, 2.2)`. The logic is in `logic.hpp` and covered by
`test.cpp`.
