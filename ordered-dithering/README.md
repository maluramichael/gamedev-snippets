# ordered-dithering

Ordered dithering with a Bayer threshold map. A generated test image (gray ramp, lit sphere, hue chart) is reduced to a
tiny palette: black and white, 4 grays, 8 or 64 colors. Every pixel gets a fixed threshold from a tiled 2x2, 4x4 or 8x8
Bayer matrix (or white noise, or none for plain rounding) and picks the lower or upper palette neighbour, so the share of
bright pixels over a tile matches the value. The mouse moves a split between original and result, a loupe shows both
side by side. The color space chip shows the classic mistake: measuring that share in sRGB code values makes midtones too
bright, measuring it in linear light keeps the brightness. A block error (mean difference of block-averaged linear
luminance) puts a number on it. The logic is in `logic.hpp` and covered by `test.cpp`.
