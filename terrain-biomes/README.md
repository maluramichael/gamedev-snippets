# terrain-biomes

Biomes from two noise maps and a lit relief. The height map alone gives stripes: temperature falls with height, so every
biome is a band along a contour line. A second noise map for moisture adds the other axis of a 4x4 Whittaker style table
(cold to hot, dry to wet), and the bands break up into patches. The usual mistake is to pick one table cell per pixel, which
draws the cell borders as hard outlines. Mixing the colors of the surrounding cell centers fades them (the blend width
slider, 0 is hard). Hillshade lights the height map from a sun, with the formula from Esri: 255 * (cos(zenith) cos(slope) +
sin(zenith) sin(slope) cos(azimuth - aspect)), slope and aspect from a 3x3 neighborhood.

Chips: 1 to 4 pick the view (biomes, height, moisture, shade only), H toggles the hillshade, N rolls a new seed, 5 and 6 switch
between height only and height plus moisture, 7 and 8 between hard and soft edges. Sliders set the moisture offset, the
temperature at the shore, the blend width and the sun's azimuth and altitude. Hover the map: the table on the right marks where
that point sits, and the panel shows height, moisture, temperature, shade and the biome. The line below the table counts how many
of the 16 table cells appear on the map. The noise is a small gradient noise written for this demo. The logic is in
`logic.hpp` and covered by `test.cpp`.

Ideas from Red Blob Games, "Making maps with noise functions" (https://www.redblobgames.com/maps/terrain-from-noise/), and Esri,
"How Hillshade works" (https://doc.esri.com/en/arcgis-pro/latest/tool-reference/spatial-analyst/how-hillshade-works.html). Own
code, own words.
