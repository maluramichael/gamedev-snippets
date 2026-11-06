#!/bin/sh
# Collects every finished preview build (build/web-<name>/<name>) into ONE gallery at build/all, with thumbs
# and demos.json, and serves it. Run it again after new previews appeared: it refreshes the copy and keeps serving.
# Usage: scripts/preview-all.sh [port]     (default 8180)  ->  http://localhost:8180/
set -eu
PORT=${1:-8180}
OUT=build/all
mkdir -p "$OUT/_shared"
cp site/index.js site/bar.js site/gallery.js site/categories.js site/site.css "$OUT/_shared/"
cp -r site/fonts "$OUT/_shared/"
cp site/index.html "$OUT/index.html"
for dir in */; do
    name=${dir%/}
    [ -f "$name/demo.json" ] || continue
    src=build/web-$name/$name
    [ -d "$src" ] || continue
    mkdir -p "$OUT/$name"
    rsync -a --delete "$src/" "$OUT/$name/"
    cp "$name/demo.json" "$OUT/$name/demo.json"
    [ -f "$name/thumb.png" ] && cp "$name/thumb.png" "$OUT/$name/thumb.png"
done
sh site/reindex.sh "$PWD/$OUT"
echo "gallery in $OUT: $(tr -cd ',' < "$OUT/demos.json" | wc -c | tr -d ' ') + 1 demos"
if ! lsof -nP -iTCP:"$PORT" -sTCP:LISTEN > /dev/null 2>&1; then
    nohup uv run python -m http.server "$PORT" -d "$OUT" > /tmp/gds-preview-all.log 2>&1 &
    echo "serving http://localhost:$PORT/ (pid $!)"
else
    echo "already serving http://localhost:$PORT/"
fi
