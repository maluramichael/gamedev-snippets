#!/bin/sh
# Builds ONE demo of the working tree (uncommitted changes included) on vault and fetches the web build to
# build/web-<name>, with the shared site files next to it. Other demo folders are left out, so unfinished demos of
# parallel sessions cannot break your build. Each name has its own persistent build directory on vault (incremental).
# Usage: scripts/preview.sh <name> [web|native|both]     (default web)
#   web:    serve with  uv run python -m http.server <port> -d build/web-<name>   and open /<name>/?embed=1
#   native: cmake + ctest on vault (smoke tests and <name>/test.cpp), output in /tmp/gds-<name>.log on vault
set -eu
NAME=${1:?usage: preview.sh <name> [web|native|both]}
MODE=${2:-web}
HOST=vault
DIR=/tmp/gds-preview-$NAME
EXCLUDES=""
for f in */demo.json; do
    d=${f%/demo.json}
    [ "$d" = "$NAME" ] || EXCLUDES="$EXCLUDES --exclude /$d/"
done
# shellcheck disable=SC2086
rsync -a --delete --exclude .git --exclude '/build/' $EXCLUDES ./ "$HOST:$DIR/"
run() { ssh "$HOST" "cd $DIR && sh scripts/build.sh $1 > /tmp/gds-$NAME-$1.log 2>&1"; }
fail() {
    ssh "$HOST" "grep -E 'error|Error|Failed|FAILED' /tmp/gds-$NAME-$1.log | head -20" >&2 || true
    echo "$1 build failed, full log: ssh $HOST cat /tmp/gds-$NAME-$1.log" >&2
    exit 1
}
if [ "$MODE" = native ] || [ "$MODE" = both ]; then
    run native || fail native
    ssh "$HOST" "grep -E 'tests passed|tests failed' /tmp/gds-$NAME-native.log"
fi
if [ "$MODE" = web ] || [ "$MODE" = both ]; then
    run web || fail web
    OUT=build/web-$NAME
    mkdir -p "$OUT"
    rsync -a --delete "$HOST:$DIR/build/web/" "$OUT/"
    mkdir -p "$OUT/_shared"
    cp site/index.js site/bar.js site/gallery.js site/categories.js site/site.css "$OUT/_shared/"
    cp -r site/fonts "$OUT/_shared/"
    cp site/index.html "$OUT/index.html"
    echo "preview ready in $OUT"
fi
