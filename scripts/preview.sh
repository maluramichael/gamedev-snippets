#!/bin/sh
# Builds the web version of the working tree (uncommitted changes included) on vault and fetches it to build/web,
# with the shared site files next to it. Serve it with: uv run python -m http.server 8099 -d build/web
# Usage: scripts/preview.sh
set -eu
HOST=vault
DIR=/tmp/gds-preview
rsync -a --delete --exclude .git --exclude build ./ "$HOST:$DIR/"
if ! ssh "$HOST" "cd $DIR && sh scripts/build.sh web > /tmp/gds-preview.log 2>&1"; then
    ssh "$HOST" "grep -E 'error|Error' /tmp/gds-preview.log | head -20" >&2
    echo "web build failed, full log: ssh $HOST cat /tmp/gds-preview.log" >&2
    exit 1
fi
mkdir -p build/web
rsync -a "$HOST:$DIR/build/web/" build/web/
mkdir -p build/web/_shared
cp site/index.js site/bar.js site/gallery.js site/site.css build/web/_shared/
cp -r site/fonts build/web/_shared/
cp site/index.html build/web/index.html
ssh "$HOST" "rm -rf $DIR /tmp/gds-preview.log"
echo "preview ready in build/web"
