#!/bin/bash
# post-receive for gamedev-snippets.git on vault. Installed as hooks/post-receive (see README of the hook install).
# Builds all demos in the pinned Emscripten image and publishes them to demos.malura.de (VPS, /srv/www/demos).
# Safety: nothing is copied unless the build produced demos; --delete only runs inside a demo's own folder,
# so folders dumped by hand stay untouched.
set -euo pipefail
unset GIT_DIR GIT_WORK_TREE GIT_INDEX_FILE GIT_QUARANTINE_PATH GIT_ALTERNATE_OBJECT_DIRECTORIES

REPO=/tank/development/git/maluramichael/gamedev-snippets.git
BRANCH=master
TARGET_HOST=malura.de
TARGET_DIR=/srv/www/demos
export HOME="${HOME:-/tank/development/git}"

while read -r old new ref; do
    if [ "$ref" != "refs/heads/$BRANCH" ]; then
        echo "--> $ref is not refs/heads/$BRANCH, no deploy."
        continue
    fi
    if [ "$new" = "0000000000000000000000000000000000000000" ]; then
        echo "--> $BRANCH was deleted, no deploy."
        continue
    fi

    WORK=$(mktemp -d /tmp/deploy-gamedev-snippets.XXXXXX)
    trap 'rm -rf "$WORK"' EXIT
    echo "--> checkout $new to $WORK"
    git --git-dir="$REPO" archive "$new" | tar -x -C "$WORK"
    cd "$WORK"

    echo "--> site tests"
    node --test site/test/index.test.mjs site/test/reindex.test.mjs
    echo "--> native build and ctest"
    sh scripts/build.sh native
    echo "--> web build"
    sh scripts/build.sh web
    sh scripts/fetch-pyodide.sh build/web/_shared/pyodide

    mkdir -p build/web/_shared
    cp site/index.js site/bar.js site/gallery.js site/site.css site/reindex.sh build/web/_shared/
    cp -r site/fonts build/web/_shared/fonts
    cp site/index.html build/web/index.html

    # Guard against a silently failed build: at least one demo and the Pyodide runtime must exist.
    DEMOS=$(find build/web -mindepth 2 -maxdepth 2 -name demo.json -not -path 'build/web/_shared/*' | wc -l)
    if [ "$DEMOS" -lt 1 ] || [ ! -s build/web/_shared/pyodide/pyodide.mjs ] || [ ! -s build/web/index.html ]; then
        echo "ERROR: build produced no demos or no runtime, deploy aborted." >&2
        exit 1
    fi

    RSYNC="rsync -a --no-o --no-g --chmod=D755,F644"
    ssh "$TARGET_HOST" "mkdir -p $TARGET_DIR/_shared"
    $RSYNC --delete build/web/_shared/ "$TARGET_HOST:$TARGET_DIR/_shared/"
    $RSYNC build/web/index.html "$TARGET_HOST:$TARGET_DIR/index.html"
    for dir in build/web/*/; do
        name=$(basename "$dir")
        [ "$name" = "_shared" ] && continue
        [ -f "$dir/demo.json" ] || continue
        $RSYNC --delete "$dir" "$TARGET_HOST:$TARGET_DIR/$name/"
    done
    ssh "$TARGET_HOST" "chown -R www-data:www-data $TARGET_DIR && sh $TARGET_DIR/_shared/reindex.sh $TARGET_DIR"
    echo "==> deployed $DEMOS demos -> https://demos.malura.de/"
done
