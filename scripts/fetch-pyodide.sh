#!/bin/sh
# Copies the parts of a pinned Pyodide release that the Python demos need (runtime, standard library,
# lock file and the pygame-ce wheel) into <outdir>. The download is verified against a pinned SHA-256.
# Usage: scripts/fetch-pyodide.sh build/web/_shared/pyodide
set -eu
VERSION=314.0.7
SHA256=192b5864e6e6d30ab074861af800cb8b4acb0998ef0f9342c3367448aeb86645
URL="https://github.com/pyodide/pyodide/releases/download/$VERSION/pyodide-$VERSION.tar.bz2"
OUT=${1:?usage: fetch-pyodide.sh <outdir>}
CACHE=${XDG_CACHE_HOME:-$HOME/.cache}/gamedev-snippets
ARCHIVE="$CACHE/pyodide-$VERSION.tar.bz2"

sha_ok() {
    if command -v sha256sum >/dev/null 2>&1; then
        echo "$SHA256  $1" | sha256sum -c - >/dev/null 2>&1
    else
        echo "$SHA256  $1" | shasum -a 256 -c - >/dev/null 2>&1
    fi
}

mkdir -p "$CACHE" "$OUT"
if [ ! -f "$ARCHIVE" ] || ! sha_ok "$ARCHIVE"; then
    curl -fsSL -o "$ARCHIVE.part" "$URL"
    mv "$ARCHIVE.part" "$ARCHIVE"
    sha_ok "$ARCHIVE" || { echo "checksum mismatch for $ARCHIVE" >&2; rm -f "$ARCHIVE"; exit 1; }
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
tar -xjf "$ARCHIVE" -C "$WORK" \
    pyodide/pyodide.mjs pyodide/pyodide.asm.mjs pyodide/pyodide.asm.wasm \
    pyodide/python_stdlib.zip pyodide/pyodide-lock.json \
    pyodide/pygame_ce-2.5.7-cp314-cp314-pyemscripten_2026_0_wasm32.whl
cp "$WORK"/pyodide/* "$OUT"/
echo "Pyodide $VERSION copied to $OUT"
