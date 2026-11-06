#!/bin/sh
# Writes <root>/demos.json: the sorted names of all folders that contain a demo.json.
# Folders starting with "_" or "." and names outside [a-z0-9-] are ignored.
# Usage: reindex.sh [root]   (default /srv/www/demos). Plain POSIX sh, runs on the VPS without Node.
set -eu
ROOT=${1:-/srv/www/demos}
cd "$ROOT"
TMP=$(mktemp "$ROOT/.demos.json.XXXXXX")
trap 'rm -f "$TMP"' EXIT
printf '[' > "$TMP"
FIRST=1
for D in */; do
    D=${D%/}
    case "$D" in _*|.*|*[!a-z0-9-]*) continue ;; esac
    [ -f "$D/demo.json" ] || continue
    [ "$FIRST" = 1 ] || printf ',' >> "$TMP"
    FIRST=0
    printf '"%s"' "$D" >> "$TMP"
done
printf ']\n' >> "$TMP"
chmod 644 "$TMP"
mv "$TMP" demos.json
trap - EXIT
