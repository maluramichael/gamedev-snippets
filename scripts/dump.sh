#!/bin/sh
# "Dump" a folder into the live demo site without the repo: copy it and rebuild the demo list.
# The folder needs a demo.json. Usage: scripts/dump.sh <folder> [name]
set -eu
DIR=${1:?usage: dump.sh <folder> [name]}
NAME=${2:-$(basename "$DIR")}
HOST=malura.de
ROOT=/srv/www/demos
[ -f "$DIR/demo.json" ] || { echo "$DIR/demo.json missing" >&2; exit 1; }
case "$NAME" in ""|*[!a-z0-9-]*) echo "name must match [a-z0-9-]+: $NAME" >&2; exit 1 ;; esac
rsync -a --delete --no-o --no-g "$DIR"/ "$HOST:$ROOT/$NAME/"
ssh "$HOST" "chown -R www-data:www-data '$ROOT/$NAME' && chmod -R u=rwX,go=rX '$ROOT/$NAME' && sh '$ROOT/_shared/reindex.sh' '$ROOT'"
echo "https://demos.malura.de/$NAME/"
