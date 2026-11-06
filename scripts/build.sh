#!/bin/sh
# Build inside the pinned Emscripten image (also has g++ 13, CMake, make).
# Usage: scripts/build.sh native|web   (run from the repo root, on a Linux host with Docker)
set -eu
IMAGE=emscripten/emsdk:6.0.13
MODE=${1:?usage: build.sh native|web}
case "$MODE" in
  native) CMD='cmake --preset native -DSDL_UNIX_CONSOLE_BUILD=ON && cmake --build --preset native -j4 && ctest --preset native' ;;
  web)    CMD='cmake --preset web && cmake --build --preset web -j4' ;;
  *) echo "unknown mode: $MODE" >&2; exit 2 ;;
esac
exec docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/src" -w /src "$IMAGE" sh -c "$CMD"
