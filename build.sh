#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS="$ROOT/deps"
mkdir -p "$DEPS"
if pkg-config --exists raylib 2>/dev/null; then
    RAYLIB_CFLAGS=$(pkg-config --cflags raylib)
    RAYLIB_LIBS=$(pkg-config --libs raylib)
else
    if [ ! -f "$DEPS/raylib/src/libraylib.a" ]; then
        git clone --depth 1 --branch 5.5 https://github.com/raysan5/raylib.git "$DEPS/raylib"
        make -C "$DEPS/raylib/src" PLATFORM=PLATFORM_DESKTOP -j"$(nproc)"
    fi
    RAYLIB_CFLAGS="-I$DEPS/raylib/src"
    RAYLIB_LIBS="$DEPS/raylib/src/libraylib.a -lm -lpthread -ldl -lrt -lX11"
fi
mkdir -p "$ROOT/build"
g++ -std=c++17 -O2 -I "$ROOT/external" $RAYLIB_CFLAGS "$ROOT/src/main.cpp" -o "$ROOT/build/reach_me" $RAYLIB_LIBS
echo "Build selesai. Jalankan dengan: ./build/reach_me"
