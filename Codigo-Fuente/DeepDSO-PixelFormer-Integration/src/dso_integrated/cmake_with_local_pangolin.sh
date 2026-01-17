#!/bin/bash

PANGOLIN_ROOT="$HOME/DeepDSO-PixelFormer-Integration/Pangolin/build"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-O3 -std=c++14 -march=native -Wno-deprecated" \
    -DPangolin_DIR="$PANGOLIN_ROOT" \
    -DCMAKE_PREFIX_PATH="$PANGOLIN_ROOT" \
    -DCMAKE_LIBRARY_PATH="$PANGOLIN_ROOT" \
    -DCMAKE_INCLUDE_PATH="$PANGOLIN_ROOT/../components/pango_core/include;$PANGOLIN_ROOT/../components/pango_display/include;$PANGOLIN_ROOT/../components/pango_opengl/include"

