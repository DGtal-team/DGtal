#!/bin/bash
set -e

# Build directory
cd "$BUILD_DIR"

# Common build options
export BTYPE="$BTYPE -DBUILD_TESTING=$BUILD_TESTS -DBUILD_EXAMPLES=$BUILD_EXAMPLES -DCMAKE_CXX_COMPILER=$CXXCOMPILER -DCMAKE_C_COMPILER=$CCOMPILER"

# Cmake
echo "Using C++ = $CXXCOMPILER"
echo "CMake options = $BTYPE"
cmake "$SRC_DIR" $BTYPE
