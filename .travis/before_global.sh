#!/bin/bash
set -e

# Useful paths
export SRC_DIR="$TRAVIS_BUILD_DIR"
export BUILD_DIR="$SRC_DIR/build"

# Main steps configuration
export BUILD_DOC="false"
export UPLOAD_DOC="false"
export BUILD_DGTAL="false"
export BUILD_EXAMPLES="false"
export BUILD_TESTS="false"
export BUILD_DEC="false"
export BUILD_ALL="false" # Build DGtal, examples and tests

# CMake default options
export BTYPE=

# Compiler configuration
export CCOMPILER=$CC
export CXXCOMPILER=$CXX
if [ $CC == "gcc" ]; then  export CCOMPILER=gcc-5 ; export CXXCOMPILER=g++-5; fi


export MAGICK_CONFIG_PATH=".travis/delegate.mgk"
$MAGICK_CODER_MODULE_PATH
$MAGICK_FILTER_MODULE_PATH

# Preparing folders
mkdir -p "$SRC_DIR/deps/local"
