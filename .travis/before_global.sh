#!/bin/bash

# Useful paths
export SRC_DIR="$TRAVIS_BUILD_DIR"
export BUILD_DIR="$SRC_DIR/build"

# Options send to the set command.
# e to exit immediately if a command fails
# v to print each input lines as they are read (useful for debugging)
export SET_OPTIONS=e

# Commands at script begin and end
export SCRIPT_BEGIN="set -$SET_OPTIONS"
export SCRIPT_END="set +$SET_OPTIONS ; cd \"$SRC_DIR\""

# Debug
echo $SCRIPT_BEGIN
echo $SCRIPT_END

$SCRIPT_BEGIN

# Main steps configuration
export BUILD_DOC="false"
export UPLOAD_DOC="false"
export BUILD_DGTAL="false"
export BUILD_EXAMPLES="false"
export BUILD_TESTS="false"
export BUILD_DEC="false"
export BUILD_ALL="false" # Build DGtal, examples and tests

# Build default options
export BTYPE=
export BJOBS=2 # See https://docs.travis-ci.com/user/reference/overview/#Virtualisation-Environment-vs-Operating-System

# Compiler configuration
export CCOMPILER=$CC
export CXXCOMPILER=$CXX
if [ "$CC" == "gcc" ]
then
    export CCOMPILER=gcc-5
    export CXXCOMPILER=g++-5
fi

# Build directory
mkdir -p "$BUILD_DIR"

export MAGICK_CONFIG_PATH=".travis/delegate.mgk"
$MAGICK_CODER_MODULE_PATH
$MAGICK_FILTER_MODULE_PATH

# Preparing folders
mkdir -p "$SRC_DIR/deps/local"

$SCRIPT_END
