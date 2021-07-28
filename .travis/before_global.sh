#!/bin/bash

# Useful paths
export SRC_DIR="$TRAVIS_BUILD_DIR"
export BUILD_DIR="$SRC_DIR/build"
export DEPS_DIR="$SRC_DIR/deps"

# Options send to the set command.
# e to exit immediately if a command fails
# v to print each input lines as they are read (useful for debugging)
export SET_OPTIONS=v

# Commands at script begin and end
export SCRIPT_BEGIN="set -$SET_OPTIONS"
export SCRIPT_END="set +$SET_OPTIONS ; cd \"$SRC_DIR\""

# Debug
echo $SCRIPT_BEGIN
echo $SCRIPT_END

$SCRIPT_BEGIN

# Compiler configuration
export CCOMPILER=$CC
export CXXCOMPILER=$CXX

# Build default options
export BTYPE="-DCMAKE_CXX_COMPILER=$CXXCOMPILER -DCMAKE_C_COMPILER=$CCOMPILER"
export BJOBS=2 # See https://docs.travis-ci.com/user/reference/overview/#Virtualisation-Environment-vs-Operating-System

if [ $TRAVIS_OS_NAME == linux ]
then
 if [ "$CC" == "gcc" ]
 then
    export CCOMPILER=gcc-5
    export CXXCOMPILER=g++-5
 fi
fi

# Preparing folders
# mkdir -p fails on OSX jobs, cd command also...
echo Preparing folders $BUILD_DIR and $DEPS_DIR
mkdir "$BUILD_DIR"
mkdir "$DEPS_DIR"

export MAGICK_CONFIG_PATH=".travis/delegate.mgk"
$MAGICK_CODER_MODULE_PATH
$MAGICK_FILTER_MODULE_PATH

# General environment variables
export PATH="${DEPS_DIR}/bin:${PATH}"
export CPATH="${DEPS_DIR}/include:${CPATH}"
export CMAKE_PREFIX_PATH="${DEPS_DIR}:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="${DEPS_DIR}/lib:${LD_LIBRARY_PATH}"

$SCRIPT_END
