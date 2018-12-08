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

#############################
#############################

export CONFIG="Debug,Magick,GMP,ITK,FFTW3,Debug,Cairo,QGLviewer,HDF5,EIGEN"

# OS dependent deps
if [ $TRAVIS_OS_NAME == osx ]; then source "$SRC_DIR/.travis/install_deps_macos.sh"; fi
source "$SRC_DIR/.travis/install_eigen.sh"
echo $EIGEN_ROOT

export BTYPE="$BTYPE -DBUILD_EXAMPLES=true -DBUILD_TESTING=true"
export BTYPE="$BTYPE -DCMAKE_BUILD_TYPE=Debug -DWITH_MAGICK=true -DWITH_GMP=true\
                     -DWITH_FFTW3=true -DWARNING_AS_ERROR=ON -DCMAKE_BUILD_TYPE=Debug \
                     -DWITH_HDF5=true -DWITH_CAIRO=true -DWITH_QGLVIEWER=true -DWITH_EIGEN=true\
                     -DWARNING_AS_ERROR=OFF -DEIGEN3_INCLUDE_DIR='$EIGEN_ROOT/include/eigen3'"

#############################
#     cmake
#############################
#Build directory
cd "$BUILD_DIR"

# Common build options
export BTYPE="$BTYPE -DCMAKE_CXX_COMPILER=$CXXCOMPILER -DCMAKE_C_COMPILER=$CCOMPILER"

# Cmake
echo "Using C++ = $CXXCOMPILER"
echo "CMake options = $BTYPE"
cmake "$SRC_DIR" $BTYPE

#############################
#     make all
#############################
cd "$BUILD_DIR"
source "$SRC_DIR/.travis/build_dec.sh"   #Sequential DEC examples, this would also build library
make -j $BJOBS



$SCRIPT_END
