#!/bin/bash
$SCRIPT_BEGIN

# OS dependent deps
"$SRC_DIR/.travis/install_eigen.sh"

# Build options
BTYPE="$BTYPE -DBUILD_EXAMPLES=false -DBUILD_TESTING=true"
BTYPE="$BTYPE -DCMAKE_BUILD_TYPE=Debug -DWITH_MAGICK=true -DWITH_GMP=true\
              -DWITH_FFTW3=true -DWARNING_AS_ERROR=ON -DCMAKE_BUILD_TYPE=Debug \
              -DWITH_HDF5=true -DWITH_CAIRO=true -DWITH_QGLVIEWER=true \
              -DWITH_EIGEN=true -DDGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS=true \
              -DWARNING_AS_ERROR=OFF -DCMAKE_INSTALL_PREFIX='$DEPS_DIR'"

# OSX dependencies and build customization (hence the source)
if [ $TRAVIS_OS_NAME == osx ]
then
  source "$SRC_DIR/.travis/install_deps_macos.sh"
fi

#############################
#     cmake
#############################
#Build directory
cd "$BUILD_DIR"

# Cmake
echo "Using C++ = $CXXCOMPILER"
echo "CMake options = $BTYPE"
cmake "$SRC_DIR" $BTYPE -G Ninja

#############################
#     make all
#############################
echo "Building..."
ninja

$SCRIPT_END
