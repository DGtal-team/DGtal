#!/bin/bash
$SCRIPT_BEGIN

# OS dependent deps
source "$SRC_DIR/.travis/install_eigen.sh"

BTYPE="$BTYPE -DBUILD_EXAMPLES=true -DBUILD_TESTING=false"
BTYPE="$BTYPE -DCMAKE_BUILD_TYPE=Debug -DWITH_MAGICK=true -DWITH_GMP=true\
                     -DWITH_FFTW3=true -DWARNING_AS_ERROR=ON -DCMAKE_BUILD_TYPE=Debug \
                     -DWITH_HDF5=true -DWITH_CAIRO=true -DWITH_QT5=true -DWITH_QGLVIEWER=true -DWITH_EIGEN=true\
                     -DDGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS=true \
                     -DWARNING_AS_ERROR=OFF"

#############################
#     cmake
#############################
#Build directory
cd "$BUILD_DIR"
echo "Build folder= $BUILD_DIR"

# Cmake
echo "Using C++ = $CXXCOMPILER"
echo "CMake options = $BTYPE"
cmake "$SRC_DIR" $BTYPE

### Downloading tag file for DGtalTools
wget --no-check-certificate -O "$BUILD_DIR/DGtalTools-tagfile" http://dgtal.org/doc/tags/DGtalTools-tagfile;

### DGtal Doc
make doc > buildDoc.log

$SCRIPT_END
