#!/bin/bash

set -e

mkdir build
cd build

### Cmake
echo "Using C++ = $CXXCOMPILER"
cmake ..  $BTYPE -DCMAKE_CXX_COMPILER=$CXXCOMPILER -DCMAKE_C_COMPILER=$CCOMPILER 

### DGtal doc
if [ $BUILD_DOC = "true" ];
then
    echo "Building the doc"
    make doc
fi
