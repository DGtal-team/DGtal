#!/bin/bash

set -e

mkdir build
cd build

### Cmake
echo "Using C++ = $CXXCOMPILER"
cmake ..  $BTYPE -DCMAKE_CXX_COMPILER=$CXXCOMPILER -DCMAKE_C_COMPILER=$CCOMPILER 


### DGtal  build
make -j 4

echo "NeedExample $NEEDEXAMPLESANDTESTS"
### DGtal Examples and Examples
if [ $NEEDEXAMPLESANDTESTS = "true" ];
then
    ctest -j 3 --output-on-failure
fi

### DGtal doc
if [ $BUILD_DOC = "true" ];
then
    echo "Building the doc"
    make doc > buildDoc.log
fi
