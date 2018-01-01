#!/bin/bash

set -e

mkdir build
cd build

### Cmake
echo "Using C++ = $CXXCOMPILER"
cmake ..  $BTYPE -DCMAKE_CXX_COMPILER=$CXXCOMPILER -DCMAKE_C_COMPILER=$CCOMPILER


### DGtal Core build
if [ $NEEDCORE = "true" ];
then
    make -j 3 DGtal
fi

echo "NeedExample $NEEDEXAMPLESANDTESTS"
### DGtal Examples and Examples
if [ $NEEDEXAMPLESANDTESTS = "true" ];
then
    cd examples ; make  -j 3
    cd ../tests ;  make -j 3
    ctest -j 3--output-on-failure

    if [ -f io/writers/testMagickWriter ]; then
      io/writers/testMagickWriter -s
    fi
    if [ -f io/readers/testMagickReader ]; then
      io/readers/testMagickReader
    fi
fi

### DGtal doc
if [ $BUILD_DOC = "true" ];
then
    echo "Building the doc"
    make doc > buildDoc.log
fi
