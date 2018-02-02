#!/bin/bash

set -e

mkdir build
cd build

### Cmake
echo "Using C++ = $CXXCOMPILER"
cmake ..  $BTYPE -DCMAKE_CXX_COMPILER=$CXXCOMPILER -DCMAKE_C_COMPILER=$CCOMPILER 
make DGtal;
if [ $DEC = "true" ];
    then
        echo "Compile Dec in non parallel mode to save memory (to fix gcc internal compiler error(Killed)";
        make exampleDiscreteExteriorCalculusChladni;
        make exampleDiscreteExteriorCalculusSolve;
        make exampleDECSurface;
        make examplePropagation;
        make testDiscreteExteriorCalculusExtended;        
fi
    
### DGtal  build
make -j 4

echo "NeedExample $NEEDEXAMPLESANDTESTS"
### DGtal Examples and Examples
if [ $NEEDEXAMPLESANDTESTS = "true" ];
then
    cd examples ; make  -j 3
    cd ../tests ;  make -j 3

    if [ -f io/writers/testMagickWriter ]; then
      io/writers/testMagickWriter -s
    fi
    if [ -f io/readers/testMagickReader ]; then
      io/readers/testMagickReader
    fi
    ctest -j 3--output-on-failure


fi

### DGtal doc
if [ $BUILD_DOC = "true" ];
then
    echo "Building the doc"
    make doc > buildDoc.log
fi
