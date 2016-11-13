#!/bin/bash

set -e

mkdir build
cd build

### Cmake
cmake ..  $BTYPE -DCMAKE_CXX_COMPILER=$CXXCOMPILER -DCMAKE_C_COMPILER=$CCOMPILER 
 
### DGtal Core build
if [ $NEEDCORE == "true" ]
then
    make -j 3 DGtal && make -j 3 DGtalIO
fi


### DGtal Examples and Examples
if [ $NEEDEXAMPLESANDTESTS == "true" ]
then
    cd examples && make  -j 3 && cd ../tests &&  make -j 3 && make test  ARGS=--output-on-failure
fi

