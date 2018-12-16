#!/bin/bash
$SCRIPT_BEGIN


## Get and test if DGtalTools compiles
DGTALPATH="$SRC_DIR"
echo "DGtal path = $DGTALPATH"
echo "Build = $BUILD"
echo "TRAVIS_BUILD_DIR= $TRAVIS_BUILD_DIR"
git clone --depth 1 git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
cmake .. -DDGtal_DIR=$TRAVIS_BUILD_DIR/build  #inplace
echo "Running cmake .. -DDGtal_DIR=$TRAVIS_BUILD_DIR/build"
echo cmake .. -DDGtal_DIR=$TRAVIS_BUILD_DIR/build  #inplace
  #inplace
make -j $BJOBS
$SCRIPT_END
