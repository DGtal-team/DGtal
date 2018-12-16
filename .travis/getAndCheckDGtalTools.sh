#!/bin/bash
$SCRIPT_BEGIN

export SRC_DIR="$TRAVIS_BUILD_DIR"

## Get and test if DGtalTools compiles
DGTALPATH="$SRC_DIR"
echo "DGtal path = $DGTALPATH"
echo "Build = $BUILD"
echo "TRAVIS_BUILD_DIR= $TRAVIS_BUILD_DIR"
git clone --depth 1 git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
cmake .. -DDGtal_DIR="$SRC_DIR" $BUILD   #inplace
make -j $BJOBS

$SCRIPT_END
