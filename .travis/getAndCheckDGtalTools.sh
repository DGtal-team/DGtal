#!/bin/bash
$SCRIPT_BEGIN

## Get and test if DGtalTools compiles
DGTALPATH="$SRC_DIR"
echo "DGtal path = $DGTALPATH"
git clone --depth 1 git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
cmake .. -DDGtal_DIR="$BUILD_DIR" $BUILD
make -j $BJOBS

$SCRIPT_END
