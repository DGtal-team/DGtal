#!/bin/bash
$SCRIPT_BEGIN

## Get and test if DGtalTools compiles
DGTALPATH="$SRC_DIR"
echo "DGtal path = $DGTALPATH"
git clone --depth 1 git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
cmake .. -DDGtal_DIR="$SRC_DIR" $BUILD   #inplace
make -j $BJOBS

$SCRIPT_END
