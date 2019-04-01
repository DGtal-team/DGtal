#!/bin/bash
$SCRIPT_BEGIN

## Get and test if DGtalTools compiles

cd "$SRC_DIR/.."
git clone --depth 1 git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
cmake .. $BTYPE -G Ninja
  #inplace
ninja

$SCRIPT_END
