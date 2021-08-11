#!/bin/bash
$SCRIPT_BEGIN

## Get and test if DGtalTools compiles

cd "$SRC_DIR/.."
git clone --depth 1 git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
cmake .. $BTYPE -DDGTALTOOLS_RANDOMIZED_BUILD_THRESHOLD=25  -DDGTALTOOLS_RANDOMIZED_BUILD_WHITELIST="$(cat ../cmake/whiteListBuildTools.txt)"  -G Ninja
  #inplace
ninja

$SCRIPT_END
