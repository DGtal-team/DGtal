## Get and test if DGtalTools compiles in default mode

git clone --depth 1 git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
cmake .. -DDGtal_DIR=../../
make -j 2
