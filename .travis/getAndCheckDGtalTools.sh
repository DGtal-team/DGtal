## Get and test if DGtalTools compiles in default mode

cd .. && git clone git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
cmake .. -DDGtal_DIR=../../
make -j 2
