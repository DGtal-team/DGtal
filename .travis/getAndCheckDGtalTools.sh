## Get and test if DGtalTools compiles
DGTALPATH=`pwd`
echo "DGtal path = $DGTALPATH"
git clone --depth 1 git://github.com/DGtal-team/DGtalTools.git
cd DGtalTools
mkdir build ; cd build
echo cmake .. -DDGtal_DIR=$DGTALPATH
make -j 2
