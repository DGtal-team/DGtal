#!/bin/bash


## Temporary HDF5 build issue
export BTYPE="$BTYPE -DWITH_HDF5=false" && echo "Disabling HDF5 on MacOS";
export BTYPE="$BTYPE -DWITH_QT5=true"  && echo "Forcing Qt5 on MacOS";

#
# Note: gmp and boost already installed
#

brew update
brew install qt5 doxygen homebrew/science/hdf5 graphviz graphicsmagick fftw eigen homebrew/boneyard/libqglviewer
