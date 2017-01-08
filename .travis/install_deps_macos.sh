#!/bin/bash

#
# Note: gmp and boost already installed
#
brew update
brew install qt5 doxygen  graphviz graphicsmagick fftw eigen homebrew/boneyard/libqglviewer

## Temporary HDF5 build issue
export BTYPE="$BTYPE -DWITH_HDF5=false" && echo "Disabling HDF5 on MacOS";
export BTYPE="$BTYPE -DWITH_QT5=true -DCMAKE_PREFIX_PATH=$(brew --prefix qt5)" && echo "Forcing Qt5 on MacOS";
