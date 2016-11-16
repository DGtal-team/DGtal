#!/bin/bash

set -ev

## Temporary HDF5 build issue
export BTYPE="$BTYPE -DWITH_HDF5=false" && echo "Disabling HDF5 on MacOS"; 

#
# Note: gmp and boost already installed
#

brew update
brew install doxygen homebrew/science/hdf5 graphviz graphicsmagick fftw eigen  homebrew/boneyard/libqglviewer
