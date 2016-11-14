#!/bin/bash

set -e

BTYPE="$BTYPE -DWITH_HDF5=false" && echo "Disabling HDF5 on MacOS"; 

brew update && brew install boost doxygen homebrew/science/hdf5 graphviz graphicsmagick  gmp libqglviewer fftw

brew install homebrew/boneyard/libqglviewer
