#!/bin/bash

set_options=$- # Backup of set options
set +e # The brew install has errors

#
# Note: gmp and boost already installed
#
brew update
#brew install qt5 doxygen homebrew/science/hdf5 graphviz graphicsmagick fftw eigen
brew install qt5 graphicsmagick fftw eigen ninja
# Explicit install of libqglviewer
brew install http://liris.cnrs.fr/david.coeurjolly/misc/libqglviewer.rb

## Temporary HDF5 build issue
BTYPE="$BTYPE -DWITH_HDF5=false" && echo "Disabling HDF5 on MacOS";
BTYPE="$BTYPE -DWITH_QT5=true -DCMAKE_PREFIX_PATH=$(brew --prefix qt5)" && echo "Forcing Qt5 on MacOS";

# Restoring set options
set -$set_options

