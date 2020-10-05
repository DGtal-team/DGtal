#!/bin/bash

set_options=$- # Backup of set options
set +e # The brew install has errors


## Temporary HDF5 build issue
BTYPE="$BTYPE -DWITH_HDF5=false" && echo "Disabling HDF5 on MacOS";
BTYPE="$BTYPE -DWITH_QGLVIEWER=false" && echo "Disabling QGLViewer on MacOS";

# Restoring set options
set -$set_options
