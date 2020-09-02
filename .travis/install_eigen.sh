#!/bin/bash
$SCRIPT_BEGIN

#
# Local install of Eigen on linux system
#
if [ "$TRAVIS_OS_NAME" == linux ];
then
    cd "$HOME"

    wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2
    bunzip2 eigen-3.3.7.tar.bz2
    tar xf eigen-3.3.7.tar

    cd eigen-3.3.7
    mkdir build ; cd build

    cmake .. -DCMAKE_INSTALL_PREFIX="$DEPS_DIR"
    make -j $BJOBS
    make install
fi

$SCRIPT_END
