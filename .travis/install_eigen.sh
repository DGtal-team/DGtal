#!/bin/bash
$SCRIPT_BEGIN

#
# Local install of Eigen on linux system
#
if [ "$TRAVIS_OS_NAME" == linux ];
then
    cd "$HOME"

    wget http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
    bunzip2 3.2.10.tar.bz2
    tar xf 3.2.10.tar

    cd eigen-eigen-b9cd8366d4e8
    mkdir build ; cd build

    cmake .. -DCMAKE_INSTALL_PREFIX="$DEPS_DIR"
    make -j $BJOBS
    make install
fi

$SCRIPT_END
