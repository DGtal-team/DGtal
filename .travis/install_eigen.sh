#!/bin/bash


#
# Local install of Eigen on linux system
#

export EIGEN_ROOT="/usr/local/"

if [ $TRAVIS_OS_NAME == linux ];
then
    cd deps
    wget http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2

    bunzip2 3.2.10.tar.bz2
    tar xvf 3.2.10.tar

    cd eigen-eigen-b9cd8366d4e8
    mkdir build ; cd build

    cmake .. -DCMAKE_INSTALL_PREFIX="${SRC_DIR}/deps/local"
    make && make install && cd ${SRC_DIR} && export EIGEN_ROOT="$SRC_DIR/deps/local"

fi
