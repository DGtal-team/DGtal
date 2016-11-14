#!/bin/bash

export BUILD_DOC="false"
export UPLOAD_DOC="false"
export NEEDCORE="true"
export NEEDEXAMPLESANDTESTS="true";
export SRC_DIR="`pwd`"

export CCOMPILER="gcc-4.8"
export CXXCOMPILER="g++-4.8"

##Preparing folders
mkdir deps/
mkdir deps/local
