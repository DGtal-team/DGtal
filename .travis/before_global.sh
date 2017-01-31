#!/bin/bash

export BUILD_DOC="false"
export UPLOAD_DOC="false"
export NEEDCORE="true"

export NEEDEXAMPLESANDTESTS="true";
export SRC_DIR="`pwd`"

export CCOMPILER=$CC
export CXXCOMPILER=$CXX

##Preparing folders
mkdir deps/
mkdir deps/local
