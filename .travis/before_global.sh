#!/bin/bash

export BUILD_DOC="false"
export UPLOAD_DOC="false"
export NEEDCORE="true"

export NEEDEXAMPLESANDTESTS="true";
export SRC_DIR="`pwd`"

export CCOMPILER=$CC
export CXXCOMPILER=$CXX
export MAGICK_CONFIG_PATH=".travis/delegate.mgk"
$MAGICK_CODER_MODULE_PATH
$MAGICK_FILTER_MODULE_PATH
##Preparing folders
mkdir deps/
mkdir deps/local
