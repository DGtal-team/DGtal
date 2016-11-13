#!/bin/bash

BUILD_DOC="false"
UPLOAD_DOC="false"
NEED_CORE="true"
NEEDEXAMPLESANDTESTS="true";
SRC_DIR="`pwd`"

CCOMPILER="gcc-4.8"
CXXCOMPILER="g++-4.8"

echo "CXXCOMPILER = $CXXCOMPILER"

##Preparing folders
mkdir deps/
mkdir deps/local
