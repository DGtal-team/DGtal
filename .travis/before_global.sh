#!/bin/bash

set -v

BUILD_DOC="false"
UPLOAD_DOC="false"
NEED_CORE="true"
NEEDEXAMPLESANDTESTS="true";
SRC_DIR="`pwd`"

CCOMPILER="cc"
CXXCOMPILER="c++"


##Preparing folders
mkdir deps/
mkdir deps/local
