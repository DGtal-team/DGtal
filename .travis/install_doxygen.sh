#!/bin/bash
$SCRIPT_BEGIN

#
##Download and install 1.8.1
#

mkdir ~/doxygen && cd ~/doxygen
wget http://ftp.stack.nl/pub/users/dimitri/doxygen-1.8.10.linux.bin.tar.gz && tar xzf doxygen-1.8.10.linux.bin.tar.gz

$SCRIPT_END
