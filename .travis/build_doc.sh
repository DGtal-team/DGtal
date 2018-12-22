#!/bin/bash
$SCRIPT_BEGIN

### Downloading tag file for DGtalTools
wget --no-check-certificate -O "$BUILD_DIR/DGtalTools-tagfile" http://dgtal.org/doc/tags/DGtalTools-tagfile;

### DGtal Doc
make doc > buildDoc.log
$SCRIPT_END
