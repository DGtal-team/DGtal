#!/bin/bash
$SCRIPT_BEGIN

### DGtal Doc
cd "$BUILD_DIR"
make doc > buildDoc.log && cd ""$SRC_DIR"
$SCRIPT_END
