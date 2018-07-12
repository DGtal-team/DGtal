#!/bin/bash
$SCRIPT_BEGIN

### DGtal Examples
cd "$BUILD_DIR/examples"
make -j $BJOBS

$SCRIPT_END
