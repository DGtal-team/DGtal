#!/bin/bash
$SCRIPT_BEGIN

### DGtal Tests
cd "$BUILD_DIR/tests"
make -j $BJOBS

$SCRIPT_END
