#!/bin/bash
$SCRIPT_BEGIN

### DGtal Tests
cd "$BUILD_DIR/tests"

if [ -f io/writers/testMagickWriter ]; then
    io/writers/testMagickWriter -s
fi

if [ -f io/readers/testMagickReader ]; then
    io/readers/testMagickReader
fi

ctest -j $BJOBS --output-on-failure

$SCRIPT_END
