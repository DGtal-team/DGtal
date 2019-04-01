#!/bin/bash
$SCRIPT_BEGIN

### DGtal Tests
echo "Running the unit tests."
cd "$BUILD_DIR"
ctest -j $BJOBS --output-on-failure

$SCRIPT_END


#if [ -f io/writers/testMagickWriter ]; then
#    io/writers/testMagickWriter -s
#fi
#
#if [ -f io/readers/testMagickReader ]; then
#    io/readers/testMagickReader
#fi
