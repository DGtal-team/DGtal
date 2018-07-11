#!/bin/bash
set -e

### DGtal Tests
cd "$BUILD_DIR/tests"

if [ -f io/writers/testMagickWriter ]; then
    io/writers/testMagickWriter -s
fi

if [ -f io/readers/testMagickReader ]; then
    io/readers/testMagickReader
fi

ctest -j 3 --output-on-failure
