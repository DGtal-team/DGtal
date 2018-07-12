#!/bin/bash
set -e

### DGtal Tests
cd "$BUILD_DIR/tests"
make -j 2
