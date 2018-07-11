#!/bin/bash
set -e

### DGtal Doc
cd "$BUILD_DIR"
make doc > buildDoc.log
