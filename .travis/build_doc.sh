#!/bin/bash
$SCRIPT_BEGIN

### DGtal Doc
cd build
ninja doc > buildDoc.log
cat buildDoc.log
$SCRIPT_END
