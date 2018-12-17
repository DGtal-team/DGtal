#!/bin/bash
$SCRIPT_BEGIN

### DGtal Doc
ninja doc > buildDoc.log
cat buildDoc.log
$SCRIPT_END
