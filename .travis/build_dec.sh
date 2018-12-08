#!/bin/bash
$SCRIPT_BEGIN

### Dec
echo "op"
echo cd "$BUILD_DIR"
echo $BUILD_DIR
echo make DGtal -j $BJOBS
echo make exampleDiscreteExteriorCalculusChladni
#make exampleDiscreteExteriorCalculusSolve
#make exampleDECSurface
make examplePropagation
make testDiscreteExteriorCalculusExtended

$SCRIPT_END
