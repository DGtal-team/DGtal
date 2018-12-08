#!/bin/bash
$SCRIPT_BEGIN

### Dec
cd "$BUILD_DIR"
make DGtal -j $BJOBS
make exampleDiscreteExteriorCalculusChladni
#make exampleDiscreteExteriorCalculusSolve
#make exampleDECSurface
make examplePropagation
make testDiscreteExteriorCalculusExtended

$SCRIPT_END
