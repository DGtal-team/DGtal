#!/bin/bash
$SCRIPT_BEGIN

### Dec
echo "Compile Dec in non parallel mode to save memory (to fix gcc internal compiler error(Killed))";

cd "$BUILD_DIR"
make exampleDiscreteExteriorCalculusChladni
#make exampleDiscreteExteriorCalculusSolve
#make exampleDECSurface
make examplePropagation
make testDiscreteExteriorCalculusExtended

$SCRIPT_END
