#!/bin/bash
set -e

### Dec
echo "Compile Dec in non parallel mode to save memory (to fix gcc internal compiler error(Killed))";

cd "$BUILD_DIR"
make exampleDiscreteExteriorCalculusChladni
#make exampleDiscreteExteriorCalculusSolve
#make exampleDECSurface
make examplePropagation
make testDiscreteExteriorCalculusExtended
