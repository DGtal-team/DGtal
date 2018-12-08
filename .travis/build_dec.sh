#!/bin/bash
$SCRIPT_BEGIN

### Dec
cd "$BUILD_DIR"
echo $BUILD_DIR
make examplePropagation
make testDiscreteExteriorCalculusExtended
make exampleDiscreteExteriorCalculusChladni

$SCRIPT_END
