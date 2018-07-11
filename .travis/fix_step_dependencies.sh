#!/bin/bash
set -e

if [[ "$BUILD_DEC" == "true" ]]
then
  export BUILD_TESTS="true"
  export BUILD_EXAMPLES="true"
fi

if [[ "$BUILD_ALL" == "true" ]]
then
  export BUILD_TESTS="true"
  export BUILD_EXAMPLES="true"
  export BUILD_DGTAL="true"
fi

if [[ "$BUILD_TESTS" == "true" ]] || [[ "$BUILD_EXAMPLES" == "true" ]]
then
  export BUILD_DGTAL="true"
fi

if [[ "$UPLOAD_DOC" == "true" ]]
then
  export BUILD_DOC="true"
fi
