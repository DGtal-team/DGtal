#!/bin/bash

return_code=0

HOMEPATH=$PWD
BUILDPATH=$PWD/build
DOXYGENLOG=${BUILDPATH}/doxygen.log


## We first check that the doxygen.log is empty
if [[ -f "$DOXYGENLOG" ]]
then
    if [[ -s "$DOXYGENLOG" ]]
    then
        return_code=1
        echo "Doxygen log file not empty !"
        echo "====================================="
        cat "$DOXYGENLOG"
        echo "====================================="
    else
        echo "Doxygen log OK"
        return_code=0
    fi
else
  return_code=1
  echo "Doxygen log file not found !"
fi


## We check src code consitency
cd src/
"$HOMEPATH/.travis/check_src_file_tag.sh"
if [[ $? == 0 ]]
then
    echo "@file tag OK"
else
    return_code=1;
fi
cd ..

## We check examples consistency
#
# TODO
# 

exit $return_code
