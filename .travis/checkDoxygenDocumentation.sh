#!/bin/bash

return_code=0
return_code2=0
return_code3=0
HOMEPATH=$PWD


## We first check that the doxygen.log is empty
if [[ -s doxygen.log ]]
then
    return_code=1
    echo "Doxygen log file not empty !"
    echo "====================================="
    cat doxygen.log
    echo "====================================="
else
    return_code=0
fi

## We check src code consitency
cd src/
$HOMEPATH/.travis/check_src_file_tag.sh
if [[ $? -ne 0 ]]
then
    return_code2=1;
fi
cd ..

## We check examples consistency
#
# TODO
# 

return_code=$((return_code + return_code2 + return_code3))
exit $return_code
