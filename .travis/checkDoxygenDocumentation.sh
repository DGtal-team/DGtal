#!/bin/bash

return_code=0
return_code2=0
return_code3=0
HOMEPATH=$PWD


## We first check that the doxygen.log is empty
if [ -s doxygen.log ]
then
    return_code=0
else
    return_code=1
    echo "Doxygen log file not empty !"
    cat doxygen.log
fi
   
## We check src code consitency
cd src/
if ! $( $HOMEPATH/.travis/check_src_file_tag.sh  )
then
    return_code2=1;
fi
cd ..

## We check examples consistency
cd examples/
if ! $( $HOMEPATH/.travis/check_examples_file_tag.sh  )
then
    return_code3=1;
fi
cd ..

return_code=$((return_code + return_code2 + return_code3)) 
echo $return_code

exit $return_code
