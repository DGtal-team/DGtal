#!/bin/bash

return_code=0

# Checking that examples has proper @file tag
for file in `find * -regextype posix-extended -regex '.*\.[ch](pp|xx|)'`
do
  expected_name=$file
  if ! $(grep -aqP "^\s*(\**|//[/!]|/\*[\*!])\s*?[@\\\\]example(\s+${expected_name})?\s*$" $file)
  then
    echo -E "Error in file $file:"
    echo -E "  expecting \" * @example ${expected_name}\""
    echo -E "  but found \"$(grep -m 1 -aP '[@\\](example|file)' $file)\""
    echo

    return_code=1
  fi
done

exit $return_code
