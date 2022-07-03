#!/bin/bash
$SCRIPT_BEGIN

return_code=0

DOXYGENLOG="$BUILD_DIR/doxygen.log"

## We first check that the doxygen.log is empty
if [[ -f "$DOXYGENLOG" ]]
then

    # Filtering doxygen log (e.g. to ignore some bugs):
    # 1) "warning: unexpected token TK_EOF as the argument of ref"
    #    at src/DGtal/topology/doc/moduleDigitalTopology.dox:300-316
    #    See https://github.com/doxygen/doxygen/issues/6352
    # 2) "warning: argument 'surface' of command @param is not found in the argument list of DGtal::ShortcutsGeometry< TKSpace >::getKSpace(typename TDigitalSurfaceContainer)"
    #    at src/DGtal/helpers/Shortcuts.h 386 & 397
    #    Probably a bug (e.g. removing the template parameter solves the warning)
    # 3) warning: argument 'set' of command @param is not found in the argument list of DGtal::VoxelComplex< TKSpace, TCellContainer >::construct(typename TDigitalSet)
    #     Might be related to: Inheritance with different names of template parameter:
    #       https://github.com/doxygen/doxygen/issues/8181
    rm -f /tmp/doxygen.*.log
    awk '\
            /unexpected token TK_EOF as the argument of ref/ \
        ||  /argument .surface. of command @param is not found in the argument list/ \
        ||  /argument .set. of command @param is not found in the argument list/ \
                {print $0 > "/tmp/doxygen.ignored.log"; next} \
                {print $0 > "/tmp/doxygen.kept.log"} \
        ' "$DOXYGENLOG"

    if [[ -s "/tmp/doxygen.kept.log" ]]
    then
        return_code=1
        echo "Doxygen log file not empty !"
        echo "====================================="
        cat "/tmp/doxygen.kept.log"
        echo "====================================="
    else
        echo "Doxygen log OK"
        return_code=0
    fi

    if [[ -s "/tmp/doxygen.ignored.log" ]]
    then
        echo "Ignored doxygen log messages:"
        echo "====================================="
        cat "/tmp/doxygen.ignored.log"
        echo "====================================="
    fi
else
  return_code=1
  echo "Doxygen log file not found !"
fi


## We check src code consistency
cd "$SRC_DIR/src/DGtal"
"$SRC_DIR/.github/workflows/check_src_file_tag.sh"
if [[ $? == 0 ]]
then
    echo "@file tag OK"
else
    return_code=1;
fi
cd ../..

## We check examples consistency
#
# TODO
#

$SCRIPT_END

exit $return_code
