#!/bin/bash
# Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
# Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France

if ! test -d "${DGtal}"; then
    echo "Environment variable DGtal is undefined."
    exit 1
fi
SCRIPTS_DIR=${DGtal}/dev/scripts
source ${SCRIPTS_DIR}/common.sh

if test \( "$#" != "3" \) -a \( "$#" != "2" \);
then 
    echo "usage: $0 test_name subdir [namespace]" ;
    echo "       - creates a C++ skeleton file (.cpp) designed to test"
    echo "         a class [test_name]. Tests are expected to be in a"
    echo "         directory of the form: tests/subdir/test[test_name]."
    echo "       - defaut namespace is ${namespace}."
    exit 1
fi


if test -w "${TESTS_DIR}/$2/test$1.cpp" ;
then
    echo "File ${TESTS_DIR}/$2/test$1.cpp exists and is writable. Please remove it before." ;
    exit 2;
fi

echo "--- Creating file ${TESTS_DIR}/$2/test$1.cpp"

if test "$#" = "3"; then namespace=$3; fi
enspace="s@YYY@${namespace}@g"
esubdir="s@ZZZ@$2@g"
ename="s@XXX@$1@g"
#etoday='s/2000\/??\/??/'`date '+20%y\/%m\/%d'`'/g'
etoday='s@2000/??/??@'"${today}"'@g'
eauthor="s@AUTHOR@${author}@g"
eemail="s/EMAIL/${email}/g"
einstitution="s@INSTITUTION@${institution}@g"


# MODELS_DIR=${DGtal}/dev/models
# PROFILES_DIR=${DGtal}/dev/profiles

# profile=`whoami`
# author="${profile} (login)"
# email="Unknown"
# institution="Unknown"
# if test -r "${PROFILES_DIR}/profiles.defs"; then
#     author=${AUTHOR}
#     email=${EMAIL}
#     institution=${INSTITUTION}
# fi

if test ! -r "${MODELS_DIR}/testXXX.cpp"; then
    echo "Missing model testXXX.cpp in ${MODELS_DIR}."
    exit 2
fi

cat "${MODELS_DIR}/testXXX.cpp" | sed -e "${enspace}" -e "${esubdir}" -e "${ename}" -e "${etoday}" -e "${eauthor}" -e "${eemail}" -e "${einstitution}"  > "${TESTS_DIR}/$2/test$1.cpp"

#TODO: Do we have to insert an entry for the test in CMakeLists.txt ?
#      Here is an easy way to do it (but it doesn't check if the entry is
#      already in the list DGTAL_TESTS_SRC)
#echo "--- Add test$1 to CMakeLists.txt"
#sed -i -e "s@SET(DGTAL_TESTS_SRC@SET(DGTAL_TESTS_SRC\n   test$1@" "${TESTS_DIR}$2/CMakeLists.txt"

echo "--> done."
