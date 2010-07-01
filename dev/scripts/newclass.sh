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
    echo "usage: $0 module_name subdir [namespace]" ;
    echo "       - creates three C++ skeleton files (.h .ih and .cpp) designed"
    echo "         for a class [module_name]. Modules are expected to be in a"
    echo "         directory of the form: namespace/subdir/module_name."
    echo "       - defaut namespace is ${namespace}."
    exit 1
fi


if test -w "${INCLUDE_DIR}/$2/$1.h" ;
then
    echo "File ${INCLUDE_DIR}/$2/$1.h exists and is writable. Please remove it before." ;
    exit 2;
fi
if test -w "${INCLUDE_DIR}/$2/$1.ih" ;
then
    echo "File ${INCLUDE_DIR}/$2/$1.ih exists and is writable. Please remove it before." ;
    exit 2;
fi
if test -w "${SRC_DIR}/$2/$1.cpp" ;
then
    echo "File ${SRC_DIR}/$2/$1.cpp exists and is writable. Please remove it before." ;
    exit 2;
fi

echo "--- Creating files ${INCLUDE_DIR}/$2/$1.h, ${INCLUDE_DIR}/$2/$1.ih and ${SRC_DIR}/$2/$1.cpp"

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

if test ! -r "${MODELS_DIR}/XXX.h"; then
    echo "Missing model XXX.h in ${MODELS_DIR}."
    exit 2
fi
if test ! -r "${MODELS_DIR}/XXX.ih"; then
    echo "Missing model XXX.ih in ${MODELS_DIR}."
    exit 2
fi
if test ! -r "${MODELS_DIR}/XXX.cpp"; then
    echo "Missing model XXX.cpp in ${MODELS_DIR}."
    exit 2
fi

cat "${MODELS_DIR}/XXX.h" | sed -e "${enspace}" -e "${esubdir}" -e "${ename}" -e "${etoday}" -e "${eauthor}" -e "${eemail}" -e "${einstitution}" > "${INCLUDE_DIR}/$2/$1.h"
cat "${MODELS_DIR}/XXX.ih" | sed -e "${enspace}" -e "${esubdir}" -e "${ename}" -e "${etoday}" -e "${eauthor}" -e "${eemail}" -e "${einstitution}"  > "${INCLUDE_DIR}/$2/$1.ih"
cat "${MODELS_DIR}/XXX.cpp" | sed -e "${enspace}" -e "${esubdir}" -e "${ename}" -e "${etoday}" -e "${eauthor}" -e "${eemail}" -e "${einstitution}"  > "${SRC_DIR}/$2/$1.cpp"
echo "--> done."
