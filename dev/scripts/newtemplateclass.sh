#!/bin/bash

if ! test -d "${DGtal}"; then
    echo "Environment variable DGtal is undefined."
    exit 1
fi
SCRIPTS_DIR=${DGtal}/dev/scripts
source ${SCRIPTS_DIR}/common.sh

if test \( "$#" != "3" \) -a \( "$#" != "2" \);
then 
    echo "usage: $0 module_name subdir [namespace]" ;
    echo "       - creates two templated C++ skeleton files (.h and .ih) designed"
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

echo "--- Creating files ${INCLUDE_DIR}/$2/$1.h and ${INCLUDE_DIR}/$2/$1.ih"

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

if test ! -r "${MODELS_DIR}/TXXX.h"; then
    echo "Missing model TXXX.h in ${MODELS_DIR}."
    exit 2
fi
if test ! -r "${MODELS_DIR}/TXXX.ih"; then
    echo "Missing model TXXX.ih in ${MODELS_DIR}."
    exit 2
fi

cat "${MODELS_DIR}/TXXX.h" | sed -e "${enspace}" -e "${esubdir}" -e "${ename}" -e "${etoday}" -e "${eauthor}" -e "${eemail}" -e "${einstitution}" > "${INCLUDE_DIR}/$2/$1.h"
cat "${MODELS_DIR}/TXXX.ih" | sed -e "${enspace}" -e "${esubdir}" -e "${ename}" -e "${etoday}" -e "${eauthor}" -e "${eemail}" -e "${einstitution}"  > "${INCLUDE_DIR}/$2/$1.ih"
echo "--> done."
