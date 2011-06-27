#!/bin/bash
# David Coeurjolly

if ! test -d "${DGtal}"; then
    echo "Environment variable DGtal is undefined."
    exit 1
fi
SCRIPTS_DIR=${DGtal}/dev/scripts
source ${SCRIPTS_DIR}/common.sh

if test  \( "$#" != "1" \);
then 
    echo "usage: $0 documenatation_name subdir" ;
    echo "       - creates a doxygen documentation skeleton file (.dox) in the doc folder."
    exit 1
fi

if test -w "${DGtal}/doc/$1.dox" ;
then
    echo "File ${DGtal}/doc/$1.dox exists and is writable. Please remove it before." ;
    exit 2;
fi

echo "--- Creating files ${DGtal}/doc/$1.dox"

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

if test ! -r "${MODELS_DIR}/XXX.dox"; then
    echo "Missing model XXX.dox in ${MODELS_DIR}."
    exit 2
fi

cat "${MODELS_DIR}/XXX.dox" | sed -e "${enspace}" -e "${esubdir}" -e "${ename}" -e "${etoday}" -e "${eauthor}" -e "${eemail}" -e "${einstitution}"  > "${DGtal}/doc/$1.dox"

echo "--> done."
