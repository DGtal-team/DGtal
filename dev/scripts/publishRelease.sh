#!/bin/bash

if ! test -d "${DGtal}"; then
    echo "Environment variable DGtal is undefined."
    exit 1
fi

rsync -azv --delete --delete-after DGtal*.tar.gz dgtal@liris.cnrs.fr:/home/dgtal/public_html/releases


echo "--> done."
