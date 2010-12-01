#!/bin/bash

if ! test -d "${DGtal}"; then
    echo "Environment variable DGtal is undefined."
    exit 1
fi

rsync -azv --delete --delete-after ${DGtal}/doc/html/ dgtal@liris.cnrs.fr:/home/dgtal/public_html/doc/$1/

echo "--> done."
