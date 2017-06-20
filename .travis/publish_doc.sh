#!/bin/bash

set -ev

echo "PATH ="
echo $PWD
ls -alrt

rsync -azv --delete --delete-after -e 'ssh -oStrictHostKeyChecking=no -i  .travis/dgtal_rsa' html/ dgtal@liris.cnrs.fr:/home/dgtal/public_html/doc/nightly/


##DOCSET build
cd html
make
wget http://dgtal.org/doc/docset/template.tgz
tar zxvf template.tgz ; mv template/* org.dgtal.docset
tar zcvf DGtal-devel.tgz org.dgtal.docset

cd ..

rsync  -azv --delete --delete-after -e 'ssh -oStrictHostKeyChecking=no -i  .travis/dgtal_rsa' html/DGtal-devel.tgz dgtal@liris.cnrs.fr:/home/dgtal/public_html/doc/docset


###TAGS for DGtalTools
scp -i  .travis/dgtal_rsa  DGtal-tagfile dgtal@liris.cnrs.fr:/home/dgtal/public_html/doc/tags/;scp -i  .travis/dgtal_rsa  Board-tagfile dgtal@liris.cnrs.fr:/home/dgtal/public_html/doc/tags/
