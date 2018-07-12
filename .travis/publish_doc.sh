#!/bin/bash

$SCRIPT_BEGIN

cd "$BUILD_DIR"
echo $HOME
rsync -azv --delete --delete-after -e 'ssh -oStrictHostKeyChecking=no -i  /home/travis/build/DGtal-team/DGtal/.travis/dgtal_rsa' html/ dgtal@connect.liris.cnrs.fr:/home-projets/dgtal/public_html/doc/nightly/


##DOCSET build
cd html
make
wget --no-check-certificate http://dgtal.org/doc/docset/template.tgz
tar zxvf template.tgz ; mv template/* org.dgtal.docset
tar zcvf DGtal-devel.tgz org.dgtal.docset

cd ..

rsync  -azv --delete --delete-after -e 'ssh -oStrictHostKeyChecking=no -i  .travis/dgtal_rsa' html/DGtal-devel.tgz dgtal@connect.liris.cnrs.fr:/home-projets/dgtal/public_html/doc/docset


###TAGS for DGtalTools
scp -i  /home/travis/build/DGtal-team/DGtal/.travis/dgtal_rsa  DGtal-tagfile dgtal@connect.liris.cnrs.fr:/home-projets/dgtal/public_html/doc/tags/
scp -i  /home/travis/build/DGtal-team/DGtal/.travis/dgtal_rsa  Board-tagfile dgtal@connect.liris.cnrs.fr:/home-projets/dgtal/public_html/doc/tags/

$SCRIPT_END
