#!/usr/bin/env bash

script_dir=$(cd $(dirname $0) || exit 1; pwd)

pushd $script_dir/../.. # top CMakeLists.txt folder
mkdir -p $script_dir/dist
docker build -f ./wrap/deploy/Dockerfile-dockcross-manylinux2014-wheel -t phcerdan/dgtal-linux-wheel .
docker cp $(docker create phcerdan/dgtal-linux-wheel:latest):/work/dist $script_dir/dist
popd
