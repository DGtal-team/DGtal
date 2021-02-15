#!/usr/bin/env bash

# Versions can be restricted by passing them in as arguments to the script
# For example,
# manylinux-build-wheels.sh cp38
if [[ $# -eq 0 ]]; then
  PYBIN=(/opt/python/*/bin)
  PYBINARIES=()
  for version in "${PYBIN[@]}"; do
    if [[  ${version} == *"cp36"* || ${version} == *"cp37"* || ${version} == *"cp38"* || ${version} == *"cp39"* ]]; then
      PYBINARIES+=(${version})
    fi
  done
else
  PYBINARIES=()
  for version in "$@"; do
    PYBINARIES+=(/opt/python/*${version}*/bin)
  done
fi

# i686 or x86_64 ?
case $(uname -p) in
    i686)
        ARCH=x86
        ;;
    x86_64)
        ARCH=x64
        ;;
    *)
        die "Unknown architecture $(uname -p)"
        ;;
esac

echo "Building wheels for $ARCH"
# -----------------------------------------------------------------------

echo "BOOST_BUILD_DIR: ${BOOST_BUILD_DIR}"

script_dir=$(cd $(dirname $0) || exit 1; pwd)
deploy_dir=${script_dir}
pushd ${deploy_dir}
# Compile wheels re-using standalone project and archive cache
for PYBIN in "${PYBINARIES[@]}"; do
    PYTHON_EXECUTABLE=${PYBIN}/python
    PYTHON_INCLUDE_DIR=$( find -L ${PYBIN}/../include/ -name Python.h -exec dirname {} \; )
    PYTHON_INCLUDE_DIRS=${PYTHON_INCLUDE_DIR}

    echo ""
    echo "PYTHON_EXECUTABLE:${PYTHON_EXECUTABLE}"
    echo "PYTHON_INCLUDE_DIR:${PYTHON_INCLUDE_DIR}"

    # Remove when scikit-build includes cmake_target PR:
    # https://github.com/scikit-build/scikit-build/pull/477
    ${PYBIN}/python -m pip uninstall scikit-build -y
    ${PYBIN}/python -m pip install -r requirements-deploy.txt

    # TODO: Switch BOOST_ROOT to these two when CMake is at least 3.15 and remove Boost_ROOT
    # -DCMAKE_FIND_PACKAGE_PREFER_CONFIG:BOOL=ON \
    # -DBoost_DIR:STRING=${BOOST_CMAKE_CONFIG_FOLDER} \
    ${PYBIN}/python setup.py bdist_wheel --build-type Release -G Ninja -- \
      -DCMAKE_CXX_COMPILER_TARGET:STRING=$(uname -p)-linux-gnu \
      -DBOOST_ROOT=${BOOST_BUILD_DIR} \
      -DPYTHON_EXECUTABLE:FILEPATH=${PYTHON_EXECUTABLE} \
      -DPYTHON_INCLUDE_DIR=${PYTHON_INCLUDE_DIR} \
    || exit 1
    # ${PYBIN}/python setup.py clean
done

# auditwheel will bundle shared libraries in the wheel,
# but they have to be found first using LD_LIBRARY_PATH
export DEPENDENCIES_LD_LIBRARY_PATH="/work/boost-build/lib"
# This step will fixup the wheel switching from 'linux' to 'manylinux2014' tag and include third party libraries
for whl in dist/*linux_$(uname -p).whl; do
    LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${DEPENDENCIES_LD_LIBRARY_PATH} auditwheel repair ${whl} -w /work/dist/
    rm ${whl}
done
popd
