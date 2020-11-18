[![Build Status](https://dev.azure.com/DGtal/DGtal/_apis/build/status/DGtal-team.DGtal?branchName=master)](https://dev.azure.com/DGtal/DGtal/_build/latest?definitionId=2&branchName=master)

Python
======

DGtal is wrapped to python using pybind11, and uploaded regularly to pypi for all platforms (Linux, Windows, MacOS)
and multiple `python` version (from `3.5` to latest) using azure-pipelines.

```
pip install dgtal
```

```python
import dgtal
```

Development
===========

Due to the heavy templated nature of DGtal, only a subset of types
are available in Python. All the types in Z2i and Z3i namespaces are wrapped, there are defined in [StdDefs.h](https://github.com/DGtal-team/DGtal/blob/master/src/DGtal/helpers/StdDefs.h).

## Steps to add a new wrap:
Imagine we want to add wrappings for the class `HyperRectDomain` from the `kernel` module.

- Inside the folder `wrap/kernel` add the files:
  - `HyperRectDomain_py.cpp`. In this file we choose what specifc types of `HyperRectDomain` we are wrapping.
  Here we define the function `init_HyperRectDomain`:
  ```cpp
  void init_HyperRectDomain(py::module & m) {
    using Z2i  = SpaceND<2, DGtal::PythonInteger>;
    using DomainZ2i = HyperRectDomain <Z2i>;
    auto py_class_DomainZ2i = declare_HyperRectDomain<DomainZ2i>(m, "DomainZ2i");
  }
  ```

  - `HyperRectDomain_declare_py.h`. This is where the templated function declare_HyperRectDomain is defined, and where the actual pybind11 wrappings are done.

- Add a call to `init_HyperRectDomain` to the `init_dgtal_kernel` function inside the `kernel_init.cpp`.
  If you are adding a new module, i.e. `foo`, the module `foo` has to be added to the parent init file `dgtal_init_py.cpp` in the `wrap` folder.
  All this modularity allows parallel compilation to speed up the build process.

- Add the `.cpp` files to the apropiate `CMakeLists.txt`.
  In this case we add `HyperRectDomain_py.cpp` to `wrap/kernel/CMakeLists.txt`.
  In the case we are adding a new module, use `add_subdirectory(foo)` in `wrap/CMakeLists.txt`

- Add python tests for your new wraps.
  Add the file `test_HyperRectDomain.py` to `wrap/test/kernel`, and using
  `pytest` exercise all the constructors, functions and access to data members exposed in your wrap.

  Also, add the test file to `wrap/test/kernel/CMakeLists.txt` to allow `ctest` to discover and execute the test.
  You can then `ctest -R test_Hyper* -V` to execute all the tests in that file.
  Please remember that pytest requires that your test functions start with `test_`, for example `def test_constructors():`.
  Also, `pytest` allows to parametrize your test functions, allowing to test the wrappings for multiple types automatically.
  See existing examples to guide you in the process.


