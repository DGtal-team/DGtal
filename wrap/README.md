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
  - `HyperRectDomain_types_py.h`, to add (and reuse later) the specific types of `HyperRectDomain` that are going to be wrapped into python.
  ```
  #include "Common_types_py.h" // For DGtal::Python::Integer
  namespace DGtal {
      namespace Python {
          using Z2i  = DGtal::SpaceND<2, DGtal::Python::Integer>;
          using DomainZ2i = DGtal::HyperRectDomain <Z2i>;
      } // namespace Python
  } // namespace DGtal
  ```

  - `HyperRectDomain_py.cpp`. This is the file to compile where we define the function `init_HyperRectDomain`.
  Using the types from HyperRectDomain_types_py we wrap that type with the template function `declare_HyperRectDomain`.
  ```cpp
  #include "dgtal_pybind11_common.h"

  #include "HyperRectDomain_types_py.h" // For DGtal::Python::DomainZ2i
  #include "HyperRectDomain_declare_py.h"

  void init_HyperRectDomain(py::module & m) {
    auto py_class_DomainZ2i = declare_HyperRectDomain<DGtal::Python::DomainZ2i>(m, "DomainZ2i");
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

## Classes wrapped

### StdDefs:
- [x] PointVector: `Point2D`, `Point3D`, `RealPoint2D`, `RealPoint3D`
- [x] HyperRectDomain: `DomainZ2i`, `DomainZ3i`
- [x] DigitalSetBySTLVector: `DigitalSetZ2i`, `DigitalSetZ3i`
- [x] KhalimskyPreSpaceND: `PreCell2D`, `SPreCell2D`, `KPreSpace2D` (and 3D)
- [x] KhalimskySpaceND: `Cell2D`, `SCell2D`, `KSpace2D` (and 3D)
- [x] MetricAdjacency: 2D: `Adj4`, `Adj8`, 3D: `Adj6`, `Adj18`, `Adj26`
- [x] DigitalTopology: `DT4_8`, `DT8_4`, `DT6_18`, `DT18_6`, `DT6_26`, `DT26_6`
- [x] Object: `Object4_8`, `Object8_4`, `Object6_18`, `Object18_6`, `Object6_26`, `Object26_6`

### IO:
- Provide bridge from numpy to ImageContainersBySTLVector.
  Check test_ImageContainerBySTLVector.py for usage, in a nutshell:
  Use external library (ITK, opencv) and use their numpy bridge.
  ```python
  import itk
  import dgtal
  itk_image = itk.imread(image_filename)
  itk_np_array = itk.GetArrayViewFromImage(itk_image)
  # itk_np_array = itk_np_array.astype('int32') # if conversion is needed.
  ImageContainer = dgtal.images.ImageContainerByVector2DUnsignedChar
  dgtal_image = ImageContainer(itk_np_array)
  ```
- [NA] Wrap internal DGtal IO classes. Discarded for now (@dcoeurjo)

### Other:
- [x] CubicalComplex `CubicalComplex2D`, `CubicalComplex3D` (using `std::unordered_map` as CellContainer).
- [ ] VoxelComplex
- [ ] DigitalSurface


Dev notes
==========
- [x] Add `data()` to DigitalSetBySTLVector.h, to expose low level details to python.
  This would allow slicing, pickling, copy, setitem, getitem and other python goodies.
  Instead of `data()`, we add `container()` to DigitalSets and ImageContainers PR #1532

Building wraps
==============
Compile DGtal with option `DGTAL_WRAP_PYTHON=ON`.
This will create a folder in your build tree named `dgtal`.
From the build folder this will work: `python -c 'import dgtal'`, to use the package from the build tree elswhere, create a virtualenviroment:
```
mkvirtualenv dgtal-build
```
And then create a file named: `dgtal.pth` in `~/.virtualenvs/dgtal-build/lib/python3.X/site-packages` with the content:
```
/path/to/dgtal_build_folder
/path/to/dgtal_build_folder/dgtal
```

Now you can use `workon dgtal-build` from anywhere, and `python -c 'import dgtal'` will work.
Remember that if you modify and rebuild DGtal wrappings, and you are in a `ipython` session, or `jupyter`, you will need to restart or reload
the ipython kernel for the changes to be loaded.


