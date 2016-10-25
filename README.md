![DGtal logo](doc/images/logoDGtal-small.png)

Main website: http://dgtal.org

Description
===========

DGtal is a generic open source library for Digital Geometry
programming for which the main objective is to structure different
developments from the digital geometry and topology community. The
aims are numerous: make easier the appropriation of our tools for a
neophyte (new PhD students, researchers from other topics,...), permit
better comparisons from new methods with already existing approaches
and to construct a federative project. Another objective of DGtal is
to simplify the construction of demonstration tools to share new
results and potential efficiency of the proposed work.

DGtal is an opensource free software written by researchers and students from the
DGtal research community. We wish to keep an estimate of the number of persons
using the library. This is really important for us to justify the ressources spent
in developing this software. Please, take a few seconds to fill in the registration
form: http://dgtal.org/registration/

Quick Install
=============

* Linux: [Binary packages](https://software.opensuse.org/download.html?project=home%3Acopyme%3ADGtal&package=DGtal) (Arch, CentOS, Fedora, openSUSE, RHEL, SL)
* MacOS (using [homebrew](http://brew.sh)):
 
       ```brew install dgtal```


(```brew options dgtal``` to enable optional features)

* Windows: no binary package, please compile the library.


Additional instructions are available in the
[documentation](http://dgtal.org/doc/nightly/moduleBuildDGtal.html).

Quick Build Instructions
========================

More details are given in the [documentation pages](http://dgtal.org/download/). We just sketch the main instructions on linux/unix-based systems:

```shell
git clone https://github.com/DGtal-team/DGtal.git
cd DGtal ; mkdir buid ; cd build
cmake ..
make install
```

Minimum system requirements: C++11 enabled compiler, [cmake](http://cmake.org), [boost](http://boost.org) (>= 1.46).

DGtal can be compiled on Microsoft Windows system using Visual Studio 2014 (or newer): Generate the Visual Studio project using windows [cmake](http://cmake.org) tool and compile the DGtal solution (you may also need to set the cmake variable ```BUILD_SHARED_LIBS``` to false).

More Information
================

* Project homepage http://dgtal.org
* Related DGtalTools project: http://dgtal.org/tools, https://github.com/DGtal-team/DGtalTools, https://github.com/DGtal-team/DGtalTools-contrib

* Release 0.9.2 [![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.56430.svg)](http://dx.doi.org/10.5281/zenodo.56430)
* Release 0.9.1 [![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.45125.svg)](http://dx.doi.org/10.5281/zenodo.45125)
* Release 0.9 [![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.31884.svg)](http://dx.doi.org/10.5281/zenodo.31884)
* Release 0.8 [![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.11586.png)](http://dx.doi.org/10.5281/zenodo.11586)
* Continuous Integration [![Build Status](https://travis-ci.org/DGtal-team/DGtal.svg?branch=master)](https://travis-ci.org/DGtal-team/DGtal)
