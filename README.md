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


Quick Install
=============

* MacOS (using [homebrew](http://brew.sh)):

       brew tap DGtal-team/DGtal
       brew install dgtal


(```brew options dgtal``` to enable optional features)

* Linux and windows: no binary package, please compile the library.


Additional instructions are available in the
[documentation](http://dgtal.org/doc/stable/moduleBuildDGtal.html).

Quick Build Instructions
========================

More details are given in the [documentation pages](http://dgtal.org/download/). We just sketch the main instructions on linux/unix-based systems:

```shell
git clone https://github.com/DGtal-team/DGtal.git
cd DGtal ; mkdir build ; cd build
cmake ..
make install
```

Minimum system requirements: C++11 enabled compiler, [cmake](http://cmake.org), [boost](http://boost.org) (>= 1.50).

DGtal can be compiled on Microsoft Windows system using Visual Studio 2014 (or newer): Generate the Visual Studio project using windows [cmake](http://cmake.org) tool and compile the DGtal solution (you may also need to set the cmake variable ```BUILD_SHARED_LIBS``` to false).


Docker
======

A Docker file is included in the repository in order to allow the users to build a docker image with the DGtal library installed. Inside the docker container, the library is installed with many optional dependencies (gmp, eigen, cgal, itk, cairo, qglviewer, imagemagick, openmp, fftw3) so the user can start developing without the need to install DGtal on the system.
                                                                                                                                                                                                                        
To build the Docker image, Docker should have been installed in the system (to [install Docker](https://docs.docker.com/engine/install/)) download the *Dockerfile* inside the [DGtal repository](https://github.com/DGtal-team/DGtal), in the same directory where the Dockerfile is located execute    

```shell
docker build -t dgtal:lastest .
```
To run an interactive terminal inside the docker container and see the examples 
```shell
docker run -it --user=digital dgtal:lastest bash
cd /home/digital/git/DGtal/build/examples
```

To enable Viewer3D/QGLViewer features, please check [this discussion](https://github.com/DGtal-team/DGtal/pull/1580).

More Information
================

* Project homepage http://dgtal.org
* Related DGtalTools project: http://dgtal.org/dgtaltools, [DGtalTools](https://github.com/DGtal-team/DGtalTools), [DGtalTools-contrib](https://github.com/DGtal-team/DGtalTools-contrib)

* Release 1.2 [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4892404.svg)](https://doi.org/10.5281/zenodo.4892404)
* Release 1.1 [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4075246.svg)](https://doi.org/10.5281/zenodo.4075246) 
* Release 1.0 [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.2611275.svg)](https://doi.org/10.5281/zenodo.2611275)
* Release 0.9.4 [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1203577.svg)](https://doi.org/10.5281/zenodo.1203577)
* Release 0.9.3 [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.290419.svg)](https://doi.org/10.5281/zenodo.290419)
* Release 0.9.2 [![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.56430.svg)](http://dx.doi.org/10.5281/zenodo.56430)
* Release 0.9.1 [![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.45125.svg)](http://dx.doi.org/10.5281/zenodo.45125)
* Release 0.9 [![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.31884.svg)](http://dx.doi.org/10.5281/zenodo.31884)
* Release 0.8 [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.11586.svg)](https://doi.org/10.5281/zenodo.11586)
* Continuous Integration (Linux/MacOS) [![Build Status](https://travis-ci.org/DGtal-team/DGtal.svg?branch=master)](https://travis-ci.org/DGtal-team/DGtal)
* Continuous Integration (Windows) [![Build status](https://ci.appveyor.com/api/projects/status/7labe8ft0jf30ng7/branch/master?svg=true)](https://ci.appveyor.com/project/kerautret/dgtal-8op01/branch/master)
