#Download base image 
ARG UBUNTU_VERSION=20.04
FROM ubuntu:${UBUNTU_VERSION} as base
# LABEL about the custom image
LABEL maintainer="msalazar@centrogeo.edu.mx"
LABEL version="0.0.1"
LABEL description="This Docker is for the Dgtal library installation."
ARG DEBIAN_FRONTEND=noninteractive



###Update 
RUN apt-get update && apt-get install -y curl

##### Basics
RUN apt install -y build-essential
#### Install Git
RUN apt install -y git 
#### Install G++ 
RUN apt install g++


#### gl libraries
RUN apt -y update --fix-missing
#RUN apt -y install libx11-dev 
RUN apt -y install mesa-common-dev libglm-dev mesa-utils


### Install cmake
RUN apt -y install cmake 
###Install boost 
RUN apt -y  install  libboost-all-dev
###Install clang-9
RUN apt -y install clang-9

RUN apt -y install libcgal*

RUN apt -y install libmagick++-dev

RUN apt -y install graphicsmagick*

RUN apt -y install doxygen

RUN apt -y install libcgal-dev

RUN apt -y install libinsighttoolkit4-dev

RUN apt -y install libqglviewer-dev-qt5

RUN apt -y install libgmp-dev

RUN  apt -y install libeigen3-dev

RUN  apt -y install libfftw3-dev

#### User to install 
RUN groupadd -g 1000 digital
RUN useradd -d /home/digital -s /bin/bash -m digital -u 1000 -g 1000
RUN usermod -aG sudo digital
####

RUN apt -y install mesa-common-dev libglm-dev mesa-utils

### Directory to  store the git 
RUN mkdir /home/digital/git/
RUN mkdir /home/digital/git/DGtal


#### clone git and install 
RUN git clone https://github.com/DGtal-team/DGtal.git /home/digital/git/DGtal

RUN mkdir /home/digital/git/DGtal/build 
RUN cd /home/digital/git/DGtal/build &&  cmake .. -DWITH_GMP=true -DWITH_EIGEN=true -DWITH_FFTW3=true -DWITH_CGAL=true -DWITH_ITK=true -DWITH_OPENMP=true -DWITH_CAIRO=true -DWITH_QGLVIEWER=true -DWITH_MAGICK=true && make install


