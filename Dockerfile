#Download base image 
ARG UBUNTU_VERSION=22.04
FROM ubuntu:${UBUNTU_VERSION} as base
# LABEL about the custom image
LABEL maintainer="david.coeurjolly@cnrs.fr"
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
RUN apt -y  install  libboost-dev

### Install optional deps
RUN apt -y install libmagick++-dev

RUN apt -y install graphicsmagick*

RUN apt -y install doxygen

RUN apt -y install libcgal-dev

#RUN apt-get -y install libinsighttoolkit5-dev

#RUN apt -y install libqglviewer-dev-qt5

#RUN apt -y install libgmp-dev

#RUN  apt -y install libfftw3-dev

#### User to install 
RUN groupadd -g 1000 digital
RUN useradd -d /home/digital -s /bin/bash -m digital -u 1000 -g 1000
RUN usermod -aG sudo digital
####

#RUN apt -y install mesa-common-dev libglm-dev mesa-utils

### Directory to  store the git 
RUN mkdir /home/digital/git/
RUN mkdir /home/digital/git/DGtal


#### clone git and install 
RUN git clone https://github.com/DGtal-team/DGtal.git /home/digital/git/DGtal

#RUN mkdir /home/digital/git/DGtal/build 
RUN cd /home/digital/git/DGtal/build &&  cmake .. -DDGTAL_WITH_GMP=true -DWITH_EIGEN=true -DDGTAL_WITH_FFTW3=true - DDGTAL_WITH_CGAL=true  -DDGTAL_WITH_ITK=true -DDGTAL_WITH_OPENMP=true  -DDGTAL_WITH_CAIRO=true -DWITH_QGLVIEWER=true -DWITH_MAGICK=true && make install


