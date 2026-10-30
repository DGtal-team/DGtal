#Download base image
ARG UBUNTU_VERSION=22.04
FROM ubuntu:${UBUNTU_VERSION} as base
# LABEL about the custom image
LABEL maintainer="david.coeurjolly@cnrs.fr"
LABEL version="0.0.1"
LABEL description="This Docker is for the Dgtal library installation."
ARG DEBIAN_FRONTEND=noninteractive



###Update and needed package
RUN apt-get update && apt-get install -y build-essential cmake libgl-dev  libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev  m4 imagemagick git curl zlib1g-dev  bash  clang  binutils wget libcairo2-dev






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

RUN mkdir /home/digital/git/DGtal/build

RUN cd /home/digital/git/DGtal/build &&  cmake ..  -DDGTAL_WITH_CAIRO=true -DDGTAL_WITH_POLYSCOPE_VIEWER=true && make install
