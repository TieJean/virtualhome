# Based off of python:3.6-slim, except that we are using ubuntu instead of debian.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ensure local python is preferred over distribution python
ENV PATH /usr/local/bin:$PATH

# http://bugs.python.org/issue19846
# > At the moment, setting "LANG=C" on a Linux system *fundamentally breaks Python 3*, and that's not OK.
ENV LANG C.UTF-8

# Disable apt sandbox (required for rootless podman — _apt user can't do DNS)
RUN echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/no-sandbox

# runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
		-o Acquire::Retries=10 \
		ca-certificates \
		libexpat1 \
		libffi7 \
		libgdbm6 \
		libreadline8 \
		libsqlite3-0 \
		libgl1-mesa-glx \
		libgl1-mesa-dri \
		libx11-6 \
		libxext6 \
		mesa-utils \
		&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
		python3 \
		python3-dev \
		python3-pip \
		python-is-python3 \
	&& rm -rf /var/lib/apt/lists/*

# Manually fix missing OpenGL library symlink
# RUN ln -s /usr/lib/x86_64-linux-gnu/mesa/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so.1

RUN apt-get update && apt-get install -y wget && wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb/download -O /tmp/virtualgl.deb && apt-get install -y /tmp/virtualgl.deb && rm /tmp/virtualgl.deb


# Copy and set up run script
COPY ./run_sim_gpu.sh /run_sim.sh
RUN chmod +x /run_sim.sh

RUN apt-get update && apt-get -y upgrade

# xvfb is used to do CPU based rendering of Unity
RUN apt-get install -y xvfb


# port 8080 is the port used in in Editor training.
EXPOSE 8080

ENTRYPOINT ["./run_sim.sh"]
