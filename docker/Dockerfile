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


# ---------------------------------------------------------------------------
# Mesa 25.1.9 llvmpipe, replacing jammy's 23.2.1 (installed above as
# libgl1-mesa-dri). REASON: 23.2.1's llvmpipe anisotropic filter scans an
# ellipse bounding box computed as box_u = 2/d * sqrt(d*C*F), d = 4AC-B^2, with
# NO clamp (lp_bld_sample_soa.c, lp_build_sample_aniso). When the ellipse
# degenerates d -> 0 and the sample count explodes: a live loop bound of
# 3926-7918 was measured against a max_anisotropy of 9. That is the ~400 s
# single-threaded stall that trips Unity's 500 s HTTP timeout and fails bags.
# Mesa 25.1 rewrote that function to iterate a bounded sample rate instead.
# NOTE 24.x is NOT fixed -- lp_build_sample_aniso there is identical to 23.2.1.
#
# The tree is pre-built (llvmpipe only, LLVM 19 statically linked, so there is
# no libLLVM to ship) and must be present in the build context as ./mesa251.
# See README.mesa251.md for how to produce it. Apptainer users can instead build
# Apptainer.mesa251.def, which overlays the same tree onto an existing image.
COPY ./mesa251 /opt/mesa251

# LD_LIBRARY_PATH is the load-bearing variable: Mesa 25.1 has no per-driver
# megadriver, it ships a versioned libgallium that the GLVND vendor libs pull in
# via DT_NEEDED, so LIBGL_DRIVERS_PATH alone does not switch the driver.
ENV LD_LIBRARY_PATH=/opt/mesa251/lib:/opt/mesa251/lib/dri
ENV LIBGL_DRIVERS_PATH=/opt/mesa251/lib/dri
ENV __GLX_VENDOR_LIBRARY_NAME=mesa
# NOTE: do NOT set __EGL_VENDOR_LIBRARY_FILENAMES. It makes Unity choose the EGL
# path, which needs a DRM render node (/dev/dri/cardN) and fails on CPU-only
# nodes -- the sim then never starts. GLX is the path Unity uses here.

# Keep renderer caches off $HOME: home has a hard quota on SLURM and when it
# fills, Unity Perception throws "Disk quota exceeded" rather than anything that
# points at disk.
ENV MESA_SHADER_CACHE_DIR=/tmp/mesa_shader_cache

# port 8080 is the port used in in Editor training.
EXPOSE 8080

ENTRYPOINT ["./run_sim.sh"]
