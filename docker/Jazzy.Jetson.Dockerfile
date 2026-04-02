FROM arm64v8/ros:jazzy-perception-noble

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

# Environment variables
ENV CUDA_HOME=/usr/local/cuda \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=jazzy \
    PIP_BREAK_SYSTEM_PACKAGES=1

# --- Handle user creation ---
RUN if id -u $USER_UID ; then userdel "$(id -un $USER_UID)" ; fi

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python-is-python3 \
    git \
    openssh-client \
    wget \
    vim \
    curl \
    libeigen3-dev \
    build-essential \
    locales \
    software-properties-common \
    lsb-release \
    gnupg2 && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN add-apt-repository universe

RUN rosdep init && rosdep update

RUN rm -rf /var/lib/apt/lists/* /tmp/*




ARG TORCH_CUDA_ARCH_LIST="7.2;7.0+PTX"
ENV FORCE_CUDA="1"
RUN pip3 install 'git+https://github.com/facebookresearch/detectron2.git'
RUN pip3 install 'git+https://github.com/openai/CLIP.git'