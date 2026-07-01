# See https://github.com/BumblebeeAS/isaac-ros-docker/ for updated Dockerfiles

FROM ardusub_sim:humble

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG HOME_DIR=/root

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-shapely \
    libgl1-mesa-dri \
    mesa-utils \
    ros-humble-example-interfaces \
    ros-humble-foxglove-bridge \
    ros-humble-foxglove-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    software-properties-common \
    tmux && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir tmuxp

RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --ignore-installed \
    --extra-index-url https://download.pytorch.org/whl/cu128 \
    imutils \
    kornia \
    "numpy<2" \
    "onnx<1.18" \
    onnxruntime-gpu \
    onnxslim \
    "opencv-python<4.12" \
    protobuf==3.20.3 \
    rich \
    scikit-learn \
    shapely \
    supervision \
    torch==2.7.0+cu128 \
    torchvision==0.22.0+cu128 \
    transforms3d \
    typer \
    ultralytics

# Ultralytics overrides the setuptools version, which breaks some ROS packages
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 uninstall -y setuptools

WORKDIR ${HOME_DIR}
