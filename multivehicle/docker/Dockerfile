FROM multivehicle_sim:humble

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG HOME_DIR=/root

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-tf-transformations && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --ignore-installed "transforms3d>=0.4" "numpy>=1.25,<2"
RUN pip3 install --no-cache-dir mavsdk

WORKDIR ${HOME_DIR}
