#!/usr/bin/env bash

set -e

# Builds the multivehicle_examples Docker image FROM multivehicle_sim:humble.
# Run from this directory (src/examples/multivehicle): ./build.bash
#
# SPDX-License-Identifier: MIT
image_name=multivehicle_examples
image_tag=humble

if [ ! -f "docker/Dockerfile" ]; then
    echo "Err: docker/Dockerfile not found. Run from src/examples/multivehicle."
    exit 1
fi

if ! docker image inspect multivehicle_sim:humble >/dev/null 2>&1; then
    echo "Err: base image multivehicle_sim:humble not found."
    echo "Build it first from the multivehicle_sim repo:"
    echo "  cd ../../multivehicle_sim && ./build.bash"
    exit 1
fi

image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --rm -t $image_plus_tag -f docker/Dockerfile docker && \
docker tag $image_plus_tag $image_name:$image_tag && \
echo "Built $image_plus_tag and tagged as $image_name:$image_tag"
