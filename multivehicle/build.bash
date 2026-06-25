#!/usr/bin/env bash

set -e

image_name=multivehicle_examples
image_tag=humble

if [ ! -f "docker/Dockerfile" ]; then
    echo "Err: docker/Dockerfile not found. Run from src/examples/multivehicle."
    exit 1
fi

if ! docker image inspect multivehicle_sim:humble >/dev/null 2>&1; then
    echo "Err: base image multivehicle_sim:humble not found."
    echo "Build it first from src/multivehicle_sim."
    exit 1
fi

image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --rm -t $image_plus_tag -f docker/Dockerfile docker && \
docker tag $image_plus_tag $image_name:$image_tag && \
echo "Built $image_plus_tag and tagged as $image_name:$image_tag"
