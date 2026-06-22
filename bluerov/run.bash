#!/usr/bin/env bash

set -e

# Run the examples image with NVIDIA and X11 support through Rocker.
# The workspace containing this repository is mounted at the path expected by
# the tmuxp session files.

IMAGE_NAME="${1:-bluerov_ws:humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

rocker \
  --devices /dev/dri \
  --dev-helpers \
  --nvidia \
  --x11 \
  --git \
  --volume "${WORKSPACE_DIR}:/root/HOST/bluerov_ws" \
  --network=host \
  "${IMAGE_NAME}"
