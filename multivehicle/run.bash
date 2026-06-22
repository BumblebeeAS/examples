#!/usr/bin/env bash

set -e

# Run the multivehicle_examples image with NVIDIA + X11 via rocker.
# Mounts ONLY this workspace at /root/HOST/mvsim_ws (the path the tmuxp sessions
# and setup_dashboard.sh expect). This script lives in src/examples/multivehicle,
# so the workspace root is three levels up.
#
# SPDX-License-Identifier: MIT

IMAGE_NAME="${1:-multivehicle_examples:humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

rocker \
  --devices /dev/dri \
  --dev-helpers \
  --nvidia \
  --x11 \
  --git \
  --volume "${WORKSPACE_DIR}:/root/HOST/mvsim_ws" \
  --network=host \
  "${IMAGE_NAME}"
