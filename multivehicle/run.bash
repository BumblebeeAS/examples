#!/usr/bin/env bash

set -e

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
