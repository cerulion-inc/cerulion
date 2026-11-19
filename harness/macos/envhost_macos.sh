#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# macOS only: starts the benchmark's simulator host through envhost_macos.py (Apple's OpenGL, offscreen), with the
# same clean environment the stock launcher passes on Linux. Linux does not use this file.
# The episode launcher calls it when RCPX_EXT_ENVHOST names it (run-cell, body_record and the sim server set it):
#   envhost_macos.sh RUNTIME ENV_SOCK SCORING_DIR GL        (the working folder is the code tree)
RUNTIME=$1; SOCK=$2; SCORING=$3; GL=$4
: "${LIBERO_BX:?LIBERO_BX must name the benchmark folder}"
TREE=${PLAYGROUND_SNAPSHOT:-$PWD}
cd "$TREE" || exit 2
CFG=libero-config; [ "$RUNTIME" = pro ] && CFG=liberopro-config
exec env -i PATH="$PATH" HOME="$HOME" USER="$USER" LANG="${LANG:-C}" TMPDIR="${TMPDIR:-/tmp}" \
  LIBERO_CONFIG_PATH="$LIBERO_BX/$CFG" PLAYGROUND_SNAPSHOT="$TREE" PYTHONHASHSEED=0 \
  PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  LP_NUM_THREADS="${LP_NUM_THREADS:-2}" LIBERO_BX="$LIBERO_BX" \
  "$LIBERO_BX/libero-env/bin/python" -s "$(dirname "$0")/envhost_macos.py" --runtime "$RUNTIME" --sock "$SOCK" \
  --scoring-dir "$SCORING" --gl "$GL"
