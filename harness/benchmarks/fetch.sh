#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Fetch the benchmark at the revisions the measured runs used, into one folder:
#   LIBERO      github.com/Lifelong-Robot-Learning/LIBERO  commit 8f1084e3132a39270c3a13ebe37270a43ece2a01
#   LIBERO-PRO  github.com/Zxy-MLlab/LIBERO-PRO            commit eafdb809426b13153aa1e4c42d6601844217dfec
#   160 task and start-state files of the Hugging Face dataset zhouxueyang/LIBERO-Pro at revision
#   c86fc3b8293185a6f373677018ff3e37f8391602, checked against LIBERO-PRO-data.sha256
# usage: fetch.sh TARGET_DIR [SIM_PYTHON]
#   SIM_PYTHON is the simulator environment's Python 3.10; when given, LIBERO is installed into it.
#   FETCH_LINKED=1: TARGET_DIR/LIBERO and TARGET_DIR/LIBERO-PRO are already there (install.sh --bench-from);
#   nothing is cloned, the data files are still fetched where missing and checked.
#   UV names the uv program (default: uv on PATH).
set -euo pipefail
BX=${1:?usage: fetch.sh TARGET_DIR [SIM_PYTHON]}
SIM_PY=${2:-}
UV=${UV:-uv}
HERE=$(cd "$(dirname "$0")" && pwd)
LIBERO_SHA=8f1084e3132a39270c3a13ebe37270a43ece2a01
PRO_SHA=eafdb809426b13153aa1e4c42d6601844217dfec
DATA_REV=c86fc3b8293185a6f373677018ff3e37f8391602
DATA_URL=https://huggingface.co/datasets/zhouxueyang/LIBERO-Pro/resolve/$DATA_REV
mkdir -p "$BX"

clone_at() {  # URL DIR SHA
  if [ ! -d "$2/.git" ]; then
    git init -q "$2"
    git -C "$2" remote add origin "$1"
  fi
  echo "      $1 at $3"
  git -C "$2" fetch --depth 1 origin "$3"
  git -C "$2" checkout -q FETCH_HEAD
  test "$(git -C "$2" rev-parse HEAD)" = "$3"
}
if [ -z "${FETCH_LINKED:-}" ]; then
  clone_at https://github.com/Lifelong-Robot-Learning/LIBERO.git "$BX/LIBERO" "$LIBERO_SHA"
  clone_at https://github.com/Zxy-MLlab/LIBERO-PRO.git "$BX/LIBERO-PRO" "$PRO_SHA"
fi

DEST=$BX/LIBERO-PRO/libero/libero
TOTAL=$(grep -c . "$HERE/LIBERO-PRO-data.files.txt")
echo "      $TOTAL task and start-state files of the dataset at revision $DATA_REV"
n=0
while IFS= read -r rel; do
  [ -n "$rel" ] || continue
  mkdir -p "$DEST/$(dirname "$rel")"
  if [ ! -s "$DEST/$rel" ]; then
    # Written beside the file and moved into place whole, so a stopped download never stands as the file.
    curl -fsSL --retry 5 --retry-delay 5 -o "$DEST/$rel.part" "$DATA_URL/$rel"
    mv -f "$DEST/$rel.part" "$DEST/$rel"
  fi
  n=$((n + 1))
  if [ $((n % 40)) -eq 0 ] || [ "$n" -eq "$TOTAL" ]; then echo "      $n of $TOTAL"; fi
done < "$HERE/LIBERO-PRO-data.files.txt"
if command -v sha256sum >/dev/null; then
  (cd "$DEST" && sha256sum --quiet -c "$HERE/LIBERO-PRO-data.sha256")
else   # macOS has no sha256sum; shasum reads the same list
  (cd "$DEST" && shasum -a 256 -c "$HERE/LIBERO-PRO-data.sha256" | { grep -v ': OK$' || true; })
  (cd "$DEST" && shasum -a 256 -s -c "$HERE/LIBERO-PRO-data.sha256")
fi

write_config() {  # CONFIG_DIR TREE
  mkdir -p "$1"
  cat > "$1/config.yaml" <<YAML
benchmark_root: $2/libero/libero
bddl_files: $2/libero/libero/bddl_files
init_states: $2/libero/libero/init_files
datasets: $2/libero/datasets
assets: $2/libero/libero/assets
YAML
}
write_config "$BX/libero-config" "$BX/LIBERO"
write_config "$BX/liberopro-config" "$BX/LIBERO-PRO"

if [ -n "$SIM_PY" ]; then
  "$UV" pip install --python "$SIM_PY" --no-deps -e "$BX/LIBERO" --config-settings editable_mode=compat
fi
echo "benchmark ready in $BX"
