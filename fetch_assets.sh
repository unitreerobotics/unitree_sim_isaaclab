#!/bin/bash
# Download the scene / robot assets file by file from the HuggingFace dataset
# unitreerobotics/unitree_sim_isaaclab_usds (folder assets/ of the dataset -> ./assets).
#
#   bash fetch_assets.sh                          # all assets
#   bash fetch_assets.sh --robot g1d              # only what the tasks of these robots need
#   bash fetch_assets.sh --robot g129,r1a7d         #   (g129 | h1_2 | g1d | r1a7d | r17a | all; comma list or repeated)
#   bash fetch_assets.sh robots/g1-29dof-dex1-base-fix-usd urdf objects/PackingTable
#                                                 # only these folders / files (paths below assets/)
#
# Files that are already present and unchanged are skipped, so re-running only fetches what is missing or was
# updated; local files that are not in the dataset (e.g. assets/robots/.urdf_cache, your own scenes) are kept.
# A mirror can be used with HF_ENDPOINT, e.g. HF_ENDPOINT=https://hf-mirror.com bash fetch_assets.sh

set -e
set -o pipefail

REPO_ID="unitreerobotics/unitree_sim_isaaclab_usds"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-python}"
command -v "$PYTHON" >/dev/null 2>&1 || PYTHON=python3

# assets (below assets/) used by the tasks of each robot; the USDs only reference files inside their own folder
COMMON_ASSETS="objects/PackingTable"  # table of the pick & place cylinder tasks (all robots)
# scenes of the red block / stack / drawer tasks
BLOCK_SCENES="objects/small_warehouse objects/small_warehouse_digital_twin objects/table_with_yellowbox.usd objects/drawers"
robot_assets() {
    case "$1" in
        g129) echo "robots/g1-29dof-dex1-base-fix-usd robots/g1-29dof-dex3-base-fix-usd robots/g1-29dof-inspire-base-fix-usd" \
                   "robots/g1-29dof_wholebody_dex1 robots/g1-29dof_wholebody_dex3 robots/g1-29dof_wholebody_inspire" \
                   "objects/PackingTable_2 objects/small_warehouse objects/small_warehouse_digital_twin" \
                   "objects/table_with_yellowbox.usd objects/drawers model" ;;
        h1_2) echo "robots/h1_2-26dof-inspire-base-fix-usd objects/small_warehouse_digital_twin objects/table_with_yellowbox.usd" ;;
        g1d)  echo "urdf/g1_d_description $BLOCK_SCENES" ;;
        r1a7d)  echo "urdf/r1_d_description $BLOCK_SCENES" ;;
        r17a) echo "urdf/r1_7a_description objects/small_warehouse_digital_twin objects/table_with_yellowbox.usd" ;;
        *)    return 1 ;;
    esac
}

PATHS=()
ALL=0
while [ $# -gt 0 ]; do
    case "$1" in
        --robot|-r)
            [ -n "${2:-}" ] || { echo "Error: --robot needs a value (g129, h1_2, g1d, r1a7d, r17a or all)" >&2; exit 1; }
            for r in ${2//,/ }; do
                if [ "$r" = "all" ]; then
                    ALL=1
                elif list=$(robot_assets "$r"); then
                    PATHS+=($COMMON_ASSETS $list)
                else
                    echo "Error: unknown robot type '$r' (g129, h1_2, g1d, r1a7d, r17a or all)" >&2
                    exit 1
                fi
            done
            shift 2 ;;
        *)
            PATHS+=("$1")
            shift ;;
    esac
done
[ "$ALL" -eq 1 ] && PATHS=()
# unique paths, in order
mapfile -t PATHS < <(printf '%s\n' "${PATHS[@]}" | awk 'NF && !seen[$0]++')
if [ ${#PATHS[@]} -gt 0 ]; then
    echo "Assets: ${PATHS[*]}"
else
    echo "Assets: all"
fi

# httpx (used by huggingface_hub) needs the socksio package for a SOCKS proxy; if it is missing and an HTTP(S)
# proxy is configured as well, use that one instead of failing.
case "${ALL_PROXY:-${all_proxy:-}}" in
    socks*)
        if ! "$PYTHON" -c "import socksio" >/dev/null 2>&1; then
            if [ -n "${https_proxy:-${HTTPS_PROXY:-}}" ]; then
                echo "Note: ALL_PROXY is a SOCKS proxy but 'socksio' is not installed; using the HTTP(S) proxy instead."
                unset ALL_PROXY all_proxy
            else
                echo "Error: ALL_PROXY is a SOCKS proxy but 'socksio' is not installed (pip install socksio)." >&2
                exit 1
            fi
        fi
        ;;
esac

if ! "$PYTHON" -c "import huggingface_hub" >/dev/null 2>&1; then
    echo "Installing huggingface_hub..."
    "$PYTHON" -m pip install -q huggingface_hub
fi

echo "Downloading assets from $REPO_ID into $SCRIPT_DIR/assets ..."
"$PYTHON" - "$REPO_ID" "$SCRIPT_DIR" "${PATHS[@]}" <<'EOF'
import sys
from huggingface_hub import snapshot_download

repo_id, root, *paths = sys.argv[1:]
patterns = []
for p in paths or [""]:
    p = p.strip("/")
    if p.startswith("assets/"):
        p = p[len("assets/"):]
    # a folder (everything below it) or a single file / glob
    patterns += [f"assets/{p}/**", f"assets/{p}"] if p else ["assets/**"]
snapshot_download(repo_id=repo_id, repo_type="dataset", allow_patterns=patterns, local_dir=root, max_workers=8)
EOF

if [ ! -d "$SCRIPT_DIR/assets/robots" ] && [ ${#PATHS[@]} -eq 0 ]; then
    echo "Error: download finished but $SCRIPT_DIR/assets/robots is missing" >&2
    exit 1
fi
echo "✅ All done! Assets are in $SCRIPT_DIR/assets"
