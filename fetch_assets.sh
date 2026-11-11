#!/bin/bash
# Download the scene / robot assets file by file from the HuggingFace dataset
# unitreerobotics/unitree_sim_isaaclab_usds (folder assets/ of the dataset -> ./assets).
#
#   bash fetch_assets.sh                          # all assets
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
"$PYTHON" - "$REPO_ID" "$SCRIPT_DIR" "$@" <<'EOF'
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

if [ ! -d "$SCRIPT_DIR/assets/robots" ] && [ $# -eq 0 ]; then
    echo "Error: download finished but $SCRIPT_DIR/assets/robots is missing" >&2
    exit 1
fi
echo "✅ All done! Assets are in $SCRIPT_DIR/assets"
