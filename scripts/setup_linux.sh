#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "$(uname -s)" != "Linux" ]]; then
  echo "Use Linux with an NVIDIA driver and Vulkan support." >&2
  exit 1
fi
command -v uv >/dev/null || { echo "Install uv first: https://docs.astral.sh/uv/getting-started/installation/"; exit 1; }
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python --override constraints/mplib.txt -e '.[sim,planning,dev]'
uv venv --python 3.12 .venv-data
uv pip install --python .venv-data/bin/python -e '.[data]'
echo 'Activate with: source .venv/bin/activate'
echo 'Set the isolated dataset writer: export SCI_SIM_DATA_PYTHON="$PWD/.venv-data/bin/python"'
echo 'Then run: sci-sim prepare-assets'
echo 'Import local instrument assets: sci-sim import-instruments --source ../atombridge'
