#!/usr/bin/env bash
set -euo pipefail
project_dir="${1:?Pass the Linux path of the sci-sim project}"
if [[ "$(id -u)" != 0 ]]; then
  echo 'Provision through wsl --user root; this script installs system packages.' >&2
  exit 1
fi
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y python3-venv python3-pip ca-certificates curl git \
  libvulkan1 vulkan-tools mesa-vulkan-drivers libgl1 libegl1 libx11-6 libxext6 libxrender1 ffmpeg
if ! id scisim >/dev/null 2>&1; then
  useradd --create-home --shell /bin/bash scisim
fi
# No password or passwordless sudo is created. Provisioning is explicit through wsl -u root.
python3 -m venv /opt/scisim-bootstrap
/opt/scisim-bootstrap/bin/pip install uv
if [[ ! -x /home/scisim/.venvs/sci-sim/bin/python ]]; then
  runuser -u scisim -- /opt/scisim-bootstrap/bin/uv venv --python 3.12 /home/scisim/.venvs/sci-sim
fi
cd "$project_dir"
# WSL has CPU PhysX support; avoid downloading an unused multi-GB CUDA runtime.
runuser -u scisim -- /opt/scisim-bootstrap/bin/uv pip install \
  --python /home/scisim/.venvs/sci-sim/bin/python \
  torch==2.7.1+cpu torchvision==0.22.1+cpu --index https://download.pytorch.org/whl/cpu
runuser -u scisim -- /opt/scisim-bootstrap/bin/uv pip install \
  --python /home/scisim/.venvs/sci-sim/bin/python --override constraints/mplib.txt \
  -e '.[sim,planning,dev]'
if [[ ! -x /home/scisim/.venvs/sci-data/bin/python ]]; then
  runuser -u scisim -- /opt/scisim-bootstrap/bin/uv venv --python 3.12 /home/scisim/.venvs/sci-data
fi
runuser -u scisim -- /opt/scisim-bootstrap/bin/uv pip install \
  --python /home/scisim/.venvs/sci-data/bin/python \
  torch==2.9.1+cpu torchvision==0.24.1+cpu --index https://download.pytorch.org/whl/cpu
runuser -u scisim -- /opt/scisim-bootstrap/bin/uv pip install \
  --python /home/scisim/.venvs/sci-data/bin/python -e '.[data]' torchcodec==0.9.1
runuser -u scisim -- /home/scisim/.venvs/sci-sim/bin/sci-sim prepare-assets
if [[ -d "$project_dir/../atombridge/model/instrument" ]]; then
  runuser -u scisim -- /home/scisim/.venvs/sci-sim/bin/sci-sim import-instruments --source ../atombridge
fi
printf '\nReady. Use wsl -d SciSim-Ubuntu -u scisim, then:\n'
printf 'source /home/scisim/.venvs/sci-sim/bin/activate\n'
printf 'cd %q\n' "$project_dir"
printf 'source scripts/activate_wsl.sh\n'
printf 'vulkaninfo --summary\n'
printf 'sci-sim render --layout lab --output outputs/overview.png\n'
