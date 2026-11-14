# Source this file from the project root in SciSim-Ubuntu.
source /home/scisim/.venvs/sci-sim/bin/activate
export SCI_SIM_DATA_PYTHON=/home/scisim/.venvs/sci-data/bin/python
# This WSL machine exposes only Mesa software Vulkan to SAPIEN.
# Scope the explicit ICD to this shell; do not change native Linux GPU settings.
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json
