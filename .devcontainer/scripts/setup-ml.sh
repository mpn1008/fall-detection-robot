#!/usr/bin/env bash
set -euo pipefail
case "${1:-cpu}" in
    cpu) index=https://download.pytorch.org/whl/cpu ;;
    cuda) index=https://download.pytorch.org/whl/cu128 ;;
    *) echo 'Usage: setup-ml.sh [cpu|cuda]' >&2; exit 2 ;;
esac

# CUDA 12.8 wheels support Blackwell GPUs such as the RTX 5060 Ti.
# The host supplies the driver; do not install a Linux NVIDIA driver here.
/opt/venv/bin/python -m pip install --upgrade pip
/opt/venv/bin/python -m pip install --upgrade --index-url "$index" \
    'torch>=2.7,<3' 'torchvision>=0.22,<1'
/opt/venv/bin/python -m pip install 'ultralytics>=8.3,<9'
/opt/venv/bin/python -c 'import torch; print("PyTorch:", torch.__version__, "CUDA available:", torch.cuda.is_available())'
