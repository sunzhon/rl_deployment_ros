#!/usr/bin/bash

if [ $# -lt 1 ]
then
echo "please set OS, 'jetson' or 'ubuntu' "
return -1
fi

# Jetson
if [[ $1 == *"jetson"* ]]
then
echo "1) install pytorch on jetson"
sudo apt-get -y update; 
sudo apt-get -y install python3-pip libopenblas-dev
export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
python3 -m pip install --upgrade pip && python3 -m pip install numpy==1.22.4 && python3 -m pip install --no-cache $TORCH_INSTALL
else
# Ubuntu not Jetson
echo "1) install pytorch on ubuntu"
pip install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
fi

echo "2) install python standard package"
pip install -r requirement.txt
sudo apt-get install ros-$(rosversion -d)-ros-numpy

echo "3) install rls_rl"

cd ./../rsl_rl/ && pip install -e .




