#!/bin/bash

#export PATH=$PATH:/opt/arm-2009q1/bin
export CROSS_COMPILE=arm-none-linux-gnueabi-
export ARCH=arm

make modules 
export INSTALL_MOD_PATH="/home/open-nandra/projects/htc_wizard/nfs_root"
make modules_install 