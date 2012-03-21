#!/bin/bash -eu
# Build u-boot

[ -f ~/.bash-android ] && source ~/.bash-android
[ -f $CLOUDSURFER_ROOT/bin/bash-android ] && source $CLOUDSURFER_ROOT/bin/bash-android

make="make -j4 CROSS_COMPILE=arm-eabi- "
target=u-boot.bin

# exec &> $0.OUT

# Preconfigure
$make mrproper
$make omap3_logic_config

# Build
$make ${*:-all}

if [ $? -ne 0 ] || [ ! -e $target ]; then
  echo FAIL
  exit 1
fi

echo SUCCESS
exit 0
