#!/bin/bash -eu
# Build u-boot

[ -f ~/.bash-android ] && source ~/.bash-android
[ -f $CLOUDSURFER_ROOT/bin/bash-android ] && source $CLOUDSURFER_ROOT/bin/bash-android

make="make -j4"
target=u-boot.bin

# exec &> $0.OUT

# Preconfigure
if [ ! -e include/config.mk ]; then
  $make omap3_logic_config
fi

# Erase existing binary
if [ -e $target ]; then
  rm -rf $target
fi

# Build
$make CROSS_COMPILE=arm-eabi- ${*:-all}

if [ $? -ne 0 ] || [ ! -e $target ]; then
  echo FAIL
  exit 1
fi

echo SUCCESS
exit 0
