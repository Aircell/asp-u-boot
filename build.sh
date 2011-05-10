#!/bin/bash -eu
# Build u-boot

make="make -j4"
target=u-boot.bin

if [ -f ../bin/bash-android ]; then
  source ../bin/bash-android
fi

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
