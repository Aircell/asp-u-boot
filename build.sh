#!/bin/bash -eu
# Build u-boot

make="make -j4"
target=u-boot.bin

workspace=$PWD/..
source ~/.bash-android

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

## Install
INSTALLDIR=$workspace/../install.d
[ -d $INSTALLDIR ] || mkdir -p $INSTALLDIR
cp $target $INSTALLDIR

echo SUCCESS
exit 0
