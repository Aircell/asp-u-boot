#!/bin/bash -eu
# Build u-boot

target=u-boot.bin

workspace=$PWD/..
PATH=/usr/lib/jvm/java-1.5.0-sun/bin:/usr/bin:/bin
PATH+=:$workspace/cs-android-kernel/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin

exec &> OUT

# Preconfigure
if [ ! -e include/config.mk ]; then
  make omap3_logic_config
fi

# Erase existing binary
if [ -e $target ]; then
  rm -rf $target
fi

# Build
make CROSS_COMPILE=arm-eabi- all

if [ $? -ne 0 ] || [ ! -e $target ]; then
  echo FAIL
  exit 1
fi

## Install
INSTALLDIR=$workspace/out
[ -d $INSTALLDIR ] || mkdir -p $INSTALLDIR
cp $target $INSTALLDIR

echo SUCCESS
exit 0
