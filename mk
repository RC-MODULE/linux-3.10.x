#!/bin/bash

mkdir -p build
mkdir -p target
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabi-
export INSTALL_PATH=../target
export INSTALL_MOD_PATH=../target
export KBUILD_OUTPUT=./build
export KBUILD_VERBOSE=1
export KCFLAGS=-pipe

make -j 8 $1 $2 $3 $4 $5 $6 $7 $8 $9
