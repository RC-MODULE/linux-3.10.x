#!/bin/bash
./mk mt143-05.dtb && \
./mk uImage && \
cp -p ./build/arch/arm/boot/dts/mt143-05.dtb ./target && \
./mk uinstall
