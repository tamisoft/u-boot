export CC=arm-linux-gnueabihf-
make ARCH=arm CROSS_COMPILE=${CC} distclean
make ARCH=arm CROSS_COMPILE=${CC} mx53sigbox_defconfig
make ARCH=arm CROSS_COMPILE=${CC} -j 16
#cp -vf u-boot.bin /var/lib/tftpboot/sigbox
#cp -vf u-boot.imx /var/lib/tftpboot/sigbox
