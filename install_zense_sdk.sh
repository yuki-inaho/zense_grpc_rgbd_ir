#!/bin/bash -e

PICOZENSE_LIB="PicoZenseSDK_Ubuntu18.04_20190605_v2.4.0.4_DCAM710"
PICOZENSE_INSTALL_DIR=/usr/local/PicoZenseSDK

add-apt-repository ppa:nilarimogard/webupd8
apt-get update
apt-get install libvdpau-va-gl1
apt-get install i965-va-driver
apt-get install vdpauinfo

curl -L https://zstatic.picovr.com/zense/sdk/DCAM710/Ubuntu/$PICOZENSE_LIB.tar.bz2 |
	tar jx -C /usr/local

mv /usr/local/$PICOZENSE_LIB $PICOZENSE_INSTALL_DIR

chown -R root:root $PICOZENSE_INSTALL_DIR
cd $PICOZENSE_INSTALL_DIR

./install.sh

cat <<EOF > /usr/lib/pkgconfig/libpicozense.pc
prefix=$PICOZENSE_INSTALL_DIR
exec_prefix=\${prefix}
includedir=\${prefix}/Include
libdir=\${exec_prefix}/Lib/x64
Name: libpicozense
Description: The Library for Pico Zense
Version: 1.0.0
Cflags: -I\${includedir}/
Libs: -L\${libdir} -lpicozense_api
EOF

ln -sf $(pkg-config --libs-only-L libpicozense | sed 's/^-L//')/* /usr/local/lib/

ldconfig
