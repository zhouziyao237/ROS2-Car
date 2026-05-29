#!/usr/bin/env bash

cd /host-home/Software || exit 1

mkdir -p /tmp/groot_home
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root

env -i \
  HOME=/tmp/groot_home \
  USER=root \
  LOGNAME=root \
  DISPLAY=$DISPLAY \
  XAUTHORITY=$XAUTHORITY \
  XDG_RUNTIME_DIR=/tmp/runtime-root \
  QT_X11_NO_MITSHM=1 \
  QT_OPENGL=software \
  QT_QUICK_BACKEND=software \
  QSG_RENDER_LOOP=basic \
  NO_AT_BRIDGE=1 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  ./Groot-1.0.0-x86_64.AppImage
