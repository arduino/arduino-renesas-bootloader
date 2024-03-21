#!/bin/bash

if [ x$TINYUSB_ROOT == x ]; then
echo Please set TINYUSB_ROOT env variable
exit 1
fi

PROJECT_NAME=`basename $PWD`

if [ -d _build ]; then
  rm -R _build
fi

if [ -d distrib ]; then
  rm -R distrib
fi

mkdir distrib

make -f Makefile.wifi  -j8
mv _build/uno_r4/$PROJECT_NAME.hex distrib/dfu_wifi.hex
rm -rf _build

make -f Makefile.minima  -j8
mv _build/uno_r4/$PROJECT_NAME.hex distrib/dfu_minima.hex
rm -rf _build

make -f Makefile.c33  -j8
mv _build/portenta_c33/$PROJECT_NAME.hex distrib/dfu_c33.hex
rm -rf _build

make -f Makefile.opta-analog  -j8
mv _build/uno_r4/$PROJECT_NAME.hex distrib/opta-analog.hex
rm -rf _build

make -f Makefile.opta-digital  -j8
mv _build/uno_r4/$PROJECT_NAME.hex distrib/opta-digital.hex
rm -rf _build