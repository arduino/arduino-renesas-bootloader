# don't forget to patch
# -/* Board has bootloader */
# -FLASH_IMAGE_START = 0x10000;
# in hw/bsp/ra/boards/.../*.ld

TINYUSB_ROOT=path_to_tinyusb_repo make -f Makefile.wifi  -j8
TINYUSB_ROOT=path_to_tinyusb_repo make -f Makefile.c33  -j8
TINYUSB_ROOT=path_to_tinyusb_repo make -f Makefile.minima  -j8
