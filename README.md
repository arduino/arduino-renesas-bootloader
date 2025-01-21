**Arduino bootloader for Renesas boards**

```
git clone https://github.com/arduino/arduino-renesas-bootloader
git clone https://github.com/hathach/tinyusb.git
cd tinyusb
git checkout 0.17.0
python ./tools/get_deps.py ra
export TINYUSB_ROOT=$PWD
patch -p1 < ../arduino-renesas-bootloader/0001-fix-arduino-bootloaders.patch
cd ..
cd arduino-renesas-bootloader
./compile.sh
```
