# Building in Ubuntu

Building for Ubuntu platform is remarkably easy. The only trick to understand is that the Ubuntu toolchain,
which they are downstreaming from Debian, might not be compatible with Cleanflight. This applies to Ubuntu versions
at least up to (and including) 16.04. For example, the cross-compiler that comes with Ubuntu 16.04 is version 4.9.3, 
released in May 2015 while at the time of writing this article (October 2017), the most recent version was 6.3.1.
You can check your version by

```
arm-none-eabi-gcc --version
```

When using the version 4.9.3 the compilation will succeed but you might see a lot of warnings of the following type:

```
  .... uses 4-byte wchar_t yet the output is to use 2-byte wchar_t; use of wchar_t values across objects may fail
```

If any version of the ARM crosscompiler ("arm-none-eabi-gcc") is installed, Cleanflight will use it. So, you must make sure that a correct
and compatible version is installed. You do this by first uninstalling the existing, incompatible version by
```
sudo apt-get remove binutils-arm-none-eabi gcc-arm-none-eabi
```
After this, you need to install a good version of "arm-none-eabi-gcc". 

The steps you need to take depend on the version of Cleanflight you want to create. Starting with Cleanflight 2.0, the procedure is very easy, as the Makefile that comes with Cleanflight will take care of almost everything automatically - in particular, it will download the ARM cross-compiler if there is none installed. So, all you need to do after un-installing old incompatible versions of the cross-compiler is to say (example: for SPRACINGF3EVO):
```
$ git clone git@github.com:cleanflight/cleanflight.git
$ cd cleanflight
$ make TARGET=SPRACINGF3EVO
```

You will see a message that the cross-compiler is downloaded and installed. Then, you'll see a set of files being compiled, and finally linked, yielding both an ELF and then a HEX:
```
...
arm-none-eabi-size ./obj/main/cleanflight_SPRACINGF3EVO.elf
   text	   data	    bss	    dec	    hex	filename
 135880	   1928	  20664	 158472	  26b08	./obj/main/cleanflight_SPRACINGF3EVO.elf
arm-none-eabi-objcopy -O ihex --set-start 0x8000000 obj/main/cleanflight_SPRACINGF3EVO.elf obj/cleanflight_SPRACINGF3EVO.hex

leco@Hutia:~/repositories/git/cleanflight$ ls -la obj/
-rw-rw-r--  1 leco leco 387709 Okt 28 00:00 cleanflight_SPRACINGF3EVO.hex

```

You can use the Cleanflight-Configurator to flash the ```obj/cleanflight_SPRACINGF3EVO.hex``` file.

If you would like to compile an old version of Cleanflight (before 2.0), the Makefile will not download the 
current crosscompiler for you. It is easiest to use a Cleanflight version 2.x, and make any TARGET, e.g. SPRACINGF3EVO (just as described above). The cross-compiler is downloaded and installed into the directory cleanflight/tools. Then, checkout - with git - the old version of Cleanflight you want to compile, set your PATH variable to point to the bin/ subdirectory of the gcc-arm-none-eabi.../ folder inside tools/, and make your TARGET again (do not forget to "make clean TARGET=SPRACINGF3EVO" before making the final hex file).


## Flashing the resulting hex file
You can install the cp210x driver with
```
$ sudo modprobe cp210x
$ lsmod | grep cp210x
cp210x                 24576  0
usbserial              53248  1 cp210x
```
When plugging in the USB cable with the FC, "dmesg" shows
```
$ dmesg
[ 5742.227149] usb 1-6: new full-speed USB device number 47 using xhci_hcd
[ 5742.356226] usb 1-6: New USB device found, idVendor=0483, idProduct=5740
[ 5742.356235] usb 1-6: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 5742.356240] usb 1-6: Product: STM32 Virtual COM Port  
[ 5742.356244] usb 1-6: Manufacturer: STMicroelectronics
[ 5742.356248] usb 1-6: SerialNumber: 2059365F5433
[ 5742.356510] usb 1-6: ep 0x82 - rounding interval to 1024 microframes, ep desc says 2040 microframes
[ 5742.356945] cdc_acm 1-6:1.0: ttyACM0: USB ACM device
```
Sometimes, however, you get a problem and no device ttyACMx will be created. This may look like this:
```
[ 2776.729734] cdc_acm 1-6:1.0: ttyACM0: USB ACM device
[ 2777.557676] usb 1-6: USB disconnect, device number 20
[ 2777.557864] cdc_acm 1-6:1.0: failed to set dtr/rts
```

Should you encounter this problem, it is probably the modem manager service that prevents Ubuntu from creating the device. Try killing it, and if that works, you might want to prevent it from restarting at the next reboot:
```
leco@Hutia:~$ sudo systemctl --all | grep -i modem # lets find out the name of the service
  ModemManager.service  
$ sudo service ModemManager stop
$ # to make it persistent through reboot do:
$ sudo systemctl disable ModemManager
```

In order to flash, the bootloader device for STM32 MCUs must be writable for the user (you), which, by default, is not the case. The following line establishes a rule that the STM32 bootloader device is in the 'plugdev' Unix group:
```
$ (echo '# DFU (Internal bootloader for STM32 MCUs)'
 echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", GROUP="plugdev"') | sudo tee /etc/udev/rules.d/45-stdfu-permissions.rules > /dev/null
```
Anyone who is in the 'plugdev' group may now write to the STM32 bootloader device. If you are not, make yourself member by

```
sudo usermod -a -G plugdev <username>
```

Also it says you should create a file 
`/etc/udev/rules.d/90-ttyACM-group-plugdev.rules`
that contains 
`KERNEL=="ttyACM[0-9]", GROUP="plugdev"`
 




We suggest that you take an
alternative PPA from Terry Guo, found here:
https://launchpad.net/~terry.guo/+archive/ubuntu/gcc-arm-embedded

This PPA has several compiler versions and platforms available. For many hardware platforms (notably Naze)
the 4.9.3 compiler will work fine. For some, older compiler 4.8 (notably Sparky) is more appropriate. We
suggest you build with 4.9.3 first, and try to see if you can connect to the CLI or run the Configurator.
If you cannot, please see the section below for further hints on what you might do.

## Setup GNU ARM Toolchain

Note specifically the last paragraph of Terry's PPA documentation -- Ubuntu carries its own package for
`gcc-arm-none-eabi`, so you'll have to remove it, and then pin the one from the PPA.
For your release, you should first remove any older pacakges (from Debian or Ubuntu directly), introduce
Terry's PPA, and update:
```
sudo apt-get remove binutils-arm-none-eabi gcc-arm-none-eabi
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
```

For Ubuntu 14.10 (current release, called Utopic Unicorn), you should pin:
```
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0utopic12
```

For Ubuntu 14.04 (an LTS as of Q1'2015, called Trusty Tahr), you should pin:
```
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0trusty12
```

For Ubuntu 12.04 (previous LTS, called Precise Penguin), you should pin:
```
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0precise12
```

## Building on Ubuntu

After the ARM toolchain from Terry is installed, you should be able to build from source.
```
cd src
git clone git@github.com:cleanflight/cleanflight.git
cd cleanflight
make TARGET=NAZE
```

You'll see a set of files being compiled, and finally linked, yielding both an ELF and then a HEX:
```
...
arm-none-eabi-size ./obj/main/cleanflight_NAZE.elf 
   text    data     bss     dec     hex filename
  97164     320   11080  108564   1a814 ./obj/main/cleanflight_NAZE.elf
arm-none-eabi-objcopy -O ihex --set-start 0x8000000 obj/main/cleanflight_NAZE.elf obj/cleanflight_NAZE.hex
$ ls -la obj/cleanflight_NAZE.hex                                                                                                                                                 
-rw-rw-r-- 1 pim pim 274258 Jan 12 21:45 obj/cleanflight_NAZE.hex
```

You can use the Cleanflight-Configurator to flash the ```obj/cleanflight_NAZE.hex``` file.

## Bricked/Bad build?

Users have reported that the 4.9.3 compiler for ARM produces bad builds, for example on the Sparky hardware platform.
It is very likely that using an older compiler would be fine -- Terry happens to have also a 4.8 2014q2 build in his
PPA - to install this, you can fetch the `.deb` directly:
http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu/pool/main/g/gcc-arm-none-eabi/

and use `dpkg` to install:
```
sudo dpkg -i gcc-arm-none-eabi_4-8-2014q2-0saucy9_amd64.deb
```

Make sure to remove `obj/` and `make clean`, before building again.

## Updating and rebuilding

Navigate to the local cleanflight repository and use the following steps to pull the latest changes and rebuild your version of cleanflight:

```
cd src/cleanflight
git reset --hard
git pull
make clean TARGET=NAZE
make
```

Credit goes to K.C. Budd, AKfreak for testing, and pulsar for doing the long legwork that yielded this very short document.
