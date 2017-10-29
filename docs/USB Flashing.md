# USB Flashing
Some newer boards with full USB support must be flashed in USB DFU mode. This is a straightforward process in Configurator versions 0.67 and newer. The standard flashing procedure should work successfully with the caveat of some platform specific problems as noted below. The "No reboot sequence" checkbox has no effect as the device will automatically be detected when already in bootloader mode (a DFU device will appear in the connect dropdown if this is the case). The Full chip erase checkbox operates as normal. The baudrate checkbox is ignored as it has no relevance to USB.

### Charging-Only Cables
If you see no signs of life on your host computer when you plug in your board, check your cable with your mobile phone or some other USB device - some charging cables have only the power pins connected. These will power up the board, so the leds light up, but the host computer will not react to the device at all. You need a proper USB cable to connect your board to the Cleanflight Configurator.

## Platform Specific: Linux
Linux requires udev rules to allow write access to USB devices for users. An example shell command to acheive this on Ubuntu is shown here:
```
(echo '# DFU (Internal bootloader for STM32 MCUs)'
 echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", GROUP="plugdev"') | sudo tee /etc/udev/rules.d/45-stdfu-permissions.rules > /dev/null
```

This assigns the device to the plugdev group(a standard group in Ubuntu). To check that your account is in the plugdev group type `groups` in the shell and ensure plugdev is listed. If not you can add yourself as shown (replacing `<username>` with your username):
```
sudo usermod -a -G plugdev <username>
```

If you see your ttyUSB device disappear right after the board is connected, chances are that the ModemManager service (that handles network connectivity for you) thinks it is a GSM modem. If this happens, you can issue the following command to disable the service:
```
sudo systemctl stop ModemManager.service 
```

If your system lacks the systemctl command, use any equivalent command that works on your system to disable services. You can likely add your device ID to a blacklist configuration file to stop ModemManager from touching the device, if you need it for cellural networking, but that is beyond the scope of cleanflight documentation.

If you see the ttyUSB device appear and immediately disappear from the list in Cleanflight Configurator when you plug in your flight controller via USB, chances are that NetworkManager thinks your board is a GSM modem and hands it off to the ModemManager daemon as the flight controllers are not known to the blacklisted


## Platform Specific: Windows
Chrome can have problems accessing USB devices on Windows. A driver should be automatically installed by Windows for the ST Device in DFU Mode but this doesn't always allow access for Chrome. The solution is to replace the ST driver with a libusb driver. The easiest way to do that is to download [Zadig](http://zadig.akeo.ie/). 
With the board connected and in bootloader mode (reset it by sending the character R via serial, or simply attempt to flash it with the correct serial port selected in Configurator): 
* Open Zadig
* Choose Options > List All Devices
* Select `STM32 BOOTLOADER` in the device list
* Choose `WinUSB (v6.x.x.x)` in the right hand box
![Zadig Driver Procedure](assets/images/zadig-dfu.png)
* Click Replace Driver
* Restart Chrome (make sure it is completely closed, logout and login if unsure)
* Now the DFU device should be seen by Configurator



# Troubleshooting, and some background (Linux)

Sometimes, connecting the Configurator program to the FC, or flashing the FC using the configurator, fail. The following section provides some background that help to identify the reason of the problem.

First, when connecting your FC to the computer, the LED on the board should light up - if they dont, your cable, your FC board, or your computer USB port are broken. If they do, wait a few seconds, then check whether your Linux and your board have properly established a connection. You do this with the `lsusb` command which will list all USB devices that are currently connected. In the example below, the FC was found as Device 025 on Bus 001. With `lsusb -vs bus:device`, detailed information about the device can be queried:

```
$ lsusb
Bus 001 Device 025: ID 0483:5740 STMicroelectronics STM32F407
$ lsusb -vs 001:025
Bus 001 Device 025: ID 0483:5740 STMicroelectronics STM32F407
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            2 Communications
  bDeviceSubClass         0 
  bDeviceProtocol         0 
  bMaxPacketSize0        64
  idVendor           0x0483 STMicroelectronics
  idProduct          0x5740 STM32F407
  bcdDevice            2.00
  iManufacturer           1 STMicroelectronics
  iProduct                2 STM32 Virtual COM Port  
...
```



```
$ sudo dmesg
# connect a NAZE
[ 8308.198937] usb 1-5: new full-speed USB device number 10 using xhci_hcd
[ 8308.328547] usb 1-5: New USB device found, idVendor=10c4, idProduct=ea60
[ 8308.328555] usb 1-5: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 8308.328560] usb 1-5: Product: CP2102 USB to UART Bridge Controller
[ 8308.328563] usb 1-5: Manufacturer: Silicon Labs
[ 8308.328567] usb 1-5: SerialNumber: 0001
[ 8309.435542] usbcore: registered new interface driver usbserial
[ 8309.435583] usbcore: registered new interface driver usbserial_generic
[ 8309.435623] usbserial: USB Serial support registered for generic
[ 8309.450879] usbcore: registered new interface driver cp210x
[ 8309.450891] usbserial: USB Serial support registered for cp210x
[ 8309.450910] cp210x 1-5:1.0: cp210x converter detected
[ 8309.452253] usb 1-5: cp210x converter now attached to ttyUSB0
# disconnect the NAZE:
[ 8404.849369] usb 1-5: USB disconnect, device number 10
[ 8404.849739] cp210x ttyUSB0: cp210x converter now disconnected from ttyUSB0
[ 8404.849774] cp210x 1-5:1.0: device disconnected
# connect a SPRACINGF3EVO:
[ 9342.990591] usb 1-5: new full-speed USB device number 15 using xhci_hcd
[ 9343.102638] usb 1-5: device descriptor read/64, error -71
[ 9343.318613] usb 1-5: device descriptor read/64, error -71
[ 9343.534605] usb 1-5: new full-speed USB device number 16 using xhci_hcd
[ 9343.646691] usb 1-5: device descriptor read/64, error -71
[ 9343.862639] usb 1-5: device descriptor read/64, error -71
[ 9344.358639] usb 1-5: new full-speed USB device number 18 using xhci_hcd
[ 9344.488031] usb 1-5: New USB device found, idVendor=0483, idProduct=5740
[ 9344.488039] usb 1-5: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 9344.488044] usb 1-5: Product: STM32 Virtual COM Port  
[ 9344.488048] usb 1-5: Manufacturer: STMicroelectronics
[ 9344.488051] usb 1-5: SerialNumber: 206830544234
[ 9344.488363] usb 1-5: ep 0x82 - rounding interval to 1024 microframes, ep desc says 2040 microframes
[ 9344.488919] cdc_acm 1-5:1.0: ttyACM0: USB ACM device
# start flashing the SPRACINGF3EVO by using the configurator (hit the 'Flash Firmware' button)
[ 9444.715745] usb 1-5: USB disconnect, device number 18
[ 9445.025906] usb 1-5: new full-speed USB device number 19 using xhci_hcd
[ 9445.154828] usb 1-5: New USB device found, idVendor=0483, idProduct=df11
[ 9445.154836] usb 1-5: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 9445.154841] usb 1-5: Product: STM32  BOOTLOADER
[ 9445.154845] usb 1-5: Manufacturer: STMicroelectronics
[ 9445.154848] usb 1-5: SerialNumber: 206830542030
# here the flashing happens... then, once it is done, the following happens
#  (no intervention from the user)
[ 9466.888400] usb 1-5: USB disconnect, device number 19
[ 9467.326622] usb 1-5: new full-speed USB device number 20 using xhci_hcd
[ 9467.456005] usb 1-5: New USB device found, idVendor=0483, idProduct=5740
[ 9467.456014] usb 1-5: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 9467.456018] usb 1-5: Product: STM32 Virtual COM Port  
[ 9467.456022] usb 1-5: Manufacturer: STMicroelectronics
[ 9467.456026] usb 1-5: SerialNumber: 206830544234
[ 9467.456325] usb 1-5: ep 0x82 - rounding interval to 1024 microframes, ep desc says 2040 microframes
[ 9467.456851] cdc_acm 1-5:1.0: ttyACM0: USB ACM device
# disconnect the SPRACINGF3EVO:
[ 9703.396560] usb 1-5: USB disconnect, device number 20
```

If you see your ttyUSB device disappear right after the board is connected, chances are that the ModemManager service (that handles network connectivity for you) thinks it is a GSM modem. If this happens, you can issue the following command to disable the service:
```
sudo systemctl stop ModemManager.service 
```

If your system lacks the systemctl command, use any equivalent command that works on your system to disable services. You can likely add your device ID to a blacklist configuration file to stop ModemManager from touching the device, if you need it for cellural networking, but that is beyond the scope of cleanflight documentation.

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
Anyone who is in the 'plugdev' group may now write to the STM32 bootloader device. If you are not - you find out by entering the `groups` command and checking whether 'plugdev' is in the resulting llist -, make yourself member by

```
sudo usermod -a -G plugdev <username>
```

Also it says you should create a file 
`/etc/udev/rules.d/90-ttyACM-group-plugdev.rules`
that contains 
`KERNEL=="ttyACM[0-9]", GROUP="plugdev"`
 

