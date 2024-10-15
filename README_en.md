English / [简体中文](./README.md)

## 2024-10-15 note

This keyboard is quite old. To be honest, I don't really like it. The keycaps are bad and non-standard, difficult to buy a third-party replacement. I haven't used it for a long time, so there may not be any updates in the future for this firmware.

## Logitech G610 gaming keyboard firmware

## INTRO:

This is a non-original firmware of the Logitech G610 mechanical keyboard.

You'll need to disassemble the keyboard and flash the bootloader with an ST-Link for the first time.

Warning: flashing a non-original firmware may break your keyboard, be careful.

## Feature:

1. Configs (brightness) save to EEPROM (delay 5 seconds).

2. Game mode will turn off the two LEDs of the GUI key.

3. Menu(Application) key turn into an "FN" key now.

4. Switchable Insert key, by pressing FN + Insert, combination key like Ctrl + Insert are not affected.

5. DFU upgrade mode, by pressing FN + Game Mode.

6. Back to the official firmware by FN + F1.

7. Holding GameMode when plugging in the USB cable, will go into DFU mode.

8. Holding the backlight when plugging in the USB cable, will run the official firmware.

9. Smearing light effect switch by FN + Light.

10.NumLock guard, 'don't touch my NumLock', switch by FN + NumLock.


## Flash mapping
![image](./PIC/flash%20mapping.png)


## Note:

Use ./Src/usbd_hid.c and ./Inc/usbd_hid.h instead of the pair in ./Middlewares/ST/STM32_USB_Device_Library/Class/HID

coz every time the STM32CubeMX regenerate code they will be overwritten, so I move them out from the lib;

And don't forget to remove ./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c from MDK source list

Bootloader:

https://github.com/lingex/STM32L_DFU_Solution/tree/Branch_128k

## Important:
This branch is a non-standard use of STM32L100R8, by using the 64k~128k internal flash, to keep both official firmware and our firmware
inside, can not download into the device directly using MDK, and no debug, st-link utility and dfu are feasible.


## Flash steps

(You will need to install 2 software (not including the IDE):  STM32 ST-LINK Utility  and  ST DfuSe Demo)

I did some modifications to the DfuSe Demo, see here: https://github.com/lingex/ST-DFUSe/releases

a.flash bootloader

  STM32 ST-LINK Utility: Bootloader .hex     (need a debug tool like ST-Link)
  
  These are the SWD pins
  
  ![image](./PIC/downloading/STLINK%20IO.png)
  
  ![image](./PIC/downloading/bootloader.png)
  
b.flash official fw

  STM32 ST-LINK Utility: Official_no_tail.hex   (need a debug tool like ST-Link)
  
  or ST DfuSe Demo： Official_app_only.dfu

c.flash this fw
  
  STM32 ST-LINK Utility: G610Rebuild.hex   (need a debug tool like ST-Link)
  
  or ST DfuSe Demo: G610Rebuild.dfu


DFU Tool
https://github.com/lingex/ST-DFUSe/releases

  
## Flashback to official firmware (will also remove the bootloader):

  STM32 ST-LINK Utility : Official_bootloader_and_app.bin  (offset: 0x8000000)


## Knowing issues

 - The volume roller is not good enough compared to the official version, especially at low speed.

 - NKRO required another endpoint, but can not send messages through it, it may be solved by using multi interfaces.

 - NKRO may not work on some linux OS: https://static.wongcornall.com/ibm-capsense-usb-web/ibm-capsense-usb.html#x1-140003.3


## Thanks to:

 - media keys
 
 https://github.com/diabolo38/HidKbd.git

 - ztask
 
 https://github.com/tomzbj/ztask

 - hex2dfu
 
https://github.com/nanoframework/hex2dfu
