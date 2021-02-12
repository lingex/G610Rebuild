# G610Rebuild
A non-original firmware of Logitech G610 mechanical keyboard.

# Feature:
1.Configs (brightness, game mode) save to eeprom.

2.Game mode will turn off the two LED of the GUI key.

3.Menu(Application) key turn into be a "FN" key now.

4.Switchable Insert key, by pressing CTRL + INSERT, the LED will show the status.

5.Dfu upgrade mode, by pressing FN(Menu) + Game Mode.


# TODO:
1."NKRO" support.



# Note:
Use ./Src/usbd_hid.c and ./Inc/usbd_hid.h instead of the pair in ./Middlewares/ST/STM32_USB_Device_Library/Class/HID

coz every time the cubemx regenerate code they will be overwrite;

And don't forget to remove ./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c from MDK source list

Bootloader:

https://github.com/lingex/STM32L_DFU_Solution/tree/Branch_128k

# Important:
This branch is a non-standard use of STM32L100R8, by using the 64k~128k internal flash, to keep both official firmware and this firmware
inside, can not download into device directly using MDK, and no debug, st-link utility and dfu are feasible.


Default load is this firmware, two ways to switch to official firmware:
1.press and hold backlight then plug in the usb cable,
2.press Fn (menu) + F1 when running this firmware.


# Thanks to:

media keys
https://github.com/diabolo38/HidKbd.git

ztask
https://www.amobbs.com/thread-5722920-1-1.html

hex2dfu
https://github.com/nanoframework/hex2dfu
