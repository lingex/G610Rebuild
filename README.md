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

2. some backlight effects.



# Note:
Use ./Src/usbd_hid.c and ./Inc/usbd_hid.h instead of the pair in ./Middlewares/ST/STM32_USB_Device_Library/Class/HID

coz every time the cubemx regenerate code they will be overwrite;

And don't forget to remove ./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c from MDK source list

Bootloader:
https://github.com/lingex/STM32L10X_MSD_BOOTLOADER.git


# Thanks to:

media keys
https://github.com/diabolo38/HidKbd.git

ztask
https://www.amobbs.com/thread-5722920-1-1.html

hex2dfu
https://github.com/nanoframework/hex2dfu
