# G610Rebuild
A non-original firmware of Logitech G610 mechanical keyboard.

Feature:
1.Configs (brightness, game mode) save to eeprom.
2.Game mode will turn off the two LED of the GUI key.
3.Menu(Application) key turn into be a "FN" key now.
4.Switchable Insert key, by pressing CTRL + INSERT, the LED will show the status.



TODO:
1.Volume up and down are not finish yet.
2."NKRO" support.



Note:
use ./Src/usbd_hid.c and ./Inc/usbd_hid.h instead of the pair in ./Middlewares/ST/STM32_USB_Device_Library/Class/HID
coz every time the cubemx regenerate code they will be overwrite;
And don't forget to remove ./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c from MDK source list



Thanks to:

media keys
https://github.com/diabolo38/HidKbd.git

bootloader
https://github.com/sfyip/STM32F103_MSD_BOOTLOADER.git

ztask
https://www.amobbs.com/thread-5722920-1-1.html
