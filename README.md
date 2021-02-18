## G610Rebuild

## CHS:
这是适用于罗技G610机械键盘的非官方固件(注意: 刷入第三方固件可能会损坏你的键盘，如果你的键盘是健康的/你对它没什么不满的话不建议尝试)。

不过也不必太担心，只要别搞坏硬件，基本上是成不了砖的，官方很厚道，固件没有加密，可以直接读出来，这里备份了一个(Fw/Official_bootloader_and_app.bin)，重新刷回去就恢复原厂状态了。



# 特性:

1.五档背光亮度调节，带记忆，设置完毕5S后保存。

2.游戏模式下GUI键(WIN键)将背光将被关闭，同时启用nkro模式。

3.Menu键功能取消了，作为 FN 功能键。

4.Insert键可开关，通过按住 FN + Insert 键切换，关闭状态下不影响组合键(如 Ctrl + Insert)的使用。

5.DFU升级功能，通过按住FN + GAMEMODE键进入。

6.与官方固件并存，FN + F1可切换到官方固件，当然这时想要再切回我们的固件就需要重新插拔键盘了。

7.按住GAMEMODE键上电，键盘直接进入DFU升级模式。

8.按住背光键上电，键盘进入官方固件。

9.默认开启拖尾等效，如不需要可把 main.h 中 #define TAILING_EFFECT 1 改为 #define TAILING_EFFECT 0


# flash 分配图
![image](https://github.com/lingex/G610Rebuild/blob/master/PIC/flash%20mapping.png)


# 注意事项：

a.使用 ./Src/usbd_hid.c 和 ./Inc/usbd_hid.h 
文件组合而不是 ./Middlewares/ST/STM32_USB_Device_Library/Class/HID下的同名文件

原因是基础项目是通过cubeMx生成的DEMO，这两个文件需要做很多的修改，每次修改都会被重新覆盖掉

如果使用MDK编译，需要把./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c文件从源文件列表中移除

其他IDE也类似。

b.本项目依赖于以下Bootloader:

https://github.com/lingex/STM32L_DFU_Solution/tree/Branch_128k


从flash分配图可以看出，这是一种非常规用法，flash标准容量是64KB，实际上采用的是128KB的晶圆，所以总共有128KB的容量可以使用，从产品的角度是不推荐这样使用的。



## 刷机步骤

a.刷入bootloader

  STM32 ST-LINK Utility : Bootloader.hex       (需要调试工具，如ST-Link)
  
b.刷入官方固件  

  方法1 STM32 ST-LINK Utility : Official_no_tail.hex    (需要调试工具，如ST-Link)
  
  方法2 ST DfuSe Demo ：Official_app_only.dfu    (无需额外工具)

c.刷入自定义固件

  ST DfuSe Demo : g610App.dfu      (无需额外工具)


  
# 恢复回官方固件状态(同时将清除bootloader):

STM32 ST-LINK Utility : Official_bootloader_and_app.bin  (偏移地址: 0x8000000)



## —————————————————————————————


## EN:

A non-original firmware of Logitech G610 mechanical keyboard(Using non-original firmware may damage your keyboard).

# Feature:

1.Configs (brightness) save to eeprom.

2.Game mode will turn off the two LED of the GUI key.

3.Menu(Application) key turn into be a "FN" key now.

4.Switchable Insert key, by pressing FN + Insert, combination key like Ctrl + Insert are not affected.

5.DFU upgrade mode, by pressing FN + Game Mode.

6.Back to the official firmware by FN + F1.

7.Holding GameMode when plug in the usb cable, will go into DFU mode.

8.Holding backlight when plug in the usb cable, will run the official firmware.

9.Using tailing light effect by default, turn off: in main.h : #define TAILING_EFFECT 1 --> #define TAILING_EFFECT 0


# flash mapping
![image](https://github.com/lingex/G610Rebuild/blob/master/PIC/flash%20mapping.png)


# Note:

Use ./Src/usbd_hid.c and ./Inc/usbd_hid.h instead of the pair in ./Middlewares/ST/STM32_USB_Device_Library/Class/HID

coz every time the cubemx regenerate code they will be overwrite;

And don't forget to remove ./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c from MDK source list

Bootloader:

https://github.com/lingex/STM32L_DFU_Solution/tree/Branch_128k

# Important:
This branch is a non-standard use of STM32L100R8, by using the 64k~128k internal flash, to keep both official firmware and this firmware
inside, can not download into device directly using MDK, and no debug, st-link utility and dfu are feasible.


# Flash steps

a.flash bootloader

  STM32 ST-LINK Utility : Bootloader.hex     (need a debug tool like ST-Link)
  
b.flash official fw

  STM32 ST-LINK Utility : Official_no_tail.hex   (need a debug tool like ST-Link)
  
  or ST DfuSe Demo ：Official_app_only.dfu

c.flash this fw

  ST DfuSe Demo : g610App.dfu


  
# flash into official state(will also remove bootloader):

  STM32 ST-LINK Utility : Official_bootloader_and_app.bin  (offset: 0x8000000)


# Thanks to:

media keys
https://github.com/diabolo38/HidKbd.git

ztask
https://www.amobbs.com/thread-5722920-1-1.html

hex2dfu
https://github.com/nanoframework/hex2dfu
