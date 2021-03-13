## Logitech G610 gaming keyboard firmware

## CHS:
这是适用于罗技G610机械键盘的非官方固件(注意: 刷入第三方固件可能会损坏你的键盘，如果你的键盘是健康的/你对它没什么不满的话不建议尝试)。

其实也不必太担心，只要别搞坏硬件，基本上是成不了砖的，官方很厚道，固件并没有加密，可以直接读出来，这里备份了一个(Fw/Official_bootloader_and_app.bin)，重新刷回去就恢复原厂状态了。

首次刷机需要拆开键盘外壳，使用ST-Link下载bootloader, 此后升级就不再需要拆了。


# 特性:

1.五档背光亮度调节，带记忆，设置完毕5S后保存。

2.游戏模式下 GUI键(WIN键)将被关闭，同时启用NKRO模式。

3.Menu键功能取消了，作为 FN 功能键。

4.Insert键可开关，通过按住 FN + Insert 键切换，关闭状态下不影响组合键(如 Ctrl + Insert)的使用。

5.DFU升级功能，通过按住FN + GAMEMODE键进入。

6.与官方固件并存，FN + F1可切换到官方固件，当然这时想要再切回我们的固件就需要重新插拔键盘了。

7.按住GAMEMODE键上电，键盘直接进入DFU升级模式。

8.按住背光键上电，键盘进入官方固件。

9.拖尾灯效，可通过 FN + Light 键切换开关。

10.NumLock 守卫，妈妈再也不用我的 NumLock 被关了，可通过 FN + Light 键切换开关。


# Flash 分配图
![image](https://github.com/lingex/G610Rebuild/blob/master/PIC/flash%20mapping.png)

固件取巧使用了 MCU 标准容量之后的 64K flash (正式产品中不建议这样使用) 来存放我们自己写的键盘App, 这样就可以跟原厂的 App 共存了。

# 注意事项：

a.使用 ./Src/usbd_hid.c 和 ./Inc/usbd_hid.h 
文件组合而不是 ./Middlewares/ST/STM32_USB_Device_Library/Class/HID下的同名文件

原因是基础项目是通过cubeMx生成的DEMO，这两个文件需要做很多的修改，每次用cubeMx重新生成都会被覆盖掉，所以我把它们从驱动库中移出来了。

如果使用MDK编译，需要把./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c文件从源文件列表中移除

其他IDE也类似。

b.本项目依赖于以下Bootloader:

https://github.com/lingex/STM32L_DFU_Solution/tree/Branch_128k



## 刷机步骤

(除编译代码的IDE外，需要安装两个软件 STM32 ST-LINK Utility 和 ST DfuSe Demo)

其中ST DfuSe Demo是 ST 开源的，我在其基础上做了些修改，也可以用我修改的: https://github.com/lingex/ST-DFUSe/releases


a.刷入bootloader

  STM32 ST-LINK Utility : Bootloader.hex       (需要调试工具，如ST-Link)
  
  这里是SWD调试口
  
  ![image](https://github.com/lingex/G610Rebuild/blob/master/PIC/downloading/STLINK%20IO.png)
  
  可以这样下载而不需要焊接排针
  
  ![image](https://github.com/lingex/G610Rebuild/blob/master/PIC/downloading/bootloader.png)
  
  
b.刷入官方固件  

  方法1 STM32 ST-LINK Utility : Official_no_tail.hex    (需要调试工具，如ST-Link)
  
  方法2 ST DfuSe Demo ：Official_app_only.dfu    (无需额外工具)

c.刷入自定义固件
  
  方法1 STM32 ST-LINK Utility : G610Rebuild.hex (需要调试工具，如ST-Link)

  方法2 ST DfuSe Demo : G610Rebuild.dfu      (无需额外工具)


  
# 恢复到官方固件状态(同时将清除bootloader):

STM32 ST-LINK Utility : Official_bootloader_and_app.bin  (偏移地址: 0x8000000)


# 已知问题

1.音量滚轮体验大约能达到官方固件 80% 的水平，主要是低速下表现不佳，还可以改进；

2.NKRO是通过增加一个USB端点(端点2)来实现，但是通过端点2发送不出去，原因未知。所以使用了端点1来发送，改进方向是通过多个interface来实现；

3.NKRO键值映射可能在linux等一些系统下有问题，参见https://static.wongcornall.com/ibm-capsense-usb-web/ibm-capsense-usb.html#x1-140003.3
  实在碰到了就别开NKRO吧。


## —————————————————————————————


## EN:

A non-original firmware of Logitech G610 mechanical keyboard(Using non-original firmware may damage your keyboard).

You'll need to disassemble the keyboard and flash the bootloader with a ST-Link for the first time,

# Feature:

1.Configs (brightness) save to eeprom (delay 5 seconds).

2.Game mode will turn off the two LED of the GUI key.

3.Menu(Application) key turn into be a "FN" key now.

4.Switchable Insert key, by pressing FN + Insert, combination key like Ctrl + Insert are not affected.

5.DFU upgrade mode, by pressing FN + Game Mode.

6.Back to the official firmware by FN + F1.

7.Holding GameMode when plug in the usb cable, will go into DFU mode.

8.Holding backlight when plug in the usb cable, will run the official firmware.

9.Smearing light effect switch by FN + Light.

10.NumLock guard, 'don't touch my NumLock', switch by FN + NumLock.


# flash mapping
![image](https://github.com/lingex/G610Rebuild/blob/master/PIC/flash%20mapping.png)


# Note:

Use ./Src/usbd_hid.c and ./Inc/usbd_hid.h instead of the pair in ./Middlewares/ST/STM32_USB_Device_Library/Class/HID

coz every time the cubemx regenerate code they will be overwrite, so I move them out from the lib;

And don't forget to remove ./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c from MDK source list

Bootloader:

https://github.com/lingex/STM32L_DFU_Solution/tree/Branch_128k

# Important:
This branch is a non-standard use of STM32L100R8, by using the 64k~128k internal flash, to keep both official firmware and our firmware
inside, can not download into device directly using MDK, and no debug, st-link utility and dfu are feasible.


# Flash steps

(You will need to install 2 software (not include the IDE):  STM32 ST-LINK Utility  and  ST DfuSe Demo)

I did some modify on the DfuSe Demo, see here: https://github.com/lingex/ST-DFUSe/releases

a.flash bootloader

  STM32 ST-LINK Utility : Bootloader.hex     (need a debug tool like ST-Link)
  
  this are the SWD pins
  
  ![image](https://github.com/lingex/G610Rebuild/blob/master/PIC/downloading/STLINK%20IO.png)
  
  ![image](https://github.com/lingex/G610Rebuild/blob/master/PIC/downloading/bootloader.png)
  
b.flash official fw

  STM32 ST-LINK Utility : Official_no_tail.hex   (need a debug tool like ST-Link)
  
  or ST DfuSe Demo ：Official_app_only.dfu

c.flash this fw
  
  STM32 ST-LINK Utility : G610Rebuild.hex   (need a debug tool like ST-Link)
  
  or ST DfuSe Demo : G610Rebuild.dfu


DFU Tool
https://github.com/lingex/ST-DFUSe/releases

  
# Flash into official state(will also remove bootloader):

  STM32 ST-LINK Utility : Official_bootloader_and_app.bin  (offset: 0x8000000)


# Thanks to:

media keys
https://github.com/diabolo38/HidKbd.git

ztask
https://www.amobbs.com/thread-5722920-1-1.html

hex2dfu
https://github.com/nanoframework/hex2dfu
