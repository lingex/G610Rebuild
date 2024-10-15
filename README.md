[English](./README_en.md) / 简体中文

## 2024-10-15 注
  这个键盘已经很老了，说实话并不是很喜欢，键帽质量很差还非标，想自己换第三方都不好买。其实已经很久不用了，这个固件后续大概也不会再改进了。


## 罗技 G610 机械键盘非官方固件

## 简介:

这是适用于罗技G610机械键盘的非官方固件。

刷非官方固件需要较强的动手能力，对STM32单片机有一定了解，并且需要额外的刷机工具如ST-LINK, 操作不当可能会导致键盘损坏。

小心操作别搞坏硬件，基本上是成不了砖的。官方很大气，原厂固件并没有加密，可以直接读出来，这个仓库里备份了一个(Fw/Official_bootloader_and_app.bin)，重新刷回去就恢复原厂状态了。

首次刷机需要拆开键盘外壳，使用ST-Link下载bootloader, 此后升级就不再需要拆了。


## 特性:

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


## Flash 分配表
![image](./PIC/flash%20mapping.png)

固件取巧使用了 MCU 标准容量之后的 64K flash (正式产品中不建议这样使用) 来存放我们自己写的键盘App, 这样就可以跟原厂的 App 共存了。

## 注意事项：

a.使用 ./Src/usbd_hid.c 和 ./Inc/usbd_hid.h 
文件组合而不是 ./Middlewares/ST/STM32_USB_Device_Library/Class/HID下的同名文件

原因是基础项目是通过cubeMx生成的DEMO，这两个文件需要做很多的修改，每次用cubeMx重新生成都会被覆盖掉，所以我把它们从驱动库中移出来了。

如果使用MDK编译，需要把./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c文件从源文件列表中移除

其他IDE也类似。

b.本项目依赖于以下Bootloader:

https://github.com/lingex/STM32L_DFU_Solution/tree/Branch_128k



## 刷机步骤

(除编译代码的IDE外，需要安装两个软件 STM32 ST-LINK Utility 和 ST DfuSe Demo)

其中ST DfuSe Demo是 ST 开源的，我在其基础上做了些修改更方便使用: https://github.com/lingex/ST-DFUSe/releases

使用官方原版或者我修改的版本均可。


a.刷入bootloader

  STM32 ST-LINK Utility: Bootloader.hex       (需要调试工具，如ST-Link)
  
  这里是SWD调试口
  
  ![image](./PIC/downloading/STLINK%20IO.png)
  
  可以这样下载而不需要焊接排针
  
  ![image](./PIC/downloading/bootloader.png)
  
  
b.刷入官方固件  

  方法1 STM32 ST-LINK Utility : Official_no_tail.hex    (需要调试工具，如ST-Link)
  
  方法2 ST DfuSe Demo ：Official_app_only.dfu    (无需额外工具)

c.刷入自定义固件
  
  方法1 STM32 ST-LINK Utility : G610Rebuild.hex (需要调试工具，如ST-Link)

  方法2 ST DfuSe Demo : G610Rebuild.dfu      (无需额外工具)


  
## 恢复到官方固件状态(同时将清除bootloader):

STM32 ST-LINK Utility : Official_bootloader_and_app.bin  (偏移地址: 0x8000000)


## 已知问题

 - 音量滚轮体验相较官方固件还有些差距，主要是低转速下表现不佳；

 - NKRO是通过增加一个USB端点(端点2)来实现，但是通过端点2发送不出去，原因未知。所以使用了端点1来发送，改进方向是通过多个interface来实现；

 - NKRO键值映射可能在linux等一些系统下有问题，参见https://static.wongcornall.com/ibm-capsense-usb-web/ibm-capsense-usb.html#x1-140003.3
  实在碰到了就别开NKRO吧。

## 感谢

 - media keys

https://github.com/diabolo38/HidKbd.git

 - ztask

https://github.com/tomzbj/ztask

 - hex2dfu

https://github.com/nanoframework/hex2dfu
