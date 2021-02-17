## G610Rebuild

## CHS:
�����������޼�G610��е���̵ķǹٷ��̼�(ע��: ˢ��������̼����ܻ�����ļ��̣������ļ����ǽ�����/�����ûʲô�����Ļ������鳢��)��



# ����:

1.�嵵�������ȵ��ڣ������䣬�������5S�󱣴档

2.��Ϸģʽ��GUI��(WIN��)�����⽫���رգ�ͬʱ����nkroģʽ��

3.Menu������ȡ���ˣ���Ϊ FN ���ܼ���

4.Insert���ɿ��أ�ͨ����ס FN + Insert ���л����ر�״̬�²�Ӱ����ϼ�(�� Ctrl + Insert)��ʹ�á�

5.DFU�������ܣ�ͨ����סFN + GAMEMODE�����롣

6.��ٷ��̼����棬FN + F1���л����ٷ��̼�����Ȼ��ʱ��Ҫ���л����ǵĹ̼�����Ҫ���²�μ����ˡ�

7.��סGAMEMODE���ϵ磬����ֱ�ӽ���DFU����ģʽ��

8.��ס������ϵ磬���̽���ٷ��̼���


# flash ����ͼ
![image](https://github.com/lingex/G610Rebuild/blob/Branch_128k/PIC/flash%20mapping.png)


# ע�����

a.ʹ�� ./Src/usbd_hid.c �� ./Inc/usbd_hid.h 
�ļ���϶����� ./Middlewares/ST/STM32_USB_Device_Library/Class/HID�µ�ͬ���ļ�

ԭ���ǻ�����Ŀ��ͨ��cubeMx���ɵ�DEMO���������ļ���Ҫ���ܶ���޸ģ�ÿ���޸Ķ��ᱻ���¸��ǵ�

���ʹ��MDK���룬��Ҫ��./Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c�ļ���Դ�ļ��б����Ƴ�

����IDEҲ���ơ�

b.����Ŀ����������Bootloader:

https://github.com/lingex/STM32L_DFU_Solution/tree/Branch_128k


��flash����ͼ���Կ���������һ�ַǳ����÷���flash��׼������64KB��ʵ���ϲ��õ���128KB�ľ�Բ�������ܹ���128KB����������ʹ�ã��Ͳ�Ʒ�ĽǶ��ǲ��Ƽ�����ʹ�õġ�



## ˢ������

a.ˢ��bootloader

  STM32 ST-LINK Utility : Bootloader.hex
  
b.ˢ��ٷ��̼�  

  ����1 STM32 ST-LINK Utility : Official_no_tail.hex
  
  ����2 ST DfuSe Demo ��Official_app_only.dfu

c.ˢ���Զ���̼�

  ST DfuSe Demo : g610App.dfu


  
# �ָ��عٷ��̼�״̬(ͬʱ�����bootloader):

STM32 ST-LINK Utility : Official_bootloader_and_app.bin  (ƫ�Ƶ�ַ: 0x8000000)



## ��������������������������������������������������������������������������


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

# flash mapping
![image](https://github.com/lingex/G610Rebuild/blob/Branch_128k/PIC/flash%20mapping.png)


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

  STM32 ST-LINK Utility : Bootloader.hex
  
b.flash official fw

  STM32 ST-LINK Utility : Official_no_tail.hex
  
  or ST DfuSe Demo ��Official_app_only.dfu

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
