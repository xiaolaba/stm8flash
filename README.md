


Uses Raspberry PI & SPI interface to mimic SWIM single wire interface for ST MCU flash programming. This version is based on stm8flash, original code supports SWIM & uses ST-LINK hardware. Anszom extended it to support SPI-based SWIM programming. Read the article of design, http://kuku.eu.org/?projects/stm8spi/stm8spi  


Raspberry Pi 3 model B, SPI pinout  
==================================  
![2021-01-27_Raspberry_Pi_3B_SPI_pinout/Pi_3B_SPI_pin.JPG](2021-01-27_Raspberry_Pi_3B_SPI_pinout/Pi_3B_SPI_pin.JPG)  

origianl schematic,
[2021-01-27_Raspberry_Pi_3B_SPI_pinout/rpi_SCH_3b_1p2_reduced.pdf](2021-01-27_Raspberry_Pi_3B_SPI_pinout/rpi_SCH_3b_1p2_reduced.pdf)


install Pi OS first,  
![2021-01-27_Raspberry_Pi_3B_SPI_pinout/install.JPG](2021-01-27_Raspberry_Pi_3B_SPI_pinout/install.JPG)  

boot Pi up, once setup done, [language pack](https://xiaolaba.wordpress.com/2016/04/10/raspberry-pi-3-chinese-characters-display-and-input/) is well prepared upon your location and timezone preference, no need any manual installation as before.

set whatever you want for this Pi computer,  
 user name : pi  
 pwd : xiao  

open terminal, build this software,  
```  
  git clone https://github.com/xiaolaba/stm8flash  
  sudo apt install libusb-1.0  
  cd stm8flash  
  make clean  
  make  
  
  ## will copy 'stm8flash' to /usr/bin/ 
  sudo make install
  
  ## run this program
  stm8flash
```  
  
the job done and ready,  
![2021-01-27_Raspberry_Pi_3B_SPI_pinout/2021-01-27-113415_1920x1080_scrot.png](2021-01-27_Raspberry_Pi_3B_SPI_pinout/2021-01-27-113415_1920x1080_scrot.png)

wiring and connection diagram  

![2021-01-27_Raspberry_Pi_3B_SPI_pinout/connection_diagram2021-01-27.jpg](2021-01-27_Raspberry_Pi_3B_SPI_pinout/connection_diagram2021-01-27.jpg)  

 
  
  
    
.    .
.  
stm8flash
=========

This is a free and opensource software distributed under the terms of GNU General Public License v2.

It also seems to be the only program that's able to communicate through the SWIM interface of ST-LINKs under Linux as for March, 2014.


Synopsis
--------

```
stm8flash -c <stlink|stlinkv2> -p <partname> [-s flash|eeprom|0x8000] [-r|-w|-v] <filename>
```

Flash examples:
```nohighlight
./stm8flash -c stlink -p stm8s003f3 -w blinky.bin
./stm8flash -c stlink -p stm8s003f3 -w blinky.ihx
./stm8flash -c stlinkv2 -p stm8s003f3 -w blinky.ihx
./stm8flash -c stlink -p stm8s105c6 -w blinky.bin
./stm8flash -c stlinkv2 -p stm8l150 -w blinky.bin
```

EEPROM examples:
```nohighlight
./stm8flash -c stlinkv2 -p stm8s003f3 -s eeprom -r ee.bin
./stm8flash -c stlinkv2 -p stm8s003f3 -s eeprom -w ee.bin
./stm8flash -c stlinkv2 -p stm8s003f3 -s eeprom -v ee.bin
```

Support table
-------------

  * ST-Link V1: flash/eeprom/opt
  * ST-Link V2: flash2/eeprom2/opt2

| MCU         | flash | eeprom | opt  | flash2 | eeprom2 | opt2  |
|-------------|-------|--------|------|--------|---------|-------|
| stlux???a   |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af526?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af528?  |  ?    |  ?     |  ?   |  32    |  ?      |  ?    |
| stm8af52a?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af6213  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af6223  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af6223a |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af6226  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af624?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af6266  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af6268  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af6269  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af628?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8af62a?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8al313?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8al314?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8al316?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8al3l4?  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8al3l6?  |  ?    |  ?     |  ?   |  ok    |  ?      |  ?    |
| stm8l051f3  |  ok   |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l052c6  |  ok   |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l052r8  |  ok   |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l101f1  |  ?    |  no    |  ?   |  ?     |  no     |  ?    |
| stm8l101?2  |  ?    |  no    |  ?   |  ?     |  no     |  ?    |
| stm8l101?3  |  ?    |  no    |  ?   |  ok    |  no     |  ?    |
| stm8l151?2  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l151?3  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l151?4  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l151?6  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l151?8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l152?4  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l152?6  |  ok   |  FAIL  |  ?   |  ok    |  ok     |  ?    |
| stm8l152?8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8l162?8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s003?3  |  ok   |  FAIL  |  ?   |  ok    |  ok     |  ?    |
| stm8s005?6  |  ok   |  ?     |  ok  |  ?     |  ?      |  ?    |
| stm8s007c8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s103f2  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s103?3  |  ?    |  ?     |  ?   |  ok    |  ?      |  ?    |
| stm8s105?4  |  ok   |  FAIL  |  ?   |  ok    |  ok     |  ?    |
| stm8s105?6  |  ok   |  ?     |  ?   |  ok    |  ?      |  ?    |
| stm8s207c8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s207cb  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s207k8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s207m8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s207mb  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s207r8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s207rb  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s207s8  |  ?    |  ?     |  ?   |  32    |  ?      |  ?    |
| stm8s207sb  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s207?6  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s208c6  |  ?    |  ?     |  ?   |  ok    |  ?      |  ?    |
| stm8s208s6  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s208?8  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8s208?b  |  ?    |  ?     |  ?   |  32    |  ?      |  ?    |
| stm8s903?3  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8splnb1  |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |
| stm8tl5??4  |  ?    |  no    |  ?   |  ?     |  no     |  ?    |
| stnrg???a   |  ?    |  ?     |  ?   |  ?     |  ?      |  ?    |

Legend:

  * `ok`   - Fully supported.
  * `no`   - Not supported.
  * `?`    - Not tested.
  * `FÁIL` - Not working. Needs fix.
  * `32`   - Lower 32K of flash works, upper doesn't.

