
# Diagnostic common interface

This is shared between the MCU firmware sections, and PC software


## KWP Routines


|RID|Desc|Bootloader|Application|Args|Original EGS supported|
|:-:|:-:|:-:|:-:|:-:|:-:|
|0x24|Burn production date|Y|N|[D(1),W(1),M(1),Y(1)]|
|0xE0|Flash erase|Y|N|[ADDR(4),BLKS(2)]|
|0xE1|Flash CRC32|Y|N|[TARGCRC32(4),STARTADDR(4),ENDADDR(4)]|
|0x30|Clear EEPROM|N|Y||Y|
|0x33|Toggle TCC Sol|N|Y|[TY(1),EN(1)]|Y|