MEMORY {
  sfr              : ORIGIN = 0x0000, LENGTH = 0x0010 /* END=0x0010, size 16 */
  peripheral_8bit  : ORIGIN = 0x0010, LENGTH = 0x00f0 /* END=0x0100, size 240 */
  peripheral_16bit : ORIGIN = 0x0100, LENGTH = 0x0100 /* END=0x0200, size 256 */
  ram (wx)         : ORIGIN = 0x0200, LENGTH = 0x0200 /* END=0x0400, size 512 */
  infomem          : ORIGIN = 0x1000, LENGTH = 0x0100 /* END=0x1100, size 256 as 4 64-byte segments */
  infod            : ORIGIN = 0x1000, LENGTH = 0x0040 /* END=0x1040, size 64 */
  infoc            : ORIGIN = 0x1040, LENGTH = 0x0040 /* END=0x1080, size 64 */
  infob            : ORIGIN = 0x1080, LENGTH = 0x0040 /* END=0x10c0, size 64 */
  infoa            : ORIGIN = 0x10c0, LENGTH = 0x0040 /* END=0x1100, size 64 */
  rom (rx)         : ORIGIN = 0xe000, LENGTH = 0x1fde /* END=0xffde, size 8158 */
  vectors          : ORIGIN = 0xffe0, LENGTH = 0x0020 /* END=0x10000, size 32 as 16 2-byte segments */
  /* Remaining banks are absent */
  bsl              : ORIGIN = 0x0000, LENGTH = 0x0000
  ram2 (wx)        : ORIGIN = 0x0000, LENGTH = 0x0000
  ram_mirror (wx)  : ORIGIN = 0x0000, LENGTH = 0x0000
  signature        : ORIGIN = 0x0000, LENGTH = 0x0000
  tinyram (wx)     : ORIGIN = 0x0000, LENGTH = 0x0000
  usbram (wx)      : ORIGIN = 0x0000, LENGTH = 0x0000
  far_rom          : ORIGIN = 0x00000000, LENGTH = 0x00000000
}
REGION_ALIAS("REGION_TEXT", rom);
REGION_ALIAS("REGION_DATA", ram);
REGION_ALIAS("REGION_FAR_ROM", far_rom); /* Legacy name, no longer used */
REGION_ALIAS("REGION_FAR_TEXT", far_rom);
REGION_ALIAS("REGION_FAR_DATA", ram2);
PROVIDE (__info_segment_size = 0x40);
PROVIDE (__infod = 0x1000);
PROVIDE (__infoc = 0x1040);
PROVIDE (__infob = 0x1080);
PROVIDE (__infoa = 0x10c0);
