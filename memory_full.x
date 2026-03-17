MEMORY
{
  # Uncommented by relivant build scripts
  # Bootloader relivant code cannot be larger than 120KB (Bootprot fuse limit)
  #FLASH_PRE (rx) : ORIGIN = 0x00000000            , LENGTH = 8K
  #FLASH_BLD (rx) : ORIGIN = 0x00000000 + 8K       , LENGTH = 112K
  #FLASH_APP (rx) : ORIGIN = 0x00000000 + 120K     , LENGTH = 0x00100000 - 120K
  CAN             : ORIGIN = 0x20000000            , LENGTH = 2K
  BL_COMM (xrw)   : ORIGIN = 0x20000000 + 2K       , LENGTH = 512
  RAM_TST(rw)     : ORIGIN = 0x20000000 + 2K + 512 , LENGTH = 128  # Buffer for RAM testing
  RAM (xrw)       : ORIGIN = 0x20000000 + 2K + 640 , LENGTH = 256K - 2K - 640
}

SECTIONS {
  .can (NOLOAD) :
  {
    *(.can .can.*);
  } > CAN

  .bl_comm (NOLOAD):  
  {
    *(.bl_comm);
  } > BL_COMM

  .ram_test (NOLOAD):  
  {
    *(.ram_test);
  } > RAM_TST
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
_ram_test_buffer_addr = ORIGIN(RAM_TST);
_ram_test_buffer_end_addr = ORIGIN(RAM_TST)+LENGTH(RAM_TST);
_ram_start_addr = ORIGIN(RAM);

_can_ram_addr = ORIGIN(CAN);
_can_ram_end_addr = ORIGIN(CAN)+LENGTH(CAN);