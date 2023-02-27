
#include <msp430.h>
#include <inttypes.h>
#include "Defines.h"

#include "SPI.h"
#include "ff.h"			/* Obtains integer types */
#include "diskio.h"


// Glue functions for the FatFS library

// Send the disk into idle state and check for correct response
// TODO: unsure if this will screw with things if FatFS uses this function while doing other things?
DSTATUS disk_status (BYTE pdrv) {
  return (SD_goIdleState() == 0x01);
}

// Initalse the disk and return disk status if succesful
DSTATUS disk_initialize (BYTE pdrv) {
  SD_init();
  return disk_status(pdrv);
}

// Read the given sectors into the buffer
DRESULT disk_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {

  if (disk_status(pdrv)) { return RES_NOTRDY; }

  // TODO: add checks for exceding SD card size
  if ((pdrv != 0)) { return RES_PARERR; }

  uint8_t token, res;
  int reads;
  for (reads = 0; reads < count; reads++) {
    res = SD_readSingleBlock(sector + reads, buff+512*reads ,&token);

    if (res || (token & 0b00000001)) {return RES_ERROR; }

  }

  return RES_OK;
}

// Writes to the given sectors from the buffers. Note, will be slow as individual write calls are not recomended.
DRESULT disk_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
  if (disk_status(pdrv)) { return RES_NOTRDY; }

  // TODO: add checks for exceding SD card size
  if ((pdrv != 0)) { return RES_PARERR; }

  uint8_t token, res;
  int writes;
  for (writes = 0; writes < count; writes++) {
    res = SD_writeSingleBlock(sector + writes, buff+512*writes, &token);
    if (res || (token & 0b00000001)) {return RES_ERROR; }

  }

  return RES_OK;
}

// Controls various device specific features
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff) {

  if (disk_status(pdrv)) { return RES_NOTRDY; }

  if (pdrv != 0) { return RES_PARERR; }

  // Switch of required commands
  switch (cmd) {
    case CTRL_SYNC : return RES_OK;
    case GET_SECTOR_COUNT :
    // send CMD9
    // index: 9
    // argument: RCA Addr (0 I think), stuff bits
    // TODO: do I need a CRC?, also, RCA things may be wrong
    // spi_transfer(0xFF); // 0xFF send before and after CS for safety (see notes 29/12)
    // CS_ENABLE();
    // spi_transfer(0xFF);
    //
    // uint8_t res[17];
    // SD_command(9, 0, 0);
    // SD_readRes2(&res);
    //
    // // deassert chip select
    // spi_transfer(0xFF);
    // CS_DISABLE();
    // spi_transfer(0xFF);


//     Get the two parameters burried in the data
//    // C_SIZE has 2 bits in res[7], all of res[8] and 2 in res[9]
//    int c_size = ((res[9] & (BIT0 | BIT1)) << 10) | (res[8] << 2) | (res[7] & (BIT6 | BIT7));
//    // C_SIZE_MILTI has 1 bit in res[5] and 2 in res[6]
//    int c_size_multi = ((res[6] & (BIT0 | BIT1)) << 1) | (res[5] & BIT7);
//    int multi = (int) pow(2,c_size_multi);

    // *(DWORD*)buff = (c_size+1) * multi;
    *(DWORD*)buff = 8000000;
    return RES_OK;
    case GET_SECTOR_SIZE : *(DWORD*)buff = 512; return RES_OK;
    case GET_BLOCK_SIZE : *(DWORD*)buff = 1; return RES_OK;
    case CTRL_TRIM : return RES_OK;
    default : return RES_PARERR;
  }

  }


DWORD get_fattime (void) { return 0;}

 /* fp [OUT] File object to move to the end of */
FRESULT setAppend (FIL* fp )
{
    /* Seek to end of the file to append data */
    fr = f_lseek(fp, f_size(fp));
    if (fr != FR_OK)
        f_close(fp);
    }
    return fr;
}
