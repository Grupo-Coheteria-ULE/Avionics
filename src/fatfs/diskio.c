/**
 * @file diskio.c
 * @brief FatFs disk I/O bridge to sd_spi driver.
 */

#include "ff.h"
#include "diskio.h"
#include "sd_spi.h"

extern sd_spi_t g_sd;

DSTATUS disk_initialize(BYTE pdrv)
{
    (void)pdrv;
    return 0;
}

DSTATUS disk_status(BYTE pdrv)
{
    (void)pdrv;
    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;
    return (sd_spi_read(&g_sd, sector, buff, count) == SD_SPI_OK) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;
    return (sd_spi_write(&g_sd, sector, buff, count) == SD_SPI_OK) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)pdrv;

    switch (cmd)
    {
        case CTRL_SYNC:
            return RES_OK;
        case GET_SECTOR_COUNT:
            *(DWORD *)buff = sd_spi_get_sector_count(&g_sd);
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}

DWORD get_fattime(void)
{
    return ((DWORD)(2026 - 1980) << 25) | ((DWORD)1 << 21) | ((DWORD)1 << 16);
}
