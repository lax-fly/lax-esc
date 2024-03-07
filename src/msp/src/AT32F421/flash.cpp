#include "flash.h"
#include "at32f421_conf.h"
#include <string.h>

#ifndef FLASH_BASE
#define FLASH_BASE 0x08000000U
#endif
#define FLASH_SIZE 0x10000U // 64kB
#define FLASH_END (FLASH_BASE + FLASH_SIZE)
#define PAGE_SIZE 0x400U

#define EEPROM_SIZE (PAGE_SIZE)
#define EEPROM_ADDR (FLASH_END - EEPROM_SIZE)

static int flash_storage_erase(uint32_t address)
{
    /* wait for last operation  */
    while (flash_flag_get(FLASH_OBF_FLAG))
    {
    }

    /* clear all pending flags */
    flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);

    /* erase the flash pages */
    for (; address < FLASH_END; address += PAGE_SIZE)
    {
        if (flash_sector_erase(address) != FLASH_OPERATE_DONE)
        {
            break;
        }
    }

    return (address >= FLASH_END) ? 0 : -1;
}

static int flash_storage_write(uint32_t const *data, uint32_t size)
{
    uint32_t address = EEPROM_ADDR;

    if (!data || !size || EEPROM_SIZE < size)
        return -1;

    /*
        Returns false results when changing only currPowerdB.
        Commenting out for now to force writing.
    */

    /* unlock the flash program/erase controller */
    flash_unlock();

    if (flash_storage_erase(address) < 0)
    {
        // Erase failed
        flash_lock();
        return -1;
    }

    /* program flash */
    while (size-- && address < FLASH_END)
    {
        if (flash_word_program(address, *data++) != FLASH_OPERATE_DONE)
        {
            break;
        }
        address += 4U;
    }

    /* lock the main FMC after the program operation */
    flash_lock();

    if (size)
        return -1;

    return 0;
}

void Flash::write(uint8_t *data, uint32_t sz)
{
    flash_storage_write((uint32_t *)data, ((sz + 3) / 4));
}

void Flash::read(uint8_t *data, uint32_t sz)
{
    memcpy(data, (uint8_t *)EEPROM_ADDR, sz);
}

static Flash flash;

FlashIf *FlashIf::singleton()
{
    return &flash;
}