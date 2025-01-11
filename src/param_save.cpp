/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "param_save.h"

#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/flash.h>

#include "hwdefs.h"
#include "my_string.h"
#include "params.h"

#define NUM_PARAMS ((PARAM_BLKSIZE - 8) / sizeof(PARAM_ENTRY))
#define PARAM_WORDS (PARAM_BLKSIZE / 4)

#ifdef STM32F4
#define FLASH_SECTOR_0 0
#define FLASH_SECTOR_1 1
#define FLASH_SECTOR_2 2
#define FLASH_SECTOR_3 3
#define FLASH_SECTOR_4 4
#define FLASH_SECTOR_5 5
#define FLASH_SECTOR_6 6
#define FLASH_SECTOR_7 7
#define FLASH_SECTOR_8 8
#define FLASH_SECTOR_9 9
#define FLASH_SECTOR_10 10
#define FLASH_SECTOR_11 11
#endif

typedef struct
{
   uint16_t key;
   uint8_t dummy;
   uint8_t flags;
   uint32_t value;
} PARAM_ENTRY;

typedef struct
{
   PARAM_ENTRY data[NUM_PARAMS];
   uint32_t crc;
   uint32_t padding;
} PARAM_PAGE;

static uint32_t GetFlashAddress()
{
   uint32_t flashSize = desig_get_flash_size();

   //Always save parameters to last flash page
   return FLASH_BASE + flashSize * 1024 - PARAM_BLKNUM * PARAM_BLKSIZE;
}

#ifdef STM32F4
int GetFlashSector(uint32_t address)
{
    if (address < 0x08004000) return FLASH_SECTOR_0;
    if (address < 0x08008000) return FLASH_SECTOR_1;
    if (address < 0x0800C000) return FLASH_SECTOR_2;
    if (address < 0x08010000) return FLASH_SECTOR_3;
    if (address < 0x08020000) return FLASH_SECTOR_4;
    if (address < 0x08040000) return FLASH_SECTOR_5;
    if (address < 0x08060000) return FLASH_SECTOR_6;
    if (address < 0x08080000) return FLASH_SECTOR_7;
    if (address < 0x080A0000) return FLASH_SECTOR_8;
    if (address < 0x080C0000) return FLASH_SECTOR_9;
    if (address < 0x080E0000) return FLASH_SECTOR_10;
    return FLASH_SECTOR_11; // Up to 1MB of flash
}
#endif


/**
* Save parameters to flash
*
* @return CRC of parameter flash page
*/
uint32_t parm_save()
{
    PARAM_PAGE parmPage;
    uint32_t idx;
    uint32_t paramAddress = GetFlashAddress();
    uint32_t check = 0xFFFFFFFF;
    uint32_t* baseAddress = (uint32_t*)paramAddress;

    // Check if the flash page/sector is already erased
    for (int i = 0; i < PARAM_WORDS; i++, baseAddress++) {
        check &= *baseAddress;
    }

    crc_reset();
    memset32((int*)&parmPage, 0xFFFFFFFF, PARAM_WORDS);

    // Copy parameter values and keys to block structure
    for (idx = 0; idx < NUM_PARAMS && idx < Param::PARAM_LAST; idx++) {
        if (Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM) {
            const Param::Attributes* pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
            parmPage.data[idx].flags = (uint8_t)Param::GetFlag((Param::PARAM_NUM)idx);
            parmPage.data[idx].key = pAtr->id;
            parmPage.data[idx].value = Param::Get((Param::PARAM_NUM)idx);
        }
    }

    parmPage.crc = crc_calculate_block(((uint32_t*)&parmPage), (2 * NUM_PARAMS));
    flash_unlock();

#ifdef STM32F1
    // STM32F1: Erase page
    if (check != 0xFFFFFFFF) {
        flash_erase_page(paramAddress);
    }
#else
    // STM32F4: Erase sector
    if (check != 0xFFFFFFFF) {
        int sector = GetFlashSector(paramAddress); // Determine the sector
        flash_erase_sector(sector, FLASH_CR_PROGRAM_X32);
    }
#endif

    // Program the flash with new data
    for (idx = 0; idx < PARAM_WORDS; idx++) {
        uint32_t* pData = ((uint32_t*)&parmPage) + idx;
        flash_program_word(paramAddress + idx * sizeof(uint32_t), *pData);
    }

    flash_lock();
    return parmPage.crc;
}

/**
* Load parameters from flash
*
* @retval 0 Parameters loaded successfully
* @retval -1 CRC error, parameters not loaded
*/
int parm_load()
{
   uint32_t paramAddress = GetFlashAddress();
   PARAM_PAGE *parmPage = (PARAM_PAGE *)paramAddress;

   crc_reset();
   uint32_t crc = crc_calculate_block(((uint32_t*)parmPage), (2 * NUM_PARAMS));

   if (crc == parmPage->crc)
   {
      for (unsigned int idxPage = 0; idxPage < NUM_PARAMS; idxPage++)
      {
         Param::PARAM_NUM idx = Param::NumFromId(parmPage->data[idxPage].key);
         if (idx != Param::PARAM_INVALID && Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM)
         {
            Param::SetFixed(idx, parmPage->data[idxPage].value);
            Param::SetFlagsRaw(idx, parmPage->data[idxPage].flags);
         }
      }
      return 0;
   }

   return -1;
}
