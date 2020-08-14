/**************************************************************************//**
 * @file     dfmc.c
 * @version  V1.00
 * @brief    M471 series DFMC driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>

#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup DFMC_Driver DFMC Driver
  @{
*/


/** @addtogroup DFMC_EXPORTED_FUNCTIONS DFMC Exported Functions
  @{
*/

/**
  * @brief Disable DFMC ISP function.
  * @return None
  */
void DFMC_Close(void)
{
    DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief Execute DFMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 256 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 256 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  */
int32_t DFMC_Erase(uint32_t u32PageAddr)
{
    int32_t  ret = 0;

    DFMC->ISPCMD = DFMC_ISPCMD_PAGE_ERASE;
    DFMC->ISPADDR = u32PageAddr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    while (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) { }

    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        ret = -1;
    }
    return ret;
}

/**
  * @brief Enable DFMC ISP function
  * @return None
  */
void DFMC_Open(void)
{
    DFMC->ISPCTL |=  DFMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute DFMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  */
uint32_t DFMC_Read(uint32_t u32Addr)
{
    DFMC->ISPCMD = DFMC_ISPCMD_READ;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
    while (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) { }

    return DFMC->ISPDAT;
}

/**
  * @brief Execute ISP DFMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return None
  */
void DFMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    DFMC->ISPCMD = DFMC_ISPCMD_PROGRAM;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPDAT = u32Data;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
    while (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) { }

    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
    }
}


/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 256 bytes.
  * @return Success or not.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter.
  */
int32_t  DFMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
    int32_t   ret;

    if ((u32addr % 256UL) || (u32count % 256UL))
    {
        ret = 0xFFFFFFFF;
    }
    else
    {
        DFMC->ISPCMD  = DFMC_ISPCMD_RUN_CKS;
        DFMC->ISPADDR = u32addr;
        DFMC->ISPDAT  = u32count;
        DFMC->ISPTRG  = DFMC_ISPTRG_ISPGO_Msk;

        while (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk) { }


        if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
        {
            DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
            ret = -1;
        }

        DFMC->ISPCMD = DFMC_ISPCMD_READ_CKS;
        DFMC->ISPADDR    = u32addr;
        DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

        while (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk) { }

        if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
        {
            DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
            ret = -1;
        }
        ret = DFMC->ISPDAT;
    }

    return ret;
}


/**
  * @brief Run flash all one verification and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 256 bytes.
  * @retval   DREAD_ALLONE_YES       The contents of verified flash area are 0xFFFFFFFF.
  * @retval   DREAD_ALLONE_NOT       Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   DREAD_ALLONE_CMD_FAIL  Unexpected error occurred.
  */
int32_t  DFMC_CheckAllOne(uint32_t u32addr, uint32_t u32count)
{
    int32_t  ret = DREAD_ALLONE_CMD_FAIL;

    DFMC->ISPSTS = DFMC_ISPSTS_ALLONE_Msk;   /* clear check all one bit */

    DFMC->ISPCMD   = DFMC_ISPCMD_RUN_ALL1;
    DFMC->ISPADDR  = u32addr;
    DFMC->ISPDAT   = u32count;
    DFMC->ISPTRG   = DFMC_ISPTRG_ISPGO_Msk;

    while (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk) { }


    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        ret = -1;
    }

    do
    {
        DFMC->ISPCMD = DFMC_ISPCMD_READ_ALL1;
        DFMC->ISPADDR    = u32addr;
        DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
        while (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk) { }
    }
    while (DFMC->ISPDAT == 0UL);

    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        ret = -1;
    }

    if (DFMC->ISPDAT == DREAD_ALLONE_YES)
    {
        ret = DFMC->ISPDAT;
    }

    if (DFMC->ISPDAT == DREAD_ALLONE_NOT)
    {
        ret = DFMC->ISPDAT;
    }
    return ret;
}

/*@}*/ /* end of group DFMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group DFMC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


