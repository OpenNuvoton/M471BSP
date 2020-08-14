/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 20/07/13 11:22a $
 * @brief    M471 series FMC driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/


/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/


/**
  * @brief Disable FMC ISP function.
  * @return None
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief     Config XOM Region
  * @param[in] u32XomNum    The XOM number(0~3)
  * @param[in] u32XomBase   The XOM region base address.
  * @param[in] u8XomPage    The XOM page number of region size.
  *
  * @retval    0   Success
  * @retval    1   XOM is has already actived.
  * @retval   -1   Program failed.
  * @retval   -2   Invalid XOM number.
  *
  * @details  Program XOM base address and XOM size(page)
  */
int32_t FMC_ConfigXOM(uint32_t u32XomNum, uint32_t u32XomBase, uint8_t u8XomPage)
{
    int32_t  ret = 0;

    if(u32XomNum >= 4UL)
    {
        ret = -2;
    }

    if(ret == 0)
    {
        ret = FMC_GetXOMState(u32XomNum);
    }

    if(ret == 0)
    {
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (u32XomNum * 0x10u);
        FMC->ISPDAT = u32XomBase;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}

        if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            ret = -1;
        }
    }

    if(ret == 0)
    {
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (u32XomNum * 0x10u + 0x04u);
        FMC->ISPDAT = u8XomPage;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}

        if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            ret = -1;
        }
    }

    if(ret == 0)
    {
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (u32XomNum * 0x10u + 0x08u);
        FMC->ISPDAT = 0u;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}

        if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            ret = -1;
        }
    }

    return ret;
}

/**
  * @brief Execute FMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 2048 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 2048 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    int32_t  ret = 0;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
    }
    return ret;
}

/**
  * @brief Execute FMC_ISPCMD_BANK_ERASE command to erase a flash block.
 * @param[in]  u32BankAddr Base address of the flash bank to be erased.
 *
 * @return     ISP bank erase success or not.
 * @retval     0  Success
 * @retval    -1  Erase failed
 */
int32_t FMC_Erase_Bank(uint32_t u32BankAddr)
{
    int32_t ret = 0;

    FMC->ISPCMD = FMC_ISPCMD_BANK_ERASE;
    FMC->ISPADDR = u32BankAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
    }
    return ret;
}

/**
  * @brief  Execute Erase XOM Region
  *
  * @param[in]  u32XomNum  The XOMRn(n=0~3)
  *
  * @return   XOM erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  * @retval   -2  Invalid XOM number.
  *
  * @details Execute FMC_ISPCMD_PAGE_ERASE command to erase XOM.
  */
int32_t FMC_EraseXOM(uint32_t u32XomNum)
{
    uint32_t u32Addr = 0;
    int32_t i32Active, err = 0;

    if(u32XomNum >= 4UL)
    {
        err = -2;
    }

    if(err == 0)
    {
        i32Active = FMC_GetXOMState(u32XomNum);

        if(i32Active)
        {
            switch(u32XomNum)
            {
            case 0u:
                u32Addr = (FMC->XOMR0STS & 0xFFFFFF00u) >> 8u;
                break;
            case 1u:
                u32Addr = (FMC->XOMR1STS & 0xFFFFFF00u) >> 8u;
                break;
            case 2u:
                u32Addr = (FMC->XOMR2STS & 0xFFFFFF00u) >> 8u;
                break;
            case 3u:
                u32Addr = (FMC->XOMR3STS & 0xFFFFFF00u) >> 8u;
                break;
            default:
                break;
            }
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPDAT = 0x55aa03u;
            FMC->ISPTRG = 0x1u;
#if ISBEN
            __ISB();
#endif
            while(FMC->ISPTRG) {}

            /* Check ISPFF flag to know whether erase OK or fail. */
            if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
            {
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
                err = -1;
            }
        }
        else
        {
            err = -1;
        }
    }
    return err;
}

/**
  * @brief  Check the XOM is actived or not.
  *
  * @param[in] u32XomNum    The xom number(0~3).
  *
  * @retval    1   XOM is actived.
  * @retval    0   XOM is not actived.
  * @retval   -2   Invalid XOM number.
  *
  * @details To get specify XOMRn(n=0~3) active status
  */
int32_t FMC_GetXOMState(uint32_t u32XomNum)
{
    uint32_t u32act;
    int32_t  ret = 0;

    if(u32XomNum >= 4UL)
    {
        ret = -2;
    }

    if(ret >= 0)
    {
        u32act = (((FMC->XOMSTS) & 0xful) & (1ul << u32XomNum)) >> u32XomNum;
        ret = (int32_t)u32act;
    }
    return ret;
}

/**
  * @brief Get the current boot source.
  * @return The current boot source.
  * @retval   0  Is boot from APROM.
  * @retval   1  Is boot from LDROM.
  */
int32_t FMC_GetBootSource (void)
{
    int32_t  ret = 0;

    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
    {
        ret = 1;
    }

    return ret;
}

/**
  * @brief Enable FMC ISP function
  * @return None
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief Execute FMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

    return FMC->ISPDAT;
}

/**
  * @brief Execute FMC_ISPCMD_READ_64 command to read a double-word from flash.
  * @param[in]  u32addr   Address of the flash location to be read.
  *             It must be a double-word aligned address.
  * @param[out] u32data0  Place holder of word 0 read from flash address u32addr.
  * @param[out] u32data1  Place holder of word 0 read from flash address u32addr+4.
  * @return    0   Success
  * @return   -1   Failed
  */
int32_t FMC_Read_64(uint32_t u32addr, uint32_t * u32data0, uint32_t * u32data1)
{
    int32_t  ret = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_64;
    FMC->ISPADDR    = u32addr;
    FMC->ISPDAT = 0x0UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        ret = -1;
    }
    else
    {
        *u32data0 = FMC->MPDAT0;
        *u32data1 = FMC->MPDAT1;
    }
    return ret;
}

/**
  * @brief      Set boot source from LDROM or APROM after next software reset
  * @param[in]  i32BootSrc
  *                1: Boot from LDROM
  *                0: Boot from APROM
  * @return    None
  * @details   This function is used to switch APROM boot or LDROM boot. User need to call
  *            FMC_SetBootSource to select boot source first, then use CPU reset or
  *            System Reset Request to reset system.
  */
void FMC_SetBootSource(int32_t i32BootSrc)
{
    if(i32BootSrc)
    {
        FMC->ISPCTL |= FMC_ISPCTL_BS_Msk; /* Boot from LDROM */
    }
    else
    {
        FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;/* Boot from APROM */
    }
}

/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return None
  */
void FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
}

/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM_64 to program a double-word to flash.
  * @param[in]  u32addr Address of the flash location to be programmed.
  *             It must be a double-word aligned address.
  * @param[in]  u32data0   The word data to be programmed to flash address u32addr.
  * @param[in]  u32data1   The word data to be programmed to flash address u32addr+4.
  * @return    0   Success
  * @return   -1   Failed
  */
int32_t FMC_Write8Bytes(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1)
{
    int32_t  ret = 0;

    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_64;
    FMC->ISPADDR = u32addr;
    FMC->MPDAT0  = u32data0;
    FMC->MPDAT1  = u32data1;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        ret = -1;
    }
    return ret;
}

/**
  * @brief Execute FMC_ISPCMD_READ command to read User Configuration.
  * @param[out]  u32Config A three-word array.
  *              u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count Available word count in u32Config.
  * @return Success or not.
  * @retval    0  Success.
  * @retval   -1  Invalid parameter.
  */
int32_t FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count)
{
    int32_t   ret = 0;

    u32Config[0] = FMC_Read(FMC_CONFIG_BASE);

    if (u32Count > 3UL)
    {
        ret = -1;
    }
    else
    {
        if(u32Count > 1UL)
        {
            u32Config[1] = FMC_Read(FMC_CONFIG_BASE+4UL);
        }
        if(u32Count > 2UL)
        {
            u32Config[2] = FMC_Read(FMC_CONFIG_BASE+8UL);
        }
    }
    return ret;
}


/**
  * @brief Execute ISP commands to erase then write User Configuration.
  * @param[in] u32Config   A two-word array.
  *            u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count    The number of User Configuration words to be written.
  * @return Success or not.
  * @retval    0   Success
  * @retval   -1   Failed
  */
int32_t FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count)
{
    int   i;

    FMC_ENABLE_CFG_UPDATE();
    FMC_Erase(FMC_CONFIG_BASE);

    if ((FMC_Read(FMC_CONFIG_BASE) != 0xFFFFFFFF) || (FMC_Read(FMC_CONFIG_BASE+4) != 0xFFFFFFFF) ||
            (FMC_Read(FMC_CONFIG_BASE+8) != 0xFFFFFF5A))
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    for (i = 0; i < u32Count; i++)
    {
        FMC_Write(FMC_CONFIG_BASE+i*4UL, u32Config[i]);

        if (FMC_Read(FMC_CONFIG_BASE+i*4UL) != u32Config[i])
        {
            FMC_DISABLE_CFG_UPDATE();
            return -1;
        }
    }

    FMC_DISABLE_CFG_UPDATE();
    return 0;
}

/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 2048 bytes.
  * @return Success or not.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter.
  */
uint32_t  FMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
    uint32_t   ret;

    if ((u32addr % 2048UL) || (u32count % 2048UL))
    {
        ret = 0xFFFFFFFF;
    }
    else
    {
        FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
        FMC->ISPADDR = u32addr;
        FMC->ISPDAT  = u32count;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

        FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
        FMC->ISPADDR    = u32addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

        while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

        ret = FMC->ISPDAT;
    }

    return ret;
}


/**
  * @brief Run flash all one verification and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 2048 bytes.
  * @retval   READ_ALLONE_YES      The contents of verified flash area are 0xFFFFFFFF.
  * @retval   READ_ALLONE_NOT  Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   READ_ALLONE_CMD_FAIL  Unexpected error occurred.
  */
uint32_t  FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count)
{
    uint32_t  ret = READ_ALLONE_CMD_FAIL;

    FMC->ISPSTS = 0x80UL;   /* clear check all one bit */

    FMC->ISPCMD   = FMC_ISPCMD_RUN_ALL1;
    FMC->ISPADDR  = u32addr;
    FMC->ISPDAT   = u32count;
    FMC->ISPTRG   = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    do
    {
        FMC->ISPCMD = FMC_ISPCMD_READ_ALL1;
        FMC->ISPADDR    = u32addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }
    }
    while (FMC->ISPDAT == 0UL);

    if (FMC->ISPDAT == READ_ALLONE_YES)
    {
        ret = FMC->ISPDAT;
    }

    if (FMC->ISPDAT == READ_ALLONE_NOT)
    {
        ret = FMC->ISPDAT;
    }

    return ret;
}

/**
 * @brief      Write Multi-Word bytes to flash
 *
 * @param[in]  u32Addr    Start flash address in APROM where the data chunk to be programmed into.
 *                        This address must be 8-bytes aligned to flash address.
 * @param[in]  pu32Buf    Buffer that carry the data chunk.
 * @param[in]  u32Len     Length of the data chunk in bytes.
 *
 * @retval     >=0  Number of data bytes were programmed.
 * @return     -1   Invalid address.
 *
 * @detail     Program Multi-Word data into specified address of flash.
 */
#if defined ( __CC_ARM )
#pragma arm section code="fastcode"
int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len)

#elif defined ( __ICCARM__ )
int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len) @ "fastcode"

#elif defined ( __GNUC__ )
#pragma GCC push_options
#pragma GCC optimize ("O0")
__attribute__ ((used, long_call, section(".fastcode"))) int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len)

#else
int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len)
#endif
{

    uint32_t i, idx, u32OnProg, retval = 0;
    int32_t err;

    if ((u32Addr % 8) != 0)
    {
        return -1;
    }

    idx = 0u;
    FMC->ISPCMD = FMC_ISPCMD_MULTI_PROG;
    FMC->ISPADDR = u32Addr;
    retval += 16;
    do
    {
        err = 0;
        u32OnProg = 1u;
        FMC->MPDAT0 = pu32Buf[idx + 0u];
        FMC->MPDAT1 = pu32Buf[idx + 1u];
        FMC->MPDAT2 = pu32Buf[idx + 2u];
        FMC->MPDAT3 = pu32Buf[idx + 3u];
        FMC->ISPTRG = 0x1u;
        idx += 4u;

        for (i = idx; i < (FMC_MULTI_WORD_PROG_LEN / 4u); i += 4u) /* Max data length is 256 bytes (512/4 words)*/
        {
            __set_PRIMASK(1u); /* Mask interrupt to avoid status check coherence error*/
            do
            {
                if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0u)
                {
                    __set_PRIMASK(0u);

                    FMC->ISPADDR = FMC->MPADDR & (~0xful);
                    idx = (FMC->ISPADDR - u32Addr) / 4u;
                    err = -1;
                }
            }
            while ((FMC->MPSTS & (3u << FMC_MPSTS_D0_Pos)) && (err == 0));

            if (err == 0)
            {
                retval += 8;

                /* Update new data for D0 */
                FMC->MPDAT0 = pu32Buf[i];
                FMC->MPDAT1 = pu32Buf[i + 1u];
                do
                {
                    if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0u)
                    {
                        __set_PRIMASK(0u);
                        FMC->ISPADDR = FMC->MPADDR & (~0xful);
                        idx = (FMC->ISPADDR - u32Addr) / 4u;
                        err = -1;
                    }
                }
                while ((FMC->MPSTS & (3u << FMC_MPSTS_D2_Pos)) && (err == 0));

                if (err == 0)
                {
                    retval += 8;

                    /* Update new data for D2*/
                    FMC->MPDAT2 = pu32Buf[i + 2u];
                    FMC->MPDAT3 = pu32Buf[i + 3u];
                    __set_PRIMASK(0u);
                }
            }

            if (err < 0)
            {
                break;
            }
        }
        if (err == 0)
        {
            u32OnProg = 0u;
            while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }
        }
    }
    while (u32OnProg);
    return retval;
}
#if defined ( __CC_ARM )
#pragma arm section

#elif defined ( __GNUC__ )
#pragma GCC pop_options

#endif

/*@}*/ /* end of group FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group FMC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


