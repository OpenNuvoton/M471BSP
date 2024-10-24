/**************************************************************************//**
 * @file     system_M471.c
 * @version  V1.000
 * @brief    CMSIS Cortex-M4 Core Peripheral Access Layer Source File for M471
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock  = __SYSTEM_CLOCK;    /*!< System Clock Frequency (Core Clock)*/
uint32_t CyclesPerUs      = (__HSI / 1000000UL); /* Cycles per micro second */
uint32_t PllClock         = __HSI;             /*!< PLL Output Clock Frequency         */
uint32_t gau32ClkSrcTbl[] = {__HXT, __LXT, 0UL, __LIRC, 0UL, 0UL, 0UL, __HIRC};

/**
  * @brief      Set PF.2 and PF.3 to input mode
  * @param      None
  * @return     None
  * @details    GPIO default state could be configured as input or quasi through user config.
  *             To use HXT, PF.2 and PF.3 must not set as quasi mode. This function changes
  *             PF.2 and PF.3 to input mode no matter which mode they are working at.
  */
static __INLINE void HXTInit(void)
{
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

}

/**
 * @brief  Initialize the System
 *
 * @param  none
 * @return none
 */
void SystemInit (void)
{
    /* Add your system initialize code here.
       Do not use global variables because this function is called before
       reaching pre-main. RW section maybe overwritten afterwards.          */


    /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                   (3UL << 11*2)  );               /* set CP11 Full Access */
#endif

    /* Set access cycle for CPU @ 120MHz */
    FMC->CYCCTL = (FMC->CYCCTL & ~FMC_CYCCTL_CYCLE_Msk) | (6 << FMC_CYCCTL_CYCLE_Pos);

    HXTInit();

    /* Enable ICLKON bit by BSP. */
    /* This bit can be disabled to save system power if no special application concern. */
    GPIO->DBCTL |= GPIO_DBCTL_ICLKON_Msk;
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
