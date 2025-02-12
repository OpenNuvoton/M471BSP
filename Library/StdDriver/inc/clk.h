/**************************************************************************//**
 * @file     clk.h
 * @version  V0.10
 * $Revision: 3 $
 * $Date: 20/07/15 11:39a $
 * @brief    M471 Series Clock Controller (CLK) Driver Header File
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

/** @addtogroup CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/


#define FREQ_4MHZ           4000000     /*!< Define frequency macro   4MHz  \hideinitializer */
#define FREQ_8MHZ           8000000     /*!< Define frequency macro   8MHz  \hideinitializer */
#define FREQ_12MHZ         12000000     /*!< Define frequency macro  12MHz  \hideinitializer */
#define FREQ_16MHZ         16000000     /*!< Define frequency macro  16MHz  \hideinitializer */
#define FREQ_24MHZ         24000000     /*!< Define frequency macro  24MHz  \hideinitializer */
#define FREQ_25MHZ         25000000     /*!< Define frequency macro  25MHz  \hideinitializer */
#define FREQ_32MHZ         32000000     /*!< Define frequency macro  32MHz  \hideinitializer */
#define FREQ_48MHZ         48000000     /*!< Define frequency macro  48MHz  \hideinitializer */
#define FREQ_50MHZ         50000000     /*!< Define frequency macro  50MHz  \hideinitializer */
#define FREQ_51MHZ         51000000     /*!< Define frequency macro  51MHz  \hideinitializer */
#define FREQ_64MHZ         64000000     /*!< Define frequency macro  64MHz  \hideinitializer */
#define FREQ_68MHZ         68000000     /*!< Define frequency macro  68MHz  \hideinitializer */
#define FREQ_72MHZ         72000000     /*!< Define frequency macro  72MHz  \hideinitializer */
#define FREQ_96MHZ         96000000     /*!< Define frequency macro  96MHz  \hideinitializer */
#define FREQ_100MHZ       100000000     /*!< Define frequency macro 100MHz  \hideinitializer */
#define FREQ_120MHZ       120000000     /*!< Define frequency macro 120MHz  \hideinitializer */
#define FREQ_144MHZ       144000000     /*!< Define frequency macro 144MHz  \hideinitializer */
#define FREQ_240MHZ       240000000     /*!< Define frequency macro 240MHz  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCTL constant definitions.  (Write-protection)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PWRCTL_HXTGAIN_L0   (0)     /*!< Setting HXT Gain Control to level 0 for lower than 4MHz external crystal   \hideinitializer */
#define CLK_PWRCTL_HXTGAIN_L1   (1)     /*!< Setting HXT Gain Control to level 1 for  4MHz ~  8MHz external crystal \hideinitializer */
#define CLK_PWRCTL_HXTGAIN_L2   (2)     /*!< Setting HXT Gain Control to level 2 for  8MHz ~ 12MHz external crystal \hideinitializer */
#define CLK_PWRCTL_HXTGAIN_L3   (3)     /*!< Setting HXT Gain Control to level 3 for 12MHz ~ 16MHz external crystal \hideinitializer */
#define CLK_PWRCTL_HXTGAIN_L4   (4)     /*!< Setting HXT Gain Control to level 4 for 16MHz ~ 24MHz external crystal \hideinitializer */
#define CLK_PWRCTL_HXTGAIN_L5   (5)     /*!< Setting HXT Gain Control to level 5                                    \hideinitializer */
#define CLK_PWRCTL_HXTGAIN_L6   (6)     /*!< Setting HXT Gain Control to level 6                                    \hideinitializer */
#define CLK_PWRCTL_HXTGAIN_L7   (7)     /*!< Setting HXT Gain Control to level 7 for 24MHz ~ 32MHz external crystal \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.  (Write-protection)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HCLKSEL_HXT           (0x00UL<<CLK_CLKSEL0_HCLKSEL_Pos)     /*!< Setting clock source as external X'tal                 \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LXT           (0x01UL<<CLK_CLKSEL0_HCLKSEL_Pos)     /*!< Setting clock source as external X'tal 32.768KHz       \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_PLL           (0x02UL<<CLK_CLKSEL0_HCLKSEL_Pos)     /*!< Setting clock source as PLL output                     \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LIRC          (0x03UL<<CLK_CLKSEL0_HCLKSEL_Pos)     /*!< Setting clock source as internal low speed RC clock    \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_HIRC          (0x07UL<<CLK_CLKSEL0_HCLKSEL_Pos)     /*!< Setting clock source as internal high speed RC clock   \hideinitializer */

#define CLK_CLKSEL0_STCLKSEL_HXT          (0x00UL<<CLK_CLKSEL0_STCLKSEL_Pos)    /*!< Setting SysTick clock source as external X'tal             \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_LXT          (0x01UL<<CLK_CLKSEL0_STCLKSEL_Pos)    /*!< Setting SysTick clock source as external X'tal 32.768KHz   \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_HXT_DIV2     (0x02UL<<CLK_CLKSEL0_STCLKSEL_Pos)    /*!< Setting SysTick clock source as external X'tal/2           \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_HCLK_DIV2    (0x03UL<<CLK_CLKSEL0_STCLKSEL_Pos)    /*!< Setting SysTick clock source as HCLK/2                     \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_HIRC_DIV2    (0x07UL<<CLK_CLKSEL0_STCLKSEL_Pos)    /*!< Setting SysTick clock source as internal high speed RC clock/2     \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_HCLK         (0x01UL<<SysTick_CTRL_CLKSOURCE_Pos)  /*!< Setting SysTick clock source as HCLK                       \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL1_WDTSEL_LXT           (0x1UL<<CLK_CLKSEL1_WDTSEL_Pos)        /*!< Setting WDT clock source as external X'tal 32.768KHz       \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  (0x2UL<<CLK_CLKSEL1_WDTSEL_Pos)        /*!< Setting WDT clock source as HCLK/2048                      \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_LIRC          (0x3UL<<CLK_CLKSEL1_WDTSEL_Pos)        /*!< Setting WDT clock source as internal low speed RC clock    \hideinitializer */

#define CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048 (0x2UL<<CLK_CLKSEL1_WWDTSEL_Pos)       /*!< Setting WWDT clock source as HCLK/2048                     \hideinitializer */
#define CLK_CLKSEL1_WWDTSEL_LIRC         (0x3UL<<CLK_CLKSEL1_WWDTSEL_Pos)       /*!< Setting WWDT clock source as internal low speed RC clock   \hideinitializer */

#define CLK_CLKSEL1_CLKOSEL_HXT          (0x0UL<<CLK_CLKSEL1_CLKOSEL_Pos)       /*!< Setting CLKO clock source as external X'tal            \hideinitializer */
#define CLK_CLKSEL1_CLKOSEL_LXT          (0x1UL<<CLK_CLKSEL1_CLKOSEL_Pos)       /*!< Setting CLKO clock source as external X'tal 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_CLKOSEL_HCLK         (0x2UL<<CLK_CLKSEL1_CLKOSEL_Pos)       /*!< Setting CLKO clock source as HCLK                      \hideinitializer */
#define CLK_CLKSEL1_CLKOSEL_HIRC         (0x3UL<<CLK_CLKSEL1_CLKOSEL_Pos)       /*!< Setting CLKO clock source as external internal high speed RC clock \hideinitializer */

#define CLK_CLKSEL1_TMR0SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR0SEL_Pos)       /*!< Setting Timer 0 clock source as external X'tal             \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR0SEL_Pos)       /*!< Setting Timer 0 clock source as external X'tal 32.768KHz   \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_PCLK0        (0x2UL<<CLK_CLKSEL1_TMR0SEL_Pos)       /*!< Setting Timer 0 clock source as PCLK0                      \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_EXT_TRG      (0x3UL<<CLK_CLKSEL1_TMR0SEL_Pos)       /*!< Setting Timer 0 clock source as external trigger           \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_LIRC         (0x5UL<<CLK_CLKSEL1_TMR0SEL_Pos)       /*!< Setting Timer 0 clock source as internal low speed RC clock    \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_HIRC         (0x7UL<<CLK_CLKSEL1_TMR0SEL_Pos)       /*!< Setting Timer 0 clock source as internal high speed RC clock   \hideinitializer */

#define CLK_CLKSEL1_TMR1SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR1SEL_Pos)       /*!< Setting Timer 1 clock source as external X'tal             \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR1SEL_Pos)       /*!< Setting Timer 1 clock source as external X'tal 32.768KHz   \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_PCLK0        (0x2UL<<CLK_CLKSEL1_TMR1SEL_Pos)       /*!< Setting Timer 1 clock source as PCLK0                      \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_EXT_TRG      (0x3UL<<CLK_CLKSEL1_TMR1SEL_Pos)       /*!< Setting Timer 1 clock source as external trigger           \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_LIRC         (0x5UL<<CLK_CLKSEL1_TMR1SEL_Pos)       /*!< Setting Timer 1 clock source as internal low speed RC clock    \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_HIRC         (0x7UL<<CLK_CLKSEL1_TMR1SEL_Pos)       /*!< Setting Timer 1 clock source as internal high speed RC clock   \hideinitializer */

#define CLK_CLKSEL1_TMR2SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR2SEL_Pos)       /*!< Setting Timer 2 clock source as external X'tal             \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR2SEL_Pos)       /*!< Setting Timer 2 clock source as external X'tal 32.768KHz   \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_PCLK1        (0x2UL<<CLK_CLKSEL1_TMR2SEL_Pos)       /*!< Setting Timer 2 clock source as PCLK1                      \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_EXT_TRG      (0x3UL<<CLK_CLKSEL1_TMR2SEL_Pos)       /*!< Setting Timer 2 clock source as external trigger           \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_LIRC         (0x5UL<<CLK_CLKSEL1_TMR2SEL_Pos)       /*!< Setting Timer 2 clock source as internal low speed RC clock    \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_HIRC         (0x7UL<<CLK_CLKSEL1_TMR2SEL_Pos)       /*!< Setting Timer 2 clock source as internal high speed RC clock   \hideinitializer */

#define CLK_CLKSEL1_TMR3SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR3SEL_Pos)       /*!< Setting Timer 3 clock source as external X'tal             \hideinitializer */
#define CLK_CLKSEL1_TMR3SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR3SEL_Pos)       /*!< Setting Timer 3 clock source as external X'tal 32.768KHz   \hideinitializer */
#define CLK_CLKSEL1_TMR3SEL_PCLK1        (0x2UL<<CLK_CLKSEL1_TMR3SEL_Pos)       /*!< Setting Timer 3 clock source as PCLK1                      \hideinitializer */
#define CLK_CLKSEL1_TMR3SEL_EXT_TRG      (0x3UL<<CLK_CLKSEL1_TMR3SEL_Pos)       /*!< Setting Timer 3 clock source as external trigger           \hideinitializer */
#define CLK_CLKSEL1_TMR3SEL_LIRC         (0x5UL<<CLK_CLKSEL1_TMR3SEL_Pos)       /*!< Setting Timer 3 clock source as internal low speed RC clock    \hideinitializer */
#define CLK_CLKSEL1_TMR3SEL_HIRC         (0x7UL<<CLK_CLKSEL1_TMR3SEL_Pos)       /*!< Setting Timer 3 clock source as internal high speed RC clock   \hideinitializer */

#define CLK_CLKSEL1_UART0SEL_HXT         (0x0UL<<CLK_CLKSEL1_UART0SEL_Pos)      /*!< Setting UART0 clock source as external X'tal   \hideinitializer */
#define CLK_CLKSEL1_UART0SEL_PLL         (0x1UL<<CLK_CLKSEL1_UART0SEL_Pos)      /*!< Setting UART0 clock source as external PLL     \hideinitializer */
#define CLK_CLKSEL1_UART0SEL_LXT         (0x2UL<<CLK_CLKSEL1_UART0SEL_Pos)      /*!< Setting UART0 clock source as external X'tal 32.768KHz     \hideinitializer */
#define CLK_CLKSEL1_UART0SEL_HIRC        (0x3UL<<CLK_CLKSEL1_UART0SEL_Pos)      /*!< Setting UART0 clock source as external internal high speed RC clock    \hideinitializer */

#define CLK_CLKSEL1_UART1SEL_HXT         (0x0UL<<CLK_CLKSEL1_UART1SEL_Pos)      /*!< Setting UART1 clock source as external X'tal   \hideinitializer */
#define CLK_CLKSEL1_UART1SEL_PLL         (0x1UL<<CLK_CLKSEL1_UART1SEL_Pos)      /*!< Setting UART1 clock source as external PLL     \hideinitializer */
#define CLK_CLKSEL1_UART1SEL_LXT         (0x2UL<<CLK_CLKSEL1_UART1SEL_Pos)      /*!< Setting UART1 clock source as external X'tal 32.768KHz     \hideinitializer */
#define CLK_CLKSEL1_UART1SEL_HIRC        (0x3UL<<CLK_CLKSEL1_UART1SEL_Pos)      /*!< Setting UART1 clock source as external internal high speed RC clock    \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL2 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL2_EPWM0SEL_HCLK        (0x0UL<<CLK_CLKSEL2_EPWM0SEL_Pos)      /*!< Setting EPWM0 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL2_EPWM0SEL_PCLK0       (0x1UL<<CLK_CLKSEL2_EPWM0SEL_Pos)      /*!< Setting EPWM0 clock source as PCLK0 \hideinitializer */

#define CLK_CLKSEL2_EPWM1SEL_HCLK        (0x0UL<<CLK_CLKSEL2_EPWM1SEL_Pos)      /*!< Setting EPWM1 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL2_EPWM1SEL_PCLK1       (0x1UL<<CLK_CLKSEL2_EPWM1SEL_Pos)      /*!< Setting EPWM1 clock source as PCLK1 \hideinitializer */

#define CLK_CLKSEL2_SPI0SEL_HXT          (0x0UL<<CLK_CLKSEL2_SPI0SEL_Pos)       /*!< Setting SPI0 clock source as HXT    \hideinitializer */
#define CLK_CLKSEL2_SPI0SEL_PLL          (0x1UL<<CLK_CLKSEL2_SPI0SEL_Pos)       /*!< Setting SPI0 clock source as PLL    \hideinitializer */
#define CLK_CLKSEL2_SPI0SEL_PCLK1        (0x2UL<<CLK_CLKSEL2_SPI0SEL_Pos)       /*!< Setting SPI0 clock source as PCLK1  \hideinitializer */
#define CLK_CLKSEL2_SPI0SEL_HIRC         (0x3UL<<CLK_CLKSEL2_SPI0SEL_Pos)       /*!< Setting SPI0 clock source as HIRC   \hideinitializer */

#define CLK_CLKSEL2_SPI1SEL_HXT          (0x0UL<<CLK_CLKSEL2_SPI1SEL_Pos)       /*!< Setting SPI1 clock source as HXT    \hideinitializer */
#define CLK_CLKSEL2_SPI1SEL_PLL          (0x1UL<<CLK_CLKSEL2_SPI1SEL_Pos)       /*!< Setting SPI1 clock source as PLL    \hideinitializer */
#define CLK_CLKSEL2_SPI1SEL_PCLK0        (0x2UL<<CLK_CLKSEL2_SPI1SEL_Pos)       /*!< Setting SPI1 clock source as PCLK0  \hideinitializer */
#define CLK_CLKSEL2_SPI1SEL_HIRC         (0x3UL<<CLK_CLKSEL2_SPI1SEL_Pos)       /*!< Setting SPI1 clock source as HIRC   \hideinitializer */

#define CLK_CLKSEL2_BPWM0SEL_HCLK        (0x0UL<<CLK_CLKSEL2_BPWM0SEL_Pos)      /*!< Setting BPWM0 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL2_BPWM0SEL_PCLK0       (0x1UL<<CLK_CLKSEL2_BPWM0SEL_Pos)      /*!< Setting BPWM0 clock source as PCLK0 \hideinitializer */

#define CLK_CLKSEL2_BPWM1SEL_HCLK        (0x0UL<<CLK_CLKSEL2_BPWM1SEL_Pos)      /*!< Setting BPWM1 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL2_BPWM1SEL_PCLK1       (0x1UL<<CLK_CLKSEL2_BPWM1SEL_Pos)      /*!< Setting BPWM1 clock source as PCLK1 \hideinitializer */

#define CLK_CLKSEL2_CIR0SEL_HXT          (0x0UL<<CLK_CLKSEL2_CIR0SEL_Pos)       /*!< Setting CIR0 clock source as HXT    \hideinitializer */
#define CLK_CLKSEL2_CIR0SEL_LXT          (0x1UL<<CLK_CLKSEL2_CIR0SEL_Pos)       /*!< Setting CIR0 clock source as LXT    \hideinitializer */
#define CLK_CLKSEL2_CIR0SEL_TM0          (0x2UL<<CLK_CLKSEL2_CIR0SEL_Pos)       /*!< Setting CIR0 clock source as TM0    \hideinitializer */
#define CLK_CLKSEL2_CIR0SEL_LIRC         (0x3UL<<CLK_CLKSEL2_CIR0SEL_Pos)       /*!< Setting CIR0 clock source as LIRC   \hideinitializer */
#define CLK_CLKSEL2_CIR0SEL_HIRC         (0x4UL<<CLK_CLKSEL2_CIR0SEL_Pos)       /*!< Setting CIR0 clock source as HIRC   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL3 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL3_UART2SEL_HXT         (0x0UL<<CLK_CLKSEL3_UART2SEL_Pos)      /*!< Setting UART2 clock source as external X'tal   \hideinitializer */
#define CLK_CLKSEL3_UART2SEL_PLL         (0x1UL<<CLK_CLKSEL3_UART2SEL_Pos)      /*!< Setting UART2 clock source as external PLL     \hideinitializer */
#define CLK_CLKSEL3_UART2SEL_LXT         (0x2UL<<CLK_CLKSEL3_UART2SEL_Pos)      /*!< Setting UART2 clock source as external X'tal 32.768KHz \hideinitializer */
#define CLK_CLKSEL3_UART2SEL_HIRC        (0x3UL<<CLK_CLKSEL3_UART2SEL_Pos)      /*!< Setting UART2 clock source as external internal high speed RC clock    \hideinitializer */

#define CLK_CLKSEL3_UART3SEL_HXT         (0x0UL<<CLK_CLKSEL3_UART3SEL_Pos)      /*!< Setting UART3 clock source as external X'tal   \hideinitializer */
#define CLK_CLKSEL3_UART3SEL_PLL         (0x1UL<<CLK_CLKSEL3_UART3SEL_Pos)      /*!< Setting UART3 clock source as external PLL     \hideinitializer */
#define CLK_CLKSEL3_UART3SEL_LXT         (0x2UL<<CLK_CLKSEL3_UART3SEL_Pos)      /*!< Setting UART3 clock source as external X'tal 32.768KHz \hideinitializer */
#define CLK_CLKSEL3_UART3SEL_HIRC        (0x3UL<<CLK_CLKSEL3_UART3SEL_Pos)      /*!< Setting UART3 clock source as external internal high speed RC clock    \hideinitializer */

#define CLK_CLKSEL3_UART4SEL_HXT         (0x0UL<<CLK_CLKSEL3_UART4SEL_Pos)      /*!< Setting UART4 clock source as external X'tal   \hideinitializer */
#define CLK_CLKSEL3_UART4SEL_PLL         (0x1UL<<CLK_CLKSEL3_UART4SEL_Pos)      /*!< Setting UART4 clock source as external PLL     \hideinitializer */
#define CLK_CLKSEL3_UART4SEL_LXT         (0x2UL<<CLK_CLKSEL3_UART4SEL_Pos)      /*!< Setting UART4 clock source as external X'tal 32.768KHz \hideinitializer */
#define CLK_CLKSEL3_UART4SEL_HIRC        (0x3UL<<CLK_CLKSEL3_UART4SEL_Pos)      /*!< Setting UART4 clock source as external internal high speed RC clock    \hideinitializer */

#define CLK_CLKSEL3_UART5SEL_HXT         (0x0UL<<CLK_CLKSEL3_UART5SEL_Pos)      /*!< Setting UART5 clock source as external X'tal   \hideinitializer */
#define CLK_CLKSEL3_UART5SEL_PLL         (0x1UL<<CLK_CLKSEL3_UART5SEL_Pos)      /*!< Setting UART5 clock source as external PLL     \hideinitializer */
#define CLK_CLKSEL3_UART5SEL_LXT         (0x2UL<<CLK_CLKSEL3_UART5SEL_Pos)      /*!< Setting UART5 clock source as external X'tal 32.768KHz \hideinitializer */
#define CLK_CLKSEL3_UART5SEL_HIRC        (0x3UL<<CLK_CLKSEL3_UART5SEL_Pos)      /*!< Setting UART5 clock source as external internal high speed RC clock    \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV0_HCLK(x)  (((x)-1) << CLK_CLKDIV0_HCLKDIV_Pos)   /*!< CLKDIV Setting for HCLK clock divider.  It could be 1~16   \hideinitializer */
#define CLK_CLKDIV0_UART0(x) (((x)-1) << CLK_CLKDIV0_UART0DIV_Pos)  /*!< CLKDIV Setting for UART0 clock divider. It could be 1~16   \hideinitializer */
#define CLK_CLKDIV0_UART1(x) (((x)-1) << CLK_CLKDIV0_UART1DIV_Pos)  /*!< CLKDIV Setting for UART1 clock divider. It could be 1~16   \hideinitializer */
#define CLK_CLKDIV0_EADC(x)  (((x)-1) << CLK_CLKDIV0_EADCDIV_Pos)   /*!< CLKDIV Setting for EADC clock divider.  It could be 1~256  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV4 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV4_UART2(x) (((x)-1) << CLK_CLKDIV4_UART2DIV_Pos)  /*!< CLKDIV Setting for UART2 clock divider. It could be 1~16   \hideinitializer */
#define CLK_CLKDIV4_UART3(x) (((x)-1) << CLK_CLKDIV4_UART3DIV_Pos)  /*!< CLKDIV Setting for UART3 clock divider. It could be 1~16   \hideinitializer */
#define CLK_CLKDIV4_UART4(x) (((x)-1) << CLK_CLKDIV4_UART4DIV_Pos)  /*!< CLKDIV Setting for UART4 clock divider. It could be 1~16   \hideinitializer */
#define CLK_CLKDIV4_UART5(x) (((x)-1) << CLK_CLKDIV4_UART5DIV_Pos)  /*!< CLKDIV Setting for UART5 clock divider. It could be 1~16   \hideinitializer */
#define CLK_CLKDIV4_TRACE(x) (((x)-1) << CLK_CLKDIV4_TRACEDIV_Pos)  /*!< CLKDIV Setting for TRACE clock divider. It could be 1~256   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  PCLKDIV constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PCLKDIV_APB0DIV_DIV1       (0x0UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider  1. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV2       (0x1UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider  2. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV4       (0x2UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider  4. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV8       (0x3UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider  8. \hideinitializer */
#define CLK_PCLKDIV_APB0DIV_DIV16      (0x4UL<<CLK_PCLKDIV_APB0DIV_Pos)     /*!< PCLKDIV Setting for APB0 clock divider 16. \hideinitializer */

#define CLK_PCLKDIV_APB1DIV_DIV1       (0x0UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  1. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV2       (0x1UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  2. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV4       (0x2UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  4. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV8       (0x3UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider  8. \hideinitializer */
#define CLK_PCLKDIV_APB1DIV_DIV16      (0x4UL<<CLK_PCLKDIV_APB1DIV_Pos)     /*!< PCLKDIV Setting for APB1 clock divider 16. \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  PLLCTL constant definitions. PLL = FIN * NF / NR / NO                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PLLCTL_PLLSRC_HXT       (0x0UL << CLK_PLLCTL_PLLSRC_Pos)    /*!< For PLL clock source is HXT.  4~12MHz < FIN < 24MHz    \hideinitializer */
#define CLK_PLLCTL_PLLSRC_HIRC_DIV4 (0x1UL << CLK_PLLCTL_PLLSRC_Pos)    /*!< For PLL clock source is HIRC/4. 12 MHz< FIN < 12MHz    \hideinitializer */

#define CLK_PLLCTL_NR(x)        ((x-2)<<CLK_PLLCTL_INDIV_Pos)   /*!< For PLL input divider. x must be constant and 2 <= x <= 7 since constraint 1.6MHz < FIN/NR < 16MHz and NR = INDIV+2 >= 2   \hideinitializer */
#define CLK_PLLCTL_NF(x)        ((x-2)<<CLK_PLLCTL_FBDIV_Pos)   /*!< For PLL feedback divider. x must be constant and 17 <= x/NR <= 41 since constraint 200MHz < FIN*NF/NR < 500MHz.            \hideinitializer */

#define CLK_PLLCTL_NO_1         (0x0UL << CLK_PLLCTL_OUTDIV_Pos)    /*!< For PLL output divider is 1    \hideinitializer */
#define CLK_PLLCTL_NO_2         (0x1UL << CLK_PLLCTL_OUTDIV_Pos)    /*!< For PLL output divider is 2    \hideinitializer */
#define CLK_PLLCTL_NO_4         (0x3UL << CLK_PLLCTL_OUTDIV_Pos)    /*!< For PLL output divider is 4    \hideinitializer */

#define CLK_PLLCTL_64MHz_HXT        (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(64)     | CLK_PLLCTL_NO_4)          /*!< Predefined PLLCTL setting for 64MHz PLL output with HXT(12MHz X'tal)       \hideinitializer */
#define CLK_PLLCTL_68MHz_HXT        (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(68)     | CLK_PLLCTL_NO_4)          /*!< Predefined PLLCTL setting for 68MHz PLL output with HXT(12MHz X'tal)       \hideinitializer */
#define CLK_PLLCTL_72MHz_HXT        (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(72)     | CLK_PLLCTL_NO_4)          /*!< Predefined PLLCTL setting for 72MHz PLL output with HXT(12MHz X'tal)       \hideinitializer */
#define CLK_PLLCTL_96MHz_HXT        (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(96)     | CLK_PLLCTL_NO_4)          /*!< Predefined PLLCTL setting for 96MHz PLL output with HXT(12MHz X'tal)       \hideinitializer */
#define CLK_PLLCTL_120MHz_HXT       (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(4)  | CLK_PLLCTL_NF(160)    | CLK_PLLCTL_NO_4)          /*!< Predefined PLLCTL setting for 120MHz PLL output with HXT(12MHz X'tal)      \hideinitializer */
#define CLK_PLLCTL_144MHz_HXT       (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(72)     | CLK_PLLCTL_NO_2)          /*!< Predefined PLLCTL setting for 144MHz PLL output with HXT(12MHz X'tal)      \hideinitializer */
#define CLK_PLLCTL_64MHz_HIRC_DIV4  (CLK_PLLCTL_PLLSRC_HIRC_DIV4  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(64)     | CLK_PLLCTL_NO_4)    /*!< Predefined PLLCTL setting for 64MHz PLL output with HIRC_DIV4(12MHz IRC)   \hideinitializer */
#define CLK_PLLCTL_68MHz_HIRC_DIV4  (CLK_PLLCTL_PLLSRC_HIRC_DIV4  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(68)     | CLK_PLLCTL_NO_4)    /*!< Predefined PLLCTL setting for 68MHz PLL output with HIRC_DIV4(12MHz IRC)   \hideinitializer */
#define CLK_PLLCTL_72MHz_HIRC_DIV4  (CLK_PLLCTL_PLLSRC_HIRC_DIV4  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(72)     | CLK_PLLCTL_NO_4)    /*!< Predefined PLLCTL setting for 72MHz PLL output with HIRC_DIV4(12MHz IRC)   \hideinitializer */
#define CLK_PLLCTL_96MHz_HIRC_DIV4  (CLK_PLLCTL_PLLSRC_HIRC_DIV4  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(96)     | CLK_PLLCTL_NO_4)    /*!< Predefined PLLCTL setting for 96MHz PLL output with HIRC_DIV4(12MHz IRC)   \hideinitializer */
#define CLK_PLLCTL_120MHz_HIRC_DIV4 (CLK_PLLCTL_PLLSRC_HIRC_DIV4  | CLK_PLLCTL_NR(4)  | CLK_PLLCTL_NF(160)    | CLK_PLLCTL_NO_4)    /*!< Predefined PLLCTL setting for 120MHz PLL output with HIRC_DIV4(12MHz X'tal)\hideinitializer */
#define CLK_PLLCTL_144MHz_HIRC_DIV4 (CLK_PLLCTL_PLLSRC_HIRC_DIV4  | CLK_PLLCTL_NR(3)  | CLK_PLLCTL_NF(72)     | CLK_PLLCTL_NO_2)    /*!< Predefined PLLCTL setting for 144MHz PLL output with HIRC_DIV4(12MHz IRC)  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
/* APBCLK(31:30)|CLKSEL(29:28)|CLKSEL_Msk(27:25) |CLKSEL_Pos(24:20)|CLKDIV(19:18)|CLKDIV_Msk(17:10)|CLKDIV_Pos(9:5)|IP_EN_Pos(4:0) */

#define MODULE_APBCLK(x)        (((x) >>30) & 0x3UL)    /*!< Calculate AHBCLK/APBCLK offset on MODULE index, 0x0:AHBCLK, 0x1:APBCLK0, 0x2:APBCLK1           \hideinitializer */
#define MODULE_CLKSEL(x)        (((x) >>28) & 0x3UL)    /*!< Calculate CLKSEL offset on MODULE index, 0x0:CLKSEL0, 0x1:CLKSEL1, 0x2:CLKSEL2, 0x3:CLKSEL3    \hideinitializer */
#define MODULE_CLKSEL_Msk(x)    (((x) >>25) & 0x7UL)    /*!< Calculate CLKSEL mask offset on MODULE index       \hideinitializer */
#define MODULE_CLKSEL_Pos(x)    (((x) >>20) & 0x1fUL)   /*!< Calculate CLKSEL position offset on MODULE index   \hideinitializer */
#define MODULE_CLKDIV(x)        (((x) >>18) & 0x3UL)    /*!< Calculate CLKDIV offset on MODULE index, 0x0:CLKDIV0, 0x1:CLKDIV1, 0x2:CLKDIV3, 0x3:CLKDIV4    \hideinitializer */
#define MODULE_CLKDIV_Msk(x)    (((x) >>10) & 0xffUL)   /*!< Calculate CLKDIV mask offset on MODULE index       \hideinitializer */
#define MODULE_CLKDIV_Pos(x)    (((x) >>5 ) & 0x1fUL)   /*!< Calculate CLKDIV position offset on MODULE index   \hideinitializer */
#define MODULE_IP_EN_Pos(x)     (((x) >>0 ) & 0x1fUL)   /*!< Calculate APBCLK offset on MODULE index            \hideinitializer */
#define MODULE_NoMsk            0x0                     /*!< Not mask on MODULE index                           \hideinitializer */
#define NA                      MODULE_NoMsk            /*!< Not Available                                      \hideinitializer */

#define MODULE_APBCLK_ENC(x)        (((x) & 0x03UL) << 30)   /*!< MODULE index, 0x0:AHBCLK, 0x1:APBCLK0, 0x2:APBCLK1    \hideinitializer */
#define MODULE_CLKSEL_ENC(x)        (((x) & 0x03UL) << 28)   /*!< CLKSEL offset on MODULE index, 0x0:CLKSEL0, 0x1:CLKSEL1, 0x2:CLKSEL2, 0x3:CLKSEL3     \hideinitializer */
#define MODULE_CLKSEL_Msk_ENC(x)    (((x) & 0x07UL) << 25)   /*!< CLKSEL mask offset on MODULE index                    \hideinitializer */
#define MODULE_CLKSEL_Pos_ENC(x)    (((x) & 0x1fUL) << 20)   /*!< CLKSEL position offset on MODULE index                \hideinitializer */
#define MODULE_CLKDIV_ENC(x)        (((x) & 0x03UL) << 18)   /*!< APBCLK CLKDIV on MODULE index, 0x0:CLKDIV, 0x1:CLKDIV1, 0x2:CLKDIV3, 0x3:CLKDIV4      \hideinitializer */
#define MODULE_CLKDIV_Msk_ENC(x)    (((x) & 0xffUL) << 10)   /*!< CLKDIV mask offset on MODULE index                    \hideinitializer */
#define MODULE_CLKDIV_Pos_ENC(x)    (((x) & 0x1fUL) <<  5)   /*!< CLKDIV position offset on MODULE index                \hideinitializer */
#define MODULE_IP_EN_Pos_ENC(x)     (((x) & 0x1fUL) <<  0)   /*!< AHBCLK/APBCLK offset on MODULE index                  \hideinitializer */


//AHBCLK
#define PDMA_MODULE    (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_PDMACKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< PDMA Module   \hideinitializer */

#define ISP_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_ISPCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< ISP Module    \hideinitializer */

#define CRC_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_CRCCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< CRC Module    \hideinitializer */

#define TRACE_MODULE   (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_TRACECKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)  |MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC( 3)|MODULE_CLKDIV_Msk_ENC(0xFF)|MODULE_CLKDIV_Pos_ENC(CLK_CLKDIV4_TRACEDIV_Pos))     /*!< TRACE Module  \hideinitializer */

#define GPA_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPACKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPA Module  \hideinitializer */

#define GPB_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPBCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPB Module  \hideinitializer */

#define GPC_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPCCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPC Module  \hideinitializer */

#define GPD_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPDCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPD Module  \hideinitializer */

#define GPE_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPECKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPE Module  \hideinitializer */

#define GPF_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPFCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPF Module  \hideinitializer */

#define GPG_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPGCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPG Module  \hideinitializer */

#define GPH_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPHCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPH Module  \hideinitializer */

#define GPI_MODULE     (MODULE_APBCLK_ENC( 0)|MODULE_IP_EN_Pos_ENC(CLK_AHBCLK_GPICKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))     /*!< GPI Module  \hideinitializer */

//APBCLK0
#define WDT_MODULE     (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_WDTCKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC( 3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_WDTSEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< WDT Module     \hideinitializer */

#define WWDT_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_WDTCKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC( 3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_WWDTSEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< WWDT Module    \hideinitializer */

#define RTC_MODULE     (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_RTCCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< RTC Module     \hideinitializer */

#define TMR0_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_TMR0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC( 7)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_TMR0SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< TMR0 Module    \hideinitializer */

#define TMR1_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_TMR1CKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC( 7)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_TMR1SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< TMR1 Module    \hideinitializer */

#define TMR2_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_TMR2CKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC( 7)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_TMR2SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< TMR2 Module    \hideinitializer */

#define TMR3_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_TMR3CKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC( 7)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_TMR3SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< TMR3 Module    \hideinitializer */

#define CLKO_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_CLKOCKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC( 3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_CLKOSEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< CLKO Module    \hideinitializer */

#define ACMP01_MODULE  (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_ACMP01CKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< ACMP Module    \hideinitializer */

#define I2C0_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_I2C0CKEN_Pos) |\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< I2C0 Module    \hideinitializer */

#define I2C1_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_I2C1CKEN_Pos) |\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< I2C1 Module    \hideinitializer */

#define SPI0_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_SPI0CKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 2)|MODULE_CLKSEL_Msk_ENC( 3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL2_SPI0SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< SPI0 Module    \hideinitializer */

#define SPI1_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_SPI1CKEN_Pos) |\
                        MODULE_CLKSEL_ENC( 2)|MODULE_CLKSEL_Msk_ENC( 3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL2_SPI1SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< SPI1 Module    \hideinitializer */

#define UART0_MODULE   (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_UART0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC(  3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_UART0SEL_Pos)|\
                        MODULE_CLKDIV_ENC( 0)|MODULE_CLKDIV_Msk_ENC(0xF)|MODULE_CLKDIV_Pos_ENC(CLK_CLKDIV0_UART0DIV_Pos))   /*!< UART0 Module   \hideinitializer */

#define UART1_MODULE   (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_UART1CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 1)|MODULE_CLKSEL_Msk_ENC(  3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL1_UART1SEL_Pos)|\
                        MODULE_CLKDIV_ENC( 0)|MODULE_CLKDIV_Msk_ENC(0xF)|MODULE_CLKDIV_Pos_ENC(CLK_CLKDIV0_UART1DIV_Pos))   /*!< UART1 Module   \hideinitializer */

#define UART2_MODULE   (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_UART2CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 3)|MODULE_CLKSEL_Msk_ENC(  3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL3_UART2SEL_Pos)|\
                        MODULE_CLKDIV_ENC( 3)|MODULE_CLKDIV_Msk_ENC(0xF)|MODULE_CLKDIV_Pos_ENC(CLK_CLKDIV4_UART2DIV_Pos))   /*!< UART2 Module   \hideinitializer */

#define UART3_MODULE   (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_UART3CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 3)|MODULE_CLKSEL_Msk_ENC(  3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL3_UART3SEL_Pos)|\
                        MODULE_CLKDIV_ENC( 3)|MODULE_CLKDIV_Msk_ENC(0xF)|MODULE_CLKDIV_Pos_ENC(CLK_CLKDIV4_UART3DIV_Pos))   /*!< UART3 Module   \hideinitializer */

#define UART4_MODULE   (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_UART4CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 3)|MODULE_CLKSEL_Msk_ENC(  3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL3_UART4SEL_Pos)|\
                        MODULE_CLKDIV_ENC( 3)|MODULE_CLKDIV_Msk_ENC(0xF)|MODULE_CLKDIV_Pos_ENC(CLK_CLKDIV4_UART4DIV_Pos))   /*!< UART4 Module   \hideinitializer */

#define UART5_MODULE   (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_UART5CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 3)|MODULE_CLKSEL_Msk_ENC(  3)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL3_UART5SEL_Pos)|\
                        MODULE_CLKDIV_ENC( 3)|MODULE_CLKDIV_Msk_ENC(0xF)|MODULE_CLKDIV_Pos_ENC(CLK_CLKDIV4_UART5DIV_Pos))   /*!< UART5 Module   \hideinitializer */

#define EADC_MODULE    (MODULE_APBCLK_ENC( 1)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK0_EADCCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)  |MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC( 0)|MODULE_CLKDIV_Msk_ENC(0xFF)|MODULE_CLKDIV_Pos_ENC(CLK_CLKDIV0_EADCDIV_Pos))   /*!< EADC Module    \hideinitializer */

//APBCLK1
#define DAC_MODULE     (MODULE_APBCLK_ENC( 2)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK1_DACCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)  |MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< DAC Module    \hideinitializer */

#define CIR0_MODULE    (MODULE_APBCLK_ENC( 2)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK1_CIR0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2)|MODULE_CLKSEL_Msk_ENC( 7)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL2_CIR0SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< CIR0 Module    \hideinitializer */

#define EPWM0_MODULE   (MODULE_APBCLK_ENC( 2)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK1_EPWM0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2)|MODULE_CLKSEL_Msk_ENC( 1)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL2_EPWM0SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< EPWM0 Module    \hideinitializer */

#define EPWM1_MODULE   (MODULE_APBCLK_ENC( 2)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK1_EPWM1CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2)|MODULE_CLKSEL_Msk_ENC( 1)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL2_EPWM1SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< EPWM1 Module    \hideinitializer */

#define BPWM0_MODULE   (MODULE_APBCLK_ENC( 2)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK1_BPWM0CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2)|MODULE_CLKSEL_Msk_ENC( 1)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL2_BPWM0SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< BPWM0 Module    \hideinitializer */

#define BPWM1_MODULE   (MODULE_APBCLK_ENC( 2)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK1_BPWM1CKEN_Pos)|\
                        MODULE_CLKSEL_ENC( 2)|MODULE_CLKSEL_Msk_ENC( 1)|MODULE_CLKSEL_Pos_ENC(CLK_CLKSEL2_BPWM1SEL_Pos)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< BPWM1 Module    \hideinitializer */

#define PRNG_MODULE    (MODULE_APBCLK_ENC( 2)|MODULE_IP_EN_Pos_ENC(CLK_APBCLK1_PRNGCKEN_Pos)|\
                        MODULE_CLKSEL_ENC(NA)|MODULE_CLKSEL_Msk_ENC(NA)  |MODULE_CLKSEL_Pos_ENC(NA)|\
                        MODULE_CLKDIV_ENC(NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))      /*!< PRNG Module    \hideinitializer */

#define CLK_TIMEOUT_ERR             (-1)    /*!< Clock timeout error value \hideinitializer */

/*@}*/ /* end of group CLK_EXPORTED_CONSTANTS */

extern int32_t g_CLK_i32ErrCode;

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

/**
  * @brief    Get PLL Clock Output Frequency
  * @param    None
  * @return   PLL clock output frequency
  * @details  To get actual PLL clock output frequency. The clock uint is in Hz.
  * \hideinitializer
  */
static __INLINE uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq;
    uint32_t u32FIN, u32NF, u32NR, u32NO;
    uint8_t au8NoTbl[4] = {1, 2, 2, 4}; /* OUTDIV :DEF: {1, 2, 2, 4} */
    uint32_t u32Reg;

    u32PllFreq = 0;
    u32Reg = CLK->PLLCTL;

    if ((u32Reg & (CLK_PLLCTL_PD_Msk | CLK_PLLCTL_OE_Msk)) == 0)
    {
        /* PLL is enabled and output enabled */
        if (u32Reg & CLK_PLLCTL_PLLSRC_Msk)
        {
            u32FIN = (__HIRC >> 2);
        } else
            u32FIN = __HXT;

        if (u32Reg & CLK_PLLCTL_BP_Msk)
        {
            /* PLL is in bypass mode */
            u32PllFreq = u32FIN;
        }
        else
        {
            /* PLL is in normal work mode */
            u32NO = au8NoTbl[((u32Reg & CLK_PLLCTL_OUTDIV_Msk) >> CLK_PLLCTL_OUTDIV_Pos)];
            u32NF  = ((u32Reg & CLK_PLLCTL_FBDIV_Msk) >> CLK_PLLCTL_FBDIV_Pos) + 2;
            u32NR  = ((u32Reg & CLK_PLLCTL_INDIV_Msk) >> CLK_PLLCTL_INDIV_Pos) + 2;
            /* u32FIN is shifted 2 bits to avoid overflow */
            u32PllFreq = (((u32FIN >> 2) * u32NF) / (u32NR * u32NO) << 2);
        }
    }

    return u32PllFreq;
}

/**
  * @brief      This function execute delay function.
  * @param[in]  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                             50MHz => 335544us, 48MHz => 349525us, 28MHz => 699050us ...
  * @return     None
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e. the same as system core clock.
  *             User can use SystemCoreClockUpdate() to calculate CyclesPerUs automatically before using this function.
  * \hideinitializer
  */
__STATIC_INLINE void CLK_SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    /* Disable SysTick counter */
    SysTick->CTRL = 0;
}


/**
  * @brief  Get current UART0 clock frquency.
  * @param  None.
  * @return UART0 clock frquency. The clock UNIT is in Hz.
  * \hideinitializer
  */
static __INLINE uint32_t CLK_GetUARTFreq(void)
{
    uint32_t u32Freqout, u32AHBDivider, u32ClkSel;

    u32Freqout = 0;
    u32ClkSel = CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk ;

    if (u32ClkSel == CLK_CLKSEL1_UART0SEL_HXT)  /* external HXT crystal clock */
    {
        u32Freqout = __HXT;
    }
    else if(u32ClkSel == CLK_CLKSEL1_UART0SEL_PLL)      /* PLL clock */
    {
        u32Freqout = CLK_GetPLLClockFreq();
    }
    else if(u32ClkSel == CLK_CLKSEL1_UART0SEL_LXT)      /* LXT clock */
    {
        u32Freqout = __LXT;
    }
    else if(u32ClkSel == CLK_CLKSEL1_UART0SEL_HIRC)     /* HIRC clock */
    {
        u32Freqout = __HIRC;
    }

    u32AHBDivider = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) + 1 ;

    return (u32Freqout/u32AHBDivider);
}


uint32_t CLK_WaitClockReady(uint32_t);
void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHXTFreq(void);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_DisablePLL(void);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_DisableSysTick(void);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_PowerDown(void);
void CLK_Idle(void);

/*@}*/ /* end of group CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group CLK_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __CLK_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
