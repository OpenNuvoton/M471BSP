/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to get average conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/1 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable EADC peripheral clock */
    /* Note: The EADC_CLK speed should meet datasheet spec (<36MHz) and rules in following table.   */
    /* +--------------+------------------+                                                          */
    /* | PCLK divider | EADC_CLK divider |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 1            | 1, 2, 3, 4, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 2, 4, 8, 16  | 2, 4, 6, 8, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(2));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB.0 - PB.1 to input mode */
    GPIO_SetMode(PB, BIT0|BIT1, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.1 ADC analog input pins. */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk)) |
                    (SYS_GPB_MFPL_PB0MFP_EADC0_CH0 | SYS_GPB_MFPL_PB1MFP_EADC0_CH1);
    /* Disable the PB.0 - PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData;

    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 1;   /* Use Sample Module 1 */

    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);

    printf("\n");
    printf("+---------------------------------------------------+\n");
    printf("|             EADC Average sample code              |\n");
    printf("+---------------------------------------------------+\n");

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    while(1)
    {
        printf("Select test items:\n");
        printf("  [1] Basic EADC conversion (channel 0 only)\n");
        printf("  [2] Basic EADC conversion (channel 1 only)\n");
        printf("  Other keys: exit EADC test\n");
        u8Option = getchar();

        if (u8Option == '1')
            u32ChannelNum = 0;
        else if (u8Option == '2')
            u32ChannelNum = 1;
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 0 */
        EADC_SetExtendSampleTime(EADC, u32ModuleNum, 0);

        /* Enable Accumulate feature */
        EADC_ENABLE_ACU(EADC, u32ModuleNum, EADC_MCTL1_ACU_32);

        /* Enable Average feature */
        EADC_ENABLE_AVG(EADC, u32ModuleNum);

        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
        NVIC_EnableIRQ(EADC0_INT0_IRQn);

        /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
        g_u32AdcIntFlag = 0;
        EADC_START_CONV(EADC, u32ModuleMask);

        /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
        while(g_u32AdcIntFlag == 0);

        /* Get the conversion result of the sample module */
        i32ConversionData = EADC_GET_CONV_DATA(EADC, u32ModuleNum);
        printf("Conversion result of channel %u averaged 32 times : 0x%X (%d)\n\n", u32ChannelNum, i32ConversionData, i32ConversionData);

        /* Disable Average feature */
        EADC_DISABLE_ACU(EADC, u32ModuleNum);

        /* Disable Accumulate feature */
        EADC_DISABLE_AVG(EADC, u32ModuleNum);

        /* Disable the sample module interrupt. */
        EADC_DISABLE_INT(EADC, u32IntMask);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
        NVIC_DisableIRQ(EADC0_INT0_IRQn);
    }   /* end of while(1) */
}

void EADC0_INT0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    printf("Exit EADC sample code\n");

    while(1);
}
