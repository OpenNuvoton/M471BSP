/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Convert temperature sensor (Sample module 25) and print conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable EADC peripheral clock */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Enable temperature sensor */
    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;

    /* Set reference voltage to external pin */
    SYS_SetVRef(SYS_VREFCTL_VREF_PIN);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void EADC_FunctionTest()
{
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      Temperature sensor test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 25 external sampling time to 0x3F */
    EADC_SetExtendSampleTime(EADC, 25, 0x3F);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 25 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT0);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT25);//Enable sample module 25 interrupt.
    NVIC_EnableIRQ(EADC0_INT0_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 25 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC, BIT25);

    /* Wait EADC conversion done */
    while(g_u32AdcIntFlag == 0);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, BIT0);

    /* Get the conversion result of the sample module 25 */
    i32ConversionData = EADC_GET_CONV_DATA(EADC, 25);
    printf("Conversion result of temperature sensor: 0x%X (%d)\n", i32ConversionData, i32ConversionData);

    /* The equation of converting to real temperature is as below
     * (25+(((float)i32ConversionData/4095*3300)-675)/(-1.83)), 3300 means ADCVREF=3.3V
     * If ADCREF set to 1.6V, the equation should be updated as below
     * (25+(((float)i32ConversionData/4095*1600)-675)/(-1.83)), 1600 means ADCVREF=1.6V
     */
    printf("Current Temperature = %2.1f\n\n", (25+(((float)i32ConversionData/4095*3300)-675)/(-1.83)));
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

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while(1);
}
