/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    To receive data from an IR transmitter that was driven by timer PWM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/**
  * Receive error int:
  *     The CIR_STATUS_RERRF_Msk will be raised due to following reasons.
  *     1. The coming pulse width does not meet the upper and lower bound of all patterns.
  *     2. The first pulse width of coming pattern is not header pattern if pattern type is CIR_POSITIVE_EDGE or CIR_NEGATIVE_EDGE.
  *
  * Due to the repeat code (special + stop patterns) appears alone and without header patteren ahead. It will cause the CIR_STATUS_RERRF_Msk flag to be raised.
  * Programmer needs to judge if the error event caused by repeat code or stop pattern
  */

volatile uint32_t gu32ReceivedData0 = 0;
volatile uint32_t gu32ReceivedData1 = 0;
volatile uint32_t gu32TransmitData = 0x12345678;
void CIR_IRQHandler(void)
{
    uint32_t status = CIR_GetIntFlag(CIR0) & CIR_GetEnabledIntMask(CIR0);

    if(status & CIR_STATUS_RERRF_Msk)
    {
        uint32_t u32Hb, u32Lb;
        /*
         * Due to the repeat code, it will raise a error event.
         * Programmer needs to judge if the event caused by repeat code (special + stop patterns).
         */
        CIR_ClearIntFlag(CIR0, CIR_STATUS_RERRF_Msk);

        CIR_GetPatternBoundary(CIR0, CIR_SPECIAL_PAT, &u32Hb, &u32Lb);
        if((CIR_GetLatchedTimerValue(CIR0)< u32Hb) && (CIR_GetLatchedTimerValue(CIR0) > u32Lb))
            printf("Repeat\n");

        CIR_GetPatternBoundary(CIR0, CIR_END_PAT, &u32Hb, &u32Lb);
        if((CIR_GetLatchedTimerValue(CIR0)< u32Hb) && (CIR_GetLatchedTimerValue(CIR0) > u32Lb))
            printf("End\n");
        else
            printf("Error\n");
    }
    else if(status & CIR_STATUS_EPMF_Msk)
    {
        uint32_t u32Data0, u32Data1;
        /* End Patter match int */
        CIR_ClearIntFlag(CIR0, CIR_STATUS_EPMF_Msk);

        CIR_GetData(CIR0, &u32Data0, &u32Data1);
        gu32ReceivedData0 = u32Data0;
        /* Clear data fields and bit count for next receiving */
        CIR_ClearDataFieldBitCount(CIR0);
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set Timer PWM source clock to PCLK */
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select CIR module clock source as LIRC */
    CLK_SetModuleClock(CIR0_MODULE, CLK_CLKSEL2_CIR0SEL_LIRC, (uint32_t)NULL);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CIR module clock */
    CLK_EnableModuleClock(CIR0_MODULE);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(TMR3_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set GPF multi-function pin for TimerPWM driving IR LED */
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF11MFP_Msk));
    SYS->GPF_MFPH |= SYS_GPF_MFPH_PF11MFP_TM3;

    /* Set GPC multi-function pin for CIR */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk) | SYS_GPC_MFPH_PC14MFP_CIR0_RXD;

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


/**
 *    Use AI-26C generic remote controller by setting TV system to Panasonic:
 *    Pressed "Setup" up to 3 seconds, then pressed "0", "0", "1" to assign the remote controller to operate on Panasonic mode
 *
 *    CIR Clock Source From LIRC. T = 26.04us
 *
 *    unit: us
 *    Header    9100 + 4450
 *    Spec      9237 + 2000
 *    D0        576 + 557
 *    D1        576 + 1690
 *    Stop      542 + 17000
 */
typedef struct tagTiming
{
    uint32_t u16Low;
    uint32_t u16High;
    uint32_t u16Deviation;
} S_TIME;

typedef struct tagPat
{
    S_TIME sHeadPat;
    S_TIME sD0Pat;
    S_TIME sD1Pat;
    S_TIME sSpecPat;
    S_TIME sEndPat;
} S_PAT;

S_PAT sPatten[1] =
{
    9100, 4450,  50,
    576,  557,   15,
    576,  1690,  15,
    9065, 2282,  15,
    542,  17000, 15
};

/**
 *    Initialize CIR
 *    According to the timing of IR sent, to set related registers of CIR
 *
 *    CIR Clock Source From LIRC. T = 26.04us ~= 26us
 *
 *    unit: us
 *    Header    9100 + 4450
 *    Spec      9237 + 2000
 *    D0        576 + 557
 *    D1        576 + 1690
 *    Stop      542 + 17000
 */
void CIR_Init(void)
{
    uint32_t u32UpperBond, u32LowerBond;

    CIR_SetClockPrescaler(CIR0, CIR_PRESCALER_1);

    /* Set Header Pattern Boundary */
    u32UpperBond = ((sPatten[0].sHeadPat.u16Low+sPatten[0].sHeadPat.u16High)/26) + sPatten[0].sHeadPat.u16Deviation;
    u32LowerBond = ((sPatten[0].sHeadPat.u16Low+sPatten[0].sHeadPat.u16High)/26) - sPatten[0].sHeadPat.u16Deviation;
    CIR_SetPatternBoundary(CIR0, CIR_HEADER_PAT, u32UpperBond, u32LowerBond);

    /* Set Data0 Pattern Boundary */
    u32UpperBond = ((sPatten[0].sD0Pat.u16Low+sPatten[0].sD0Pat.u16High)/26) + sPatten[0].sD0Pat.u16Deviation;
    u32LowerBond = ((sPatten[0].sD0Pat.u16Low+sPatten[0].sD0Pat.u16High)/26) - sPatten[0].sD0Pat.u16Deviation;
    CIR_SetPatternBoundary(CIR0, CIR_DATA0_PAT, u32UpperBond, u32LowerBond);

    /* Set Data1 Pattern Boundary */
    u32UpperBond = ((sPatten[0].sD1Pat.u16Low+sPatten[0].sD1Pat.u16High)/26) + sPatten[0].sD1Pat.u16Deviation;
    u32LowerBond = ((sPatten[0].sD1Pat.u16Low+sPatten[0].sD1Pat.u16High)/26) - sPatten[0].sD1Pat.u16Deviation;
    CIR_SetPatternBoundary(CIR0, CIR_DATA1_PAT, u32UpperBond, u32LowerBond);

    /* Set Special Pattern Boundary */
    u32UpperBond = ((sPatten[0].sSpecPat.u16Low+sPatten[0].sSpecPat.u16High)/26) + sPatten[0].sSpecPat.u16Deviation;
    u32LowerBond = ((sPatten[0].sSpecPat.u16Low+sPatten[0].sSpecPat.u16High)/26) + sPatten[0].sSpecPat.u16Deviation;
    CIR_SetPatternBoundary(CIR0, CIR_SPECIAL_PAT, u32UpperBond, u32LowerBond);

    /* Set End Pattern Boundary */
    u32UpperBond = ((sPatten[0].sEndPat.u16Low+sPatten[0].sEndPat.u16High)/26) + sPatten[0].sEndPat.u16Deviation;
    u32LowerBond = ((sPatten[0].sEndPat.u16Low+sPatten[0].sEndPat.u16High)/26) - sPatten[0].sEndPat.u16Deviation;
    CIR_SetPatternBoundary(CIR0, CIR_END_PAT, u32UpperBond, u32LowerBond);

    CIR_EnableInt(CIR0, CIR_INTCTL_PERRIEN_Msk|CIR_INTCTL_EPMIEN_Msk);
    CIR_SetInputType(CIR0, CIR_POSITIVE_EDGE, CIR_INVERSE);
    NVIC_EnableIRQ(CIR_IRQn);
    CIR_Open(CIR0);
}


void TPWM_IR_Transmitter(uint32_t u32IrData)
{
    int32_t i;
    /* Send Header pattern */
    TPWM_ENABLE_OUTPUT(TIMER3, TPWM_CH0);
    CLK_SysTickDelay(sPatten[0].sHeadPat.u16Low);
    TPWM_ENABLE_OUTPUT(TIMER3, 0);
    CLK_SysTickDelay(sPatten[0].sHeadPat.u16High);

    /* Send 32 bits Data pattern */
    for(i=0; i <= 31; i=i+1)
    {
        if( (u32IrData & BIT0) == BIT0 )
        {
            TPWM_ENABLE_OUTPUT(TIMER3, TPWM_CH0);
            CLK_SysTickDelay(sPatten[0].sD1Pat.u16Low);
            TPWM_ENABLE_OUTPUT(TIMER3, 0);
            CLK_SysTickDelay(sPatten[0].sD1Pat.u16High);
        }
        else
        {
            TPWM_ENABLE_OUTPUT(TIMER3, TPWM_CH0);
            CLK_SysTickDelay(sPatten[0].sD0Pat.u16Low);
            TPWM_ENABLE_OUTPUT(TIMER3, 0);
            CLK_SysTickDelay(sPatten[0].sD0Pat.u16Low);
        }
        u32IrData = u32IrData >> 1;
    }

    /* Send Stop pattern */
    TPWM_ENABLE_OUTPUT(TIMER3, TPWM_CH0);
    CLK_SysTickDelay(sPatten[0].sEndPat.u16Low);
    TPWM_ENABLE_OUTPUT(TIMER3, 0);
    CLK_SysTickDelay(sPatten[0].sEndPat.u16High);
}

int main(void)
{
    uint32_t u32Data;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Utilize Timer PWM to drive the IR LED */
    TPWM_ENABLE_PWM_MODE(TIMER3);
    printf("Timer3 PWM_CH0 on PF.11\n\n");

    /* Disabke output of PWM_CH0 default */
    TPWM_ENABLE_OUTPUT(TIMER3, 0);

    /* Set Timer3 PWM output frequency acts as IR's carrier frequency 38000 Hz, duty 50% in up count type */
    u32Data = TPWM_ConfigOutputFreqAndDuty(TIMER3, 38000, 50);
    if ( u32Data != 38000)
        printf("Set the frequency %d different from the user's setting\n", u32Data);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER3);

    /* Init CIR to receive data from IR LED */
    CIR_Init();

    do
    {
        /* TPWM Trigger IR LED to send data */
        TPWM_IR_Transmitter(gu32TransmitData);
        CLK_SysTickDelay(1000000);
        printf("IR transmitted data 0x%x\n", gu32TransmitData);
        printf("CIR received data 0x%x....\n\n", gu32ReceivedData0);

        /* TPWM Trigger IR LED to send data */
        TPWM_IR_Transmitter(~gu32TransmitData);
        CLK_SysTickDelay(1000000);
        printf("IR transmitted data 0x%x\n", ~gu32TransmitData);
        printf("CIR received data 0x%x....\n\n", gu32ReceivedData0);

        gu32TransmitData++;
    }
    while(gu32TransmitData != 0x123456A0);

    printf("CIR Sample Code Done\n\n");
    while(SYS->PDID);

}
