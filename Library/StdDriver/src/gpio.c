/**************************************************************************//**
 * @file     gpio.c
 * @version  V3.00
 * @brief    M471 series GPIO driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD, PE, PF, PG, PH or PI.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT15 for PA, PB, PD, PE and PF GPIO port.
 *                          It could be BIT0 ~ BIT14 for PC GPIO port.
 *                          It could be BIT2 ~ BIT4, BIT9 ~ BIT15 for PG GPIO port.
 *                          It could be BIT4 ~ BIT11 for PH GPIO port.
 *                          It could be BIT0 ~ BIT5 for PI GPIO port.
 * @param[in]   u32Mode     Operation mode.  It could be
 *                              - \ref GPIO_MODE_INPUT
 *                              - \ref GPIO_MODE_OUTPUT
 *                              - \ref GPIO_MODE_OPEN_DRAIN
 *                              - \ref GPIO_MODE_QUASI
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0ul; i < GPIO_PIN_MAX; i++)
    {
        if((u32PinMask & (1ul << i))==(1ul << i))
        {
            port->MODE = (port->MODE & ~(0x3ul << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

/**
 * @brief       Enable GPIO interrupt
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD, PE, PF, PG, PH or PI.
 * @param[in]   u32Pin      The pin of specified GPIO port.
 *                          It could be 0 ~ 15 for PA, PB, PD, PE and PF GPIO port.
 *                          It could be 0 ~ 14 for PC GPIO port.
 *                          It could be 2 ~ 4, 9 ~ 15 for PG GPIO port.
 *                          It could be 4 ~ 11 for PH GPIO port.
 *                          It could be 0 ~ 5 for PI GPIO port.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be
 *                              - \ref GPIO_INT_RISING
 *                              - \ref GPIO_INT_FALLING
 *                              - \ref GPIO_INT_BOTH_EDGE
 *                              - \ref GPIO_INT_HIGH
 *                              - \ref GPIO_INT_LOW
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    port->INTTYPE = (port->INTTYPE&~(1ul<<u32Pin)) | (((u32IntAttribs >> 24) & 0xFFUL) << u32Pin);
    port->INTEN = (port->INTEN&~(0x00010001ul<<u32Pin)) | ((u32IntAttribs & 0xFFFFFFUL) << u32Pin);
}


/**
 * @brief       Disable GPIO interrupt
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD, PE, PF, PG, PH or PI.
 * @param[in]   u32Pin      The pin of specified GPIO port.
 *                          It could be 0 ~ 15 for PA, PB, PD, PE and PF GPIO port.
 *                          It could be 0 ~ 14 for PC GPIO port.
 *                          It could be 2 ~ 4, 9 ~ 15 for PG GPIO port.
 *                          It could be 4 ~ 11 for PH GPIO port.
 *                          It could be 0 ~ 5 for PI GPIO port.
 *
 * @return      None
 *
 * @details     This function is used to disable specified GPIO pin interrupt.
 */
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin)
{
    port->INTTYPE &= ~(1UL << u32Pin);
    port->INTEN &= ~((0x00010001UL) << u32Pin);
}

/**
 * @brief       Set GPIO slew rate control
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD, PE, PF, PG, PH or PI.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT15 for PA, PB, PD, PE and PF GPIO port.
 *                          It could be BIT0 ~ BIT14 for PC GPIO port.
 *                          It could be BIT2 ~ BIT4, BIT9 ~ BIT15 for PG GPIO port.
 *                          It could be BIT4 ~ BIT11 for PH GPIO port.
 *                          It could be BIT0 ~ BIT5 for PI GPIO port.
 * @param[in]   u32Mode     Slew rate mode. It could be
 *                              - \ref GPIO_SLEWCTL_NORMAL (maximum 40 MHz at 2.7V)
 *                              - \ref GPIO_SLEWCTL_HIGH (maximum 80 MHz at 2.7V)
 *                              - \ref GPIO_SLEWCTL_FAST (maximum 100 MHz at 2.7V)
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetSlewCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0ul; i < GPIO_PIN_MAX; i++)
    {
        if(u32PinMask & (1ul << i))
        {
            port->SLEWCTL = (port->SLEWCTL & ~(0x3ul << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

/**
 * @brief       Set GPIO Pull-up and Pull-down control
 *
 * @param[in]   port        GPIO port. It could be PA, PB, PC, PD, PE, PF, PG, PH or PI.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be BIT0 ~ BIT15 for PA, PB, PD, PE and PF GPIO port.
 *                          It could be BIT0 ~ BIT14 for PC GPIO port.
 *                          It could be BIT2 ~ BIT4, BIT9 ~ BIT15 for PG GPIO port.
 *                          It could be BIT4 ~ BIT11 for PH GPIO port.
 *                          It could be BIT0 ~ BIT5 for PI GPIO port.
 * @param[in]   u32Mode       The pin mode of specified GPIO pin. It could be
 *                              - \ref GPIO_PUSEL_DISABLE
 *                              - \ref GPIO_PUSEL_PULL_UP
 *
 * @return      None
 *
 * @details     Set the pin mode of specified GPIO pin.
 */
void GPIO_SetPullCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0ul; i < GPIO_PIN_MAX; i++)
    {
        if(u32PinMask & (1ul << i))
        {
            port->PUSEL = (port->PUSEL & ~(0x3ul << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}


/*@}*/ /* end of group GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group GPIO_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
