/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x34
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"
#include "isp_user.h"
#include "spi_transfer.h"

#define ReadyPin  (PB6)

#define CONFIG_SIZE 8 // in bytes
