
/*
 * Copyright (c) 2006-2012 by Roland Riegel <feedback@roland-riegel.de>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef SD_RAW_CONFIG_H
#define SD_RAW_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * \addtogroup sd_raw
 *
 * @{
 */
/**
 * \file
 * MMC/SD support configuration (license: GPLv2 or LGPLv2.1)
 */

/**
 * \ingroup sd_raw_config
 * Controls MMC/SD write support.
 *
 * Set to 1 to enable MMC/SD write support, set to 0 to disable it.
 */
#define SD_RAW_WRITE_SUPPORT 1

/**
 * \ingroup sd_raw_config
 * Controls MMC/SD write buffering.
 *
 * Set to 1 to buffer write accesses, set to 0 to disable it.
 *
 * \note This option has no effect when SD_RAW_WRITE_SUPPORT is 0.
 */
#define SD_RAW_WRITE_BUFFERING 0

/**
 * \ingroup sd_raw_config
 * Controls MMC/SD access buffering.
 * 
 * Set to 1 to save static RAM, but be aware that you will
 * lose performance.
 *
 * \note When SD_RAW_WRITE_SUPPORT is 1, SD_RAW_SAVE_RAM will
 *       be reset to 0.
 */
#define SD_RAW_SAVE_RAM 0

/**
 * \ingroup sd_raw_config
 * Controls support for SDHC cards.
 *
 * Set to 1 to support so-called SDHC memory cards, i.e. SD
 * cards with more than 2 gigabytes of memory.
 */
#define SD_RAW_SDHC 1

/**
 * @}
 */
#if SD_RAW_SDHC
    typedef uint64_t offset_t;
#else
    typedef uint32_t offset_t;
#endif

/* configuration checks */
#if SD_RAW_WRITE_SUPPORT
#undef SD_RAW_SAVE_RAM
#define SD_RAW_SAVE_RAM 0
#else
#undef SD_RAW_WRITE_BUFFERING
#define SD_RAW_WRITE_BUFFERING 0
#endif

#ifdef __cplusplus
}
#endif

#endif

