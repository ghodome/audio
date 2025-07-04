/**
  ******************************************************************************
  * @file    l4_cs42l51.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the cs42l51.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L4_CS42L51_H
#define __L4_CS42L51_H

#include <stdint.h>
#include "drv_audio.h"

/* Includes ------------------------------------------------------------------*/

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Component
  * @{
  */ 
  
/** @addtogroup CS42L51
  * @{
  */

/** @defgroup CS42L51_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup CS42L51_Exported_Constants
  * @{
  */ 

/******************************************************************************/
/***************************  Codec User defines ******************************/
/******************************************************************************/
/* Codec output devices */
/* Codec input devices */
#define INPUT_DEVICE_MIC1             0x10

/* Volume Levels values */
#define DEFAULT_VOLMIN                0x00
#define DEFAULT_VOLMAX                0xFF
#define DEFAULT_VOLSTEP               0x04

#define AUDIO_PAUSE                   0
#define AUDIO_RESUME                  1

/* Codec POWER DOWN modes */
#define CODEC_PDWN_HW                 1
#define CODEC_PDWN_SW                 2

/* MUTE commands */
#define AUDIO_MUTE_ON                 1
#define AUDIO_MUTE_OFF                0

/* I2C function */
#define I2C_BUS_NAME                  "i2c2"
#define I2C_OPFLAG_MASK               0x00
#define CODEC_RESET                   PIN_LOW
#define CODEC_SET                     PIN_HIGH

/******************************************************************************/
/****************************** REGISTER MAPPING ******************************/
/******************************************************************************/
/** 
  * @brief  CS42L51 ID  
  */  
#define  CS42L52_ID            0x1C
#define  CS42L52_ID_MASK       0xF8
/**
  * @brief Chip ID Register: Chip I.D. and Revision Register
  *  Read only register
  *  Default value: 0x01
  *  [7:3] CHIPID[4:0]: I.D. code for the CS42L51.
  *        Default value: 11100b
  *  [2:0] REVID[2:0]: CS42L51 revision level.
  *        Default value: 
  *        000 - Rev A0
  *        001 - Rev A1
  *        010 - Rev B0
  *        011 - Rev B1
  */
#define CS42L5x_CHIPID_ADDR     0x01

#define OUTPUT_DEVICE_SPEAKER   0xFA
#define OUTPUT_DEVICE_HEADPHONE 0xAF
#define OUTPUT_DEVICE_BOTH      0xAA
#define OUTPUT_DEVICE_AUTO      0x05

#define CS42L5x_OUTPUT_MODE     OUTPUT_DEVICE_HEADPHONE 
#define CS42L5x_DEFAULT_GAIN    (rt_uint8_t)(0 - 12)

/**
  * @}
  */ 

/** @defgroup CS42L51_Exported_Macros
  * @{
  */
#define VOLUME_CONVERT(Volume)    (((Volume) > 100) ? 100 : ((rt_uint8_t)(((Volume) * 255) / 100)))
/**
  * @}
  */ 

/** @defgroup CS42L51_Exported_Functions
  * @{
  */
    
/*------------------------------------------------------------------------------
                           Audio Codec functions 
------------------------------------------------------------------------------*/
/* High Layer codec CS42L52 functions */
rt_err_t cs42l52_init(rt_uint8_t Volume, rt_uint32_t AudioFreq);
rt_err_t cs42l52_play(void);
rt_err_t cs42l52_stop(rt_bool_t power_down);
rt_err_t cs42l52_set_volume(rt_uint8_t Volume);
rt_err_t cs42l52_set_frequency(rt_uint32_t AudioFreq);
rt_err_t cs42l52_set_mute(rt_uint32_t Cmd);
rt_err_t cs42l52_pause(void);
rt_err_t cs42l52_resume(void);
rt_err_t cs42l52_set_outputmode(rt_uint8_t Output);
rt_uint8_t cs42l5x_read_id(void);

void codec_dump(int argc, char **argv);
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */ 

#endif /* __CS42L51_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
