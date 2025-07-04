/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 *
 * Change Logs:
 * 2020.02.20     jpaik
 */

#ifndef __DRV_AUDIO_H__
#define __DRV_AUDIO_H__

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>
#include "drivers/audio.h"
#ifdef  STM32L496xx
#include "stm32l4xx.h"
#elif defined (STM32F746xx)
#include "stm32f7xx.h"
#endif

//#define TLV320AIC31xx
//#define TLV320AIC3120
//#define TLV320AIC3100

#define TLV320AIC310x
#define TLV320AIC3104

/* stm32 audio dirver class */
#define DEFAULT_SAMPLING_RATE       SAI_AUDIO_FREQUENCY_16K
#define DEFAULT_BITS_RATE           16
#define DEFAULT_CHANNELS            1

#define DMA_TX_BLOCK_SIZE           RT_AUDIO_REPLAY_MP_BLOCK_SIZE
#define DMA_TX_BLOCK_COUNT          2
#define DMA_TX_BUF_SIZE             (DMA_TX_BLOCK_SIZE * DMA_TX_BLOCK_COUNT)

/* AUDIO FREQUENCY */
#define AUDIO_FREQUENCY_192K          ((rt_uint32_t)192000)
#define AUDIO_FREQUENCY_96K           ((rt_uint32_t)96000)
#define AUDIO_FREQUENCY_48K           ((rt_uint32_t)48000)
#define AUDIO_FREQUENCY_44K           ((rt_uint32_t)44100)
#define AUDIO_FREQUENCY_32K           ((rt_uint32_t)32000)
#define AUDIO_FREQUENCY_22K           ((rt_uint32_t)22050)
#define AUDIO_FREQUENCY_16K           ((rt_uint32_t)16000)
#define AUDIO_FREQUENCY_11K           ((rt_uint32_t)11025)
#define AUDIO_FREQUENCY_8K            ((rt_uint32_t)8000)

#define AUDIO_SOUND_DEVICE_NAME     "sound0"

typedef struct
{
  rt_err_t  (*init)(rt_uint8_t volume, rt_uint32_t auido_freq);
  rt_err_t  (*play)(void);
  rt_err_t  (*stop)(rt_bool_t powerdown_mode);
  rt_err_t  (*set_frequency)(rt_uint32_t auido_freq);
  rt_err_t  (*set_volume)(rt_uint8_t new_vol);
} codec_driver_t;


#endif  /* __DRV_AUDIO_H__ */
