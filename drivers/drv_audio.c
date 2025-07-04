/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-07-04     ghodome   first version
 */

#include "board.h"
#include "drv_audio.h"
#include "drv_config.h"
#include <drivers/audio.h>

#ifdef  STM32L496xx
    #include "stm32l4xx.h"
#elif defined (STM32F746xx)
    #include "stm32f7xx.h"
#endif

#ifdef  TLV320AIC31xx
    #include "drv_tlv320aic31xx.h"
#elif   defined (TLV320AIC310x)
    #include "drv_tlv320aic310x.h"
#elif   defined (CS42L52)
    #include "drv_cs42l5x.h"
#endif

#include "app_config.h"


#ifdef RT_USING_AUDIO

#define DRV_DEBUG
#define LOG_TAG                 "drv.audio"
#include <drv_log.h>

#define SAIClockDivider(__FREQUENCY__) \
        (__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 12 \
    : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? 2 \
    : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 6 \
    : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? 1 \
    : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 3 \
    : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? 0 \
    : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 2 : 1
/********************************************************************************************
********************************************************************************************/
struct stm32_audio_device
{
    struct rt_audio_device audio;
    struct rt_audio_configure replay_config;
    struct rt_audio_buf_info  *buf_info;

    codec_driver_t    *codec_ops;
    SAI_HandleTypeDef audio_sai_tx_handle;
    DMA_HandleTypeDef sai_tx_dma_handle;
    rt_uint8_t        *audio_tx_buf;

    rt_bool_t         bsp_init;
    rt_bool_t         tx_start;
    rt_uint16_t       volume;
}; 
static struct stm32_audio_device stm32_sound_drv = {0};

static rt_err_t sound_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps);
static rt_err_t sound_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps);
static rt_err_t sound_init(rt_uint8_t Volume, struct rt_audio_device *audio);
static rt_err_t sound_start(struct rt_audio_device *audio, int stream);
static rt_err_t sound_stop(struct rt_audio_device *audio, int stream);
static rt_ssize_t sound_transmit(struct rt_audio_device *audio, const void *writeBuf, void *readBuf, rt_size_t size);
static void sound_buffer_info(struct rt_audio_device *audio, struct rt_audio_buf_info *info);


    static struct rt_audio_ops snd_ops =
    {
        .getcaps     = sound_getcaps,
        .configure   = sound_configure,
        .init        = sound_init,
        .start       = sound_start,
        .stop        = sound_stop,
        .transmit    = sound_transmit,
        .buffer_info = sound_buffer_info
    };
#ifdef CS42L5x

    static codec_driver_t cs42l52_drv =
    {
        .init         = cs42l52_Init,
        .play         = cs42l52_Play,
        .stop         = cs42l52_Stop,
        .set_frequency = cs42l52_SetFrequency,
        .set_volume   = cs42l52_SetVolume
    };
#elif defined (TLV320AIC31xx)
    static codec_driver_t tlv320aic31x_drv =
    {
        .init         = tlv320aic31x_init,
        .play         = tlv320aic31x_play,
        .stop         = tlv320aic31x_stop,
        .set_frequency = tlv320aic31x_set_frequency,
        .set_volume   = tlv320aic31x_volume
    };
#elif defined (TLV320AIC310x)
    static codec_driver_t tlv320aic310x_drv =
            {
                .init         = tlv320aic310x_init,
                .play         = tlv320aic310x_play,
                .stop         = tlv320aic310x_stop,
                .set_frequency = tlv320aic310x_set_frequency,
                .set_volume   = tlv320aic310x_volume
            };
#endif


/********************************************************************
    application SAI intialize sequence.
    Just call audio_sai_config() and all done.
      1. application call audio_sai_config()
      2. audio_sai_config() call HAL_SAI_InitProtocol()
      3. HAL_SAI_InitProtocol() call HAL_SAI_MspInit()
********************************************************************/
static rt_int16_t audio_conv_samplebits_to_informat(rt_uint16_t sample_bits)
{
    rt_int16_t resolution;

    switch (sample_bits)
    {
    case 16:
        resolution = SAI_PROTOCOL_DATASIZE_16BIT;
        break;

    case 24:
        resolution = SAI_PROTOCOL_DATASIZE_24BIT;
        break;

    case 32:
        resolution = SAI_PROTOCOL_DATASIZE_32BIT;
        break;
    
    default: 
        resolution = -1;
    }

    return resolution;
}

#if defined (STM32F746xx)
    /* 
        DO NOT SAI initalize in this rouine.
        audio_sai_bsp_init
          - RCC clock config
          - GPIO config
          - DMA config
    */
    static rt_err_t audio_sai_bsp_init(struct stm32_audio_device *h_drv)
    {
        GPIO_InitTypeDef          GPIO_InitStruct;
        RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct = {0};
        SAI_HandleTypeDef         *h_sai  = &h_drv->audio_sai_tx_handle;
        DMA_HandleTypeDef         *h_dma = &h_drv->sai_tx_dma_handle;
        rt_uint32_t               source_freq  = BSP_CLOCK_SOURCE_FREQ_MHZ;

        /* SAI1 */
        if (h_sai->Instance != SAI1_Block_A)
        {
            return -RT_EIO;
        }
        /* Peripheral clock enable */
        /** Initializes the peripherals clock */
        __HAL_RCC_SAI1_CLK_ENABLE();
#if 1
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
        PeriphClkInitStruct.PLLSAI.PLLSAIN       = (source_freq == 25000000) ? 100 : 50;    //(source_freq == 25) ? 100 : 50
        PeriphClkInitStruct.PLLSAI.PLLSAIR       = 2;
        PeriphClkInitStruct.PLLSAI.PLLSAIQ       = 2;
        PeriphClkInitStruct.PLLSAI.PLLSAIP       = RCC_PLLSAIP_DIV2;
        PeriphClkInitStruct.PLLSAIDivQ           = 1;
        PeriphClkInitStruct.PLLSAIDivR           = RCC_PLLSAIDIVR_2;
        PeriphClkInitStruct.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_PLLSAI;
#endif

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            return -RT_ENOSYS;
        }
        /* Peripheral interrupt init. do not use unterrupt. */
        // HAL_NVIC_SetPriority(SAI1_IRQn, 0, 0);
        // HAL_NVIC_EnableIRQ(SAI1_IRQn);

        /**SAI1_A_Block_A GPIO Configuration
        PE2     ------> SAI1_MCLK
        PE4     ------> SAI1_FS_A
        PE5     ------> SAI1_SCK_A
        PE6     ------> SAI1_SD_A
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* Peripheral DMA init*/
        __HAL_RCC_DMA2_CLK_ENABLE();                //dma clk on
        h_dma->Init.Channel             = DMA_CHANNEL_0;
        h_dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        h_dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        h_dma->Init.MemInc              = DMA_MINC_ENABLE;
        h_dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        h_dma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        h_dma->Init.Mode                = DMA_CIRCULAR;
        h_dma->Init.Priority            = DMA_PRIORITY_LOW;
        h_dma->Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
        h_dma->Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
        h_dma->Init.MemBurst            = DMA_MBURST_SINGLE;
        h_dma->Init.PeriphBurst         = DMA_PBURST_SINGLE;

        HAL_DMA_DeInit(h_dma);  //location changed

        if (HAL_DMA_Init(h_dma) != HAL_OK)
        {
            return -RT_ENOSYS;
        }

        /* SAI DMA IRQ Channel configuration */
        HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

        /* Several peripheral DMA handle pointers point to the same DMA handle.
        Be aware that there is only one stream to perform all the requested DMAs. */
        h_sai->hdmatx = h_dma;
        h_dma->Parent = h_sai;

        return RT_EOK;
    }

    void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
    {
        struct stm32_audio_device *h_drv = &stm32_sound_drv;

        if (!h_drv->bsp_init)
        {
            if (audio_sai_bsp_init(&stm32_sound_drv) != RT_EOK)
            {
                RT_ASSERT(0);
            }
            h_drv->bsp_init = RT_TRUE;
        }
    }

    void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
    {
        struct stm32_audio_device *h_drv = &stm32_sound_drv;

        h_drv->bsp_init = RT_FALSE;
        HAL_SAI_DeInit(hsai);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
        if (hsai->hdmatx)
        {
            HAL_DMA_DeInit(hsai->hdmatx);
        }
    }

    static rt_err_t audio_sai_config(struct rt_audio_configure *cfg)
    {
        struct stm32_audio_device *h_drv = &stm32_sound_drv;
        SAI_HandleTypeDef         *hsai = &h_drv->audio_sai_tx_handle;
        rt_uint16_t res_bits;

        if (!IS_SAI_AUDIO_FREQUENCY(cfg->samplerate))
        {
            RT_ASSERT(0);
            return -RT_EINVAL;
        }

        __HAL_SAI_DISABLE(hsai);
        res_bits = audio_conv_samplebits_to_informat(cfg->samplebits);
        hsai->Init.AudioMode      = SAI_MODEMASTER_TX;
        hsai->Init.Synchro        = SAI_ASYNCHRONOUS;
        hsai->Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
        hsai->Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
        hsai->Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
        hsai->Init.AudioFrequency = cfg->samplerate;
//        hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
//        hsai->Init.Mckdiv         = SAIClockDivider(cfg->samplerate);

        hsai->Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_HF;
        hsai->Init.CompandingMode = SAI_NOCOMPANDING;
        hsai->Init.TriState       = SAI_OUTPUT_NOTRELEASED;


        /* SAI_MONOMODE SAI_STEREOMODE */
        if (cfg->channels == 1)
        {
            hsai->Init.MonoStereoMode  = SAI_MONOMODE;
        }
        else
        {
            hsai->Init.MonoStereoMode  = SAI_STEREOMODE;
        }

        /* SAI_I2S_STANDARD SAI_I2S_MSBJUSTIFIED SAI_I2S_LSBJUSTIFIED SAI_PCM_LONG SAI_PCM_SHORT */
        /* hsai : SAI_HandleTypeDef
        protocol : SAI_I2S_STANDARD, SAI_I2S_MSBJUSTIFIED, SAI_I2S_LSBJUSTIFIED, SAI_PCM_LONG, SAI_PCM_SHORT
        datasize : SAI_PROTOCOL_DATASIZE_16BIT SAI_PROTOCOL_DATASIZE_16BITEXTENDED SAI_PROTOCOL_DATASIZE_24BIT SAI_PROTOCOL_DATASIZE_32BIT
        nbslot   : number of slot. current 2. max is 16. the value must be a multiple of 2.
        */
        if (HAL_SAI_InitProtocol(hsai, SAI_I2S_STANDARD, res_bits, 2) != HAL_OK)
        {
            RT_ASSERT(0);
            return -RT_EIO;
        }

        __HAL_SAI_ENABLE(hsai);
        return RT_EOK;
    }

    static rt_err_t audio_out_init(struct stm32_audio_device *h_drv)
    {
        h_drv->replay_config.channels   = DEFAULT_CHANNELS;
        h_drv->replay_config.samplerate = DEFAULT_SAMPLING_RATE;
        h_drv->replay_config.samplebits = DEFAULT_BITS_RATE;

        /* Initialize the audio output context */
#ifdef TLV320AIC31xx
        h_drv->codec_ops = &tlv320aic31x_drv;
#elif defined (TLV320AIC310x)
        h_drv->codec_ops = &tlv320aic310x_drv;
#elif defined (CS42L5x)
        h_drv->codec_ops = &cs42l5x_drv;
#endif
        h_drv->audio_sai_tx_handle.Instance = SAI1_Block_A;

        /* SAI data transfer preparation: prepare the Media to be used for the audio transfer from memory to SAI peripheral. */
        h_drv->sai_tx_dma_handle.Instance = DMA2_Stream1;


        if (audio_sai_config(&h_drv->replay_config) != RT_EOK)
        {
            RT_ASSERT(0);
            return -RT_ERROR;
        }

        return RT_EOK;
    }

    void DMA2_Stream1_IRQHandler(void)
    {
        struct stm32_audio_device *h_drv = &stm32_sound_drv;

        if (h_drv->buf_info->free_size < DMA_TX_BUF_SIZE)
        {
            h_drv->buf_info->free_size += DMA_TX_BLOCK_SIZE;
        }
        HAL_DMA_IRQHandler(&h_drv->sai_tx_dma_handle);
    }

#elif defined (STM32L496xx)
    /* 
        DO NOT SAI initalize in this rouine.
        audio_sai_bsp_init
          - RCC clock config
          - GPIO config
          - DMA config
    */
    static rt_err_t audio_rcc_periph_config(rt_uint32_t audio_freq)
    {
        RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct = {0};

        __HAL_RCC_SAI1_CLK_ENABLE();
        /* Retrieve actual RCC configuration */
        HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInitStruct);

        /* Peripheral clock enable */
        /** Initializes the peripherals clock */
        PeriphClkInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
        if ((audio_freq == AUDIO_FREQUENCY_11K) || (audio_freq == AUDIO_FREQUENCY_22K) || (audio_freq == AUDIO_FREQUENCY_44K))
        {
            /* Configure PLLSAI prescalers */
            /* SAI clock config
            PLLSAI2_VCO= 8 Mhz * PLLSAI1N = 8 * 24 = VCO_192M
            SAI_CK_x = PLLSAI2_VCO/PLLSAI1P = 192/17 = 11.294 Mhz */
            PeriphClkInitStruct.PLLSAI2.PLLSAI2N        = 24;
            PeriphClkInitStruct.PLLSAI2.PLLSAI2P        = 17;
            PeriphClkInitStruct.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;
            PeriphClkInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI2;
        }
        else if ((audio_freq == AUDIO_FREQUENCY_8K) || (audio_freq == AUDIO_FREQUENCY_16K) || (audio_freq == AUDIO_FREQUENCY_48K) || (audio_freq == AUDIO_FREQUENCY_96K))
        {
            /* SAI clock config
            PLLSAI2_VCO= 8 Mhz * PLLSAI1N = 8 * 43 = VCO_344M
            SAI_CK_x = PLLSAI1_VCO/PLLSAI2P = 344/7 = 49.142 Mhz */
            PeriphClkInitStruct.PLLSAI2.PLLSAI2N        = 43;
            PeriphClkInitStruct.PLLSAI2.PLLSAI2P        = 7;
            PeriphClkInitStruct.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;
            PeriphClkInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI2;
        }
        else
        {
            RT_ASSERT(0);
            return -RT_ERROR;
        }
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            RT_ASSERT(0);
            return -RT_ENOSYS;
        }

        return RT_EOK;
    }

    static rt_err_t audio_sai_bsp_init(struct stm32_audio_device *h_drv)
    {
        SAI_HandleTypeDef         *h_sai = &h_drv->audio_sai_tx_handle;
        DMA_HandleTypeDef         *h_dma = &h_drv->sai_tx_dma_handle;
        GPIO_InitTypeDef          GPIO_InitStruct;

        /****************************************************************************/
        /* STEP 1 : configuration RCC clock                                         */
        /****************************************************************************/
        if (audio_rcc_periph_config(h_drv->replay_config.samplerate) != RT_EOK)
        {
            RT_ASSERT(0);
            return -RT_EIO;
        }

        /****************************************************************************/
        /* STEP 2 : GPIO configuration                                              */
        /****************************************************************************/
        /* deinit GPIO  */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);  /* SAI1_MCLK_A */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);  /* SAI1_FS_A */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10); /* SAI1_SCK_A */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);  /* SAI1_SD_A */

        /* init GPIO  */
        /* Enable GPIO clock */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;  /* SAI1_MCLK_A / SAI1_SCK_A / SAI1_FS_A */
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_3; /* SAI1_SD_A */
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /****************************************************************************/
        /* STEP 3 : DMA configuration                                              */
        /****************************************************************************/
        /* DMA deinit */
        __HAL_RCC_DMA2_CLK_ENABLE();
        h_dma->Instance                 = DMA2_Channel1;
        h_dma->Init.Request             = DMA_REQUEST_1;
        h_dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        h_dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        h_dma->Init.MemInc              = DMA_MINC_ENABLE;
        h_dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        h_dma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        h_dma->Init.Mode                = DMA_CIRCULAR;
        h_dma->Init.Priority            = DMA_PRIORITY_HIGH;

        /* Associate the DMA handle */
        /* __HAL_LINKDMA()  */
        h_sai->hdmatx = h_dma;
        h_dma->Parent = h_sai;

        /* Configure the DMA Stream */
        HAL_DMA_DeInit(h_dma);
        if (HAL_DMA_Init(h_dma) != HAL_OK)
        {
            RT_ASSERT(0);
            return -RT_ENOSYS;
        }
        /* SAI DMA IRQ Channel configuration */
        HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

        return RT_EOK;
    }

    void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
    {
        struct stm32_audio_device *h_drv = &stm32_sound_drv;

        if (!h_drv->bsp_init)
        {
            if (audio_sai_bsp_init(&stm32_sound_drv) != RT_EOK)
            {
                RT_ASSERT(0);
            }
            h_drv->bsp_init = RT_TRUE;
        }
    }

    void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
    {
        struct stm32_audio_device *h_drv = &stm32_sound_drv;

        h_drv->bsp_init = RT_FALSE;
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);  /* SAI1_MCLK_A */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);  /* SAI1_FS_A */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10); /* SAI1_SCK_A */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);  /* SAI1_SD_A */
        if (hsai->hdmatx)
        {
            HAL_DMA_DeInit(hsai->hdmatx);
        }
        __HAL_RCC_SAI1_CLK_DISABLE();
    }

    void DMA2_Channel1_IRQHandler(void)
    {
        struct stm32_audio_device *h_drv = &stm32_sound_drv;

        if (h_drv->buf_info->free_size < DMA_TX_BUF_SIZE)
        {
            h_drv->buf_info->free_size += DMA_TX_BLOCK_SIZE;
        }
        HAL_DMA_IRQHandler(&h_drv->sai_tx_dma_handle);
    }

    static rt_err_t audio_sai_config(struct rt_audio_configure *cfg)
    {
        struct stm32_audio_device *h_drv = &stm32_sound_drv;
        SAI_HandleTypeDef         *hsai = &h_drv->audio_sai_tx_handle;
        rt_uint16_t               res_bits;

        if (!IS_SAI_AUDIO_FREQUENCY(cfg->samplerate))
        {
            return -RT_EINVAL;
        }

        if (audio_rcc_periph_config(cfg->samplerate) != RT_EOK)
        {
            return -RT_EIO;
        }

         HAL_SAI_DeInit(hsai);
        __HAL_SAI_DISABLE(hsai);
        hsai->Init.AudioMode      = SAI_MODEMASTER_TX;
        hsai->Init.Synchro        = SAI_ASYNCHRONOUS;
        hsai->Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
        hsai->Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
        hsai->Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
        hsai->Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_HF;       /* SAI_FIFOTHRESHOLD_EMPTY SAI_FIFOTHRESHOLD_1QF SAI_FIFOTHRESHOLD_HF SAI_FIFOTHRESHOLD_3QF SAI_FIFOTHRESHOLD_FULL */
        hsai->Init.AudioFrequency = cfg->samplerate; 
        hsai->Init.CompandingMode = SAI_NOCOMPANDING;
        hsai->Init.TriState       = SAI_OUTPUT_NOTRELEASED;
        if (hsai->Init.AudioFrequency == SAI_AUDIO_FREQUENCY_MCKDIV)
        {
            hsai->Init.Mckdiv     = SAIClockDivider(cfg->samplerate);
        }
        /* 
        DO not config follows parameters.
        I2S or PCM set parameters.
            hsai->Init.Protocol
            hsai->Init.DataSize
            hsai->Init.FirstBit
            hsai->Init.ClockStrobing
            hsai->FrameInit
        */
        /* SAI_MONOMODE SAI_STEREOMODE */
        if (cfg->channels == 1)
        {
            hsai->Init.MonoStereoMode  = SAI_MONOMODE;
        }
        else
        {
            hsai->Init.MonoStereoMode  = SAI_STEREOMODE;
        }

        /* SAI_I2S_STANDARD SAI_I2S_MSBJUSTIFIED SAI_I2S_LSBJUSTIFIED SAI_PCM_LONG SAI_PCM_SHORT */
        /* hsai : SAI_HandleTypeDef
        protocol : SAI_I2S_STANDARD, SAI_I2S_MSBJUSTIFIED, SAI_I2S_LSBJUSTIFIED, SAI_PCM_LONG, SAI_PCM_SHORT
        datasize : SAI_PROTOCOL_DATASIZE_16BIT SAI_PROTOCOL_DATASIZE_16BITEXTENDED SAI_PROTOCOL_DATASIZE_24BIT SAI_PROTOCOL_DATASIZE_32BIT
        nbslot   : number of slot. current 2. max is 16. the value must be a multiple of 2.
        */
        res_bits = audio_conv_samplebits_to_informat(cfg->samplebits);
        if (HAL_SAI_InitProtocol(hsai, SAI_I2S_STANDARD, res_bits, 2) != HAL_OK)
        {
            RT_ASSERT(0);
            return -RT_EIO;
        }
        __HAL_SAI_ENABLE(hsai);

        return RT_EOK;
    }

    static rt_err_t audio_out_init(struct stm32_audio_device *h_drv)
    {
        h_drv->audio_sai_tx_handle.Instance = SAI1_Block_A;        /* SAI instance use SAI_A*/
        h_drv->replay_config.channels   = DEFAULT_CHANNELS;
        h_drv->replay_config.samplerate = DEFAULT_SAMPLING_RATE;
        h_drv->replay_config.samplebits = DEFAULT_BITS_RATE;

        /* Initialize the audio output context */
        h_drv->codec_ops = &cs42l52_drv;

        /* SAI data transfer preparation: prepare the Media to be used for the audio transfer from memory to SAI peripheral. */
        if (audio_sai_config(&h_drv->replay_config) != RT_EOK)
        {
            RT_ASSERT(0);
            return -RT_ERROR;
        }

        return RT_EOK;
    }
#endif // STM32L496xx or STM32F7xx

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai == &stm32_sound_drv.audio_sai_tx_handle)
    {
        rt_audio_tx_complete(&stm32_sound_drv.audio);
    }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai == &stm32_sound_drv.audio_sai_tx_handle)
    {
        rt_audio_tx_complete(&stm32_sound_drv.audio);
    }
}

static rt_err_t sound_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t err = RT_EOK;
    struct stm32_audio_device *h_drv;

    RT_ASSERT(audio != RT_NULL);
    h_drv = (struct stm32_audio_device *)audio->parent.user_data;

    switch (caps->main_type)
    {
        case AUDIO_TYPE_QUERY: /* qurey the types of hw_codec device */
            switch (caps->sub_type)
            {
                case AUDIO_TYPE_QUERY:
                    caps->udata.mask = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_MIXER;
                    break;

                default:
                    err = -RT_ERROR;
                    break;
            }
            break;

        case AUDIO_TYPE_OUTPUT: /* Provide capabilities of OUTPUT unit */
            switch (caps->sub_type)
            {
                case AUDIO_DSP_PARAM:
                    caps->udata.config.samplerate   = h_drv->replay_config.samplerate;
                    caps->udata.config.channels     = h_drv->replay_config.channels;
                    caps->udata.config.samplebits   = h_drv->replay_config.samplebits;
                    break;

                case AUDIO_DSP_SAMPLERATE:
                    caps->udata.config.samplerate   = h_drv->replay_config.samplerate;
                    break;

                case AUDIO_DSP_CHANNELS:
                    caps->udata.config.channels     = h_drv->replay_config.channels;
                    break;

                case AUDIO_DSP_SAMPLEBITS:
                    caps->udata.config.samplebits   = h_drv->replay_config.samplebits;
                    break;

                default:
                    err = -RT_ERROR;
                    break;
            }
            break;

        case AUDIO_TYPE_MIXER: /* report the Mixer Units */
            switch (caps->sub_type)
            {
                case AUDIO_MIXER_QUERY:
                    caps->udata.mask = AUDIO_MIXER_VOLUME;
                    break;

                case AUDIO_MIXER_VOLUME:
                    caps->udata.value =  h_drv->volume;
                    break;

                default:
                    err = -RT_ERROR;
                    break;
            }
            break;

        default:
            err = -RT_ERROR;
            break;
    }

    return err;
}

static rt_err_t sound_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    struct stm32_audio_device *h_drv;
    rt_err_t err = -RT_ERROR;

    RT_ASSERT(audio != RT_NULL);
    h_drv = (struct stm32_audio_device *)audio->parent.user_data;

    switch (caps->main_type)
    {
        case AUDIO_TYPE_MIXER:
            switch (caps->sub_type)
            {
                case AUDIO_MIXER_VOLUME:
                {
                    rt_uint8_t volume = caps->udata.value;

                    if (h_drv->codec_ops->set_volume)
                    {
                        err = h_drv->codec_ops->set_volume(volume);
                        if (err == RT_EOK)
                        {
                            h_drv->volume = volume;
                            LOG_D("set volume %d", volume);
                        }
                    }
                    break;
                }

                default:
                    err = -RT_ERROR;
                    break;
            }
            break;

        case AUDIO_TYPE_OUTPUT:
            switch (caps->sub_type)
            {
                case AUDIO_DSP_PARAM:
                    /* set samplerate */
                    err = audio_sai_config(&caps->udata.config);
                    if (err == RT_EOK)
                    {
                        /* save configs */
                        h_drv->replay_config.samplerate = caps->udata.config.samplerate;
                        h_drv->replay_config.channels   = caps->udata.config.channels;
                        h_drv->replay_config.samplebits = caps->udata.config.samplebits;
                        h_drv->codec_ops->set_frequency(caps->udata.config.samplerate);
                    }
                    break;

                case AUDIO_DSP_SAMPLERATE:
                    caps->udata.config.samplerate = h_drv->replay_config.samplebits;
                    caps->udata.config.channels   = h_drv->replay_config.channels;
                    err = audio_sai_config(&caps->udata.config);
                    if (err == RT_EOK)
                    {
                        h_drv->replay_config.samplerate = caps->udata.config.samplerate;
                    }
                    h_drv->codec_ops->set_frequency(caps->udata.config.samplerate);
                    break;

                case AUDIO_DSP_CHANNELS:
                    caps->udata.config.samplerate = h_drv->replay_config.samplerate;
                    caps->udata.config.samplebits = h_drv->replay_config.samplebits;
                    err = audio_sai_config(&caps->udata.config);
                    if (err == RT_EOK)
                    {
                        h_drv->replay_config.channels = caps->udata.config.channels;
                    }
                    break;

                case AUDIO_DSP_SAMPLEBITS:
                    caps->udata.config.samplerate = h_drv->replay_config.samplerate;
                    caps->udata.config.channels   = h_drv->replay_config.channels;
                    err = audio_sai_config(&caps->udata.config);
                    if (err == RT_EOK)
                    {
                        h_drv->replay_config.samplebits = caps->udata.config.samplebits;
                    }
                    break;

                default:
                    err = -RT_ERROR;
                    break;
            }
            LOG_D("set samplerate %d", h_drv->replay_config.samplerate);
            LOG_D("set bit rate   %d", h_drv->replay_config.samplebits);
            LOG_D("set channel    %d", h_drv->replay_config.channels);
            break;

        default:
            break;
    }

    LOG_D("sound_configure err %d", err);
    if (err != RT_EOK)
    {
        RT_ASSERT(0);
    }

    return err;
}

static rt_err_t sound_init(rt_uint8_t Volume ,struct rt_audio_device *audio)
{
    rt_err_t err = RT_EOK;
    struct stm32_audio_device *h_drv;

    RT_ASSERT(audio != RT_NULL);
    h_drv = (struct stm32_audio_device *)audio->parent.user_data;

    h_drv->bsp_init = RT_FALSE;
    if (audio_out_init(h_drv) != RT_EOK)
    {
        LOG_E("sound_init error");
        return -RT_ERROR;
    }
    h_drv->tx_start = RT_FALSE;

    /* Initialize the audio codec internal registers */
    if (h_drv->codec_ops->init(h_drv->volume, DEFAULT_SAMPLING_RATE) != 0)
    {
        LOG_D("codec init error");
        return -RT_ERROR;
    }

    HAL_SAI_DMAStop(&h_drv->audio_sai_tx_handle);

    return err;
}

static rt_err_t sound_start(struct rt_audio_device *audio, int stream)
{
    struct stm32_audio_device *h_drv;
    rt_err_t err = RT_EOK;

    uint8_t TxData[2] = {0x00, 0x00};

    RT_ASSERT((audio != RT_NULL) && (stream < AUDIO_STREAM_RECORD));
    h_drv = (struct stm32_audio_device *)audio->parent.user_data;

    if (HAL_SAI_Transmit(&h_drv->audio_sai_tx_handle, TxData, 2, 1000) != HAL_OK)
    {
      return -RT_ERROR;
    }

    if (audio->ops->transmit)
    {
        audio->ops->transmit(audio, audio->replay->buf_info.buffer, RT_NULL, DMA_TX_BLOCK_SIZE);
    }

    if (h_drv->codec_ops->play)
    {
        err = h_drv->codec_ops->play();
        if (err != RT_EOK)
        {
            return -RT_ERROR;
        }
    }
    
    return err;
}

static rt_err_t sound_stop(struct rt_audio_device *audio, int stream)
{
    struct stm32_audio_device *h_drv;
    rt_err_t err = RT_EOK;

    RT_ASSERT((audio != RT_NULL) && (stream < AUDIO_STREAM_RECORD));
    h_drv = (struct stm32_audio_device *)audio->parent.user_data;

    if (h_drv->codec_ops->stop)
    {
        err = h_drv->codec_ops->stop(RT_FALSE);
        if (err != RT_EOK)
        {
            return -RT_ERROR;
        }
    }

    if (stream == AUDIO_STREAM_DONE)
    {
        rt_thread_mdelay(200);
    }

    HAL_SAI_DMAStop(&h_drv->audio_sai_tx_handle);
    rt_memset(h_drv->audio_tx_buf, 0, DMA_TX_BLOCK_SIZE);
    h_drv->tx_start = RT_FALSE;

    return err;
}

static rt_ssize_t sound_transmit(struct rt_audio_device *audio, const void *writeBuf, void *readBuf, rt_size_t size)
{
    struct stm32_audio_device *h_drv;
    HAL_StatusTypeDef status;

    RT_ASSERT(writeBuf != RT_NULL);
    h_drv = (struct stm32_audio_device *)audio->parent.user_data;

    if (!h_drv->tx_start)
    {
        status = HAL_SAI_Transmit_DMA(&h_drv->audio_sai_tx_handle, (rt_uint8_t *)writeBuf, size);
        if (status != HAL_OK)
        {
            RT_ASSERT(0);
            return 0;
        }
        h_drv->tx_start = RT_TRUE;
    }

    return size;
}

static void sound_buffer_info(struct rt_audio_device *audio, struct rt_audio_buf_info *info)
{
    struct stm32_audio_device *h_drv = (struct stm32_audio_device *)audio->parent.user_data;
   
    h_drv->audio_tx_buf = (rt_uint8_t *)rt_malloc(DMA_TX_BUF_SIZE);
    RT_ASSERT(h_drv->audio_tx_buf != RT_NULL);

    rt_memset(h_drv->audio_tx_buf, 0, DMA_TX_BUF_SIZE);
    info->buffer      = (rt_uint8_t *)h_drv->audio_tx_buf;
    info->total_size  = DMA_TX_BUF_SIZE;
    info->block_size  = DMA_TX_BLOCK_SIZE;
    info->block_count = DMA_TX_BLOCK_COUNT;
    info->free_size   = DMA_TX_BUF_SIZE;
    h_drv->buf_info   = info;
}

int audio_board_init()
{
    struct stm32_audio_device *h_drv = &stm32_sound_drv;

    if ((BSP_CLOCK_SOURCE_FREQ_MHZ != 8000000) && (BSP_CLOCK_SOURCE_FREQ_MHZ != 25000000))
    {
        return -RT_EINVAL;
    }

    if (!IS_SAI_AUDIO_FREQUENCY(SAI_AUDIO_FREQUENCY_16K))
    {
        rt_kprintf("[INIT] frequency setting error\r\n");
        return -RT_EINVAL;
    }
    h_drv->replay_config.samplerate = SAI_AUDIO_FREQUENCY_16K;

    h_drv->replay_config.channels = DEFAULT_CHANNELS;

    h_drv->replay_config.samplebits = SAI_AUDIO_FREQUENCY_16K;
    if (h_drv->replay_config.samplebits == 0xffff)
    {
        return -RT_EINVAL;
    }
    h_drv->volume = AUDIO_VOLUME_DEFAULT;
    h_drv->tx_start = RT_FALSE;
    
    /* register sound device */
    h_drv->audio.ops = &snd_ops;

    return rt_audio_register(&h_drv->audio, AUDIO_SOUND_DEVICE_NAME, RT_DEVICE_FLAG_WRONLY, h_drv);
}
INIT_DEVICE_EXPORT(audio_board_init);

void audio_info(uint8_t argc, char **argv)
{
    struct stm32_audio_device *h_drv = &stm32_sound_drv;

    rt_kprintf("=== AUDIO INFO ===\r\n");
    rt_kprintf("SAMPLING RATE   : %d\r\n", h_drv->replay_config.samplerate);
    rt_kprintf("CHANNEL         : %s\r\n", (h_drv->replay_config.channels == 1) ? "MONO" : "STEREO");
    rt_kprintf("RESOLUTION BITS : %d\r\n", h_drv->replay_config.samplebits);
    rt_kprintf("volume          : %d\r\n", h_drv->volume);
}
MSH_CMD_EXPORT(audio_info, audio information);

#endif // RT_USING_AUDIO
