/**
  ******************************************************************************
  * @file    cs42l51.c
  * @author  MCD Application Team
  * @brief   This file provides the CS42L51 Audio Codec driver.   
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

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "drv_common.h"
#include "drv_cs42l5x.h"
#include "drivers/i2c.h"
#include <drivers/audio.h>

#ifdef CS42L5x

#define DRV_DODEC_DEBUG         1
#if (DRV_DODEC_DEBUG)
    #define cod_debug(fmt, ...)    {rt_kprintf("[I2C] "); rt_kprintf(fmt, ##__VA_ARGS__);}
#else
    #define cod_debug(fmt, ...);
#endif

#define AUDIO_I2C_ADDRESS                       ((uint16_t) 0x4A)
#define I2C_OPFLAG_MASK                         0x00      /* 7 bits address, use start. use stop, wait ack */

#define CODEC_GET_DEFAULT_VALUE(x)              ((x >> 8) & 0xff)
#define CODEC_GET_INIT_VALUE(x)                 (x & 0xff)


#if !defined (VERIFY_WRITTENDATA)  
#define VERIFY_WRITTENDATA
#endif /* VERIFY_WRITTENDATA */

struct cs42l5x_handle_tag
{
    rt_device_t i2c_dev;
    struct rt_i2c_bus_device *i2c_bus;

    rt_bool_t init_done;
    rt_bool_t instop;
    rt_uint16_t dev_type;
};
typedef struct cs42l5x_handle_tag cs42l5x_handle_t;

static const rt_uint16_t default_register_cs42l52[] = 
{
    /* 00 */ 0xffff,                         /* 00 : dummy address 0 register   */
    /* 01 */ 0xe0e7,                         /* 01 : Chip ID                    */
    /* 02 */ 0x0101,                         /* 02 : Power Ctl 1                */
    /* 03 */ 0x0707,                         /* 03 : Power Ctl 2                */
    /* 04 */ 0x05AF,                         /* 04 : Power Ctl 3                */
    /* 05 */ 0xa080,                         /* 05 : Clocking Ctl               */
    /* 06 */ 0x0004,                         /* 06 : Interface Ctl 1            */
    /* 07 */ 0x0000,                         /* 07 : Interface Ctl 2            */
    /* 08 */ 0x8100,                         /* 08 : Input A Select             */
    /* 09 */ 0x8100,                         /* 09 : Input B Select             */
    /* 0A */ 0xa5a5,                         /* 0A : Analog HPF Ctl             */
    /* 0B */ 0x0000,                         /* 0B : ADC HPF Corner Freq        */
    /* 0C */ 0x0000,                         /* 0C : Misc. ADC Ctl              */
    /* 0D */ 0x6010,                         /* 0D : Playback Ctl 1             */
    /* 0E */ 0x0202,                         /* 0E : Misc. Ctl                  */
    /* 0F */ 0x0032,                         /* 0F : Playback Ctl 2             */
    /* 10 */ 0x0000,                         /* 10 : MICA Amp Ctl               */
    /* 11 */ 0x0000,                         /* 11 : MICB Amp Ctl               */
    /* 12 */ 0x0000,                         /* 12 : PGAA Vol, Misc             */
    /* 13 */ 0x0000,                         /* 13 : PGAB Vol, Misc             */
    /* 14 */ 0x0000,                         /* 14 : Passthru A Vol             */
    /* 15 */ 0x0000,                         /* 15 : Passthru B Vol             */
    /* 16 */ 0x0000,                         /* 16 : ADCA Vol                   */
    /* 17 */ 0x0000,                         /* 17 : ADCB Vol                   */
    /* 18 */ 0x8080,                         /* 18 : ADCMIXA Vol                */
    /* 19 */ 0x8080,                         /* 19 : ADCMIXB Vol                */
    /* 1A */ 0x0000,                         /* 1A : PCMMIXA Vol                */
    /* 1B */ 0x0000,                         /* 1B : PCMMIXB Vol                */
    /* 1C */ 0x0000,                         /* 1C : BEEP Freq On Time          */
    /* 1D */ 0x0000,                         /* 1D : BEEP Freq Off Time         */
    /* 1E */ 0x0000,                         /* 1E : BEEP Tone Cfg.             */
    /* 1F */ 0x8080,                         /* 1F : Tone Ctl                   */
    /* 20 */ 0x0000,                         /* 20 : Master A Vol               */
    /* 21 */ 0x0000,                         /* 21 : Master B Vol               */
    /* 22 */ 0x0000,                         /* 22 : Headphone A Volume         */
    /* 23 */ 0x0000,                         /* 23 : Headphone B Volume         */
    /* 24 */ 0x0000,                         /* 24 : Speaker A Volume           */
    /* 25 */ 0x0000,                         /* 25 : Speaker B Volume           */
    /* 26 */ 0x0000,                         /* 26 : Channel Mixer and Swap     */
    /* 27 */ 0x0000,                         /* 27 : Limit Ctl 1 Thresholds     */
    /* 28 */ 0x7f7f,                         /* 28 : Limit Ctl 2 Thresholds     */
    /* 29 */ 0xc0c0,                         /* 29 : Limiter Attack Rate        */
    /* 2A */ 0x0000,                         /* 2A : ALC Ctl 1 Attack Rate      */
    /* 2B */ 0x3f3f,                         /* 2B : ALC Release Rate           */
    /* 2C */ 0x0000,                         /* 2C : ALC Thresholds             */
    /* 2D */ 0x0000,                         /* 2D : Noise Gate Ctl             */
    /* 2E */ 0x0000,                         /* 2E : Overflow and Clock Status  */
    /* 2F */ 0x0000,                         /* 2F : Battery Compensation       */
    /* 30 */ 0x0000,                         /* 30 : VP Battery Level           */
    /* 31 */ 0x0000,                         /* 31 : Speaker Status             */
    /* 32 */ 0x3b3b,                         /* 32 : Reserved                   */
    /* 33 */ 0x0000,                         /* 33 : Reserved                   */
    /* 34 */ 0x5f50                          /* 34 : Charge Pump Frequency      */
};

static cs42l5x_handle_t __cs_handle;

/** @defgroup STM32L496G_DISCOVERY_BusOperations_Functions Bus Operations Functions
  * @{
  */

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
/******************************* I2C Routines**********************************/
/**
  * @brief Discovery I2C2 Bus initialization
  * @retval None
  */
static rt_ssize_t i2c_reg_write(rt_uint8_t reg, rt_uint8_t data)
{
    cs42l5x_handle_t *h_codec = &__cs_handle;
    struct rt_i2c_msg msgs[1];
    rt_ssize_t        returns;
    rt_uint8_t        wrbuf[2];

    /* put reg address and write data   */
    wrbuf[0] = reg;
    wrbuf[1] = data;

    /* build i2c data   */
    msgs[0].addr  = AUDIO_I2C_ADDRESS; /* 0x4A */
    msgs[0].flags = (I2C_OPFLAG_MASK | RT_I2C_WR);
    msgs[0].buf   = wrbuf;
    msgs[0].len   = 2;

    /* i2c write/read operation */
    returns = rt_i2c_transfer(h_codec->i2c_bus, msgs, 1);
    if (returns <= 0)
    {
#if (DRV_DODEC_DEBUG)
        cod_debug("i2c write error 0x%02X : %d\r\n", reg, data);
        RT_ASSERT(0);
#endif
        return 0;
    }

    return returns;

}

// Function to set the active page
static rt_uint8_t i2c_reg_read(rt_uint8_t reg_addr)
{
    cs42l5x_handle_t *h_codec = &__cs_handle;
    struct rt_i2c_msg msgs[2];
    rt_ssize_t        returns;
    rt_uint8_t        wrbuf[2], rdbuf[2];

    /* put register address write operation */
    wrbuf[0]      = reg_addr;
    msgs[0].addr  = AUDIO_I2C_ADDRESS; /* 4A */
    msgs[0].flags = (I2C_OPFLAG_MASK | RT_I2C_WR);
    msgs[0].buf   = wrbuf;
    msgs[0].len   = 1;

    msgs[1].addr  = AUDIO_I2C_ADDRESS;
    msgs[1].flags = (I2C_OPFLAG_MASK | RT_I2C_RD);
    msgs[1].buf   = rdbuf;
    msgs[1].len   = 1;

    /* i2c write/read operation */
    returns = rt_i2c_transfer(h_codec->i2c_bus, msgs, 2);
    if (returns <= 0)
    {
#if (DRV_DODEC_DEBUG)
        cod_debug("i2c read error \r\n");
        RT_ASSERT(0);
#endif
        return 0xff;
    }

    return rdbuf[0];

}

static rt_err_t i2c2_init(void)
{
    cs42l5x_handle_t *h_codec = &__cs_handle;

    /* config reset output pin  */
#if defined (RESET_PIN)
    rt_pin_mode(RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RESET_PIN, CODEC_RESET);
    rt_thread_mdelay(3);
    rt_pin_write(RESET_PIN, CODEC_SET);
#endif

    h_codec->i2c_dev = rt_device_find(I2C_BUS_NAME);
    if (h_codec->i2c_dev == RT_NULL)
    {
        cod_debug("cs42l5x_init no device found\r\n");
        RT_ASSERT(h_codec->i2c_dev != RT_NULL);
        return -RT_ENOSYS;
    }
    h_codec->i2c_bus = (struct rt_i2c_bus_device *)h_codec->i2c_dev->user_data;

    return RT_EOK;
}

/**
  * @brief  Deinitializes Audio low level.
  * @retval None
  */
void codec_deinit(void)
{
    rt_uint8_t Value;

    /* Mute DAC and ADC */
    Value = i2c_reg_read(0x08);
    i2c_reg_write(0x08, (Value | 0x03));
    Value = i2c_reg_read(0x07);
    i2c_reg_write(0x07, (Value | 0x03));

    /* Disable soft ramp and zero cross */
    Value = i2c_reg_read(0x06);
    i2c_reg_write(0x06, (Value & 0xF0));

    /* Set PDN to 1 */
    Value = i2c_reg_read(0x02);
    i2c_reg_write(0x02, (Value | 0x01));

    /* Set all power down bits to 1 */
    i2c_reg_write(0x02, 0x7F);
    Value = i2c_reg_read(0x03);
    i2c_reg_write(0x03, (Value | 0x0E));

    return;
}

/**
  * @brief  AUDIO Codec delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void codec_delay_ms(rt_uint32_t Delay)
{
    rt_thread_mdelay(Delay);
}

/**
  * @brief  Deinitialize the audio codec.
  * @param  None
  * @retval  None
  */
static void cs42l52_deinit()
{
    uint16_t i = 0;
    rt_uint8_t tmp = 0; 

  
    /* Enable Master Playback Mute @0x0D */
    i2c_reg_write(0x02, 0x03);

    /* Power down CS42L52 @0x03 */    
    i2c_reg_write(0x03, 0x07);

    /* Power down CS42L52 @0x04 */
    i2c_reg_write(0x04, 0xFF);

    /* config interface @0x06 */
    i2c_reg_write(0x06, 0x03);

    /* Disable Analog (Soft Ramp & Zero Cross) and HighPassFilter @0x0A */
    i2c_reg_write(0x0A, 0x00);

    /* Digital Soft Ramp on & Zero Cross off @0x0E */
    i2c_reg_write(0x0E, 0x02);

    /* Headphone Mute, Speaker Mute @0x0F */
    i2c_reg_write(0x0F, 0xF2);

    /* PCM A Volume: Mute @0x1A */
    i2c_reg_write(0x1A, 0x80);

    /* PCM B Volume: Mute @0x1B */
    i2c_reg_write(0x1B, 0x80);

    /* Limiter Attack Rate: 0x00 @0x29 */
    i2c_reg_write(0x29, 0x00);
  
    i = 0;
    i2c_reg_write(0x02, 0x9F);
    do {
        // Power down CS42L52 @0x02
        tmp = i2c_reg_read(0x02);
        i++;
    } while (((tmp & 0x01) == 0) && (i < 10));
   
    codec_delay_ms(10);

    return;
}

/***********************************************************************************************
    * ops functions
    typedef struct
    {
        rt_err_t  (*init)(rt_uint8_t volume, rt_uint32_t auido_freq);
        rt_err_t  (*play)(rt_uint8_t volume);
        rt_err_t  (*stop)(rt_bool_t powerdown_mode);
        rt_err_t  (*set_frequency)(rt_uint32_t auido_freq);
        rt_err_t  (*set_volume)(rt_uint8_t new_vol);
    } codec_driver_t;
************************************************************************************************/
/**
  * @brief Initialize the audio codec and the control interface.
  * @param Volume:     Initial output volume level (from 0 (-100dB) to 100 (0dB)).
  * @param AudioFreq:  Initial audio frequency (currently not used).
  * @retval RT_EOK if correct communication, else wrong communication.
  */

static rt_uint32_t cs42l52_power_set(rt_bool_t turnon)
{
    rt_uint32_t counter, idx;
    rt_uint8_t  tmp, comp;

    tmp  = turnon ? 0 : 0x9F;
    comp = tmp;
    /* Power down the components */
    counter = i2c_reg_write(0x02, tmp);
    idx = 0;
    do {
        tmp = i2c_reg_read(0x02);
        idx++;
    } while (((tmp & comp) != comp) && (idx < 100));  
    rt_kprintf("PDN(%d) %d\r\n", tmp, idx);

    return counter;
}
rt_err_t cs42l52_init(rt_uint8_t Volume, rt_uint32_t AudioFreq)
{
    cs42l5x_handle_t  *h_codec = &__cs_handle;
    rt_uint32_t       counter = 0; 
    const rt_uint16_t *p_reg = default_register_cs42l52;

    /* Check if codec is already initialized */
    if (!h_codec->init_done)
    {

        /* Initialize the Control interface of the Audio Codec */
        i2c2_init();

#if defined (RESET_PIN)
        /* Power off the codec */
        rt_pin_mode(RESET_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(RESET_PIN, PIN_RESET);
        codec_delay_ms(10);
        rt_pin_write(RESET_PIN, PIN_SET);
#endif

        cs42l52_deinit();       
        h_codec->init_done = RT_TRUE;
    }

    /* Set the device power down */
    counter += i2c_reg_write(0x02, 0x01);

    /* Set the device output mode */
    counter += i2c_reg_write(0x04, CS42L5x_OUTPUT_MODE);
       
    /* Clock configuration: Auto detection */  
    counter += i2c_reg_write(0x05, CODEC_GET_INIT_VALUE(p_reg[0x05]));
  
    /* Set the Slave Mode and the audio Standard */  
    counter += i2c_reg_write(0x06, CODEC_GET_INIT_VALUE(p_reg[0x06]));
  
    /* Interface Control 2: SCLK is Re-timed signal from MCLK*/
    counter +=i2c_reg_write(0x07, CODEC_GET_INIT_VALUE(p_reg[0x07])); 
  
    /* ADCA and PGAA Select: no input selected*/
    counter +=i2c_reg_write(0x08, CODEC_GET_INIT_VALUE(p_reg[0x08]));

    /* ADCB and PGAB Select: no input selected*/
    counter +=i2c_reg_write(0x09, CODEC_GET_INIT_VALUE(p_reg[0x09])); 

    /*Play Back Control 1: headphone gain is 0.4, PCM not inverted, Master not mute*/
    counter +=i2c_reg_write(0x0D, CODEC_GET_INIT_VALUE(p_reg[0x0D]));

    /* Miscellaneous Controls: Passthrough Analog & Passthrough Mute off, Soft Ramp on @0x0E*/
    counter +=i2c_reg_write(0x0E, CODEC_GET_INIT_VALUE(p_reg[0x0E]));  

    /* Play Back Control 2: Headphone Mute off, speaker mute off, mono enabled */
    counter +=i2c_reg_write(0x0F, CODEC_GET_INIT_VALUE(p_reg[0x0F])); 

    /* PCM A Volume: PCM Mute disabled, Volume is 0db(default) */
    counter +=i2c_reg_write(0x1A, CODEC_GET_INIT_VALUE(p_reg[0x1A]));

    /* PCM B Volume: PCM Mute disabled, Volume is 0db(default) */
    counter +=i2c_reg_write(0x1B, CODEC_GET_INIT_VALUE(p_reg[0x1B])); 

    /* Headphone A Volume: Headphone Volume is -6db */
    counter +=i2c_reg_write(0x22, CODEC_GET_INIT_VALUE(p_reg[0x22]));

    /* Headphone B Volume: Headphone Volume is -6db */
    counter +=i2c_reg_write(0x23, CODEC_GET_INIT_VALUE(p_reg[0x23]));

    /* Speaker A Volume: Speaker Volume is 0db (default) */
    counter +=i2c_reg_write(0x24, CODEC_GET_INIT_VALUE(p_reg[0x24]));

    /* Speaker B Volume: Speaker Volume is 0db (default) */
    counter +=i2c_reg_write(0x25, CODEC_GET_INIT_VALUE(p_reg[0x25]));

    /* Charge Pump Frequency: 5 (default) */
    counter +=i2c_reg_write(0x34, CODEC_GET_INIT_VALUE(p_reg[0x34]));

    /* Power Control 1: power up */
    counter += i2c_reg_write(0x02, 0); 

    /* Set the Master volume */
    counter += cs42l52_set_volume(0);
    h_codec->instop = RT_TRUE;

    /* Return communication control value */
    return counter;  
}

/**
  * @brief Start the audio Codec play feature.
  * @note For this codec no Play options are required.
  * @param DeviceAddr: Device address on communication Bus.   
  * @retval 0 if correct communication, else wrong communication
  */
rt_err_t cs42l52_play()
{
    cs42l5x_handle_t  *h_codec = &__cs_handle;
    rt_uint32_t          counter = 0;

    if (h_codec->instop == 1)
    {
        /* Power control : Exit standby (PDN = 0) */
        counter += i2c_reg_write(0x02, 00);
        counter += cs42l52_set_mute(AUDIO_MUTE_OFF);
        h_codec->instop = RT_FALSE;
    }

    /* Return communication control value */
    return RT_EOK;  
}

/**
  * @brief Stop audio Codec playing. It powers down the codec.
  * @param DeviceAddr: Device address on communication Bus. 
  * @param CodecPdwnMode: selects the  power down mode (currently not used).
  * @retval 0 if correct communication, else wrong communication
  */
rt_err_t cs42l52_stop(rt_bool_t CodecPdwnMode)
{
    cs42l5x_handle_t *h_codec = &__cs_handle;

    /* Mute the output first */
    cs42l52_set_mute(AUDIO_MUTE_ON);
    if (CodecPdwnMode)
    {
        cs42l52_power_set(RT_FALSE);
    }
    h_codec->instop = RT_TRUE;

    return RT_EOK;
}

/**
  * @brief Set higher or lower the codec volume level.
  * @param Volume: output volume level (from 0 (-100dB) to 100 (0dB)).
  * @retval 0 if correct communication, else wrong communication
  */
rt_err_t cs42l52_set_volume(rt_uint8_t Volume)
{
    rt_uint32_t counter = 0;
    rt_uint8_t  convertedvol = VOLUME_CONVERT(Volume);

    if (Volume > 0xE6)
    {
        /* Set the Master volume */
        counter += i2c_reg_write(0x20, convertedvol - 0xE7);
        counter += i2c_reg_write(0x21, convertedvol - 0xE7);
    }
    else
    {
        /* Set the Master volume */
        counter += i2c_reg_write(0x20, convertedvol + 0x19);
        counter += i2c_reg_write(0x21, convertedvol + 0x19);
    }

    return RT_EOK;  
}

/**
  * @brief Set new frequency.
  * @param AudioFreq: Audio frequency used to play the audio stream.
  * @retval 0 if correct communication, else wrong communication
  */
rt_err_t cs42l52_set_frequency(rt_uint32_t AudioFreq)
{
    /* not implemented  */
    return 0;
}

/***********************************************************************************************
    other helper function
***********************************************************************************************/
/**
  * @brief Enable or disable the mute feature on the audio codec.
  * @param DeviceAddr: Device address on communication Bus.   
  * @param Cmd: AUDIO_MUTE_ON to enable the mute or AUDIO_MUTE_OFF to disable the
  *             mute mode.
  * @retval 0 if correct communication, else wrong communication
  */
rt_err_t cs42l52_set_mute(rt_uint32_t Cmd)
{
    rt_uint32_t counter = 0;  
  
    /* Set the Mute mode */
    if (Cmd == AUDIO_MUTE_ON)
    {
        counter += i2c_reg_write(0x04, 0xFF);
        counter += i2c_reg_write(0x0F, 0xF0);
        cs42l52_set_volume(0);
    }
    else /* AUDIO_MUTE_OFF Disable the Mute */
    {
        counter += i2c_reg_write(0x04, CS42L5x_OUTPUT_MODE);
        counter += i2c_reg_write(0x0F, 0x02);
    }
    codec_delay_ms(20);
  
    return counter; 
}

rt_err_t cs42l52_pause()
{  
    rt_uint32_t counter = 0;

    /* Pause the audio file playing */
    /* Mute the output first */
    counter += cs42l52_set_mute(AUDIO_MUTE_ON);
 
    /* Put the Codec in Power save mode */    
    counter += i2c_reg_write(0x02, 0x01);

    return counter;
}

rt_err_t cs42l52_resume()
{
    rt_uint32_t counter = 0;

    /* Unmute the output  */
    counter += cs42l52_set_mute(AUDIO_MUTE_OFF);
  
    /* Exit the Power save mode */
    counter += i2c_reg_write(0x02, 0x9E);

    return counter;
}

/**
  * @brief  Get the CS42L5x ID.
  * @param DeviceAddr: Device address on communication Bus.   
  * @retval The CS42L5x ID 
  */
rt_uint8_t cs42l5x_read_id()
{
    cs42l5x_handle_t *h_codec = &__cs_handle;
    rt_uint8_t Value;

    if (h_codec->init_done == 0)
    {
        /* Initialize the Control interface of the Audio Codec */
        i2c2_init(); 
    }
    Value = i2c_reg_read(CS42L5x_CHIPID_ADDR);

    return((rt_uint8_t)((Value & CS42L52_ID_MASK) >> 3));
}

#if defined (RT_USING_FINSH)
    #include <finsh.h>
    void codec_set(int argc, char **argv)
    {
        cs42l5x_handle_t *h_codec = &__cs_handle;
        rt_uint8_t reg, data;

        if (!h_codec->init_done)
        {
            i2c2_init(); 
        }
        if (argc < 3)
        {
            if (argc < 2)
            {
                data = cs42l5x_read_id();
                rt_kprintf("Usage : %s [reg] <value>\r\n", argv[0]);
                rt_kprintf(" CHIP ID : %02X. should be %02X\r\n", data, CS42L52_ID);
            }
            else
            {
                reg = (rt_uint8_t)atoi(argv[1]);
                data = i2c_reg_read(reg);
                rt_kprintf("I2C READ REG ADDR : 0x%02X => 0x%02X\r\n", reg, data);
            }
        }
        else
        {
            rt_ssize_t written;

            reg     = (rt_uint8_t)atoi(argv[1]);
            data    = (rt_uint8_t)atoi(argv[2]);
            written = i2c_reg_write(reg, data);
            rt_kprintf("I2C WRITE REG ADDR : 0x%02X => %02X, write size %d\r\n", reg, data, written);
            data = i2c_reg_read(reg);
            rt_kprintf("I2C READ REG ADDR : 0x%02X => 0x%02X\r\n", reg, data);
        }
    }
    MSH_CMD_EXPORT(codec_set, codec i2c test);

    void codec_dump(int argc, char **argv)
    {
        rt_uint32_t index;
        uint8_t     rbuf;

        for (index = 0; index <= 0x34; index++)
        {

            rbuf = i2c_reg_read(index);
            rt_kprintf("REG[%02X] => %02X\r\n", index, rbuf);
        }
        return;
    }
    MSH_CMD_EXPORT(codec_dump, codec register);
#endif
#endif

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
