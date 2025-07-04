#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>

#include "drivers/i2c.h"
#include "drv_audio.h"
#include "drv_TLV320AIC310x.h"
#include "drivers/audio.h"

#include <math.h>

//#define DRV_TLV_DEBUG              1

#ifdef TLV320AIC310x

#if (DRV_TLV_DEBUG)
    #define tlv_debug(fmt, ...)    {rt_kprintf("[TLV] "); rt_kprintf(fmt, ##__VA_ARGS__);}
#else
    #define tlv_debug(fmt, ...);
#endif

#define PAGE_CTRL_REGISTER          0x00
#define TLV320AIC310x_I2C_ADDR       0x18

struct reg_names_tag 
{
    rt_uint16_t reg;
    rt_uint8_t reg_num;
    const char* name;
};
typedef struct reg_names_tag reg_names_t;

const reg_names_t register_table[] = 
{
    {AIC310X_REG(0, 0), 0, "AIC310X_PAGECTL"},
    {AIC310X_REG(0, 1), 1, "AIC310X_RESET"},
    {AIC310X_REG(0, 2), 2, "AIC310X_SAMPLERATE"},
    {AIC310X_REG(0, 3), 3, "AIC310X_PLLQP"},
    {AIC310X_REG(0, 4), 4, "AIC310X_PLLJ"},
    {AIC310X_REG(0, 5), 5, "AIC310X_PLLD1"},
    {AIC310X_REG(0, 6), 6, "AIC310X_PLLD2"},
    {AIC310X_REG(0, 7), 7, "AIC310X_DATA_PATH"},
    {AIC310X_REG(0, 8), 8, "AIC310X_AUDIO_INTERFACE_1"},
    {AIC310X_REG(0, 9), 9, "AIC310X_AUDIO_INTERFACE_2"},
    {AIC310X_REG(0, 10), 10, "AIC310X_AUDIO_INTERFACE_3"},
    {AIC310X_REG(0, 11), 11, "AIC310X_PLLR"},
    {AIC310X_REG(0, 12), 12, "AIC310X_DIGITAL_FILTER"},
    {AIC310X_REG(0, 13), 13, "AIC310X_HDSET_DETECT"},
    {AIC310X_REG(0, 14), 14, "AIC310X_HDSET_CONFIG"},
    {AIC310X_REG(0, 37), 37, "AIC310X_DAC_POWER_AND_HPLCOM"},
    {AIC310X_REG(0, 38), 38, "AIC310X_HPRCOM"},
    {AIC310X_REG(0, 41), 41, "AIC310X_DAC_OUTPUT_SWITCH"},
    {AIC310X_REG(0, 42), 42, "AIC310X_DEPOP_TIME"},
    {AIC310X_REG(0, 43), 43, "AIC310X_DAC_LEFT_MUTE_AND_GAIN"},
    {AIC310X_REG(0, 44), 44, "AIC310X_DAC_RIGHT_MUTE_AND_GAIN"},
    {AIC310X_REG(0, 46), 46, "AIC310X_HPLOUT_PGA_L_VOLUME"},
    {AIC310X_REG(0, 47), 47, "AIC310X_HPLOUT_DAC_L1_VOLUME"},
    {AIC310X_REG(0, 49), 49, "AIC310X_HPLOUT_PGA_R_VOLUME"},
    {AIC310X_REG(0, 50), 50, "AIC310X_HPLOUT_DAC_R1_VOLUME"},
    {AIC310X_REG(0, 51), 51, "AIC310X_HPLOUT_SETUP"},
    {AIC310X_REG(0, 53), 53, "AIC310X_HPLCOM_PGA_L_VOLUME"},
    {AIC310X_REG(0, 54), 54, "AIC310X_HPLCOM_DAC_L1_VOLUME"},
    {AIC310X_REG(0, 56), 56, "AIC310X_HPLCOM_PGA_R_VOLMUE"},
    {AIC310X_REG(0, 57), 57, "AIC310X_HPLCOM_DAC_R1_VOLUME"},
    {AIC310X_REG(0, 58), 58, "AIC310X_HPLCOM_SETUP"},
    {AIC310X_REG(0, 60), 60, "AIC310X_HPROUT_PGA_L_VOLUME"},
    {AIC310X_REG(0, 61), 61, "AIC310X_HPROUT_DAC_L1_VOLUME"},
    {AIC310X_REG(0, 63), 63, "AIC310X_HPROUT_PGA_R_VOLUME"},
    {AIC310X_REG(0, 64), 64, "AIC310X_HPROUT_DAC_R1_VOLUME"},
    {AIC310X_REG(0, 65), 65, "AIC310X_HPROUT_SETUP"},
    {AIC310X_REG(0, 67), 67, "AIC310X_HPRCOM_PGA_L_VOLUME"},
    {AIC310X_REG(0, 68), 68, "AIC310X_HPRCOM_DAC_L1_VOLUME"},
    {AIC310X_REG(0, 70), 70, "AIC310X_HPRCOM_PGA_R_VOLMUE"},
    {AIC310X_REG(0, 71), 71, "AIC310X_HPRCOM_DAC_R1_VOLUME"},
    {AIC310X_REG(0, 72), 72, "AIC310X_HPRCOM_SETUP"},
    {AIC310X_REG(0, 81), 81, "AIC310X_LEFT_LINEOUT_PGA_L"},
    {AIC310X_REG(0, 82), 82, "AIC310X_LEFT_LINEOUT_DAC_L1"},
    {AIC310X_REG(0, 84), 84, "AIC310X_LEFT_LINEOUT_PGA_R"},
    {AIC310X_REG(0, 85), 85, "AIC310X_LEFT_LINEOUT_DAC_R1"},
    {AIC310X_REG(0, 86), 86, "AIC310X_LEFT_LINEOUT_SETUP"},
    {AIC310X_REG(0, 88), 88, "AIC310X_RIGHT_LINEOUT_PGA_L"},
    {AIC310X_REG(0, 89), 89, "AIC310X_RIGHT_LINEOUT_DAC_L1"},
    {AIC310X_REG(0, 91), 91, "AIC310X_RIGHT_LINEOUT_PGA_R"},
    {AIC310X_REG(0, 92), 92, "AIC310X_RIGHT_LINEOUT_DAC_R1"},
    {AIC310X_REG(0, 93), 93, "AIC310X_RIGHT_LINEOUT_SETUP"},
    {AIC310X_REG(0, 101), 101, "AIC310X_CLOCK"},
    {AIC310X_REG(0, 102), 102, "AIC310X_CLOCK_GEN"},
};

struct reg_default
{
    rt_uint16_t reg;
    rt_uint16_t val;
};

static const struct reg_default aic310x_reg_defaults[] =
{
    { AIC310X_DATA_PATH, 0x0A },
    { AIC310X_HDSET_DETECT, 0x8A},
    { AIC310X_DAC_POWER, 0xC0 },
    { AIC310X_DAC_PATH, 0x02 },
    { AIC310X_DEPOP, 0x40 },
    { AIC310X_DAC_LEFT_VOL, 0x00 },
    { AIC310X_DAC_RIGHT_VOL, 0x00 },
    { AIC310X_LEFT_LO_DAC_L1, 0x80 },
    { AIC310X_HPLOUT_DAC_L1, 0x80 },
    { AIC310X_LEFT_LO_SETUP, 0x99 },
    { AIC310X_HPLOUT_SETUP, 0x99 },
};

const float analog_gain_table[] = 
{
      0.0,  -0.5,  -1.0,  -1.5,  -2.0,  -2.5,  -3.0,  -3.5,  -4.0,  -4.5,
     -5.0,  -5.5,  -6.0,  -6.5,  -7.0,  -7.5,  -8.0,  -8.5,  -9.0,  -9.5,
    -10.0, -10.5, -11.0, -11.5, -12.0, -12.5, -13.0, -13.5, -14.0, -14.5,
    -15.0, -15.5, -16.0, -16.5, -17.0, -17.5, -18.0, -18.6, -19.1, -19.6,
    -20.1, -20.6, -21.1, -21.6, -22.1, -22.6, -23.1, -23.6, -24.1, -24.6,
    -25.1, -25.6, -26.1, -26.6, -27.1, -27.6, -28.1, -28.6, -29.1, -29.6,
    -30.1, -30.6, -31.1, -31.6, -32.1, -32.6, -33.1, -33.6, -34.1, -34.6,
    -35.1, -35.7, -36.1, -36.7, -37.1, -37.7, -38.2, -38.7, -39.2, -39.7,
    -40.2, -40.7, -41.2, -41.7, -42.1, -42.7, -43.2, -43.8, -44.3, -44.8,
    -45.2, -45.8, -46.2, -46.7, -47.4, -47.9, -48.2, -48.7, -49.3, -50.0,
    -50.3, -51.0, -51.4, -51.8, -52.2, -52.7, -53.7, -54.2, -55.3, -56.7,
    -58.3, -60.2, -62.7, -64.3, -66.2, -68.7, -72.2, -78.3, -78.3, -78.3,
    -78.3, -78.3, -78.3, -78.3, -78.3, -78.3, -78.3, -78.3
};

struct tlv_handle_tag
{
    rt_device_t i2c_dev;
    struct rt_i2c_bus_device *i2c_bus;
    rt_uint8_t  cur_page;
};
typedef struct tlv_handle_tag tlv_handle_t;

tlv_handle_t __tlv_handle;

codec_driver_t tlv320aic310x_drv;

static const char *lookup_reg_name(rt_uint16_t address) 
{
    rt_uint32_t idx;
    const reg_names_t *tbl_ptr;

    for (idx = 0; idx < sizeof(register_table)/sizeof(register_table[0]); idx++) 
    {
        tbl_ptr = &register_table[idx];
        if (tbl_ptr->reg == address) {
            return tbl_ptr->name;
        }
    }

    return "Unknown Register";
}

/* Extract page and register address from AIC310X_REG macro */
static rt_uint8_t get_page(rt_uint16_t reg) 
{
    return (reg / 128);
}

rt_uint8_t get_register(rt_uint16_t reg) 
{
    return reg % 128;
}

static rt_err_t i2c_change_page(tlv_handle_t *h_tlv, rt_uint8_t page)
{
    struct rt_i2c_msg msgs[2];
    rt_ssize_t        returns;
    rt_uint8_t        wrbuf[2], rdbuf[2];

    RT_ASSERT(page < 3);

    /* put register address and data    */
    wrbuf[0] = AIC310X_PAGECTL;
    wrbuf[1] = page;

    /* build i2c message    */
    msgs[0].addr  = TLV320AIC310x_I2C_ADDR;
    msgs[0].flags = (I2C_OPFLAG_MASK | RT_I2C_WR);
    msgs[0].buf   = wrbuf;
    msgs[0].len   = 2;

    /* i2c write operation */
    returns = rt_i2c_transfer(h_tlv->i2c_bus, msgs, 1);
    if (returns <= 0)
    {
        tlv_debug("change page write error\r\n");
        return -RT_EIO;
    }

    /* put register address and data    */
    wrbuf[0] = AIC310X_PAGECTL;

    /* build i2c message for register address write    */
    msgs[0].addr  = TLV320AIC310x_I2C_ADDR;
    msgs[0].flags = (I2C_OPFLAG_MASK | RT_I2C_WR);
    msgs[0].buf   = wrbuf;
    msgs[0].len   = 1;

    /* build i2c message for register address read    */
    msgs[1].addr  = TLV320AIC310x_I2C_ADDR;
    msgs[1].flags = (I2C_OPFLAG_MASK | RT_I2C_RD);
    msgs[1].buf   = rdbuf;
    msgs[1].len   = 1;

    /* i2c write/read operation */
    returns = rt_i2c_transfer(h_tlv->i2c_bus, msgs, 2);
    if (returns <= 0)
    {
        tlv_debug("change page i2c read error\r\n");
        return -RT_EIO;
    }

    if (rdbuf[0] != page)
    {
        tlv_debug("change page incorrect\r\n");
        return -RT_EFAULT;
    }
    else
    {
        h_tlv->cur_page = rdbuf[0];
        tlv_debug("change page to %d\r\n", rdbuf[0]);
    }

    return RT_EOK;
}

/* Function to set the active page */
static rt_ssize_t i2c_reg_write(rt_uint16_t reg_addr, rt_uint8_t data) 
{
    tlv_handle_t *h_tlv = &__tlv_handle;
    struct rt_i2c_msg msgs[1];
    rt_ssize_t        returns;
    rt_err_t          err;
    rt_uint8_t        wrbuf[2];
    rt_uint8_t        page = get_page(reg_addr);
    rt_uint8_t        reg  = get_register(reg_addr);

    if (h_tlv->cur_page != page) 
    {
        err = i2c_change_page(h_tlv, page);
#if (DRV_TLV_DEBUG)
        RT_ASSERT(err == RT_EOK);
#else
        UNUSED(err);
#endif
    }

    /* put reg address and write data   */
    wrbuf[0] = reg;
    wrbuf[1] = data;

    /* build i2c data   */
    msgs[0].addr  = TLV320AIC310x_I2C_ADDR;
    msgs[0].flags = (I2C_OPFLAG_MASK | RT_I2C_WR);
    msgs[0].buf   = wrbuf;
    msgs[0].len   = 2;

    /* i2c write/read operation */
    returns = rt_i2c_transfer(h_tlv->i2c_bus, msgs, 1);
    if (returns <= 0)
    {
#if (DRV_TLV_DEBUG)
        tlv_debug("i2c write error %s : %d\r\n", lookup_reg_name(reg_addr), data);
        RT_ASSERT(0);
#endif
        return 0;
    }
#if (DRV_TLV_DEBUG)
    tlv_debug("write %s(%d) : %d succ\r\n", lookup_reg_name(reg_addr), reg_addr, data);
#endif

    return returns;

}

// Function to set the active page
static rt_uint8_t i2c_reg_read(rt_uint16_t reg_addr) 
{
    tlv_handle_t *h_tlv = &__tlv_handle;
    struct rt_i2c_msg msgs[2];
    rt_ssize_t        returns;
    rt_err_t          err;
    rt_uint8_t        wrbuf[2], rdbuf[2];
    rt_uint8_t        page = get_page(reg_addr);
    rt_uint8_t        reg  = get_register(reg_addr);

    if (h_tlv->cur_page != page) 
    {
        err = i2c_change_page(h_tlv, page);
        if (err != RT_EOK)
        {
//#if (DRV_TLV_DEBUG)
            RT_ASSERT(err == RT_EOK);
//#else
            UNUSED(err);
            return 0xff;
//#endif
        }
    }

    /* put register address write operation */
    wrbuf[0]      = reg;
    msgs[0].addr  = TLV320AIC310x_I2C_ADDR;
    msgs[0].flags = (I2C_OPFLAG_MASK | RT_I2C_WR);
    msgs[0].buf   = wrbuf;
    msgs[0].len   = 1;

    msgs[1].addr  = TLV320AIC310x_I2C_ADDR;
    msgs[1].flags = (I2C_OPFLAG_MASK | RT_I2C_RD);
    msgs[1].buf   = rdbuf;
    msgs[1].len   = 1;

    /* i2c write/read operation */
    returns = rt_i2c_transfer(h_tlv->i2c_bus, msgs, 2);
    if (returns <= 0)
    {
#if (DRV_TLV_DEBUG)
        tlv_debug("i2c read error : %s\r\n", lookup_reg_name(reg_addr));
        RT_ASSERT(0);
#endif
        return 0xff;
    }
#if (DRV_TLV_DEBUG)
    tlv_debug("read %s : %d succ\r\n", lookup_reg_name(reg_addr),  rdbuf[0]);
#endif

    return rdbuf[0];

}

static rt_uint8_t dac2reg_value(float dB) 
{
    rt_uint16_t value;
    rt_uint8_t reg_value;

    /* Clamp dB to valid range */
    if (dB > 24.0f) dB = 24.0f;     /* Max volume: +24 dB   */
    if (dB < -63.5f) dB = -63.5f;   /* Min volume: -63.5 dB */

    /* Convert dB to register value (0.5 dB steps)  */
    value = (rt_uint16_t)(dB * 2); /* Multiply by 2 to handle 0.5 dB steps  */

    /* Adjust for the unsigned register format:
     * - Positive dB values are mapped directly (0x00 to 0x30 for 0 dB to +24 dB).
     * - Negative dB values are mapped to 2's complement (0xFF to 0x82 for -0.5 dB to -63 dB).
    */
    reg_value = (value >= 0) ? value : (0x100 + value);

    /* Ensure that reserved values (0x80) are not used */
    if (reg_value == 0x80) /* Adjust to closest valid value (-63.5 dB)   */
    {
        reg_value = 0x81; 
    }

    return reg_value;
}

static rt_uint8_t reg2adc_value(float dB) 
{
    rt_uint8_t reg_value;

    /* Clamp dB to valid range */
    if (dB > 20.0f) dB = 20.0f;   /* Max volume: +20 dB */
    if (dB < -12.0f) dB = -12.0f; /* Min volume: -12 dB */

    if (dB < 0.0f) 
    {
        /* Negative dB: map -12.0 dB to 0x68, -0.5 dB to 0x7F */
        reg_value = 0x68 + (rt_uint8_t)((dB + 12.0f) * 2);
    } 
    else 
    {
        /* Non-negative dB: map 0.0 dB to 0x00, +20.0 dB to 0x28    */
        reg_value = (rt_uint8_t)(dB * 2);
    }

    return reg_value;
}

static rt_uint8_t gain2reg_value(float gainDb) 
{
    rt_uint8_t cidx = 0, index;
    float      min_diff, diff;
    
    /* Clamp gain to valid range    */
    if (gainDb > analog_gain_table[0]) 
    {
        gainDb = analog_gain_table[0];      /* Maximum gain: 0.0 dB  */
    }

    if (gainDb < analog_gain_table[127]) 
    {
        gainDb = analog_gain_table[127];    /* Minimum gain: -78.3 dB */
    }

    /* Find the closest match in the table */
    min_diff = fabs(gainDb - analog_gain_table[0]);

    for (index = 1; index < sizeof(analog_gain_table) / sizeof(analog_gain_table[0]); ++index) 
    {
        diff = fabs(gainDb - analog_gain_table[index]);
        if (diff < min_diff) 
        {
            cidx = index;
            min_diff = diff;
        }
    }

    return cidx;
}

static rt_uint8_t pgagain2reg_value(float gainDb) 
{
    /* Clamp gain to the valid range [0, 9] */
    if (gainDb < 0.0f) 
    {
        gainDb = 0.0f;
    } 
    else if (gainDb > 9.0f) 
    {
        gainDb = 9.0f;
    }

    // Convert directly to register value
    return (rt_uint8_t)gainDb;
}

static rt_uint8_t micgain2reg_value(float gainDb) 
{
    /* Clamp gain to the valid range [0.0, 59.5] */
    if (gainDb < 0.0f) 
    {
        gainDb = 0.0f;
    } 
    else if (gainDb > 59.5f) 
    {
        gainDb = 59.5f;
    }

    /* Convert directly to register value (0.5 dB steps) */
    return (rt_uint8_t)(gainDb * 2);
}

// Function to read, modify, and write back specific bits in a register
static void modify_reg(rt_uint16_t reg, rt_uint8_t mask, rt_uint8_t value) 
{
    rt_uint8_t cur_val, shft_value, new_value;
    rt_uint8_t shift = 0;

    /* determine left shift bit count from mask */
    cur_val = i2c_reg_read(reg);
    while ((((mask >> shift) & 1) == 0) && (shift < 8))
    {
        shift++;
    }

    /* Shift the value into position */
    shft_value = (value << shift) & mask;
    new_value = (cur_val & ~mask) | shft_value;


    i2c_reg_write(reg, new_value);
}

void set_dac_samplerate(rt_uint32_t samplerate)
{
    switch(samplerate)
    {
    case AUDIO_FREQUENCY_96K:
        modify_reg(AIC310X_SAMPLERATE, AIC310X_DAC_FS_MASK,0x00);
        break;
    case AUDIO_FREQUENCY_48K:
        modify_reg(AIC310X_SAMPLERATE, AIC310X_DAC_FS_MASK,0x02);
        break;
    case AUDIO_FREQUENCY_32K:
        modify_reg(AIC310X_SAMPLERATE, AIC310X_DAC_FS_MASK,0x04);
        break;
    case AUDIO_FREQUENCY_16K:
        modify_reg(AIC310X_SAMPLERATE, AIC310X_DAC_FS_MASK,0x10);
        break;
    default:
        break;
    }
}

// High-level function to set the PLL
void set_pll(rt_uint8_t pll_p, rt_uint8_t pll_r, rt_uint8_t pll_j, rt_uint16_t pll_d) 
{

}

void set_pll_power(rt_bool_t power) 
{
    modify_reg(AIC310X_PLLQP, AIC310X_PLLQP_CONTROL_MASK, power);
}

void set_clk_mux(rt_uint8_t pll_clkin, rt_uint8_t codec_clkin) 
{
    i2c_reg_write(AIC310X_CLK_SRC, 0x01);
}

void set_word_length(rt_uint8_t wordlength) 
{

}

void set_hs_detect_int1(rt_bool_t enable) 
{
    if(enable)
    {
        i2c_reg_write(AIC310X_HDSET_DETECT, 0);
        i2c_reg_write(AIC310X_HDSET_CONFIG, 0);
    }
    else
    {
        i2c_reg_write(AIC310X_HDSET_DETECT, 0);
        i2c_reg_write(AIC310X_HDSET_CONFIG, 0);
    }
}

void enable_headset_detect(rt_bool_t enable)
{
    modify_reg(AIC310X_HDSET_DETECT, AIC310X_DTT_ENB_MASK, enable);
}

rt_bool_t is_headset_detected() 
{
    rt_bool_t detect;

    detect = (i2c_reg_read(AIC310X_HDSET_DETECT) & 0x60) ? 1 : 0;

    return detect;
}


/* High-level function to enable and unmute the DAC and route it to the output mixer */
void enable_dac() 
{
}

void set_dac_mute(rt_bool_t mute)
{
    if (mute) 
    {
    } 
    else 
    {
    }
}

void set_dac_volume(float left_dB, float right_dB) 
{
    /* Convert dB values to register format */
    rt_uint8_t leftRegValue = dac2reg_value(left_dB);
    rt_uint8_t rightRegValue = dac2reg_value(right_dB);

    // Write to left and right DAC volume registers
//    i2c_reg_write(AIC310X_RDACVOL, rightRegValue); // Right DAC volume control
}

void enable_headphone_mute(rt_bool_t mute) 
{
}

void enable_headphone_gain(float left_dB, float right_dB) 
{
}

void enable_headphone_volume(float left_dB, float right_dB) 
{
}

void enable_headphone_performance(rt_uint8_t level)
{
}

void enable_headphone_linemode(rt_bool_t line)
{
}

void enable_speaker_amp() 
{
    /* TODO: also support other channel on codecs with stereo amp */
}

void enable_speaker_mute(rt_bool_t mute)
{
    modify_reg(AIC310X_DAC_LEFT_VOL, AIC310X_DAC_MUTE_MASK, mute);
    modify_reg(AIC310X_DAC_RIGHT_VOL, AIC310X_DAC_MUTE_MASK, mute);
}

void set_dac_power(rt_bool_t power)
{
    if(power)
    {
        i2c_reg_write(AIC310X_DAC_POWER, 192);
    }
    else
    {
        i2c_reg_write(AIC310X_DAC_POWER, 0);
    }
}

void enable_speaker_volume(float left_dB) 
{

}

void enable_speaker_gain(float gaindb) 
{
    /* Round the input to the nearest integer (to handle potential float inaccuracies) */
    int rounded_gain = (int)(gaindb + 0.5f);
    rt_uint8_t gain = 0;

    switch (rounded_gain) 
    {
    case 12:
        gain = 0x1;
        break;
    case 18:
        gain = 0x2;
        break;
    case 24:
        gain = 0x3;
        break;
    default:
        gain = 0x0;
    }
}

void set_micpga_enable(rt_bool_t enable) 
{
}

void set_micpga_gain(float gain) 
{
}

void set_adc_gain(float adcGain) 
{
}

void x()
{
    tlv_reset();
}

/* High-level function to software-reset the codec (also turns off all components for power saving) */
void tlv_reset() 
{
    i2c_reg_write(AIC310X_RESET, 0x1);
}

/* TLV320AIC310x driver callback function */
rt_err_t tlv320aic310x_init(rt_uint8_t volume, rt_uint32_t auido_freq)
{
    tlv_handle_t *h_tlv = &__tlv_handle;

    /* config reset output pin  */
    rt_pin_mode(TLV310x_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(TLV310x_RESET_PIN, CODEC_RESET);
    rt_thread_mdelay(3);
    rt_pin_write(TLV310x_RESET_PIN, CODEC_SET);

    h_tlv->i2c_dev = rt_device_find(I2C_BUS_NAME);
    if (h_tlv->i2c_dev == RT_NULL)
    {
        tlv_debug("TLV320AIC310x_init no device found\r\n");
        RT_ASSERT(h_tlv->i2c_dev != RT_NULL);
        return -RT_ENOSYS;
    }
    h_tlv->i2c_bus = (struct rt_i2c_bus_device *)h_tlv->i2c_dev->user_data;

    tlv320aic310x_deinit();

    for(rt_uint8_t i = 0; i<sizeof(aic310x_reg_defaults)/sizeof(aic310x_reg_defaults[0]); i++)
    {
        i2c_reg_write(aic310x_reg_defaults[i].reg, aic310x_reg_defaults[i].val);
    }

    rt_thread_mdelay(10);

    return RT_EOK;
}

void tlv320aic310x_deinit()
{
    rt_pin_write(TLV310x_RESET_PIN, CODEC_RESET);
    rt_thread_mdelay(3);
    rt_pin_write(TLV310x_RESET_PIN, CODEC_SET);
}

rt_err_t tlv320aic310x_play(void)
{

    if(is_headset_detected())
    {

    }
    else
    {

    }

    set_dac_power(RT_TRUE);

    set_dac_mute(RT_FALSE);

    return RT_EOK;
}

rt_err_t tlv320aic310x_stop(rt_bool_t power_down)
{
    set_dac_mute(RT_TRUE);

    rt_thread_mdelay(20);

    set_dac_power(RT_FALSE);

    return RT_EOK;
}

rt_err_t tlv320aic310x_set_frequency(rt_uint32_t auido_freq)
{
    if(!((auido_freq | AUDIO_FREQUENCY_96K)
       ||(auido_freq | AUDIO_FREQUENCY_48K)
       ||(auido_freq | AUDIO_FREQUENCY_32K)
       ||(auido_freq | AUDIO_FREQUENCY_16K)))
    {
        rt_kprintf("[TLV] setting audio frequency - %d is disable\r\n", auido_freq);
        return RT_ERROR;
    }
    set_dac_samplerate(auido_freq);

    return RT_EOK;
}

rt_err_t tlv320aic310x_volume(rt_uint8_t auido_volume)
{
    float vol_gain;

    if (auido_volume >= AUDIO_VOLUME_MAX)
    {
        auido_volume = AUDIO_VOLUME_MAX - 1;
    }

    vol_gain = analog_gain_table[AUDIO_VOLUME_MAX - auido_volume - 1];
    enable_speaker_volume(vol_gain);

    return RT_EOK;
}

/* MSH command */
void reg_set(uint8_t argc, char **argv)
{
    rt_uint32_t idx;
    rt_ssize_t  size;
    rt_uint16_t regaddr;
    rt_uint8_t  val, page, reg;

    if (argc < 3)
    {
        rt_kprintf("Usage : %s page reg [value]\r\n", argv[0]);
        for (idx = 0; idx < sizeof(register_table)/sizeof(register_table[0]); idx++)
        {
            val = i2c_reg_read(register_table[idx].reg);
            rt_kprintf("[REG %d] %s : %02x\r\n", register_table[idx].reg_num, lookup_reg_name(register_table[idx].reg), val);
        }
        return;
    }

    page = (rt_uint8_t)atoi(argv[1]);
    reg  = (rt_uint8_t)atoi(argv[2]); 
    regaddr = (page * 128) + reg;
    if (argc == 3)
    {
        val = i2c_reg_read(regaddr);
        rt_kprintf("READ  : %s : %d\r\n", lookup_reg_name(regaddr), val);
    }
    else
    {
        val = (rt_uint8_t)atoi(argv[3]);
        size = i2c_reg_write(regaddr, val);
        rt_kprintf("WRITE : %s : %d write size %d\r\n", lookup_reg_name(regaddr), val, size);
    }

}
MSH_CMD_EXPORT(reg_set, tlv32oaic31x set);

void cmd_vol(uint8_t argc, char **argv)
{
    rt_uint8_t cur_vol;
    rt_int8_t new_vol;
    rt_int8_t new_gain;

    if(argc < 2)
    {
        rt_kprintf("current vol = %d\r\n", cur_vol);
    }
    else if(argc == 2)
    {
        new_vol = (rt_int8_t) atoi(argv[1]);
        if(new_vol < 0 )
        {
            new_vol = 1;
        }
        else if(new_vol > 128)
        {
            new_vol = 128;
        }
        new_gain = analog_gain_table[new_vol - 1];
        tlv320aic310x_volume(new_vol);
        rt_kprintf("set volume gain = %d \r\n", -new_gain);
    }
    else
    {
        rt_kprintf("bad arguments, check cmd\r\n");
    }
}
MSH_CMD_EXPORT(cmd_vol, set volume (1 ~ 128) );

#endif //ifdef TLV320AIC310x
