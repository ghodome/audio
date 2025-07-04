/* 
*/
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>

#include "drivers/i2c.h"
#include "drv_audio.h"
#include "drv_tlv320aic31xx.h"
#include "drivers/audio.h"

#include <math.h>

//#define DRV_TLV_DEBUG              1

#ifdef TLV320AIC31xx

#if (DRV_TLV_DEBUG)
    #define tlv_debug(fmt, ...)    {rt_kprintf("[TLV] "); rt_kprintf(fmt, ##__VA_ARGS__);}
#else
    #define tlv_debug(fmt, ...);
#endif

#define PAGE_CTRL_REGISTER          0x00
#define TLV320aic31x_I2C_ADDR       0x18

struct reg_names_tag 
{
    rt_uint16_t reg;
    const char* name;
};
typedef struct reg_names_tag reg_names_t;

const reg_names_t register_table[] = 
{
    {AIC31XX_REG(0, 0), "AIC31XX_PAGECTL"},
    {AIC31XX_REG(0, 1), "AIC31XX_RESET"},
    {AIC31XX_REG(0, 3), "AIC31XX_OT_FLAG"},
    {AIC31XX_REG(0, 4), "AIC31XX_CLKMUX"},
    {AIC31XX_REG(0, 5), "AIC31XX_PLLPR"},
    {AIC31XX_REG(0, 6), "AIC31XX_PLLJ"},
    {AIC31XX_REG(0, 7), "AIC31XX_PLLDMSB"},
    {AIC31XX_REG(0, 8), "AIC31XX_PLLDLSB"},
    {AIC31XX_REG(0, 11), "AIC31XX_NDAC"},
    {AIC31XX_REG(0, 12), "AIC31XX_MDAC"},
    {AIC31XX_REG(0, 13), "AIC31XX_DOSRMSB"},
    {AIC31XX_REG(0, 14), "AIC31XX_DOSRLSB"},
    {AIC31XX_REG(0, 16), "AIC31XX_MINI_DSP_INPOL"},
    {AIC31XX_REG(0, 18), "AIC31XX_NADC"},
    {AIC31XX_REG(0, 19), "AIC31XX_MADC"},
    {AIC31XX_REG(0, 20), "AIC31XX_AOSR"},
    {AIC31XX_REG(0, 25), "AIC31XX_CLKOUTMUX"},
    {AIC31XX_REG(0, 26), "AIC31XX_CLKOUTMVAL"},
    {AIC31XX_REG(0, 27), "AIC31XX_IFACE1"},
    {AIC31XX_REG(0, 28), "AIC31XX_DATA_OFFSET"},
    {AIC31XX_REG(0, 29), "AIC31XX_IFACE2"},
    {AIC31XX_REG(0, 30), "AIC31XX_BCLKN"},
    {AIC31XX_REG(0, 31), "AIC31XX_IFACESEC1"},
    {AIC31XX_REG(0, 32), "AIC31XX_IFACESEC2"},
    {AIC31XX_REG(0, 33), "AIC31XX_IFACESEC3"},
    {AIC31XX_REG(0, 34), "AIC31XX_I2C"},
    {AIC31XX_REG(0, 36), "AIC31XX_ADCFLAG"},
    {AIC31XX_REG(0, 37), "AIC31XX_DACFLAG1"},
    {AIC31XX_REG(0, 38), "AIC31XX_DACFLAG2"},
    {AIC31XX_REG(0, 39), "AIC31XX_OFFLAG"},
    {AIC31XX_REG(0, 44), "AIC31XX_INTRDACFLAG"},
    {AIC31XX_REG(0, 45), "AIC31XX_INTRADCFLAG"},
    {AIC31XX_REG(0, 46), "AIC31XX_INTRDACFLAG2"},
    {AIC31XX_REG(0, 47), "AIC31XX_INTRADCFLAG2"},
    {AIC31XX_REG(0, 48), "AIC31XX_INT1CTRL"},
    {AIC31XX_REG(0, 49), "AIC31XX_INT2CTRL"},
    {AIC31XX_REG(0, 51), "AIC31XX_GPIO1"},
    {AIC31XX_REG(0, 60), "AIC31XX_DACPRB"},
    {AIC31XX_REG(0, 61), "AIC31XX_ADCPRB"},
    {AIC31XX_REG(0, 63), "AIC31XX_DACSETUP"},
    {AIC31XX_REG(0, 64), "AIC31XX_DACMUTE"},
    {AIC31XX_REG(0, 65), "AIC31XX_LDACVOL"},
    {AIC31XX_REG(0, 66), "AIC31XX_RDACVOL"},
    {AIC31XX_REG(0, 67), "AIC31XX_HSDETECT"},
    {AIC31XX_REG(0, 81), "AIC31XX_ADCSETUP"},
    {AIC31XX_REG(0, 82), "AIC31XX_ADCFGA"},
    {AIC31XX_REG(0, 83), "AIC31XX_ADCVOL"},
    {AIC31XX_REG(1, 31), "AIC31XX_HPDRIVER"},
    {AIC31XX_REG(1, 32), "AIC31XX_SPKAMP"},
    {AIC31XX_REG(1, 33), "AIC31XX_HPPOP"},
    {AIC31XX_REG(1, 34), "AIC31XX_SPPGARAMP"},
    {AIC31XX_REG(1, 35), "AIC31XX_DACMIXERROUTE"},
    {AIC31XX_REG(1, 36), "AIC31XX_LANALOGHPL"},
    {AIC31XX_REG(1, 37), "AIC31XX_RANALOGHPR"},
    {AIC31XX_REG(1, 38), "AIC31XX_LANALOGSPL"},
    {AIC31XX_REG(1, 39), "AIC31XX_RANALOGSPR"},
    {AIC31XX_REG(1, 40), "AIC31XX_HPLGAIN"},
    {AIC31XX_REG(1, 41), "AIC31XX_HPRGAIN"},
    {AIC31XX_REG(1, 42), "AIC31XX_SPLGAIN"},
    {AIC31XX_REG(1, 43), "AIC31XX_SPRGAIN"},
    {AIC31XX_REG(1, 44), "AIC31XX_HPCONTROL"},
    {AIC31XX_REG(1, 46), "AIC31XX_MICBIAS"},
    {AIC31XX_REG(1, 47), "AIC31XX_MICPGA"},
    {AIC31XX_REG(1, 48), "AIC31XX_MICPGAPI"},
    {AIC31XX_REG(1, 49), "AIC31XX_MICPGAMI"},
    {AIC31XX_REG(1, 50), "AIC31XX_MICPGACM"},
    {AIC31XX_REG(3, 16), "AIC31XX_TIMERDIVIDER"}
};

struct reg_default {
    rt_uint16_t reg;
    rt_uint16_t val;
};

static const struct reg_default aic31xx_reg_defaults[] = 
{
    { AIC31XX_CLKMUX, 0x00 },
    { AIC31XX_PLLPR, 0x11 },
    { AIC31XX_PLLJ, 0x04 },
    { AIC31XX_PLLDMSB, 0x00 },
    { AIC31XX_PLLDLSB, 0x00 },
    { AIC31XX_NDAC, 0x01 },
    { AIC31XX_MDAC, 0x01 },
    { AIC31XX_DOSRMSB, 0x00 },
    { AIC31XX_DOSRLSB, 0x80 },
    { AIC31XX_NADC, 0x01 },
    { AIC31XX_MADC, 0x01 },
    { AIC31XX_AOSR, 0x80 },
    { AIC31XX_IFACE1, 0x00 },
    { AIC31XX_DATA_OFFSET, 0x00 },
    { AIC31XX_IFACE2, 0x00 },
    { AIC31XX_BCLKN, 0x01 },
    { AIC31XX_DACSETUP, 0x14 },
    { AIC31XX_DACMUTE, 0x0c },
    { AIC31XX_LDACVOL, 0x00 },
    { AIC31XX_RDACVOL, 0x00 },
    { AIC31XX_ADCSETUP, 0x00 },
    { AIC31XX_ADCFGA, 0x80 },
    { AIC31XX_ADCVOL, 0x00 },
    { AIC31XX_HPDRIVER, 0x04 },
    { AIC31XX_SPKAMP, 0x06 },
    { AIC31XX_DACMIXERROUTE, 0x00 },
    { AIC31XX_LANALOGHPL, 0x7f },
    { AIC31XX_RANALOGHPR, 0x7f },
    { AIC31XX_LANALOGSPL, 0x7f },
    { AIC31XX_RANALOGSPR, 0x7f },
    { AIC31XX_HPLGAIN, 0x02 },
    { AIC31XX_HPRGAIN, 0x02 },
    { AIC31XX_SPLGAIN, 0x00 },
    { AIC31XX_SPRGAIN, 0x00 },
    { AIC31XX_MICBIAS, 0x00 },
    { AIC31XX_MICPGA, 0x80 },
    { AIC31XX_MICPGAPI, 0x00 },
    { AIC31XX_MICPGAMI, 0x00 },
    { AIC31XX_TIMERDIVIDER, 0x80 },
};

const float analog_gain_table[] = 
{
      0.0,  -0.5,  -1.0,  -1.5,  -2.0,  -2.5,  -3.0,  -3.5,  -4.0,  -4.5,
     -5.0,  -5.5,  -6.0,  -6.5,  -7.0,  -7.5,  -8.0,  -8.5,  -9.0,  -9.5,
    -10.0, -10.5, -11.0, -11.5, -12.0, -12.5, -13.0, -13.5, -14.0, -14.5,
    -15.0, -15.5, -16.0, -16.5, -17.0, -17.5, -18.1, -18.6, -19.1, -19.6,
    -20.1, -20.6, -21.1, -21.6, -22.1, -22.6, -23.1, -23.6, -24.1, -24.6,
    -25.1, -25.6, -26.1, -26.6, -27.1, -27.6, -28.1, -28.6, -29.1, -29.6,
    -30.1, -30.6, -31.1, -31.6, -32.1, -32.6, -33.1, -33.6, -34.1, -34.6,
    -35.2, -35.7, -36.2, -36.7, -37.2, -37.7, -38.2, -38.7, -39.2, -39.7,
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

codec_driver_t tlv320aic31x_drv;

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

/* Extract page and register address from AIC31XX_REG macro */
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
    wrbuf[0] = AIC31XX_PAGECTL;
    wrbuf[1] = page;

    /* build i2c message    */
    msgs[0].addr  = TLV320aic31x_I2C_ADDR;
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
    wrbuf[0] = AIC31XX_PAGECTL;

    /* build i2c message for register address write    */
    msgs[0].addr  = TLV320aic31x_I2C_ADDR;
    msgs[0].flags = (I2C_OPFLAG_MASK | RT_I2C_WR);
    msgs[0].buf   = wrbuf;
    msgs[0].len   = 1;

    /* build i2c message for register address read    */
    msgs[1].addr  = TLV320aic31x_I2C_ADDR;
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
    msgs[0].addr  = TLV320aic31x_I2C_ADDR;
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
    msgs[0].addr  = TLV320aic31x_I2C_ADDR;
    msgs[0].flags = (I2C_OPFLAG_MASK | RT_I2C_WR);
    msgs[0].buf   = wrbuf;
    msgs[0].len   = 1;

    msgs[1].addr  = TLV320aic31x_I2C_ADDR;
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

// High-level function to set the PLL
void set_pll(rt_uint8_t pll_p, rt_uint8_t pll_r, rt_uint8_t pll_j, rt_uint16_t pll_d) 
{
    i2c_reg_write(AIC31XX_PLLJ, pll_j);
    i2c_reg_write(AIC31XX_PLLDLSB, pll_d & 0xff);
    i2c_reg_write(AIC31XX_PLLDMSB, (pll_d>>8) & 0xff);
    modify_reg(AIC31XX_PLLPR, AIC31XX_PLLPR_R_MASK, pll_r);
    modify_reg(AIC31XX_PLLPR, AIC31XX_PLLPR_P_MASK, pll_p);
}

void set_pll_power(rt_bool_t power) 
{
    modify_reg(AIC31XX_PLLPR, AIC31XX_PLLPR_POWER_MASK, power ? 1 : 0);
}

void set_clk_mux(rt_uint8_t pll_clkin, rt_uint8_t codec_clkin) 
{
    /* The source reference clock for the codec is chosen by programming the CODEC_CLKIN value on page 0 / register 4, bits D1 */
    modify_reg(AIC31XX_CLKMUX, AIC31XX_CODEC_CLKIN_MASK, codec_clkin);
    modify_reg(AIC31XX_CLKMUX, AIC31XX_PLL_CLKIN_MASK, pll_clkin);
}

void set_ndac_value(rt_uint8_t ndac) 
{
    modify_reg(AIC31XX_NDAC, AIC31XX_NDAC_MASK, ndac);
}

void set_ndac_power(rt_bool_t power) 
{
    modify_reg(AIC31XX_NDAC, AIC31XX_NDAC_POWER_MASK, power);
}

void set_mdac_value(rt_uint8_t mdac) 
{
    modify_reg(AIC31XX_MDAC, AIC31XX_MDAC_MASK, mdac);
}

void set_mdac_power(rt_bool_t power)
{
    modify_reg(AIC31XX_MDAC, AIC31XX_MDAC_POWER_MASK, power);
}

void set_dosr_value(rt_uint16_t dosr) 
{
    i2c_reg_write(AIC31XX_DOSRLSB, dosr & 0xff);
    i2c_reg_write(AIC31XX_DOSRMSB, (dosr>>8) & 0xff);
}

void set_nadc_value(rt_uint8_t nadc) 
{
    modify_reg(AIC31XX_NADC, AIC31XX_NADC_MASK, nadc);
}

void set_nadc_power(rt_bool_t power) 
{
    modify_reg(AIC31XX_NADC, AIC31XX_NADC_POWER_MASK, power);
}

void set_madc_value(rt_uint8_t madc) 
{
    modify_reg(AIC31XX_MADC, AIC31XX_MADC_MASK, madc);
}

void set_madc_power(rt_bool_t power) 
{
    modify_reg(AIC31XX_MADC, AIC31XX_MADC_POWER_MASK, power);
}

void set_aosr_value(rt_uint16_t aosr) 
{
    i2c_reg_write(AIC31XX_AOSR, aosr);
}

void set_word_length(rt_uint8_t wordlength) 
{
    modify_reg(AIC31XX_IFACE1, AIC31XX_IFACE1_DATALEN_MASK, wordlength);
}

void set_hs_detect_int1(rt_bool_t enable) 
{
    modify_reg(AIC31XX_INT1CTRL, AIC31XX_HSPLUGDET, enable);
    modify_reg(AIC31XX_GPIO1, AIC31XX_GPIO1_FUNC_MASK, AIC31XX_GPIO1_INT1);
}

void enable_headset_detect() 
{
    modify_reg(AIC31XX_HSDETECT, AIC31XX_HSD_ENABLE, 0x01);
}

rt_bool_t is_headset_detected() 
{
    return (i2c_reg_read(AIC31XX_HSDETECT) & AIC31XX_HSD_TYPE_MASK) != AIC31XX_HSD_NONE;
}


/* High-level function to enable and unmute the DAC and route it to the output mixer */
void enable_dac() 
{
    modify_reg(AIC31XX_DACSETUP, AIC31XX_DAC_POWER_MASK,0x3);
    modify_reg(AIC31XX_DACMIXERROUTE, AIC31XX_DACMIXERROUTE_DACL_MASK,0x1);
#ifndef TLV320AIC3120
    modify_reg(AIC31XX_DACMIXERROUTE, AIC31XX_DACMIXERROUTE_DACR_MASK,0x1);
#endif
}

void set_dac_mute(rt_bool_t mute)
{
    if (mute) 
    {
        modify_reg(AIC31XX_DACMUTE, AIC31XX_DACMUTE_MASK,0x3);
    } 
    else 
    {
        modify_reg(AIC31XX_DACMUTE, AIC31XX_DACMUTE_MASK,0x0);
    }
}

void set_dac_volume(float left_dB, float right_dB) 
{
    /* Convert dB values to register format */
    rt_uint8_t leftRegValue = dac2reg_value(left_dB);
//    rt_uint8_t rightRegValue = dac2reg_value(right_dB);

    // Write to left and right DAC volume registers
    i2c_reg_write(AIC31XX_LDACVOL, leftRegValue);  // Left DAC volume control
//    i2c_reg_write(AIC31XX_RDACVOL, rightRegValue); // Right DAC volume control
}

void enable_adc() 
{
    /* power */
    modify_reg(AIC31XX_ADCSETUP, AIC31XX_ADC_POWER_MASK,0x1);
    /* unmute */
    modify_reg(AIC31XX_ADCFGA, AIC31XX_ADC_MUTE_MASK,0x0);
}

/* enable the headphone amplifier */
void enable_headphone_amp() 
{
    modify_reg(AIC31XX_HPDRIVER, AIC31XX_HPD_POWER_MASK, 0x3);
}

void enable_headphone_mute(rt_bool_t mute) 
{
    modify_reg(AIC31XX_HPLGAIN, AIC31XX_HPLGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
    modify_reg(AIC31XX_HPRGAIN, AIC31XX_HPRGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
}

void enable_headphone_gain(float left_dB, float right_dB) 
{
    modify_reg(AIC31XX_HPLGAIN, AIC31XX_HPLGAIN_GAIN_MASK, pgagain2reg_value(left_dB));
#ifdef TLV320AIC3104
    modify_reg(AIC31XX_HPRGAIN, AIC31XX_HPRGAIN_GAIN_MASK, pgagain2reg_value(right_dB));
#endif
}

void enable_headphone_volume(float left_dB, float right_dB) 
{
    i2c_reg_write(AIC31XX_LANALOGHPL, gain2reg_value(left_dB));
#ifdef TLV320AIC3104
    i2c_reg_write(AIC31XX_RANALOGHPR, gain2reg_value(right_dB));
#endif
}

void enable_headphone_performance(rt_uint8_t level)
{
    modify_reg(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_PERFORMANCE_MASK, level);
}

void enable_headphone_linemode(rt_bool_t line)
{
    modify_reg(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_HPL_LINE_MASK, line);
    modify_reg(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_HPR_LINE_MASK, line);
}

void enable_speaker_amp() 
{
    modify_reg(AIC31XX_SPKAMP, AIC3100_SPKAMP_POWER_MASK, 0x1);
    /* TODO: also support other channel on codecs with stereo amp */
}

void enable_speaker_mute(rt_bool_t mute) 
{
    modify_reg(AIC31XX_SPLGAIN, AIC3100_SPKLGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
}

void enable_speaker_volume(float left_dB) 
{
    i2c_reg_write(AIC31XX_LANALOGSPL, gain2reg_value(left_dB));
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
    modify_reg(AIC31XX_SPLGAIN, AIC3100_SPKLGAIN_GAIN_MASK, gain);
}

void set_micpga_enable(rt_bool_t enable) 
{
    modify_reg(AIC31XX_MICPGA, AIC31XX_MICPGA_ENABLE_MASK, enable ? 0x0 : 1);
}

void set_micpga_gain(float gain) 
{
    modify_reg(AIC31XX_MICPGA, AIC31XX_MICPGA_GAIN_MASK, micgain2reg_value(gain));
}

void set_adc_gain(float adcGain) 
{
    i2c_reg_write(AIC31XX_ADCVOL, reg2adc_value(adcGain));
}

void x()
{
    tlv_reset();
}

/* High-level function to software-reset the codec (also turns off all components for power saving) */
void tlv_reset() 
{
    i2c_reg_write(AIC31XX_RESET, 0x1);
}

/* TLV320AIC31x driver callback function */
rt_err_t tlv320aic31x_init(rt_uint8_t volume, rt_uint32_t auido_freq)
{
    tlv_handle_t *h_tlv = &__tlv_handle;

    /* config reset output pin  */
    rt_pin_mode(TLV312x_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(TLV312x_RESET_PIN, CODEC_RESET);
    rt_thread_mdelay(3);
    rt_pin_write(TLV312x_RESET_PIN, CODEC_SET);

    h_tlv->i2c_dev = rt_device_find(I2C_BUS_NAME);
    if (h_tlv->i2c_dev == RT_NULL)
    {
        tlv_debug("tlv320aic31x_init no device found\r\n");
        RT_ASSERT(h_tlv->i2c_dev != RT_NULL);
        return -RT_ENOSYS;
    }
    h_tlv->i2c_bus = (struct rt_i2c_bus_device *)h_tlv->i2c_dev->user_data;

    tlv320aic31x_deinit();

    set_clk_mux(0, 3); // Clk Muxing : pll_clkin = mclk, codec_clkin = pll_clk

    set_pll(16, 1, 8, 0); //[04] AIC31XX_PLLPR : 148 / p | r | j | d

    set_ndac_value(1);

    set_mdac_value(1);

    i2c_reg_write(AIC31XX_IFACE1, 0);

    i2c_reg_write(AIC31XX_DACPRB, 16);

    rt_thread_mdelay(10);

    i2c_reg_write(AIC31XX_DOSRMSB, 0x00);
    i2c_reg_write(AIC31XX_DOSRLSB, 0x80);     // DOSR(Digital Oversampling Ratio) value = 512
    i2c_reg_write(AIC31XX_IFACE2,   0x00);
    i2c_reg_write(AIC31XX_BCLKN,    0x01);     // BCLK input
    i2c_reg_write(AIC31XX_DATA_OFFSET, 0x00);

    // 헤드폰 디팝/드라이버
    i2c_reg_write(AIC31XX_HPDRIVER, 0x04);
    i2c_reg_write(AIC31XX_HPPOP,    0x4E);     // 78

    // 헤드폰 아날로그 게인/볼륨
    i2c_reg_write(AIC31XX_HPLGAIN,    0x06);
    i2c_reg_write(AIC31XX_LANALOGHPL, 0x92);     // 146

    // 스피커 Class-D 앰프
    i2c_reg_write(AIC31XX_SPKAMP,     0xC6);     // 198
    i2c_reg_write(AIC31XX_SPLGAIN,    0x1C);     // 28 : gain 18dB (6~24)
    i2c_reg_write(AIC31XX_LANALOGSPL, 0x92);     // 146

    // DAC 데이터 경로 전원/볼륨/뮤트
    i2c_reg_write(AIC31XX_DACSETUP,  0x94);      // 148
    i2c_reg_write(AIC31XX_LDACVOL,   0xD4);      // 212
    i2c_reg_write(AIC31XX_DACMUTE,   0x04);

    tlv320aic31x_volume(volume);

    enable_dac();

    return RT_EOK;
}

void tlv320aic31x_deinit()
{
    rt_pin_write(TLV312x_RESET_PIN, CODEC_RESET);
    rt_thread_mdelay(3);
    rt_pin_write(TLV312x_RESET_PIN, CODEC_SET);
}

rt_err_t tlv320aic31x_play(void)
{
    set_pll_power(RT_TRUE);

    set_ndac_power(RT_TRUE);

    set_mdac_power(RT_TRUE);

    set_dac_mute(RT_FALSE);

    enable_speaker_mute(RT_FALSE);

    return RT_EOK;
}

rt_err_t tlv320aic31x_stop(rt_bool_t power_down)
{
    set_dac_mute(RT_TRUE);

    enable_speaker_mute(RT_TRUE);

    rt_thread_mdelay(20);

    set_ndac_power(RT_FALSE);

    set_mdac_power(RT_FALSE);

    set_pll_power(RT_FALSE);

    return RT_EOK;
}

rt_err_t tlv320aic31x_set_frequency(rt_uint32_t auido_freq)
{
    if (auido_freq != 16000)
        return -RT_ERROR;

    return RT_EOK;
}

rt_err_t tlv320aic31x_volume(rt_uint8_t auido_volume)
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
        for (idx = 0; idx < sizeof(register_table)/sizeof(register_table[0])-1; idx++)
        {
            val = i2c_reg_read(register_table[idx].reg);
            rt_kprintf("[%02d] %s : %d\r\n", idx, lookup_reg_name(register_table[idx].reg), val);
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

/* MSH command */
void reg_set_exam(uint8_t argc, char **argv)
{
    i2c_reg_write(AIC31XX_CLKMUX, 0);               // PLL divider setting      | value 0 : codec_clkin = MCLK(device pin) / value 3 : codec_clkin = PLL_CLK(generated on-chip)
    i2c_reg_write(AIC31XX_PLLJ, 8);                 // PLLJ setting             | value setting : value = 8
    i2c_reg_write(AIC31XX_PLLDMSB, 0);              // PLLD setting             | MSB bits value setting : 0
    i2c_reg_write(AIC31XX_PLLPR, 145);              // PLL setting              | value 128 : PLL power up / value 16 : P value = 1 / value 1 : R value = 1
    i2c_reg_write(AIC31XX_NDAC, 136);               // NDAC power up            | value 128 : DAC NDAC divider power up / value 8 : DAC NDAC divider value = 8
    i2c_reg_write(AIC31XX_MDAC, 130);               // MDAC power up            | value 128 : DAC MDAC divider power up / value 2 : DAC MDAC divider value = 2
    i2c_reg_write(AIC31XX_DOSRMSB, 128);            // Program OSR value        | DOSR value = 128
    i2c_reg_write(AIC31XX_IFACE1, 0);               // Codec Interface Control  | codec interface = I2S, word length = 16bit, BCLK = input/WCLK = input (slave mode)
    i2c_reg_write(AIC31XX_DACPRB, 16);              // DAC Processing Block     | DAC signal processing block = PRB_P16
    i2c_reg_write(AIC31XX_HPDRIVER, 4);             // Headphone setting        | output common-mode : Voltage = 1.35V
    i2c_reg_write(AIC31XX_HPPOP, 78);               // Depop setting            | Driver power on time = 610ms
    i2c_reg_write(AIC31XX_DACMIXERROUTE, 64);       // DAC routed to HPOUT      | DAC routed to mixer amplifier (HP, Lineout)
    i2c_reg_write(AIC31XX_HPLGAIN, 6);              // DAC,gain setting         | value 4 : HPOUT driver not muted , value 2 : reserved
    i2c_reg_write(AIC31XX_SPLGAIN, 28);             // Class D, gain setting    | value 16 : class-D gain = 18dB
    i2c_reg_write(AIC31XX_HPDRIVER, 130);           // Headphone setting2       | value 128 : HPOUT power up / value 2 : device maximum current limit on to the load
    i2c_reg_write(AIC31XX_SPKAMP, 198);             // Class D power up         | value 128 : output driver power up
    i2c_reg_write(AIC31XX_LANALOGHPL, 146);         // HPOUT volume control     | value 128 : routed to HPOUT / value D6 - D0 : volume
    i2c_reg_write(AIC31XX_LANALOGSPL, 146);         // Class D volume control   | value 128 : routed to Class D / value D6 - D0 : volume
    i2c_reg_write(AIC31XX_DACSETUP, 148);           // DAC Data-Path Setup      | value 128 : DAC power up
    i2c_reg_write(AIC31XX_LDACVOL, 212);            // DAC volume control       | value 212 : DAC digital gain :
    i2c_reg_write(AIC31XX_DACMUTE, 4);              // DAC mute control         | value 4 : mute off / value 0 : mute on
}
MSH_CMD_EXPORT(reg_set_exam, tlv32oaic31x set datasheet exam);

void cmd_vol(uint8_t argc, char **argv)
{
    rt_uint8_t cur_vol;
    rt_int8_t new_vol;
    rt_int8_t new_gain;

    if(argc < 2)
    {
        cur_vol = i2c_reg_read(AIC31XX_LANALOGSPL);
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
        tlv320aic31x_volume(new_vol);
        rt_kprintf("set volume gain = %d \r\n", -new_gain);
    }
    else
    {
        rt_kprintf("bad arguments, check cmd\r\n");
    }
}
MSH_CMD_EXPORT(cmd_vol, set volume (1 ~ 128) );

#endif //ifdef TLV320AIC31xx
