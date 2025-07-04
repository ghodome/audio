#ifndef TLV320AIC310X_REGS_H
#define TLV320AIC310X_REGS_H

/*   using Device   */
#define TLV320AIC3104 //for Engine Controller Jig

/* I2C function */
#define I2C_BUS_NAME                  "i2c1"
#define I2C_OPFLAG_MASK               0x00      /* 7 bits address, use start. use stop, wait ack */
#define TLV310x_RESET_PIN             GET_PIN(B, 0)
#define CODEC_RESET                   PIN_LOW
#define CODEC_SET                     PIN_HIGH

// helpers
#ifndef GENMASK
#define GENMASK(high, low) (((1U << ((high) + 1)) - 1) & ~((1U << (low)) - 1))
#endif

#ifndef BIT
#define BIT(n) (1U << (n))
#endif

#ifndef BIT_MASK
#define BIT_MASK(n) ((1U << (n)) - 1)
#endif

#ifndef SET_BIT
#define SET_BIT(value, n) ((value) | BIT(n))
#endif

#ifndef CLEAR_BIT
#define CLEAR_BIT(value, n) ((value) & ~BIT(n))
#endif

#ifdef TLV320AIC3104

#define AIC310X_STEREO_CLASS_D_BIT    1
#define AIC310X_MINIDSP_BIT        2
#define DAC31XX_BIT            3

enum aic31xx_type
{
    AIC3100    = 0,
    AIC3110 = AIC310X_STEREO_CLASS_D_BIT,
    AIC3120 = AIC310X_MINIDSP_BIT,
    AIC3111 = AIC310X_STEREO_CLASS_D_BIT | AIC310X_MINIDSP_BIT,
    DAC3100 = DAC31XX_BIT,
    DAC3101 = DAC31XX_BIT | AIC310X_STEREO_CLASS_D_BIT,
};

#define AIC310X_REG(page, reg)    ((page * 128) + reg)

#define AIC310X_PAGECTL         AIC310X_REG(0, 0) /* Page Control Register */

/* Page 0 Registers */
#define AIC310X_RESET               AIC310X_REG(0, 1)           /* Software reset register */
#define AIC310X_SAMPLERATE          AIC310X_REG(0, 2)           /* SampleRate set register */
#define AIC310X_PLLQP               AIC310X_REG(0, 3)           /* PLL - Q,P set register */
#define AIC310X_PLLJ                AIC310X_REG(0, 4)           /* PLL - J set register */
#define AIC310X_PLLD1               AIC310X_REG(0, 5)           /* PLL - D set register */
#define AIC310X_PLLD2               AIC310X_REG(0, 6)           /* PLL - D set register */
#define AIC310X_DATA_PATH           AIC310X_REG(0, 7)           /* ADC, DAC Data Path set register */
#define AIC310X_ASDI_MODE_1         AIC310X_REG(0, 8)           /* Slave/Master, WCLK BCLK, 3D-Effect set register */
#define AIC310X_ASDI_MODE_2         AIC310X_REG(0, 9)           /* TX Mode, Word Length,  */
#define AIC310X_ASDI_MODE_3         AIC310X_REG(0, 10)          /* Data offset register */
#define AIC310X_PLLR                AIC310X_REG(0, 11)          /* PLL - R set register */
#define AIC310X_DIG_FLT             AIC310X_REG(0, 12)          /* ADC, DAC Digital filter set register */
#define AIC310X_HDSET_DETECT        AIC310X_REG(0, 13)          /* Headset detect, debounce register */
#define AIC310X_HDSET_CONFIG        AIC310X_REG(0, 14)          /*  */
#define AIC310X_DAC_POWER           AIC310X_REG(0, 37)          /* L/R DAC power control register */
#define AIC310X_HPRCOM              AIC310X_REG(0, 38)          /* High Power Output set register */
#define AIC310X_DAC_PATH            AIC310X_REG(0, 41)          /* DAC path set register */
#define AIC310X_DEPOP               AIC310X_REG(0, 42)          /* power on delay, ramp-up step timing control register */
#define AIC310X_DAC_LEFT_VOL        AIC310X_REG(0, 43)          /* Left DAC Volume control register */
#define AIC310X_DAC_RIGHT_VOL       AIC310X_REG(0, 44)          /* Right DAC Volume control register */
#define AIC310X_HPLOUT_PGA_L        AIC310X_REG(0, 46)          /* PGA_L route, volume set register */
#define AIC310X_HPLOUT_DAC_L1       AIC310X_REG(0, 47)          /* DAC_L1 route, volume set register */
#define AIC310X_HPLOUT_PGA_R        AIC310X_REG(0, 49)          /* PGA_R route, volume set register */
#define AIC310X_HPLOUT_DAC_R1       AIC310X_REG(0, 50)          /* DAC_R1 route, voluime set register */
#define AIC310X_HPLOUT_SETUP        AIC310X_REG(0, 51)          /* output level, mute, power set register */
#define AIC310X_HPLCOM_PGA_L        AIC310X_REG(0, 53)          /* PGA_L route, volume set register */
#define AIC310X_HPLCOM_DAC_L1       AIC310X_REG(0, 54)          /* DAC_L1 route, volume set register */
#define AIC310X_HPLCOM_PGA_R        AIC310X_REG(0, 56)          /* PGA_R route, volume set register */
#define AIC310X_HPLCOM_DAC_R1       AIC310X_REG(0, 57)          /* DAC_R1 route, voluime set register */
#define AIC310X_HPLCOM_SETUP        AIC310X_REG(0, 58)          /* output level, mute, power set register */
#define AIC310X_HPROUT_PGA_L        AIC310X_REG(0, 60)          /* PGA_L route, volume set register */
#define AIC310X_HPROUT_DAC_L1       AIC310X_REG(0, 61)          /* DAC_L1 route, volume set register */
#define AIC310X_HPROUT_PGA_R        AIC310X_REG(0, 63)          /* PGA_R route, volume set register */
#define AIC310X_HPROUT_DAC_R1       AIC310X_REG(0, 64)          /* DAC_R1 route, voluime set register */
#define AIC310X_HPROUT_SETUP        AIC310X_REG(0, 65)          /* output level, mute, power set register */
#define AIC310X_HPRCOM_PGA_L        AIC310X_REG(0, 67)          /* PGA_L route, volume set register */
#define AIC310X_HPRCOM_DAC_L1       AIC310X_REG(0, 68)          /* DAC_L1 route, volume set register */
#define AIC310X_HPRCOM_PGA_R        AIC310X_REG(0, 70)          /* PGA_R route, volume set register */
#define AIC310X_HPRCOM_DAC_R1       AIC310X_REG(0, 71)          /* DAC_R1 route, voluime set register */
#define AIC310X_HPRCOM_SETUP        AIC310X_REG(0, 72)          /* output level, mute, power set register */
#define AIC310X_LEFT_LO_PGA_L       AIC310X_REG(0, 81)          /* PGA_L route, volume set register */
#define AIC310X_LEFT_LO_DAC_L1      AIC310X_REG(0, 82)          /* DAC_L1 route, volume set register */
#define AIC310X_LEFT_LO_PGA_R       AIC310X_REG(0, 84)          /* PGA_R route, volume set register */
#define AIC310X_LEFT_LO_DAC_R1      AIC310X_REG(0, 85)          /* DAC_R1 route, voluime set register */
#define AIC310X_LEFT_LO_SETUP       AIC310X_REG(0, 86)          /* output level, mute, power set register */
#define AIC310X_RIGHT_LO_PGA_L      AIC310X_REG(0, 88)          /* PGA_L route, volume set register */
#define AIC310X_RIGHT_LO_DAC_L1     AIC310X_REG(0, 89)          /* DAC_L1 route, volume set register */
#define AIC310X_RIGHT_LO_PGA_R      AIC310X_REG(0, 91)          /* PGA_R route, volume set register */
#define AIC310X_RIGHT_LO_DAC_R1     AIC310X_REG(0, 92)          /* DAC_R1 route, voluime set register */
#define AIC310X_RIGHT_LO_SETUP      AIC310X_REG(0, 93)          /* output level, mute, power set register */
#define AIC310X_CLK_SRC             AIC310X_REG(0, 101)         /* CODEC_CLKIN select register */
#define AIC310X_CLK_GEN             AIC310X_REG(0, 102)         /* Clock Gen control register */

/* Bits, masks, and shifts */

/* AIC310X_SAMPLERATE */
#define AIC310X_ADC_FS_MASK         GENMASK(7, 4)
#define AIC310X_DAC_FS_MASK         GENMASK(3, 0)

/* AIC310X_PLLQP */
#define AIC310X_PLLQP_CONTROL_MASK  BIT(7)
#define AIC310X_PLLQP_Q_MASK        GENMASK(6, 3)
#define AIC310X_PLLQP_P_MASK        GENMASK(2, 0)

/* AIC310X_PLLJ */
#define AIC310X_PLLJ_MASK           GENMASK(7, 2)

/* AIC310X_PLLD */
#define AIC310X_PLLD_1_MASK         GENMASK(7, 0)
#define AIC310X_PLLD_2_MASK         GENMASK(7, 2)

/* AIC310X_ASDI_Control */
#define AIC310X_ASDI_BCLK_MASK      BIT(7)
#define AIC310X_ASDI_WCLK_MASK      BIT(6)
#define AIC310X_SLAVE_MODE          0x00
#define AIC310X_MASTER_MODE         0x01
#define AIC310X_ASDI_CLK_DRV_MASK   BIT(4)
#define AIC310X_ASDI_BUS_MASK       GENMASK(7, 6)
#define AIC310X_I2C_MODE            0x00
#define AIC310X_DSP_MODE            0x01
#define AIC310X_RJ_MODE             0x02
#define AIC310X_LJ_MODE             0x03
#define AIC310X_ASDI_WORD_MASK      GENMASK(5, 4)
#define AIC310X_WORD_16BIT          0x00
#define AIC310X_WORD_20BIT          0x01
#define AIC310X_WORD_24BIT          0x02
#define AIC310X_WORD_32BIT          0x03
#define AIC310X_RESYNC_MASK         BIT(2)

/* AIC310X_HDSET */
#define AIC310X_DTT_ENB_MASK        BIT(7)
#define AIC310X_HD_DETECTION        GENMASK(6, 5)

/* AIC310X_DATA_OFFSET */
#define AIC310X_DATA_OFFSET_MASK        GENMASK(7, 0)

/* AIC310X_IFACE2 */
#define AIC310X_BCLKINV_MASK            BIT(3)
#define AIC310X_BDIVCLK_MASK            GENMASK(1, 0)
#define AIC310X_DAC2BCLK                0x00
#define AIC310X_DACMOD2BCLK             0x01
#define AIC310X_ADC2BCLK                0x02
#define AIC310X_ADCMOD2BCLK             0x03
#define AIC310X_KEEP_I2SCLK             BIT(2)

/* AIC310X_ADCFLAG */
#define AIC310X_ADCPWRSTATUS_MASK       BIT(6)

/* AIC310X_DACFLAG1 */
#define AIC310X_DAC_MUTE_MASK           BIT(7)
#define AIC310X_HPLDRVPWRSTATUS_MASK    BIT(5)
#define AIC310X_SPLDRVPWRSTATUS_MASK    BIT(4)
#define AIC310X_RDACPWRSTATUS_MASK      BIT(3)
#define AIC310X_HPRDRVPWRSTATUS_MASK    BIT(1)
#define AIC310X_SPRDRVPWRSTATUS_MASK    BIT(0)

/* AIC310X_OFFLAG */
#define AIC310X_DAC_OF_LEFT             BIT(7)
#define AIC310X_DAC_OF_RIGHT            BIT(6)
#define AIC310X_DAC_OF_SHIFTER          BIT(5)
#define AIC310X_ADC_OF                  BIT(3)
#define AIC310X_ADC_OF_SHIFTER          BIT(1)

/* AIC310X_INTRDACFLAG */
#define AIC310X_HPLSCDETECT             BIT(7)
#define AIC310X_HPRSCDETECT             BIT(6)
#define AIC310X_BUTTONPRESS             BIT(5)
#define AIC310X_HSPLUG                  BIT(4)
#define AIC310X_LDRCTHRES               BIT(3)
#define AIC310X_RDRCTHRES               BIT(2)
#define AIC310X_DACSINT                 BIT(1)
#define AIC310X_DACAINT                 BIT(0)

/* AIC310X_INT1CTRL */
#define AIC310X_HSPLUGDET               BIT(7)
#define AIC310X_BUTTONPRESSDET          BIT(6)
#define AIC310X_DRCTHRES                BIT(5)
#define AIC310X_AGCNOISE                BIT(4)
#define AIC310X_SC                      BIT(3)
#define AIC310X_ENGINE                  BIT(2)

/* AIC310X_GPIO1 */
#define AIC310X_GPIO1_FUNC_MASK        GENMASK(5, 2)
#define AIC310X_GPIO1_FUNC_SHIFT        2
#define AIC310X_GPIO1_DISABLED          0x00
#define AIC310X_GPIO1_INPUT             0x01
#define AIC310X_GPIO1_GPI               0x02
#define AIC310X_GPIO1_GPO               0x03
#define AIC310X_GPIO1_CLKOUT            0x04
#define AIC310X_GPIO1_INT1              0x05
#define AIC310X_GPIO1_INT2              0x06
#define AIC310X_GPIO1_ADC_WCLK          0x07
#define AIC310X_GPIO1_SBCLK             0x08
#define AIC310X_GPIO1_SWCLK             0x09
#define AIC310X_GPIO1_ADC_MOD_CLK       0x10
#define AIC310X_GPIO1_SDOUT             0x11

/* AIC310X_DACMUTE */
#define AIC310X_DACMUTE_MASK            GENMASK(3, 2)

#define AIC310X_DAC_POWER_MASK          GENMASK(7, 6)

/* AIC310X_HSDETECT */
#define AIC310X_HSD_ENABLE              BIT(7)
#define AIC310X_HSD_TYPE_MASK           GENMASK(6, 5)
#define AIC310X_HSD_TYPE_SHIFT          5
#define AIC310X_HSD_NONE                0x00
#define AIC310X_HSD_HP                  0x01
#define AIC310X_HSD_HS                  0x03

/* AIC310X_HPDRIVER */
#define AIC310X_HPD_OCMV_MASK           GENMASK(4, 3)
#define AIC310X_HPD_OCMV_SHIFT          3
#define AIC310X_HPD_OCMV_1_35V          0x0
#define AIC310X_HPD_OCMV_1_5V           0x1
#define AIC310X_HPD_OCMV_1_65V          0x2
#define AIC310X_HPD_OCMV_1_8V           0x3

#define AIC310X_HPD_POWER_MASK          GENMASK(7,6)

/* AIC310X_MICBIAS */
#define AIC310X_MICBIAS_MASK            GENMASK(1, 0)
#define AIC310X_MICBIAS_SHIFT           0

#define AIC310X_MICPGA_ENABLE_MASK      BIT(7)
#define AIC310X_MICPGA_GAIN_MASK        GENMASK(6, 0)

#define AIC310X_ADC_POWER_MASK          BIT(7)
#define AIC310X_ADC_MUTE_MASK           BIT(7)

#define AIC310X_HPLGAIN_MUTE_MASK       BIT(2)
#define AIC310X_HPRGAIN_MUTE_MASK       BIT(2)
#define AIC310X_HPLGAIN_GAIN_MASK       GENMASK(6, 3)
#define AIC310X_HPRGAIN_GAIN_MASK       GENMASK(6, 3)

#define AIC3100_SPKAMP_POWER_MASK       BIT(7)

#define AIC3100_SPKLGAIN_MUTE_MASK      BIT(2)
#define AIC3100_SPKLGAIN_GAIN_MASK      GENMASK(4, 3)

#define AIC310X_DACMIXERROUTE_DACL_MASK GENMASK(7, 6)
#define AIC310X_DACMIXERROUTE_DACR_MASK GENMASK(3, 2)

#define AIC310X_HPCONTROL_PERFORMANCE_MASK    GENMASK(4, 3)
#define AIC310X_HPCONTROL_HPL_LINE_MASK BIT(2)
#define AIC310X_HPCONTROL_HPR_LINE_MASK BIT(3)

#define AIC310X_TIMER_SELECT_MASK       BIT(7)

#endif

#ifdef TLV320AIC3104

#endif

extern rt_bool_t  is_headset_detected(void);
extern void set_pll(rt_uint8_t pll_p, rt_uint8_t pll_r, rt_uint8_t pll_j, rt_uint16_t pll_d);
extern void set_pll_power(rt_bool_t power);
extern void set_clk_mux(rt_uint8_t pll_clkin, rt_uint8_t codec_clkin);
extern void set_ndac_value(rt_uint8_t ndac);
extern void set_ndac_power(rt_bool_t power);
extern void set_mdac_value(rt_uint8_t mdac);
extern void set_mdac_power(rt_bool_t power);
extern void set_dosr_value(rt_uint16_t dosr);
extern void set_nadc_value(rt_uint8_t nadc);
extern void set_nadc_power(rt_bool_t power);
extern void set_madc_value(rt_uint8_t madc);
extern void set_madc_power(rt_bool_t power);
extern void set_aosr_value(rt_uint16_t aosr);
extern void set_word_length(rt_uint8_t wordlength);
extern void set_hs_detect_int1(rt_bool_t enable);
extern void enable_headset_detect(rt_bool_t enable);
extern void enable_dac(void);
extern void set_dac_mute(rt_bool_t mute);
extern void set_dac_volume(float left_dB, float right_dB);
extern void enable_adc(void);
extern void enable_headphone_amp(void);
extern void enable_headphone_mute(rt_bool_t mute);
extern void enable_headphone_gain(float left_dB, float right_dB);
extern void enable_headphone_volume(float left_dB, float right_dB);
extern void enable_headphone_performance(rt_uint8_t level);
extern void enable_headphone_linemode(rt_bool_t line);
extern void enable_speaker_amp(void);
extern void enable_speaker_mute(rt_bool_t mute);
extern void enable_speaker_volume(float left_dB);
extern void enable_speaker_gain(float gaindb);
extern void set_micpga_enable(rt_bool_t enable);
extern void set_micpga_gain(float gain);
extern void set_adc_gain(float adcGain);
extern void tlv_power_down(void);
extern void tlv_reset(void);

extern void     tlv320aic310x_deinit(void);
extern rt_err_t tlv320aic310x_init(rt_uint8_t volume, rt_uint32_t auido_freq);
extern rt_err_t tlv320aic310x_play(void);
extern rt_err_t tlv320aic310x_stop(rt_bool_t power_down);
extern rt_err_t tlv320aic310x_set_frequency(rt_uint32_t auido_freq);
extern rt_err_t tlv320aic310x_volume(rt_uint8_t auido_volume);

#endif    /* TLV320AIC310X_REGS_H */
