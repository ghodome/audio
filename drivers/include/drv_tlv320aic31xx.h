#ifndef TLV320AIC31XX_REGS_H
#define TLV320AIC31XX_REGS_H

/*   using Device   */
//#define TLV320AIC3120   //for test
//#define TLV320AIC3100 //for ReQguide

/* I2C function */
#define I2C_BUS_NAME                  "i2c1"
#define I2C_OPFLAG_MASK               0x00      /* 7 bits address, use start. use stop, wait ack */
#define TLV312x_RESET_PIN             GET_PIN(B, 0)
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

#ifdef TLV320AIC3120

#define AIC31XX_STEREO_CLASS_D_BIT    1
#define AIC31XX_MINIDSP_BIT        2
#define DAC31XX_BIT            3

enum aic31xx_type 
{
    AIC3100    = 0,
    AIC3110 = AIC31XX_STEREO_CLASS_D_BIT,
    AIC3120 = AIC31XX_MINIDSP_BIT,
    AIC3111 = AIC31XX_STEREO_CLASS_D_BIT | AIC31XX_MINIDSP_BIT,
    DAC3100 = DAC31XX_BIT,
    DAC3101 = DAC31XX_BIT | AIC31XX_STEREO_CLASS_D_BIT,
};

#define AIC31XX_REG(page, reg)    ((page * 128) + reg)

#define AIC31XX_PAGECTL        AIC31XX_REG(0, 0) /* Page Control Register */

/* Page 0 Registers */
#define AIC31XX_RESET           AIC31XX_REG(0, 1) /* Software reset register */
#define AIC31XX_OT_FLAG         AIC31XX_REG(0, 3) /* OT FLAG register */
#define AIC31XX_CLKMUX          AIC31XX_REG(0, 4) /* Clock clock Gen muxing, Multiplexers*/
#define AIC31XX_PLLPR           AIC31XX_REG(0, 5) /* PLL P and R-VAL register */
#define AIC31XX_PLLJ            AIC31XX_REG(0, 6) /* PLL J-VAL register */
#define AIC31XX_PLLDMSB         AIC31XX_REG(0, 7) /* PLL D-VAL MSB register */
#define AIC31XX_PLLDLSB         AIC31XX_REG(0, 8) /* PLL D-VAL LSB register */
#define AIC31XX_NDAC            AIC31XX_REG(0, 11) /* DAC NDAC_VAL register*/
#define AIC31XX_MDAC            AIC31XX_REG(0, 12) /* DAC MDAC_VAL register */
#define AIC31XX_DOSRMSB         AIC31XX_REG(0, 13) /* DAC OSR setting register 1, MSB value */
#define AIC31XX_DOSRLSB         AIC31XX_REG(0, 14) /* DAC OSR setting register 2, LSB value */
#define AIC31XX_MINI_DSP_INPOL  AIC31XX_REG(0, 16)
#define AIC31XX_NADC            AIC31XX_REG(0, 18) /* Clock setting register 8, PLL */
#define AIC31XX_MADC            AIC31XX_REG(0, 19) /* Clock setting register 9, PLL */
#define AIC31XX_AOSR            AIC31XX_REG(0, 20) /* ADC Oversampling (AOSR) Register */
#define AIC31XX_CLKOUTMUX       AIC31XX_REG(0, 25) /* Clock setting register 9, Multiplexers */
#define AIC31XX_CLKOUTMVAL      AIC31XX_REG(0, 26) /* Clock setting register 10, CLOCKOUT M divider value */
#define AIC31XX_IFACE1          AIC31XX_REG(0, 27) /* Audio Interface Setting Register 1 */
#define AIC31XX_DATA_OFFSET     AIC31XX_REG(0, 28) /* Audio Data Slot Offset Programming */
#define AIC31XX_IFACE2          AIC31XX_REG(0, 29) /* Audio Interface Setting Register 2 */
#define AIC31XX_BCLKN           AIC31XX_REG(0, 30) /* Clock setting register 11, BCLK N Divider */
#define AIC31XX_IFACESEC1       AIC31XX_REG(0, 31) /* Audio Interface Setting Register 3, Secondary Audio Interface */
#define AIC31XX_IFACESEC2       AIC31XX_REG(0, 32) /* Audio Interface Setting Register 4 */
#define AIC31XX_IFACESEC3       AIC31XX_REG(0, 33) /* Audio Interface Setting Register 5 */
#define AIC31XX_I2C             AIC31XX_REG(0, 34) /* I2C Bus Condition */
#define AIC31XX_ADCFLAG         AIC31XX_REG(0, 36) /* ADC FLAG */
#define AIC31XX_DACFLAG1        AIC31XX_REG(0, 37) /* DAC Flag Registers */
#define AIC31XX_DACFLAG2        AIC31XX_REG(0, 38)
#define AIC31XX_OFFLAG          AIC31XX_REG(0, 39) /* Sticky Interrupt flag (overflow) */
#define AIC31XX_INTRDACFLAG     AIC31XX_REG(0, 44) /* Sticy DAC Interrupt flags */
#define AIC31XX_INTRADCFLAG     AIC31XX_REG(0, 45) /* Sticy ADC Interrupt flags */
#define AIC31XX_INTRDACFLAG2    AIC31XX_REG(0, 46) /* DAC Interrupt flags 2 */
#define AIC31XX_INTRADCFLAG2    AIC31XX_REG(0, 47) /* ADC Interrupt flags 2 */
#define AIC31XX_INT1CTRL        AIC31XX_REG(0, 48) /* INT1 interrupt control */
#define AIC31XX_INT2CTRL        AIC31XX_REG(0, 49) /* INT2 interrupt control */
#define AIC31XX_GPIO1           AIC31XX_REG(0, 51) /* GPIO1 control */
#define AIC31XX_DACPRB          AIC31XX_REG(0, 60)
#define AIC31XX_ADCPRB          AIC31XX_REG(0, 61) /* ADC Instruction Set Register */
#define AIC31XX_DACSETUP        AIC31XX_REG(0, 63) /* DAC channel setup register */
#define AIC31XX_DACMUTE         AIC31XX_REG(0, 64) /* DAC Mute and volume control register */
#define AIC31XX_LDACVOL         AIC31XX_REG(0, 65) /* Left DAC channel digital volume control */
#define AIC31XX_RDACVOL         AIC31XX_REG(0, 66) /* Right DAC channel digital volume control */
#define AIC31XX_HSDETECT        AIC31XX_REG(0, 67) /* Headset detection */
#define AIC31XX_ADCSETUP        AIC31XX_REG(0, 81) /* ADC Digital Mic */
#define AIC31XX_ADCFGA          AIC31XX_REG(0, 82) /* ADC Digital Volume Control Fine Adjust */
#define AIC31XX_ADCVOL          AIC31XX_REG(0, 83) /* ADC Digital Volume Control Coarse Adjust */

/* Page 1 Registers */
#define AIC31XX_HPDRIVER        AIC31XX_REG(1, 31) /* Headphone drivers */
#define AIC31XX_SPKAMP          AIC31XX_REG(1, 32) /* Class-D Speakear Amplifier */
#define AIC31XX_HPPOP           AIC31XX_REG(1, 33) /* HP Output Drivers POP Removal Settings */
#define AIC31XX_SPPGARAMP       AIC31XX_REG(1, 34) /* Output Driver PGA Ramp-Down Period Control */
#define AIC31XX_DACMIXERROUTE   AIC31XX_REG(1, 35) /* DAC_L and DAC_R Output Mixer Routing */
#define AIC31XX_LANALOGHPL      AIC31XX_REG(1, 36) /* Left Analog Vol to HPL */
#define AIC31XX_RANALOGHPR      AIC31XX_REG(1, 37) /* Right Analog Vol to HPR */
#define AIC31XX_LANALOGSPL      AIC31XX_REG(1, 38) /* Left Analog Vol to SPL */
#define AIC31XX_RANALOGSPR      AIC31XX_REG(1, 39) /* Right Analog Vol to SPR */
#define AIC31XX_HPLGAIN         AIC31XX_REG(1, 40) /* HPL Driver */
#define AIC31XX_HPRGAIN         AIC31XX_REG(1, 41) /* HPR Driver */
#define AIC31XX_SPLGAIN         AIC31XX_REG(1, 42) /* SPL Driver */
#define AIC31XX_SPRGAIN         AIC31XX_REG(1, 43) /* SPR Driver */
#define AIC31XX_HPCONTROL       AIC31XX_REG(1, 44) /* HP Driver Control */
#define AIC31XX_MICBIAS         AIC31XX_REG(1, 46) /* MIC Bias Control */
#define AIC31XX_MICPGA          AIC31XX_REG(1, 47) /* MIC PGA*/
#define AIC31XX_MICPGAPI        AIC31XX_REG(1, 48) /* Delta-Sigma Mono ADC Channel Fine-Gain Input Selection for P-Terminal */
#define AIC31XX_MICPGAMI        AIC31XX_REG(1, 49) /* ADC Input Selection for M-Terminal */
#define AIC31XX_MICPGACM        AIC31XX_REG(1, 50) /* Input CM Settings */

/* Page 3 Registers */
#define AIC31XX_TIMERDIVIDER    AIC31XX_REG(3, 16) /* Timer Clock MCLK Divider */

/* Bits, masks, and shifts */

/* AIC31XX_CLKMUX */
#define AIC31XX_PLL_CLKIN_MASK      GENMASK(3, 2)
#define AIC31XX_PLL_CLKIN_SHIFT     (2)
#define AIC31XX_PLL_CLKIN_MCLK      0x00
#define AIC31XX_PLL_CLKIN_BCLK      0x01
#define AIC31XX_PLL_CLKIN_GPIO1     0x02
#define AIC31XX_PLL_CLKIN_DIN       0x03
#define AIC31XX_CODEC_CLKIN_MASK    GENMASK(1, 0)
#define AIC31XX_CODEC_CLKIN_SHIFT   (0)
#define AIC31XX_CODEC_CLKIN_MCLK    0x00
#define AIC31XX_CODEC_CLKIN_BCLK    0x01
#define AIC31XX_CODEC_CLKIN_GPIO1   0x02
#define AIC31XX_CODEC_CLKIN_PLL     0x03

/* AIC31XX_PLLPR */
#define AIC31XX_PLLPR_POWER_MASK    BIT(7)
#define AIC31XX_PLLPR_R_MASK        GENMASK(3, 0)
#define AIC31XX_PLLPR_P_MASK        GENMASK(6, 4)
/* AIC31XX_NDAC */
#define AIC31XX_NDAC_MASK           GENMASK(6, 0)
#define AIC31XX_NDAC_POWER_MASK     BIT(7)
/* AIC31XX_MDAC */
#define AIC31XX_MDAC_MASK           GENMASK(6, 0)
#define AIC31XX_MDAC_POWER_MASK     BIT(7)
/* AIC31XX_NADC */
#define AIC31XX_NADC_MASK           GENMASK(6, 0)
#define AIC31XX_NADC_POWER_MASK     BIT(7)
/* AIC31XX_MADC */
#define AIC31XX_MADC_MASK           GENMASK(6, 0)
#define AIC31XX_MADC_POWER_MASK     BIT(7)
/* AIC31XX_BCLKN */
#define AIC31XX_PLL_MASK            GENMASK(6, 0)
#define AIC31XX_PM_MASK             BIT(7)

/* AIC31XX_IFACE1 */
#define AIC31XX_IFACE1_DATATYPE_MASK    GENMASK(7, 6)
#define AIC31XX_IFACE1_DATATYPE_SHIFT   (6)
#define AIC31XX_I2S_MODE                0x00
#define AIC31XX_DSP_MODE                0x01
#define AIC31XX_RIGHT_JUSTIFIED_MODE    0x02
#define AIC31XX_LEFT_JUSTIFIED_MODE     0x03
#define AIC31XX_IFACE1_DATALEN_MASK     GENMASK(5, 4)
#define AIC31XX_IFACE1_DATALEN_SHIFT    (4)
#define AIC31XX_WORD_LEN_16BITS         0x00
#define AIC31XX_WORD_LEN_20BITS         0x01
#define AIC31XX_WORD_LEN_24BITS         0x02
#define AIC31XX_WORD_LEN_32BITS         0x03
#define AIC31XX_IFACE1_MASTER_MASK      GENMASK(3, 2)
#define AIC31XX_BCLK_MASTER             BIT(3)
#define AIC31XX_WCLK_MASTER             BIT(2)

/* AIC31XX_DATA_OFFSET */
#define AIC31XX_DATA_OFFSET_MASK        GENMASK(7, 0)

/* AIC31XX_IFACE2 */
#define AIC31XX_BCLKINV_MASK            BIT(3)
#define AIC31XX_BDIVCLK_MASK            GENMASK(1, 0)
#define AIC31XX_DAC2BCLK                0x00
#define AIC31XX_DACMOD2BCLK             0x01
#define AIC31XX_ADC2BCLK                0x02
#define AIC31XX_ADCMOD2BCLK             0x03
#define AIC31XX_KEEP_I2SCLK             BIT(2)

/* AIC31XX_ADCFLAG */
#define AIC31XX_ADCPWRSTATUS_MASK       BIT(6)

/* AIC31XX_DACFLAG1 */
#define AIC31XX_LDACPWRSTATUS_MASK      BIT(7)
#define AIC31XX_HPLDRVPWRSTATUS_MASK    BIT(5)
#define AIC31XX_SPLDRVPWRSTATUS_MASK    BIT(4)
#define AIC31XX_RDACPWRSTATUS_MASK      BIT(3)
#define AIC31XX_HPRDRVPWRSTATUS_MASK    BIT(1)
#define AIC31XX_SPRDRVPWRSTATUS_MASK    BIT(0)

/* AIC31XX_OFFLAG */
#define AIC31XX_DAC_OF_LEFT             BIT(7)
#define AIC31XX_DAC_OF_RIGHT            BIT(6)
#define AIC31XX_DAC_OF_SHIFTER          BIT(5)
#define AIC31XX_ADC_OF                  BIT(3)
#define AIC31XX_ADC_OF_SHIFTER          BIT(1)

/* AIC31XX_INTRDACFLAG */
#define AIC31XX_HPLSCDETECT             BIT(7)
#define AIC31XX_HPRSCDETECT             BIT(6)
#define AIC31XX_BUTTONPRESS             BIT(5)
#define AIC31XX_HSPLUG                  BIT(4)
#define AIC31XX_LDRCTHRES               BIT(3)
#define AIC31XX_RDRCTHRES               BIT(2)
#define AIC31XX_DACSINT                 BIT(1)
#define AIC31XX_DACAINT                 BIT(0)

/* AIC31XX_INT1CTRL */
#define AIC31XX_HSPLUGDET               BIT(7)
#define AIC31XX_BUTTONPRESSDET          BIT(6)
#define AIC31XX_DRCTHRES                BIT(5)
#define AIC31XX_AGCNOISE                BIT(4)
#define AIC31XX_SC                      BIT(3)
#define AIC31XX_ENGINE                  BIT(2)

/* AIC31XX_GPIO1 */
#define AIC31XX_GPIO1_FUNC_MASK        GENMASK(5, 2)
#define AIC31XX_GPIO1_FUNC_SHIFT        2
#define AIC31XX_GPIO1_DISABLED          0x00
#define AIC31XX_GPIO1_INPUT             0x01
#define AIC31XX_GPIO1_GPI               0x02
#define AIC31XX_GPIO1_GPO               0x03
#define AIC31XX_GPIO1_CLKOUT            0x04
#define AIC31XX_GPIO1_INT1              0x05
#define AIC31XX_GPIO1_INT2              0x06
#define AIC31XX_GPIO1_ADC_WCLK          0x07
#define AIC31XX_GPIO1_SBCLK             0x08
#define AIC31XX_GPIO1_SWCLK             0x09
#define AIC31XX_GPIO1_ADC_MOD_CLK       0x10
#define AIC31XX_GPIO1_SDOUT             0x11

/* AIC31XX_DACMUTE */
#define AIC31XX_DACMUTE_MASK            GENMASK(3, 2)

#define AIC31XX_DAC_POWER_MASK          GENMASK(7, 6)

/* AIC31XX_HSDETECT */
#define AIC31XX_HSD_ENABLE              BIT(7)
#define AIC31XX_HSD_TYPE_MASK           GENMASK(6, 5)
#define AIC31XX_HSD_TYPE_SHIFT          5
#define AIC31XX_HSD_NONE                0x00
#define AIC31XX_HSD_HP                  0x01
#define AIC31XX_HSD_HS                  0x03

/* AIC31XX_HPDRIVER */
#define AIC31XX_HPD_OCMV_MASK           GENMASK(4, 3)
#define AIC31XX_HPD_OCMV_SHIFT          3
#define AIC31XX_HPD_OCMV_1_35V          0x0
#define AIC31XX_HPD_OCMV_1_5V           0x1
#define AIC31XX_HPD_OCMV_1_65V          0x2
#define AIC31XX_HPD_OCMV_1_8V           0x3

#define AIC31XX_HPD_POWER_MASK          GENMASK(7,6)

/* AIC31XX_MICBIAS */
#define AIC31XX_MICBIAS_MASK            GENMASK(1, 0)
#define AIC31XX_MICBIAS_SHIFT           0

#define AIC31XX_MICPGA_ENABLE_MASK      BIT(7)
#define AIC31XX_MICPGA_GAIN_MASK        GENMASK(6, 0)

#define AIC31XX_ADC_POWER_MASK          BIT(7)
#define AIC31XX_ADC_MUTE_MASK           BIT(7)

#define AIC31XX_HPLGAIN_MUTE_MASK       BIT(2)
#define AIC31XX_HPRGAIN_MUTE_MASK       BIT(2)
#define AIC31XX_HPLGAIN_GAIN_MASK       GENMASK(6, 3)
#define AIC31XX_HPRGAIN_GAIN_MASK       GENMASK(6, 3)

#define AIC3100_SPKAMP_POWER_MASK       BIT(7)

#define AIC3100_SPKLGAIN_MUTE_MASK      BIT(2)
#define AIC3100_SPKLGAIN_GAIN_MASK      GENMASK(4, 3)

#define AIC31XX_DACMIXERROUTE_DACL_MASK GENMASK(7, 6)
#define AIC31XX_DACMIXERROUTE_DACR_MASK GENMASK(3, 2)

#define AIC31XX_HPCONTROL_PERFORMANCE_MASK    GENMASK(4, 3)
#define AIC31XX_HPCONTROL_HPL_LINE_MASK BIT(2)
#define AIC31XX_HPCONTROL_HPR_LINE_MASK BIT(3)

#define AIC31XX_TIMER_SELECT_MASK       BIT(7)

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
extern void enable_headset_detect(void); 
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

extern void     tlv320aic31x_deinit(void);
extern rt_err_t tlv320aic31x_init(rt_uint8_t volume, rt_uint32_t auido_freq);
extern rt_err_t tlv320aic31x_play(void);
extern rt_err_t tlv320aic31x_stop(rt_bool_t power_down);
extern rt_err_t tlv320aic31x_set_frequency(rt_uint32_t auido_freq);
extern rt_err_t tlv320aic31x_volume(rt_uint8_t auido_volume);

#endif    /* TLV320AIC31XX_REGS_H */

#endif
