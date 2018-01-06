#ifndef _FNET_LM3S_H_

#define _FNET_LM3S_H_

#if FNET_LM3S

/*********************************************************************
*
* The basic data types.
*
*********************************************************************/
typedef unsigned char fnet_uint8_t;         /*  8 bits */
typedef unsigned short int fnet_uint16_t;   /* 16 bits */
typedef unsigned long int fnet_uint32_t;    /* 32 bits */
typedef unsigned long long fnet_uint64_t;   /* 64 bits */

typedef signed char fnet_int8_t;            /*  8 bits */
typedef signed short int fnet_int16_t;      /* 16 bits */
typedef signed long int fnet_int32_t;       /* 32 bits */
typedef signed long long fnet_int64_t;      /* 64 bits */

typedef volatile fnet_uint8_t fnet_vuint8_t;     /*  8 bits */
typedef volatile fnet_uint16_t fnet_vuint16_t;   /* 16 bits */
typedef volatile fnet_uint32_t fnet_vuint32_t;   /* 32 bits */
typedef volatile fnet_uint64_t fnet_vuint64_t;   /* 64 bits */


/* Ensure that the Thumb bit is set.*/
#define FNET_CPU_ADDR_TO_INSTRUCTION(addr)    ((fnet_uint32_t)(addr)|0x1u)
#define FNET_CPU_INSTRUCTION_TO_ADDR(addr)    ((fnet_uint32_t)(addr)&(~0x1u))


/* ----------------------------------------------------------------------------
   -- UART
   ---------------------------------------------------------------------------- */
typedef struct FNET_LM3S_UART_MemMap 
{                                    
  fnet_uint32_t  DR;                                /*!< UART Data                                                             */
  
  union {
    fnet_uint32_t  UART_ALT_ECR;                    /*!< UART Receive Status/Error Clear                                       */
    fnet_uint32_t  RSR;                             /*!< UART Receive Status/Error Clear                                       */
  } ;
  fnet_uint32_t  RESERVED0[4];
  fnet_uint32_t  FR;                                /*!< UART Flag                                                             */
  fnet_uint32_t  RESERVED1;
  fnet_uint32_t  ILPR;                              /*!< UART IrDA Low-Power Register                                          */
  fnet_uint32_t  IBRD;                              /*!< UART Integer Baud-Rate Divisor                                        */
  fnet_uint32_t  FBRD;                              /*!< UART Fractional Baud-Rate Divisor                                     */
  fnet_uint32_t  LCRH;                              /*!< UART Line Control                                                     */
  fnet_uint32_t  CTL;                               /*!< UART Control                                                          */
  fnet_uint32_t  IFLS;                              /*!< UART Interrupt FIFO Level Select                                      */
  fnet_uint32_t  IM;                                /*!< UART Interrupt Mask                                                   */
  fnet_uint32_t  RIS;                               /*!< UART Raw Interrupt Status                                             */
  fnet_uint32_t  MIS;                               /*!< UART Masked Interrupt Status                                          */
  fnet_uint32_t  ICR;                               /*!< UART Interrupt Clear                                                  */
} volatile *FNET_LM3S_UART_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- UART Register Masks
   ---------------------------------------------------------------------------- */
#define FNET_LM3S_UART_DR_OE              0x00000800  // UART Overrun Error
#define FNET_LM3S_UART_DR_BE              0x00000400  // UART Break Error
#define FNET_LM3S_UART_DR_PE              0x00000200  // UART Parity Error
#define FNET_LM3S_UART_DR_FE              0x00000100  // UART Framing Error
#define FNET_LM3S_UART_DR_DATA_M          0x000000FF  // Data Transmitted or Received
#define FNET_LM3S_UART_DR_DATA_S          0

#define FNET_LM3S_UART_RSR_OE             0x00000008  // UART Overrun Error
#define FNET_LM3S_UART_RSR_BE             0x00000004  // UART Break Error
#define FNET_LM3S_UART_RSR_PE             0x00000002  // UART Parity Error
#define FNET_LM3S_UART_RSR_FE             0x00000001  // UART Framing Error

#define FNET_LM3S_UART_ECR_DATA_M         0x000000FF  // Error Clear
#define FNET_LM3S_UART_ECR_DATA_S         0

#define FNET_LM3S_UART_FR_TXFE            0x00000080  // UART Transmit FIFO Empty
#define FNET_LM3S_UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define FNET_LM3S_UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define FNET_LM3S_UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define FNET_LM3S_UART_FR_BUSY            0x00000008  // UART Busy

#define FNET_LM3S_UART_ILPR_ILPDVSR_M     0x000000FF  // IrDA Low-Power Divisor
#define FNET_LM3S_UART_ILPR_ILPDVSR_S     0

#define FNET_LM3S_UART_IBRD_DIVINT_M      0x0000FFFF  // Integer Baud-Rate Divisor
#define FNET_LM3S_UART_IBRD_DIVINT_S      0

#define FNET_LM3S_UART_FBRD_DIVFRAC_M     0x0000003F  // Fractional Baud-Rate Divisor
#define FNET_LM3S_UART_FBRD_DIVFRAC_S     0

#define FNET_LM3S_UART_LCRH_SPS           0x00000080  // UART Stick Parity Select
#define FNET_LM3S_UART_LCRH_WLEN_M        0x00000060  // UART Word Length
#define FNET_LM3S_UART_LCRH_WLEN_5        0x00000000  // 5 bits (default)
#define FNET_LM3S_UART_LCRH_WLEN_6        0x00000020  // 6 bits
#define FNET_LM3S_UART_LCRH_WLEN_7        0x00000040  // 7 bits
#define FNET_LM3S_UART_LCRH_WLEN_8        0x00000060  // 8 bits
#define FNET_LM3S_UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define FNET_LM3S_UART_LCRH_STP2          0x00000008  // UART Two Stop Bits Select
#define FNET_LM3S_UART_LCRH_EPS           0x00000004  // UART Even Parity Select
#define FNET_LM3S_UART_LCRH_PEN           0x00000002  // UART Parity Enable
#define FNET_LM3S_UART_LCRH_BRK           0x00000001  // UART Send Break

#define FNET_LM3S_UART_CTL_RXE            0x00000200  // UART Receive Enable
#define FNET_LM3S_UART_CTL_TXE            0x00000100  // UART Transmit Enable
#define FNET_LM3S_UART_CTL_LBE            0x00000080  // UART Loop Back Enable
#define FNET_LM3S_UART_CTL_SIRLP          0x00000004  // UART SIR Low-Power Mode
#define FNET_LM3S_UART_CTL_SIREN          0x00000002  // UART SIR Enable
#define FNET_LM3S_UART_CTL_UARTEN         0x00000001  // UART Enable

#define FNET_LM3S_UART_IFLS_RX_M          0x00000038  // UART Receive Interrupt FIFO Level Select
#define FNET_LM3S_UART_IFLS_RX1_8         0x00000000  // RX FIFO >= 1/8 full
#define FNET_LM3S_UART_IFLS_RX2_8         0x00000008  // RX FIFO >= 1/4 full
#define FNET_LM3S_UART_IFLS_RX4_8         0x00000010  // RX FIFO >= 1/2 full (default)
#define FNET_LM3S_UART_IFLS_RX6_8         0x00000018  // RX FIFO >= 3/4 full
#define FNET_LM3S_UART_IFLS_RX7_8         0x00000020  // RX FIFO >= 7/8 full
#define FNET_LM3S_UART_IFLS_TX_M          0x00000007  // UART Transmit Interrupt FIFO Level Select
#define FNET_LM3S_UART_IFLS_TX1_8         0x00000000  // TX FIFO <= 1/8 full
#define FNET_LM3S_UART_IFLS_TX2_8         0x00000001  // TX FIFO <= 1/4 full
#define FNET_LM3S_UART_IFLS_TX4_8         0x00000002  // TX FIFO <= 1/2 full (default)
#define FNET_LM3S_UART_IFLS_TX6_8         0x00000003  // TX FIFO <= 3/4 full
#define FNET_LM3S_UART_IFLS_TX7_8         0x00000004  // TX FIFO <= 7/8 full

#define FNET_LM3S_UART_IM_OEIM            0x00000400  // UART Overrun Error Interrupt Mask
#define FNET_LM3S_UART_IM_BEIM            0x00000200  // UART Break Error Interrupt Mask
#define FNET_LM3S_UART_IM_PEIM            0x00000100  // UART Parity Error Interrupt Mask
#define FNET_LM3S_UART_IM_FEIM            0x00000080  // UART Framing Error Interrupt Mask
#define FNET_LM3S_UART_IM_RTIM            0x00000040  // UART Receive Time-Out Interrupt Mask
#define FNET_LM3S_UART_IM_TXIM            0x00000020  // UART Transmit Interrupt Mask
#define FNET_LM3S_UART_IM_RXIM            0x00000010  // UART Receive Interrupt Mask

#define FNET_LM3S_UART_RIS_OERIS          0x00000400  // UART Overrun Error Raw Interrupt Status
#define FNET_LM3S_UART_RIS_BERIS          0x00000200  // UART Break Error Raw Interrupt Status
#define FNET_LM3S_UART_RIS_PERIS          0x00000100  // UART Parity Error Raw Interrupt Status
#define FNET_LM3S_UART_RIS_FERIS          0x00000080  // UART Framing Error Raw Interrupt Status
#define FNET_LM3S_UART_RIS_RTRIS          0x00000040  // UART Receive Time-Out Raw Interrupt Status
#define FNET_LM3S_UART_RIS_TXRIS          0x00000020  // UART Transmit Raw Interrupt Status
#define FNET_LM3S_UART_RIS_RXRIS          0x00000010  // UART Receive Raw Interrupt Status

#define FNET_LM3S_UART_MIS_OEMIS          0x00000400  // UART Overrun Error Masked Interrupt Status
#define FNET_LM3S_UART_MIS_BEMIS          0x00000200  // UART Break Error Masked Interrupt Status
#define FNET_LM3S_UART_MIS_PEMIS          0x00000100  // UART Parity Error Masked Interrupt Status
#define FNET_LM3S_UART_MIS_FEMIS          0x00000080  // UART Framing Error Masked Interrupt Status
#define FNET_LM3S_UART_MIS_RTMIS          0x00000040  // UART Receive Time-Out Masked Interrupt Status
#define FNET_LM3S_UART_MIS_TXMIS          0x00000020  // UART Transmit Masked Interrupt Status
#define FNET_LM3S_UART_MIS_RXMIS          0x00000010  // UART Receive Masked Interrupt Status

#define FNET_LM3S_UART_ICR_OEIC           0x00000400  // Overrun Error Interrupt Clear
#define FNET_LM3S_UART_ICR_BEIC           0x00000200  // Break Error Interrupt Clear
#define FNET_LM3S_UART_ICR_PEIC           0x00000100  // Parity Error Interrupt Clear
#define FNET_LM3S_UART_ICR_FEIC           0x00000080  // Framing Error Interrupt Clear
#define FNET_LM3S_UART_ICR_RTIC           0x00000040  // Receive Time-Out Interrupt Clear
#define FNET_LM3S_UART_ICR_TXIC           0x00000020  // Transmit Interrupt Clear
#define FNET_LM3S_UART_ICR_RXIC           0x00000010  // Receive Interrupt Clear

/* UART - Peripheral instance base addresses */
/* Peripheral UART0 base pointer */
#define FNET_LM3S_UART0_BASE_PTR                         ((FNET_LM3S_UART_MemMapPtr)0x4000C000UL)
/* Peripheral UART1 base pointer */
#define FNET_LM3S_UART1_BASE_PTR                         ((FNET_LM3S_UART_MemMapPtr)0x4000D000UL)
/* Peripheral UART2 base pointer */
#define FNET_LM3S_UART2_BASE_PTR                         ((FNET_LM3S_UART_MemMapPtr)0x4000E000UL)

/* ----------------------------------------------------------------------------
   -- SYSCTL
   ---------------------------------------------------------------------------- */

/* SYSCTL - Peripheral register structure */
typedef struct FNET_LM3S_SYSCTL_MemMap
{                                    
  fnet_uint32_t  DID0;                              /*!< Device Identification 0                                               */
  fnet_uint32_t  DID1;                              /*!< Device Identification 1                                               */
  fnet_uint32_t  DC0;                               /*!< Device Capabilities 0                                                 */
  fnet_uint32_t  RESERVED0;
  fnet_uint32_t  DC1;                               /*!< Device Capabilities 1                                                 */
  fnet_uint32_t  DC2;                               /*!< Device Capabilities 2                                                 */
  fnet_uint32_t  DC3;                               /*!< Device Capabilities 3                                                 */
  fnet_uint32_t  DC4;                               /*!< Device Capabilities 4                                                 */
  fnet_uint32_t  RESERVED1[4];
  fnet_uint32_t  PBORCTL;                           /*!< Brown-Out Reset Control                                               */
  fnet_uint32_t  LDOPCTL;                           /*!< LDO Power Control                                                     */
  fnet_uint32_t  RESERVED2[2];
  fnet_uint32_t  SRCR0;                             /*!< Software Reset Control 0                                              */
  fnet_uint32_t  SRCR1;                             /*!< Software Reset Control 1                                              */
  fnet_uint32_t  SRCR2;                             /*!< Software Reset Control 2                                              */
  fnet_uint32_t  RESERVED3;
  fnet_uint32_t  RIS;                               /*!< Raw Interrupt Status                                                  */
  fnet_uint32_t  IMC;                               /*!< Interrupt Mask Control                                                */
  fnet_uint32_t  MISC;                              /*!< Masked Interrupt Status and Clear                                     */
  fnet_uint32_t  RESC;                              /*!< Reset Cause                                                           */
  fnet_uint32_t  RCC;                               /*!< Run-Mode Clock Configuration                                          */
  fnet_uint32_t  PLLCFG;                            /*!< XTAL to PLL Translation                                               */
  fnet_uint32_t  RESERVED4[2];
  fnet_uint32_t  RCC2;                              /*!< Run-Mode Clock Configuration 2                                        */
  fnet_uint32_t  RESERVED5[35];
  fnet_uint32_t  RCGC0;                             /*!< Run Mode Clock Gating Control Register 0                              */
  fnet_uint32_t  RCGC1;                             /*!< Run Mode Clock Gating Control Register 1                              */
  fnet_uint32_t  RCGC2;                             /*!< Run Mode Clock Gating Control Register 2                              */
  fnet_uint32_t  RESERVED6;
  fnet_uint32_t  SCGC0;                             /*!< Sleep Mode Clock Gating Control Register 0                            */
  fnet_uint32_t  SCGC1;                             /*!< Sleep Mode Clock Gating Control Register 1                            */
  fnet_uint32_t  SCGC2;                             /*!< Sleep Mode Clock Gating Control Register 2                            */
  fnet_uint32_t  RESERVED7;
  fnet_uint32_t  DCGC0;                             /*!< Deep Sleep Mode Clock Gating Control Register 0                       */
  fnet_uint32_t  DCGC1;                             /*!< Deep-Sleep Mode Clock Gating Control Register 1                       */
  fnet_uint32_t  DCGC2;                             /*!< Deep Sleep Mode Clock Gating Control Register 2                       */
  fnet_uint32_t  RESERVED8[6];
  fnet_uint32_t  DSLPCLKCFG;                        /*!< Deep Sleep Clock Configuration                                        */
} volatile *FNET_LM3S_SYSCTL_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SYSCTL Register Masks
   ---------------------------------------------------------------------------- */
#define FNET_LM3S_SYSCTL_DID0_VER_M       0x70000000  // DID0 Version
#define FNET_LM3S_SYSCTL_DID0_VER_1       0x10000000  // Second version of the DID0 register format
#define FNET_LM3S_SYSCTL_DID0_CLASS_M     0x00FF0000  // Device Class
#define FNET_LM3S_SYSCTL_DID0_CLASS_FURY  0x00010000  // Stellaris(R) Fury-class devices
#define FNET_LM3S_SYSCTL_DID0_MAJ_M       0x0000FF00  // Major Revision
#define FNET_LM3S_SYSCTL_DID0_MAJ_REVA    0x00000000  // Revision A (initial device)
#define FNET_LM3S_SYSCTL_DID0_MAJ_REVB    0x00000100  // Revision B (first base layer revision)
#define FNET_LM3S_SYSCTL_DID0_MAJ_REVC    0x00000200  // Revision C (second base layer revision)
#define FNET_LM3S_SYSCTL_DID0_MIN_M       0x000000FF  // Minor Revision
#define FNET_LM3S_SYSCTL_DID0_MIN_0       0x00000000  // Initial device, or a major revision update
#define FNET_LM3S_SYSCTL_DID0_MIN_1       0x00000001  // First metal layer change
#define FNET_LM3S_SYSCTL_DID0_MIN_2       0x00000002  // Second metal layer change

#define FNET_LM3S_SYSCTL_DID1_VER_M       0xF0000000  // DID1 Version
#define FNET_LM3S_SYSCTL_DID1_VER_1       0x10000000  // Second version of the DID1 register format
#define FNET_LM3S_SYSCTL_DID1_FAM_M       0x0F000000  // Family
#define FNET_LM3S_SYSCTL_DID1_FAM_STELLARIS 0x00000000  // Stellaris family of microcontollers, that is, all devices with external part numbers starting with LM3S
#define FNET_LM3S_SYSCTL_DID1_PRTNO_M     0x00FF0000  // Part Number
#define FNET_LM3S_SYSCTL_DID1_PRTNO_6965  0x00730000  // LM3S6965
#define FNET_LM3S_SYSCTL_DID1_PRTNO_8962  0x00A60000  // LM3S8962
#define FNET_LM3S_SYSCTL_DID1_PINCNT_M    0x0000E000  // Package Pin Count
#define FNET_LM3S_SYSCTL_DID1_PINCNT_100  0x00004000  // 100-pin package
#define FNET_LM3S_SYSCTL_DID1_TEMP_M      0x000000E0  // Temperature Range
#define FNET_LM3S_SYSCTL_DID1_TEMP_C      0x00000000  // Commercial temperature range (0C to 70C)
#define FNET_LM3S_SYSCTL_DID1_TEMP_I      0x00000020  // Industrial temperature range (-40C to 85C)
#define FNET_LM3S_SYSCTL_DID1_TEMP_E      0x00000040  // Extended temperature range (-40C to 105C)
#define FNET_LM3S_SYSCTL_DID1_PKG_M       0x00000018  // Package Type
#define FNET_LM3S_SYSCTL_DID1_PKG_SOIC    0x00000000  // SOIC package
#define FNET_LM3S_SYSCTL_DID1_PKG_QFP     0x00000008  // LQFP package
#define FNET_LM3S_SYSCTL_DID1_PKG_BGA     0x00000010  // BGA package
#define FNET_LM3S_SYSCTL_DID1_ROHS        0x00000004  // RoHS-Compliance
#define FNET_LM3S_SYSCTL_DID1_QUAL_M      0x00000003  // Qualification Status
#define FNET_LM3S_SYSCTL_DID1_QUAL_ES     0x00000000  // Engineering Sample (unqualified)
#define FNET_LM3S_SYSCTL_DID1_QUAL_PP     0x00000001  // Pilot Production (unqualified)
#define FNET_LM3S_SYSCTL_DID1_QUAL_FQ     0x00000002  // Fully Qualified

#define FNET_LM3S_SYSCTL_DC0_SRAMSZ_M     0xFFFF0000  // SRAM Size
#define FNET_LM3S_SYSCTL_DC0_SRAMSZ_64KB  0x00FF0000  // 64 KB of SRAM
#define FNET_LM3S_SYSCTL_DC0_FLASHSZ_M    0x0000FFFF  // Flash Size
#define FNET_LM3S_SYSCTL_DC0_FLASHSZ_256K 0x0000007F  // 256 KB of Flash
#define FNET_LM3S_SYSCTL_DC0_SRAMSZ_S     16          // SRAM size shift
#define FNET_LM3S_SYSCTL_DC0_FLASHSZ_S    0           // Flash size shift

#define FNET_LM3S_SYSCTL_DC1_PWM          0x00100000  // PWM Module Present
#define FNET_LM3S_SYSCTL_DC1_ADC          0x00010000  // ADC Module Present
#define FNET_LM3S_SYSCTL_DC1_MINSYSDIV_M  0x0000F000  // System Clock Divider
#define FNET_LM3S_SYSCTL_DC1_MINSYSDIV_50 0x00003000  // Specifies a 50-MHz CPU clock with a PLL divider of 4
#define FNET_LM3S_SYSCTL_DC1_ADCSPD_M     0x00000300  // Max ADC Speed
#define FNET_LM3S_SYSCTL_DC1_ADCSPD_1M    0x00000300  // 1M samples/second
#define FNET_LM3S_SYSCTL_DC1_MPU          0x00000080  // MPU Present
#define FNET_LM3S_SYSCTL_DC1_HIB          0x00000040  // Hibernation Module Present
#define FNET_LM3S_SYSCTL_DC1_TEMP         0x00000020  // Temp Sensor Present
#define FNET_LM3S_SYSCTL_DC1_PLL          0x00000010  // PLL Present
#define FNET_LM3S_SYSCTL_DC1_WDT          0x00000008  // Watchdog Timer Present
#define FNET_LM3S_SYSCTL_DC1_SWO          0x00000004  // SWO Trace Port Present
#define FNET_LM3S_SYSCTL_DC1_SWD          0x00000002  // SWD Present
#define FNET_LM3S_SYSCTL_DC1_JTAG         0x00000001  // JTAG Present

#define FNET_LM3S_SYSCTL_DC2_COMP1        0x02000000  // Analog Comparator 1 Present
#define FNET_LM3S_SYSCTL_DC2_COMP0        0x01000000  // Analog Comparator 0 Present
#define FNET_LM3S_SYSCTL_DC2_TIMER3       0x00080000  // Timer Module 3 Present
#define FNET_LM3S_SYSCTL_DC2_TIMER2       0x00040000  // Timer Module 2 Present
#define FNET_LM3S_SYSCTL_DC2_TIMER1       0x00020000  // Timer Module 1 Present
#define FNET_LM3S_SYSCTL_DC2_TIMER0       0x00010000  // Timer Module 0 Present
#define FNET_LM3S_SYSCTL_DC2_I2C1         0x00004000  // I2C Module 1 Present
#define FNET_LM3S_SYSCTL_DC2_I2C0         0x00001000  // I2C Module 0 Present
#define FNET_LM3S_SYSCTL_DC2_QEI1         0x00000200  // QEI Module 1 Present
#define FNET_LM3S_SYSCTL_DC2_QEI0         0x00000100  // QEI Module 0 Present
#define FNET_LM3S_SYSCTL_DC2_SSI0         0x00000010  // SSI Module 0 Present
#define FNET_LM3S_SYSCTL_DC2_UART2        0x00000004  // UART Module 2 Present
#define FNET_LM3S_SYSCTL_DC2_UART1        0x00000002  // UART Module 1 Present
#define FNET_LM3S_SYSCTL_DC2_UART0        0x00000001  // UART Module 0 Present

#define FNET_LM3S_SYSCTL_DC3_32KHZ        0x80000000  // 32KHz Input Clock Available
#define FNET_LM3S_SYSCTL_DC3_CCP3         0x08000000  // CCP3 Pin Present
#define FNET_LM3S_SYSCTL_DC3_CCP2         0x04000000  // CCP2 Pin Present
#define FNET_LM3S_SYSCTL_DC3_CCP1         0x02000000  // CCP1 Pin Present
#define FNET_LM3S_SYSCTL_DC3_CCP0         0x01000000  // CCP0 Pin Present
#define FNET_LM3S_SYSCTL_DC3_ADC3         0x00080000  // ADC3 Pin Present
#define FNET_LM3S_SYSCTL_DC3_ADC2         0x00040000  // ADC2 Pin Present
#define FNET_LM3S_SYSCTL_DC3_ADC1         0x00020000  // ADC1 Pin Present
#define FNET_LM3S_SYSCTL_DC3_ADC0         0x00010000  // ADC0 Pin Present
#define FNET_LM3S_SYSCTL_DC3_PWMFAULT     0x00008000  // PWM Fault Pin Present
#define FNET_LM3S_SYSCTL_DC3_C1PLUS       0x00000400  // C1+ Pin Present
#define FNET_LM3S_SYSCTL_DC3_C1MINUS      0x00000200  // C1- Pin Present
#define FNET_LM3S_SYSCTL_DC3_C0O          0x00000100  // C0o Pin Present
#define FNET_LM3S_SYSCTL_DC3_C0PLUS       0x00000080  // C0+ Pin Present
#define FNET_LM3S_SYSCTL_DC3_C0MINUS      0x00000040  // C0- Pin Present
#define FNET_LM3S_SYSCTL_DC3_PWM5         0x00000020  // PWM5 Pin Present
#define FNET_LM3S_SYSCTL_DC3_PWM4         0x00000010  // PWM4 Pin Present
#define FNET_LM3S_SYSCTL_DC3_PWM3         0x00000008  // PWM3 Pin Present
#define FNET_LM3S_SYSCTL_DC3_PWM2         0x00000004  // PWM2 Pin Present
#define FNET_LM3S_SYSCTL_DC3_PWM1         0x00000002  // PWM1 Pin Present
#define FNET_LM3S_SYSCTL_DC3_PWM0         0x00000001  // PWM0 Pin Present

#define FNET_LM3S_SYSCTL_DC4_EPHY0        0x40000000  // Ethernet PHY Layer 0 Present
#define FNET_LM3S_SYSCTL_DC4_EMAC0        0x10000000  // Ethernet MAC Layer 0 Present
#define FNET_LM3S_SYSCTL_DC4_GPIOG        0x00000040  // GPIO Port G Present
#define FNET_LM3S_SYSCTL_DC4_GPIOF        0x00000020  // GPIO Port F Present
#define FNET_LM3S_SYSCTL_DC4_GPIOE        0x00000010  // GPIO Port E Present
#define FNET_LM3S_SYSCTL_DC4_GPIOD        0x00000008  // GPIO Port D Present
#define FNET_LM3S_SYSCTL_DC4_GPIOC        0x00000004  // GPIO Port C Present
#define FNET_LM3S_SYSCTL_DC4_GPIOB        0x00000002  // GPIO Port B Present
#define FNET_LM3S_SYSCTL_DC4_GPIOA        0x00000001  // GPIO Port A Present

#define FNET_LM3S_SYSCTL_PBORCTL_BORIOR   0x00000002  // BOR Interrupt or Reset

#define FNET_LM3S_SYSCTL_LDOPCTL_M        0x0000003F  // LDO Output Voltage
#define FNET_LM3S_SYSCTL_LDOPCTL_2_50V    0x00000000  // 2.50
#define FNET_LM3S_SYSCTL_LDOPCTL_2_45V    0x00000001  // 2.45
#define FNET_LM3S_SYSCTL_LDOPCTL_2_40V    0x00000002  // 2.40
#define FNET_LM3S_SYSCTL_LDOPCTL_2_35V    0x00000003  // 2.35
#define FNET_LM3S_SYSCTL_LDOPCTL_2_30V    0x00000004  // 2.30
#define FNET_LM3S_SYSCTL_LDOPCTL_2_25V    0x00000005  // 2.25
#define FNET_LM3S_SYSCTL_LDOPCTL_2_75V    0x0000001B  // 2.75
#define FNET_LM3S_SYSCTL_LDOPCTL_2_70V    0x0000001C  // 2.70
#define FNET_LM3S_SYSCTL_LDOPCTL_2_65V    0x0000001D  // 2.65
#define FNET_LM3S_SYSCTL_LDOPCTL_2_60V    0x0000001E  // 2.60
#define FNET_LM3S_SYSCTL_LDOPCTL_2_55V    0x0000001F  // 2.55

#define FNET_LM3S_SYSCTL_SRCR0_PWM        0x00100000  // PWM Reset Control
#define FNET_LM3S_SYSCTL_SRCR0_ADC        0x00010000  // ADC0 Reset Control
#define FNET_LM3S_SYSCTL_SRCR0_HIB        0x00000040  // HIB Reset Control
#define FNET_LM3S_SYSCTL_SRCR0_WDT        0x00000008  // WDT Reset Control

#define FNET_LM3S_SYSCTL_SRCR1_COMP1      0x02000000  // Analog Comp 1 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_COMP0      0x01000000  // Analog Comp 0 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_TIMER3     0x00080000  // Timer 3 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_TIMER2     0x00040000  // Timer 2 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_TIMER1     0x00020000  // Timer 1 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_TIMER0     0x00010000  // Timer 0 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_I2C1       0x00004000  // I2C1 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_I2C0       0x00001000  // I2C0 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_QEI1       0x00000200  // QEI1 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_QEI0       0x00000100  // QEI0 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_SSI0       0x00000010  // SSI0 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_UART2      0x00000004  // UART2 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_UART1      0x00000002  // UART1 Reset Control
#define FNET_LM3S_SYSCTL_SRCR1_UART0      0x00000001  // UART0 Reset Control

#define FNET_LM3S_SYSCTL_SRCR2_EPHY0      0x40000000  // PHY0 Reset Control
#define FNET_LM3S_SYSCTL_SRCR2_EMAC0      0x10000000  // MAC0 Reset Control
#define FNET_LM3S_SYSCTL_SRCR2_GPIOG      0x00000040  // Port G Reset Control
#define FNET_LM3S_SYSCTL_SRCR2_GPIOF      0x00000020  // Port F Reset Control
#define FNET_LM3S_SYSCTL_SRCR2_GPIOE      0x00000010  // Port E Reset Control
#define FNET_LM3S_SYSCTL_SRCR2_GPIOD      0x00000008  // Port D Reset Control
#define FNET_LM3S_SYSCTL_SRCR2_GPIOC      0x00000004  // Port C Reset Control
#define FNET_LM3S_SYSCTL_SRCR2_GPIOB      0x00000002  // Port B Reset Control
#define FNET_LM3S_SYSCTL_SRCR2_GPIOA      0x00000001  // Port A Reset Control

#define FNET_LM3S_SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status
#define FNET_LM3S_SYSCTL_RIS_BORRIS       0x00000002  // Brown-Out Reset Raw Interrupt Status

#define FNET_LM3S_SYSCTL_IMC_PLLLIM       0x00000040  // PLL Lock Interrupt Mask
#define FNET_LM3S_SYSCTL_IMC_BORIM        0x00000002  // Brown-Out Reset Interrupt Mask

#define FNET_LM3S_SYSCTL_MISC_PLLLMIS     0x00000040  // PLL Lock Masked Interrupt Status
#define FNET_LM3S_SYSCTL_MISC_BORMIS      0x00000002  // BOR Masked Interrupt Status

#define FNET_LM3S_SYSCTL_RESC_SW          0x00000010  // Software Reset
#define FNET_LM3S_SYSCTL_RESC_WDT         0x00000008  // Watchdog Timer Reset
#define FNET_LM3S_SYSCTL_RESC_BOR         0x00000004  // Brown-Out Reset
#define FNET_LM3S_SYSCTL_RESC_POR         0x00000002  // Power-On Reset
#define FNET_LM3S_SYSCTL_RESC_EXT         0x00000001  // External Reset

#define FNET_LM3S_SYSCTL_RCC_ACG          0x08000000  // Auto Clock Gating
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_M     0x07800000  // System Clock Divisor
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_2     0x00800000  // System clock /2
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_3     0x01000000  // System clock /3
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_4     0x01800000  // System clock /4
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_5     0x02000000  // System clock /5
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_6     0x02800000  // System clock /6
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_7     0x03000000  // System clock /7
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_8     0x03800000  // System clock /8
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_9     0x04000000  // System clock /9
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_10    0x04800000  // System clock /10
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_11    0x05000000  // System clock /11
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_12    0x05800000  // System clock /12
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_13    0x06000000  // System clock /13
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_14    0x06800000  // System clock /14
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_15    0x07000000  // System clock /15
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_16    0x07800000  // System clock /16
#define FNET_LM3S_SYSCTL_RCC_USESYSDIV    0x00400000  // Enable System Clock Divider
#define FNET_LM3S_SYSCTL_RCC_USEPWMDIV    0x00100000  // Enable PWM Clock Divisor
#define FNET_LM3S_SYSCTL_RCC_PWMDIV_M     0x000E0000  // PWM Unit Clock Divisor
#define FNET_LM3S_SYSCTL_RCC_PWMDIV_2     0x00000000  // PWM clock /2
#define FNET_LM3S_SYSCTL_RCC_PWMDIV_4     0x00020000  // PWM clock /4
#define FNET_LM3S_SYSCTL_RCC_PWMDIV_8     0x00040000  // PWM clock /8
#define FNET_LM3S_SYSCTL_RCC_PWMDIV_16    0x00060000  // PWM clock /16
#define FNET_LM3S_SYSCTL_RCC_PWMDIV_32    0x00080000  // PWM clock /32
#define FNET_LM3S_SYSCTL_RCC_PWMDIV_64    0x000A0000  // PWM clock /64
#define FNET_LM3S_SYSCTL_RCC_PWRDN        0x00002000  // PLL Power Down
#define FNET_LM3S_SYSCTL_RCC_BYPASS       0x00000800  // PLL Bypass
#define FNET_LM3S_SYSCTL_RCC_XTAL_M       0x000003C0  // Crystal Value
#define FNET_LM3S_SYSCTL_RCC_XTAL_1MHZ    0x00000000  // 1 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_1_84MHZ 0x00000040  // 1.8432 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_2MHZ    0x00000080  // 2 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_2_45MHZ 0x000000C0  // 2.4576 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_3_57MHZ 0x00000100  // 3.579545 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_3_68MHZ 0x00000140  // 3.6864 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_4MHZ    0x00000180  // 4 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_4_09MHZ 0x000001C0  // 4.096 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_4_91MHZ 0x00000200  // 4.9152 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_5MHZ    0x00000240  // 5 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_5_12MHZ 0x00000280  // 5.12 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_6MHZ    0x000002C0  // 6 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_6_14MHZ 0x00000300  // 6.144 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_7_37MHZ 0x00000340  // 7.3728 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_8MHZ    0x00000380  // 8 MHz
#define FNET_LM3S_SYSCTL_RCC_XTAL_8_19MHZ 0x000003C0  // 8.192 MHz
#define FNET_LM3S_SYSCTL_RCC_OSCSRC_M     0x00000030  // Oscillator Source
#define FNET_LM3S_SYSCTL_RCC_OSCSRC_MAIN  0x00000000  // MOSC
#define FNET_LM3S_SYSCTL_RCC_OSCSRC_INT   0x00000010  // IOSC
#define FNET_LM3S_SYSCTL_RCC_OSCSRC_INT4  0x00000020  // IOSC/4
#define FNET_LM3S_SYSCTL_RCC_OSCSRC_30    0x00000030  // 30 kHz
#define FNET_LM3S_SYSCTL_RCC_IOSCDIS      0x00000002  // Internal Oscillator Disable
#define FNET_LM3S_SYSCTL_RCC_MOSCDIS      0x00000001  // Main Oscillator Disable
#define FNET_LM3S_SYSCTL_RCC_SYSDIV_S     23

#define FNET_LM3S_SYSCTL_PLLCFG_F_M       0x00003FE0  // PLL F Value
#define FNET_LM3S_SYSCTL_PLLCFG_R_M       0x0000001F  // PLL R Value
#define FNET_LM3S_SYSCTL_PLLCFG_F_S       5
#define FNET_LM3S_SYSCTL_PLLCFG_R_S       0

#define FNET_LM3S_SYSCTL_RCC2_USERCC2     0x80000000  // Use RCC2
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_M   0x1F800000  // System Clock Divisor 2
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_2   0x00800000  // System clock /2
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_3   0x01000000  // System clock /3
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_4   0x01800000  // System clock /4
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_5   0x02000000  // System clock /5
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_6   0x02800000  // System clock /6
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_7   0x03000000  // System clock /7
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_8   0x03800000  // System clock /8
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_9   0x04000000  // System clock /9
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_10  0x04800000  // System clock /10
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_11  0x05000000  // System clock /11
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_12  0x05800000  // System clock /12
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_13  0x06000000  // System clock /13
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_14  0x06800000  // System clock /14
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_15  0x07000000  // System clock /15
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_16  0x07800000  // System clock /16
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_17  0x08000000  // System clock /17
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_18  0x08800000  // System clock /18
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_19  0x09000000  // System clock /19
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_20  0x09800000  // System clock /20
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_21  0x0A000000  // System clock /21
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_22  0x0A800000  // System clock /22
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_23  0x0B000000  // System clock /23
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_24  0x0B800000  // System clock /24
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_25  0x0C000000  // System clock /25
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_26  0x0C800000  // System clock /26
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_27  0x0D000000  // System clock /27
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_28  0x0D800000  // System clock /28
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_29  0x0E000000  // System clock /29
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_30  0x0E800000  // System clock /30
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_31  0x0F000000  // System clock /31
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_32  0x0F800000  // System clock /32
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_33  0x10000000  // System clock /33
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_34  0x10800000  // System clock /34
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_35  0x11000000  // System clock /35
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_36  0x11800000  // System clock /36
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_37  0x12000000  // System clock /37
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_38  0x12800000  // System clock /38
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_39  0x13000000  // System clock /39
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_40  0x13800000  // System clock /40
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_41  0x14000000  // System clock /41
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_42  0x14800000  // System clock /42
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_43  0x15000000  // System clock /43
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_44  0x15800000  // System clock /44
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_45  0x16000000  // System clock /45
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_46  0x16800000  // System clock /46
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_47  0x17000000  // System clock /47
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_48  0x17800000  // System clock /48
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_49  0x18000000  // System clock /49
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_50  0x18800000  // System clock /50
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_51  0x19000000  // System clock /51
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_52  0x19800000  // System clock /52
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_53  0x1A000000  // System clock /53
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_54  0x1A800000  // System clock /54
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_55  0x1B000000  // System clock /55
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_56  0x1B800000  // System clock /56
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_57  0x1C000000  // System clock /57
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_58  0x1C800000  // System clock /58
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_59  0x1D000000  // System clock /59
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_60  0x1D800000  // System clock /60
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_61  0x1E000000  // System clock /61
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_62  0x1E800000  // System clock /62
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_63  0x1F000000  // System clock /63
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_64  0x1F800000  // System clock /64
#define FNET_LM3S_SYSCTL_RCC2_PWRDN2      0x00002000  // Power-Down PLL 2
#define FNET_LM3S_SYSCTL_RCC2_BYPASS2     0x00000800  // PLL Bypass 2
#define FNET_LM3S_SYSCTL_RCC2_OSCSRC2_M   0x00000070  // Oscillator Source 2
#define FNET_LM3S_SYSCTL_RCC2_OSCSRC2_MO  0x00000000  // MOSC
#define FNET_LM3S_SYSCTL_RCC2_OSCSRC2_IO  0x00000010  // PIOSC
#define FNET_LM3S_SYSCTL_RCC2_OSCSRC2_IO4 0x00000020  // PIOSC/4
#define FNET_LM3S_SYSCTL_RCC2_OSCSRC2_30  0x00000030  // 30 kHz
#define FNET_LM3S_SYSCTL_RCC2_OSCSRC2_32  0x00000070  // 32.768 kHz
#define FNET_LM3S_SYSCTL_RCC2_SYSDIV2_S   23

#define FNET_LM3S_SYSCTL_RCGC0_PWM        0x00100000  // PWM Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC0_ADC        0x00010000  // ADC0 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC0_ADCSPD_M   0x00000300  // ADC Sample Speed
#define FNET_LM3S_SYSCTL_RCGC0_ADCSPD125K 0x00000000  // 125K samples/second
#define FNET_LM3S_SYSCTL_RCGC0_ADCSPD250K 0x00000100  // 250K samples/second
#define FNET_LM3S_SYSCTL_RCGC0_ADCSPD500K 0x00000200  // 500K samples/second
#define FNET_LM3S_SYSCTL_RCGC0_ADCSPD1M   0x00000300  // 1M samples/second
#define FNET_LM3S_SYSCTL_RCGC0_HIB        0x00000040  // HIB Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC0_WDT        0x00000008  // WDT Clock Gating Control

#define FNET_LM3S_SYSCTL_RCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock Gating
#define FNET_LM3S_SYSCTL_RCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock Gating
#define FNET_LM3S_SYSCTL_RCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_UART2      0x00000004  // UART2 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_UART1      0x00000002  // UART1 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC1_UART0      0x00000001  // UART0 Clock Gating Control

#define FNET_LM3S_SYSCTL_RCGC2_EPHY0      0x40000000  // PHY0 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC2_EMAC0      0x10000000  // MAC0 Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC2_GPIOG      0x00000040  // Port G Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC2_GPIOF      0x00000020  // Port F Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC2_GPIOE      0x00000010  // Port E Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC2_GPIOD      0x00000008  // Port D Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC2_GPIOC      0x00000004  // Port C Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC2_GPIOB      0x00000002  // Port B Clock Gating Control
#define FNET_LM3S_SYSCTL_RCGC2_GPIOA      0x00000001  // Port A Clock Gating Control

#define FNET_LM3S_SYSCTL_SCGC0_PWM        0x00100000  // PWM Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC0_ADC        0x00010000  // ADC0 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC0_ADCSPD_M   0x00000300  // ADC Sample Speed
#define FNET_LM3S_SYSCTL_SCGC0_ADCSPD125K 0x00000000  // 125K samples/second
#define FNET_LM3S_SYSCTL_SCGC0_ADCSPD250K 0x00000100  // 250K samples/second
#define FNET_LM3S_SYSCTL_SCGC0_ADCSPD500K 0x00000200  // 500K samples/second
#define FNET_LM3S_SYSCTL_SCGC0_ADCSPD1M   0x00000300  // 1M samples/second
#define FNET_LM3S_SYSCTL_SCGC0_HIB        0x00000040  // HIB Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC0_WDT        0x00000008  // WDT Clock Gating Control

#define FNET_LM3S_SYSCTL_SCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock Gating
#define FNET_LM3S_SYSCTL_SCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock Gating
#define FNET_LM3S_SYSCTL_SCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_UART2      0x00000004  // UART2 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_UART1      0x00000002  // UART1 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC1_UART0      0x00000001  // UART0 Clock Gating Control

#define FNET_LM3S_SYSCTL_SCGC2_EPHY0      0x40000000  // PHY0 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC2_EMAC0      0x10000000  // MAC0 Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC2_GPIOG      0x00000040  // Port G Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC2_GPIOF      0x00000020  // Port F Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC2_GPIOE      0x00000010  // Port E Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC2_GPIOD      0x00000008  // Port D Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC2_GPIOC      0x00000004  // Port C Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC2_GPIOB      0x00000002  // Port B Clock Gating Control
#define FNET_LM3S_SYSCTL_SCGC2_GPIOA      0x00000001  // Port A Clock Gating Control

#define FNET_LM3S_SYSCTL_DCGC0_PWM        0x00100000  // PWM Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC0_ADC        0x00010000  // ADC0 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC0_HIB        0x00000040  // HIB Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC0_WDT        0x00000008  // WDT Clock Gating Control

#define FNET_LM3S_SYSCTL_DCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock Gating
#define FNET_LM3S_SYSCTL_DCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock Gating
#define FNET_LM3S_SYSCTL_DCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_UART2      0x00000004  // UART2 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_UART1      0x00000002  // UART1 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC1_UART0      0x00000001  // UART0 Clock Gating Control

#define FNET_LM3S_SYSCTL_DCGC2_EPHY0      0x40000000  // PHY0 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC2_EMAC0      0x10000000  // MAC0 Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC2_GPIOG      0x00000040  // Port G Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC2_GPIOF      0x00000020  // Port F Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC2_GPIOE      0x00000010  // Port E Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC2_GPIOD      0x00000008  // Port D Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC2_GPIOC      0x00000004  // Port C Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC2_GPIOB      0x00000002  // Port B Clock Gating Control
#define FNET_LM3S_SYSCTL_DCGC2_GPIOA      0x00000001  // Port A Clock Gating Control

#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_M   0x1F800000  // Divider Field Override
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_1   0x00000000  // System clock /1
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_2   0x00800000  // System clock /2
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_3   0x01000000  // System clock /3
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_4   0x01800000  // System clock /4
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_5   0x02000000  // System clock /5
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_6   0x02800000  // System clock /6
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_7   0x03000000  // System clock /7
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_8   0x03800000  // System clock /8
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_9   0x04000000  // System clock /9
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_10  0x04800000  // System clock /10
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_11  0x05000000  // System clock /11
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_12  0x05800000  // System clock /12
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_13  0x06000000  // System clock /13
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_14  0x06800000  // System clock /14
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_15  0x07000000  // System clock /15
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_16  0x07800000  // System clock /16
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_17  0x08000000  // System clock /17
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_18  0x08800000  // System clock /18
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_19  0x09000000  // System clock /19
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_20  0x09800000  // System clock /20
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_21  0x0A000000  // System clock /21
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_22  0x0A800000  // System clock /22
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_23  0x0B000000  // System clock /23
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_24  0x0B800000  // System clock /24
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_25  0x0C000000  // System clock /25
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_26  0x0C800000  // System clock /26
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_27  0x0D000000  // System clock /27
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_28  0x0D800000  // System clock /28
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_29  0x0E000000  // System clock /29
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_30  0x0E800000  // System clock /30
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_31  0x0F000000  // System clock /31
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_32  0x0F800000  // System clock /32
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_33  0x10000000  // System clock /33
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_34  0x10800000  // System clock /34
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_35  0x11000000  // System clock /35
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_36  0x11800000  // System clock /36
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_37  0x12000000  // System clock /37
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_38  0x12800000  // System clock /38
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_39  0x13000000  // System clock /39
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_40  0x13800000  // System clock /40
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_41  0x14000000  // System clock /41
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_42  0x14800000  // System clock /42
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_43  0x15000000  // System clock /43
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_44  0x15800000  // System clock /44
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_45  0x16000000  // System clock /45
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_46  0x16800000  // System clock /46
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_47  0x17000000  // System clock /47
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_48  0x17800000  // System clock /48
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_49  0x18000000  // System clock /49
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_50  0x18800000  // System clock /50
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_51  0x19000000  // System clock /51
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_52  0x19800000  // System clock /52
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_53  0x1A000000  // System clock /53
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_54  0x1A800000  // System clock /54
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_55  0x1B000000  // System clock /55
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_56  0x1B800000  // System clock /56
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_57  0x1C000000  // System clock /57
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_58  0x1C800000  // System clock /58
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_59  0x1D000000  // System clock /59
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_60  0x1D800000  // System clock /60
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_61  0x1E000000  // System clock /61
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_62  0x1E800000  // System clock /62
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_63  0x1F000000  // System clock /63
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_64  0x1F800000  // System clock /64
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_O_M   0x00000070  // Clock Source
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_O_IGN 0x00000000  // MOSC
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_O_IO  0x00000010  // PIOSC
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_O_30  0x00000030  // 30 kHz
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_O_32  0x00000070  // 32.768 kHz
#define FNET_LM3S_SYSCTL_DSLPCLKCFG_D_S   23


/* SYSCTL - Peripheral instance base addresses */
/* Peripheral SYSCTL base pointer */
#define FNET_LM3S_SYSCTL_BASE_PTR                       ((FNET_LM3S_SYSCTL_MemMap)0x400FE000UL)

/* ----------------------------------------------------------------------------
   -- PORT
   ---------------------------------------------------------------------------- */
typedef struct FNET_LM3S_PORT_MemMapPtr
{
  fnet_uint32_t  RESERVED0[255];
  fnet_uint32_t  DATA;                              /*!< GPIO Data                                                             */
  fnet_uint32_t  DIR;                               /*!< GPIO Direction                                                        */
  fnet_uint32_t  IS;                                /*!< GPIO Interrupt Sense                                                  */
  fnet_uint32_t  IBE;                               /*!< GPIO Interrupt Both Edges                                             */
  fnet_uint32_t  IEV;                               /*!< GPIO Interrupt Event                                                  */
  fnet_uint32_t  IM;                                /*!< GPIO Interrupt Mask                                                   */
  fnet_uint32_t  RIS;                               /*!< GPIO Raw Interrupt Status                                             */
  fnet_uint32_t  MIS;                               /*!< GPIO Masked Interrupt Status                                          */
  fnet_uint32_t  ICR;                               /*!< GPIO Interrupt Clear                                                  */
  fnet_uint32_t  AFSEL;                             /*!< GPIO Alternate Function Select                                        */
  fnet_uint32_t  RESERVED1[55];
  fnet_uint32_t  DR2R;                              /*!< GPIO 2-mA Drive Select                                                */
  fnet_uint32_t  DR4R;                              /*!< GPIO 4-mA Drive Select                                                */
  fnet_uint32_t  DR8R;                              /*!< GPIO 8-mA Drive Select                                                */
  fnet_uint32_t  ODR;                               /*!< GPIO Open Drain Select                                                */
  fnet_uint32_t  PUR;                               /*!< GPIO Pull-Up Select                                                   */
  fnet_uint32_t  PDR;                               /*!< GPIO Pull-Down Select                                                 */
  fnet_uint32_t  SLR;                               /*!< GPIO Slew Rate Control Select                                         */
  fnet_uint32_t  DEN;                               /*!< GPIO Digital Enable                                                   */
  fnet_uint32_t  LOCK;                              /*!< GPIO Lock                                                             */
  fnet_uint32_t  CR;                                /*!< GPIO Commit                                                           */
} volatile *FNET_LM3S_PORT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PORT Register Masks
   ---------------------------------------------------------------------------- */
#define FNET_LM3S_GPIO_LOCK_LOCKED        0x00000001  // The GPIOCR register is locked and may not be modified
#define FNET_LM3S_GPIO_LOCK_KEY           0x1ACCE551  // Unlocks the GPIO_CR register

/* PORT - Peripheral instance base addresses */
/* Peripheral PORTA base pointer */
#define FNET_LM3S_PORTA_BASE_PTR                         ((FNET_LM3S_PORT_MemMapPtr)0x40004000UL)
/* Peripheral PORTB base pointer */
#define FNET_LM3S_PORTB_BASE_PTR                         ((FNET_LM3S_PORT_MemMapPtr)0x40005000UL)
/* Peripheral PORTC base pointer */
#define FNET_LM3S_PORTC_BASE_PTR                         ((FNET_LM3S_PORT_MemMapPtr)0x40006000UL)
/* Peripheral PORTD base pointer */
#define FNET_LM3S_PORTD_BASE_PTR                         ((FNET_LM3S_PORT_MemMapPtr)0x40007000UL)
/* Peripheral PORTE base pointer */
#define FNET_LM3S_PORTE_BASE_PTR                         ((FNET_LM3S_PORT_MemMapPtr)0x40024000UL)
/* Peripheral PORTF base pointer */
#define FNET_LM3S_PORTF_BASE_PTR                         ((FNET_LM3S_PORT_MemMapPtr)0x40025000UL)
/* Peripheral PORTG base pointer */
#define FNET_LM3S_PORTG_BASE_PTR                         ((FNET_LM3S_PORT_MemMapPtr)0x40026000UL)

/* ----------------------------------------------------------------------------
   -- TIMER
   ---------------------------------------------------------------------------- */

/* TIMER - Peripheral register structure */
typedef struct FNET_LM3S_TIMER_MemMapPtr
{
  fnet_uint32_t  CFG;                               /*!< GPTM Configuration                                                    */
  fnet_uint32_t  TAMR;                              /*!< GPTM Timer A Mode                                                     */
  fnet_uint32_t  TBMR;                              /*!< GPTM Timer B Mode                                                     */
  fnet_uint32_t  CTL;                               /*!< GPTM Control                                                          */
  fnet_uint32_t  RESERVED0[2];
  fnet_uint32_t  IMR;                               /*!< GPTM Interrupt Mask                                                   */
  fnet_uint32_t  RIS;                               /*!< GPTM Raw Interrupt Status                                             */
  fnet_uint32_t  MIS;                               /*!< GPTM Masked Interrupt Status                                          */
  fnet_uint32_t  ICR;                               /*!< GPTM Interrupt Clear                                                  */
  fnet_uint32_t  TAILR;                             /*!< GPTM Timer A Interval Load                                            */
  fnet_uint32_t  TBILR;                             /*!< GPTM Timer B Interval Load                                            */
  fnet_uint32_t  TAMATCHR;                          /*!< GPTM Timer A Match                                                    */
  fnet_uint32_t  TBMATCHR;                          /*!< GPTM Timer B Match                                                    */
  fnet_uint32_t  TAPR;                              /*!< GPTM Timer A Prescale                                                 */
  fnet_uint32_t  TBPR;                              /*!< GPTM Timer B Prescale                                                 */
  fnet_uint32_t  TAPMR;                             /*!< GPTM TimerA Prescale Match                                            */
  fnet_uint32_t  TBPMR;                             /*!< GPTM TimerB Prescale Match                                            */
  fnet_uint32_t  TAR;                               /*!< GPTM Timer A                                                          */
  fnet_uint32_t  TBR;                               /*!< GPTM Timer B                                                          */
} volatile *FNET_LM3S_TIMER_MemMapPtr;

#define FNET_LM3S_TIMER_CFG_M             0x00000007  // GPTM Configuration
#define FNET_LM3S_TIMER_CFG_32_BIT_TIMER  0x00000000  // 32-bit timer configuration
#define FNET_LM3S_TIMER_CFG_32_BIT_RTC    0x00000001  // 32-bit real-time clock (RTC) counter configuration
#define FNET_LM3S_TIMER_CFG_16_BIT        0x00000004  // 16-bit timer configuration. The function is controlled by bits 1:0 of GPTMTAMR and GPTMTBMR

#define FNET_LM3S_TIMER_TAMR_TAAMS        0x00000008  // GPTM Timer A Alternate Mode Select
#define FNET_LM3S_TIMER_TAMR_TACMR        0x00000004  // GPTM Timer A Capture Mode
#define FNET_LM3S_TIMER_TAMR_TAMR_M       0x00000003  // GPTM Timer A Mode
#define FNET_LM3S_TIMER_TAMR_TAMR_1_SHOT  0x00000001  // One-Shot Timer mode
#define FNET_LM3S_TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define FNET_LM3S_TIMER_TAMR_TAMR_CAP     0x00000003  // Capture mode

#define FNET_LM3S_TIMER_TBMR_TBAMS        0x00000008  // GPTM Timer B Alternate Mode Select
#define FNET_LM3S_TIMER_TBMR_TBCMR        0x00000004  // GPTM Timer B Capture Mode
#define FNET_LM3S_TIMER_TBMR_TBMR_M       0x00000003  // GPTM Timer B Mode
#define FNET_LM3S_TIMER_TBMR_TBMR_1_SHOT  0x00000001  // One-Shot Timer mode
#define FNET_LM3S_TIMER_TBMR_TBMR_PERIOD  0x00000002  // Periodic Timer mode
#define FNET_LM3S_TIMER_TBMR_TBMR_CAP     0x00000003  // Capture mode

#define FNET_LM3S_TIMER_CTL_TBPWML        0x00004000  // GPTM Timer B PWM Output Level
#define FNET_LM3S_TIMER_CTL_TBOTE         0x00002000  // GPTM Timer B Output Trigger Enable
#define FNET_LM3S_TIMER_CTL_TBEVENT_M     0x00000C00  // GPTM Timer B Event Mode
#define FNET_LM3S_TIMER_CTL_TBEVENT_POS   0x00000000  // Positive edge
#define FNET_LM3S_TIMER_CTL_TBEVENT_NEG   0x00000400  // Negative edge
#define FNET_LM3S_TIMER_CTL_TBEVENT_BOTH  0x00000C00  // Both edges
#define FNET_LM3S_TIMER_CTL_TBSTALL       0x00000200  // GPTM Timer B Stall Enable
#define FNET_LM3S_TIMER_CTL_TBEN          0x00000100  // GPTM Timer B Enable
#define FNET_LM3S_TIMER_CTL_TAPWML        0x00000040  // GPTM Timer A PWM Output Level
#define FNET_LM3S_TIMER_CTL_TAOTE         0x00000020  // GPTM Timer A Output Trigger Enable
#define FNET_LM3S_TIMER_CTL_RTCEN         0x00000010  // GPTM RTC Enable
#define FNET_LM3S_TIMER_CTL_TAEVENT_M     0x0000000C  // GPTM Timer A Event Mode
#define FNET_LM3S_TIMER_CTL_TAEVENT_POS   0x00000000  // Positive edge
#define FNET_LM3S_TIMER_CTL_TAEVENT_NEG   0x00000004  // Negative edge
#define FNET_LM3S_TIMER_CTL_TAEVENT_BOTH  0x0000000C  // Both edges
#define FNET_LM3S_TIMER_CTL_TASTALL       0x00000002  // GPTM Timer A Stall Enable
#define FNET_LM3S_TIMER_CTL_TAEN          0x00000001  // GPTM Timer A Enable

#define FNET_LM3S_TIMER_IMR_CBEIM         0x00000400  // GPTM Capture B Event Interrupt Mask
#define FNET_LM3S_TIMER_IMR_CBMIM         0x00000200  // GPTM Capture B Match Interrupt Mask
#define FNET_LM3S_TIMER_IMR_TBTOIM        0x00000100  // GPTM Timer B Time-Out Interrupt Mask
#define FNET_LM3S_TIMER_IMR_RTCIM         0x00000008  // GPTM RTC Interrupt Mask
#define FNET_LM3S_TIMER_IMR_CAEIM         0x00000004  // GPTM Capture A Event Interrupt Mask
#define FNET_LM3S_TIMER_IMR_CAMIM         0x00000002  // GPTM Capture A Match Interrupt Mask
#define FNET_LM3S_TIMER_IMR_TATOIM        0x00000001  // GPTM Timer A Time-Out Interrupt Mask

#define FNET_LM3S_TIMER_RIS_CBERIS        0x00000400  // GPTM Capture B Event Raw Interrupt
#define FNET_LM3S_TIMER_RIS_CBMRIS        0x00000200  // GPTM Capture B Match Raw Interrupt
#define FNET_LM3S_TIMER_RIS_TBTORIS       0x00000100  // GPTM Timer B Time-Out Raw Interrupt
#define FNET_LM3S_TIMER_RIS_RTCRIS        0x00000008  // GPTM RTC Raw Interrupt
#define FNET_LM3S_TIMER_RIS_CAERIS        0x00000004  // GPTM Capture A Event Raw Interrupt
#define FNET_LM3S_TIMER_RIS_CAMRIS        0x00000002  // GPTM Capture A Match Raw Interrupt
#define FNET_LM3S_TIMER_RIS_TATORIS       0x00000001  // GPTM Timer A Time-Out Raw Interrupt

#define FNET_LM3S_TIMER_MIS_CBEMIS        0x00000400  // GPTM Capture B Event Masked Interrupt
#define FNET_LM3S_TIMER_MIS_CBMMIS        0x00000200  // GPTM Capture B Match Masked Interrupt
#define FNET_LM3S_TIMER_MIS_TBTOMIS       0x00000100  // GPTM Timer B Time-Out Masked Interrupt
#define FNET_LM3S_TIMER_MIS_RTCMIS        0x00000008  // GPTM RTC Masked Interrupt
#define FNET_LM3S_TIMER_MIS_CAEMIS        0x00000004  // GPTM Capture A Event Masked Interrupt
#define FNET_LM3S_TIMER_MIS_CAMMIS        0x00000002  // GPTM Capture A Match Masked Interrupt
#define FNET_LM3S_TIMER_MIS_TATOMIS       0x00000001  // GPTM Timer A Time-Out Masked Interrupt

#define FNET_LM3S_TIMER_ICR_CBECINT       0x00000400  // GPTM Capture B Event Interrupt Clear
#define FNET_LM3S_TIMER_ICR_CBMCINT       0x00000200  // GPTM Capture B Match Interrupt Clear
#define FNET_LM3S_TIMER_ICR_TBTOCINT      0x00000100  // GPTM Timer B Time-Out Interrupt Clear
#define FNET_LM3S_TIMER_ICR_RTCCINT       0x00000008  // GPTM RTC Interrupt Clear
#define FNET_LM3S_TIMER_ICR_CAECINT       0x00000004  // GPTM Capture A Event Interrupt Clear
#define FNET_LM3S_TIMER_ICR_CAMCINT       0x00000002  // GPTM Capture A Match Interrupt Clear
#define FNET_LM3S_TIMER_ICR_TATOCINT      0x00000001  // GPTM Timer A Time-Out Raw Interrupt

#define FNET_LM3S_TIMER_TAILR_TAILRH_M    0xFFFF0000  // GPTM Timer A Interval Load Register High
#define FNET_LM3S_TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM Timer A Interval Load Register Low
#define FNET_LM3S_TIMER_TAILR_TAILRH_S    16
#define FNET_LM3S_TIMER_TAILR_TAILRL_S    0

#define FNET_LM3S_TIMER_TBILR_TBILRL_M    0x0000FFFF  // GPTM Timer B Interval Load Register
#define FNET_LM3S_TIMER_TBILR_TBILRL_S    0

#define FNET_LM3S_TIMER_TAMATCHR_TAMRH_M  0xFFFF0000  // GPTM Timer A Match Register High
#define FNET_LM3S_TIMER_TAMATCHR_TAMRL_M  0x0000FFFF  // GPTM Timer A Match Register Low
#define FNET_LM3S_TIMER_TAMATCHR_TAMRH_S  16
#define FNET_LM3S_TIMER_TAMATCHR_TAMRL_S  0

#define FNET_LM3S_TIMER_TBMATCHR_TBMRL_M  0x0000FFFF  // GPTM Timer B Match Register Low
#define FNET_LM3S_TIMER_TBMATCHR_TBMRL_S  0

#define FNET_LM3S_TIMER_TAPR_TAPSR_M      0x000000FF  // GPTM Timer A Prescale
#define FNET_LM3S_TIMER_TAPR_TAPSR_S      0

#define FNET_LM3S_TIMER_TBPR_TBPSR_M      0x000000FF  // GPTM Timer B Prescale
#define FNET_LM3S_TIMER_TBPR_TBPSR_S      0

#define FNET_LM3S_TIMER_TAPMR_TAPSMR_M    0x000000FF  // GPTM TimerA Prescale Match
#define FNET_LM3S_TIMER_TAPMR_TAPSMR_S    0

#define FNET_LM3S_TIMER_TBPMR_TBPSMR_M    0x000000FF  // GPTM TimerB Prescale Match
#define FNET_LM3S_TIMER_TBPMR_TBPSMR_S    0

#define FNET_LM3S_TIMER_TAR_TARH_M        0xFFFF0000  // GPTM Timer A Register High
#define FNET_LM3S_TIMER_TAR_TARL_M        0x0000FFFF  // GPTM Timer A Register Low
#define FNET_LM3S_TIMER_TAR_TARH_S        16
#define FNET_LM3S_TIMER_TAR_TARL_S        0

#define FNET_LM3S_TIMER_TBR_TBRL_M        0x0000FFFF  // GPTM Timer B
#define FNET_LM3S_TIMER_TBR_TBRL_S        0

/* TIMER - Peripheral instance base addresses */
/* Peripheral TIMER base pointer */
#define FNET_LM3S_TIMER0_BASE_PTR                        ((FNET_LM3S_TIMER_MemMapPtr)0x40030000UL)
#define FNET_LM3S_TIMER1_BASE_PTR                        ((FNET_LM3S_TIMER_MemMapPtr)0x40031000UL)
#define FNET_LM3S_TIMER2_BASE_PTR                        ((FNET_LM3S_TIMER_MemMapPtr)0x40032000UL)
#define FNET_LM3S_TIMER3_BASE_PTR                        ((FNET_LM3S_TIMER_MemMapPtr)0x40033000UL)


/*********************************************************************
* Fast Ethernet Controller (MAC)
*********************************************************************/

typedef struct FNET_LM3S_MAC_MemMapPtr
{                                    
  union {
    fnet_uint32_t  MAC_ALT_IACK;                    /*!< Ethernet MAC Raw Interrupt Status/Acknowledge                         */
    fnet_uint32_t  RIS;                             /*!< Ethernet MAC Raw Interrupt Status/Acknowledge                         */
  } ;
  fnet_uint32_t  IM;                                /*!< Ethernet MAC Interrupt Mask                                           */
  fnet_uint32_t  RCTL;                              /*!< Ethernet MAC Receive Control                                          */
  fnet_uint32_t  TCTL;                              /*!< Ethernet MAC Transmit Control                                         */
  
  union {
    fnet_uint32_t  MAC_ALT_DATA;                    /*!< Ethernet MAC Data                                                     */
    fnet_uint32_t  DATA;                            /*!< Ethernet MAC Data                                                     */
  } ;
  fnet_uint32_t  IA0;                               /*!< Ethernet MAC Individual Address 0                                     */
  fnet_uint32_t  IA1;                               /*!< Ethernet MAC Individual Address 1                                     */
  fnet_uint32_t  THR;                               /*!< Ethernet MAC Threshold                                                */
  fnet_uint32_t  MCTL;                              /*!< Ethernet MAC Management Control                                       */
  fnet_uint32_t  MDV;                               /*!< Ethernet MAC Management Divider                                       */
  fnet_uint32_t  RESERVED0;
  fnet_uint32_t  MTXD;                              /*!< Ethernet MAC Management Transmit Data                                 */
  fnet_uint32_t  MRXD;                              /*!< Ethernet MAC Management Receive Data                                  */
  fnet_uint32_t  NP;                                /*!< Ethernet MAC Number of Packets                                        */
  fnet_uint32_t  TR;                                /*!< Ethernet MAC Transmission Request                                     */
  fnet_uint32_t  TS;                                /*!< Ethernet MAC Timer Support                                            */
} volatile *FNET_LM3S_MAC_MemMapPtr;

#define FNET_LM3S_MAC0_BASE_PTR                     ((FNET_LM3S_MAC_MemMapPtr)0x40048000UL)


#if defined(__cplusplus)
extern "C" {
#endif

void fnet_lm3s_irq_enable(fnet_uint32_t irq_desc);
fnet_uint32_t fnet_lm3s_irq_disable(void);

#if defined(__cplusplus)
}
#endif

#endif /* FNET_LM3S */

#endif /*_FNET_LM3S_H_*/
