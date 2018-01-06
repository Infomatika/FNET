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
#define UART_DR_OE              0x00000800  // UART Overrun Error
#define UART_DR_BE              0x00000400  // UART Break Error
#define UART_DR_PE              0x00000200  // UART Parity Error
#define UART_DR_FE              0x00000100  // UART Framing Error
#define UART_DR_DATA_M          0x000000FF  // Data Transmitted or Received
#define UART_DR_DATA_S          0

#define UART_RSR_OE             0x00000008  // UART Overrun Error
#define UART_RSR_BE             0x00000004  // UART Break Error
#define UART_RSR_PE             0x00000002  // UART Parity Error
#define UART_RSR_FE             0x00000001  // UART Framing Error

#define UART_ECR_DATA_M         0x000000FF  // Error Clear
#define UART_ECR_DATA_S         0

#define UART_FR_TXFE            0x00000080  // UART Transmit FIFO Empty
#define UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_FR_BUSY            0x00000008  // UART Busy

#define UART_ILPR_ILPDVSR_M     0x000000FF  // IrDA Low-Power Divisor
#define UART_ILPR_ILPDVSR_S     0

#define UART_IBRD_DIVINT_M      0x0000FFFF  // Integer Baud-Rate Divisor
#define UART_IBRD_DIVINT_S      0

#define UART_FBRD_DIVFRAC_M     0x0000003F  // Fractional Baud-Rate Divisor
#define UART_FBRD_DIVFRAC_S     0

#define UART_LCRH_SPS           0x00000080  // UART Stick Parity Select
#define UART_LCRH_WLEN_M        0x00000060  // UART Word Length
#define UART_LCRH_WLEN_5        0x00000000  // 5 bits (default)
#define UART_LCRH_WLEN_6        0x00000020  // 6 bits
#define UART_LCRH_WLEN_7        0x00000040  // 7 bits
#define UART_LCRH_WLEN_8        0x00000060  // 8 bits
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_LCRH_STP2          0x00000008  // UART Two Stop Bits Select
#define UART_LCRH_EPS           0x00000004  // UART Even Parity Select
#define UART_LCRH_PEN           0x00000002  // UART Parity Enable
#define UART_LCRH_BRK           0x00000001  // UART Send Break

#define UART_CTL_RXE            0x00000200  // UART Receive Enable
#define UART_CTL_TXE            0x00000100  // UART Transmit Enable
#define UART_CTL_LBE            0x00000080  // UART Loop Back Enable
#define UART_CTL_SIRLP          0x00000004  // UART SIR Low-Power Mode
#define UART_CTL_SIREN          0x00000002  // UART SIR Enable
#define UART_CTL_UARTEN         0x00000001  // UART Enable

#define UART_IFLS_RX_M          0x00000038  // UART Receive Interrupt FIFO Level Select
#define UART_IFLS_RX1_8         0x00000000  // RX FIFO >= 1/8 full
#define UART_IFLS_RX2_8         0x00000008  // RX FIFO >= 1/4 full
#define UART_IFLS_RX4_8         0x00000010  // RX FIFO >= 1/2 full (default)
#define UART_IFLS_RX6_8         0x00000018  // RX FIFO >= 3/4 full
#define UART_IFLS_RX7_8         0x00000020  // RX FIFO >= 7/8 full
#define UART_IFLS_TX_M          0x00000007  // UART Transmit Interrupt FIFO Level Select
#define UART_IFLS_TX1_8         0x00000000  // TX FIFO <= 1/8 full
#define UART_IFLS_TX2_8         0x00000001  // TX FIFO <= 1/4 full
#define UART_IFLS_TX4_8         0x00000002  // TX FIFO <= 1/2 full (default)
#define UART_IFLS_TX6_8         0x00000003  // TX FIFO <= 3/4 full
#define UART_IFLS_TX7_8         0x00000004  // TX FIFO <= 7/8 full

#define UART_IM_OEIM            0x00000400  // UART Overrun Error Interrupt Mask
#define UART_IM_BEIM            0x00000200  // UART Break Error Interrupt Mask
#define UART_IM_PEIM            0x00000100  // UART Parity Error Interrupt Mask
#define UART_IM_FEIM            0x00000080  // UART Framing Error Interrupt Mask
#define UART_IM_RTIM            0x00000040  // UART Receive Time-Out Interrupt Mask
#define UART_IM_TXIM            0x00000020  // UART Transmit Interrupt Mask
#define UART_IM_RXIM            0x00000010  // UART Receive Interrupt Mask

#define UART_RIS_OERIS          0x00000400  // UART Overrun Error Raw Interrupt Status
#define UART_RIS_BERIS          0x00000200  // UART Break Error Raw Interrupt Status
#define UART_RIS_PERIS          0x00000100  // UART Parity Error Raw Interrupt Status
#define UART_RIS_FERIS          0x00000080  // UART Framing Error Raw Interrupt Status
#define UART_RIS_RTRIS          0x00000040  // UART Receive Time-Out Raw Interrupt Status
#define UART_RIS_TXRIS          0x00000020  // UART Transmit Raw Interrupt Status
#define UART_RIS_RXRIS          0x00000010  // UART Receive Raw Interrupt Status

#define UART_MIS_OEMIS          0x00000400  // UART Overrun Error Masked Interrupt Status
#define UART_MIS_BEMIS          0x00000200  // UART Break Error Masked Interrupt Status
#define UART_MIS_PEMIS          0x00000100  // UART Parity Error Masked Interrupt Status
#define UART_MIS_FEMIS          0x00000080  // UART Framing Error Masked Interrupt Status
#define UART_MIS_RTMIS          0x00000040  // UART Receive Time-Out Masked Interrupt Status
#define UART_MIS_TXMIS          0x00000020  // UART Transmit Masked Interrupt Status
#define UART_MIS_RXMIS          0x00000010  // UART Receive Masked Interrupt Status

#define UART_ICR_OEIC           0x00000400  // Overrun Error Interrupt Clear
#define UART_ICR_BEIC           0x00000200  // Break Error Interrupt Clear
#define UART_ICR_PEIC           0x00000100  // Parity Error Interrupt Clear
#define UART_ICR_FEIC           0x00000080  // Framing Error Interrupt Clear
#define UART_ICR_RTIC           0x00000040  // Receive Time-Out Interrupt Clear
#define UART_ICR_TXIC           0x00000020  // Transmit Interrupt Clear
#define UART_ICR_RXIC           0x00000010  // Receive Interrupt Clear

/* UART - Peripheral instance base addresses */
/* Peripheral UART0 base pointer */
#define FNET_LM3S_UART0_BASE_PTR                         ((FNET_LM3S_UART_MemMapPtr)0x4000C000UL)
/* Peripheral UART1 base pointer */
#define FNET_LM3S_UART1_BASE_PTR                         ((FNET_LM3S_UART_MemMapPtr)0x4000D000UL)
/* Peripheral UART2 base pointer */
#define FNET_LM3S_UART2_BASE_PTR                         ((FNET_LM3S_UART_MemMapPtr)0x4000E000UL)


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
