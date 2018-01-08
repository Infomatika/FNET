#ifndef _FNET_LM3S8962_CONFIG_H_
#define _FNET_LM3S8962_CONFIG_H_
                
#define FNET_LM3S                                       (1)

/* System frequency in Hz. */
#ifndef FNET_CFG_CPU_CLOCK_HZ
    #define FNET_CFG_CPU_CLOCK_HZ                       (50000000)
#endif

/* Defines the maximum number of incoming frames that may
 *           be buffered by the Ethernet module.*/
#ifndef FNET_CFG_CPU_ETH_RX_BUFS_MAX
    #define FNET_CFG_CPU_ETH_RX_BUFS_M                  (4)
#endif

/* No cache. */
#define FNET_CFG_CPU_CACHE                              (0)

/* The Flash Memory not implemented yet */
#define FNET_CFG_CPU_FLASH                              (0)

/*/ Smallest logical block which can be erased independently.*/
#define FNET_CFG_CPU_FLASH_PAGE_SIZE                    (1024)         /* 1KB sector.*/

#define FNET_CFG_CPU_FLASH_PROGRAM_SIZE                 (4)

/* On-chip Flash size.*/
#define FNET_CFG_CPU_FLASH_SIZE                         (1024 * 256)    /* 256 KB */

/* The platform does not have second Ethernet Module.*/
#define FNET_CFG_CPU_ETH1        			(0)

/* No MIB.*/
#undef FNET_CFG_MCF_ETH_MIB
#define FNET_CFG_MCF_ETH_MIB                            (0)

/* To improve the TX performance.*/
#ifndef FNET_CFG_CPU_ETH_HW_TX_IP_CHECKSUM
    #define FNET_CFG_CPU_ETH_HW_TX_IP_CHECKSUM          (0)
#endif

/* To improve the TX performance.*/
#ifndef FNET_CFG_CPU_ETH_HW_TX_PROTOCOL_CHECKSUM
    #define FNET_CFG_CPU_ETH_HW_TX_PROTOCOL_CHECKSUM    (0)
#endif

/* To improve the RX performance.*/
#ifndef FNET_CFG_CPU_ETH_HW_RX_IP_CHECKSUM
    #define FNET_CFG_CPU_ETH_HW_RX_IP_CHECKSUM          (0)
#endif

/* To improve the RX performance.*/
#ifndef FNET_CFG_CPU_ETH_HW_RX_PROTOCOL_CHECKSUM
    #define FNET_CFG_CPU_ETH_HW_RX_PROTOCOL_CHECKSUM    (0)
#endif

/* Discard of frames with MAC layer errors.*/
#ifndef FNET_CFG_CPU_ETH_HW_RX_MAC_ERR
    #define FNET_CFG_CPU_ETH_HW_RX_MAC_ERR              (1)
#endif


#endif /* _FNET_LM3S8962_CONFIG_H_ */