#ifndef _FNET_LM3S_CONFIG_H_

#define _FNET_LM3S_CONFIG_H_

#include "fnet_config.h"

#ifndef FNET_LM3S
    #define FNET_LM3S  (0)
#endif

#if FNET_LM3S

    /**************************************************************************
    *  Default serial port number.
    ******************************************************************************/
    #ifndef FNET_CFG_CPU_SERIAL_PORT_DEFAULT
        #define FNET_CFG_CPU_SERIAL_PORT_DEFAULT           (0U) /* EKI-LM3S8962 board uses the default port number 0.*/
    #endif

    /**************************************************************************
    *  Maximum Timer number that is avaiable on the used platform.
    ******************************************************************************/
    #define  FNET_CFG_CPU_TIMER_NUMBER_MAX                 (3U)

    /*******************************************************************************
     * Timer number used by the FNET. It can range from 0 to FNET_CFG_CPU_TIMER_NUMBER_MAX.
     ******************************************************************************/
    #ifndef FNET_CFG_CPU_TIMER_NUMBER
        #define FNET_CFG_CPU_TIMER_NUMBER                  (0)
    #endif

    /**************************************************************************/ /*!
    *   Vector table address.@n
    *           By default, NVIC vector table register (VTOR).
    * @showinitializer
    ******************************************************************************/
    #ifndef FNET_CFG_CPU_VECTOR_TABLE
        #define FNET_CFG_CPU_VECTOR_TABLE                  FNET_LM3S_SCB_VTOR
    #endif

    /******************************************************************************
    *  Vector number of the timer interrupt.
    *  NOTE: User application should not change this parameter.
    ******************************************************************************/
    #ifndef FNET_CFG_CPU_TIMER_VECTOR_NUMBER
        #if (FNET_CFG_CPU_TIMER_NUMBER != FNET_CFG_CPU_TIMER_MAX)
            #define FNET_CFG_CPU_TIMER_VECTOR_NUMBER       (35U + FNET_CFG_CPU_TIMER_NUMBER*2)
        #else
            #define FNET_CFG_CPU_TIMER_VECTOR_NUMBER       (51U)
        #endif
    #endif

    /******************************************************************************
    *  Vector number of the Ethernet Receive Frame vector number.
    *  NOTE: User application should not change this parameter.
    ******************************************************************************/
    #ifndef FNET_CFG_CPU_ETH0_VECTOR_NUMBER
        #define FNET_CFG_CPU_ETH0_VECTOR_NUMBER            (58U)
    #endif

    /*****************************************************************************
    *  Byte order is little endian.
    ******************************************************************************/
    #define FNET_CFG_CPU_LITTLE_ENDIAN                     (1)

    /*****************************************************************************
    *  Max priority.
    ******************************************************************************/
    #define FNET_CFG_CPU_VECTOR_PRIORITY_MAX               (7U) /* 3 bits */

    /*****************************************************************************
    *  On-chip Flash memory start address.
    ******************************************************************************/
    #ifndef FNET_CFG_CPU_FLASH_ADDRESS
        #define FNET_CFG_CPU_FLASH_ADDRESS                  (0x0U)
    #endif

    #ifndef FNET_CFG_CPU_FLASH_PROGRAM_SIZE
        #define FNET_CFG_CPU_FLASH_PROGRAM_SIZE             (4U)
    #endif


#endif /* FNET_LM3S */

#endif /* _FNET_LM3S_CONFIG_H_ */
