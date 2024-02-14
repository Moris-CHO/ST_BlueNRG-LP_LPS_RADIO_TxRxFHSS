/**
  ******************************************************************************
  * @file    main_common.h 
  * @author  RF Application Team
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2020 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_COMMON_H
#define MAIN_COMMON_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define ------------------------------------------------------------*/
#define CALIBRATION_INTERVAL_CONF   10000

#if defined CONFIG_HW_LS_RO  

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

/* Calibration must be done */
#define INITIAL_CALIBRATION TRUE
#define CALIBRATION_INTERVAL        CALIBRATION_INTERVAL_CONF

#elif defined CONFIG_HW_LS_XTAL

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        100

/* No Calibration */
#define INITIAL_CALIBRATION FALSE
#define CALIBRATION_INTERVAL        0

#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
// Channels hopping schema (Each number MUST BE unique in the sequence)
// Supported channel 0 ~ 39 (total 40 channels)
uint8_t fhss_channel_schema[] = {
                                  11,  0, 32,  2, 19,  3, 33, 30,  4, 14, 
                                   9, 13,  6,  1, 34, 39, 24, 37, 12,  8,
                                  21, 27, 23,  7, 20, 29, 16, 17,  5, 26,
                                  28, 15, 25, 35, 36, 10, 31, 22, 38, 18
                                };// You can do it as long as you want.
uint8_t fhss_channel_ptr;
uint8_t fhss_channel;
/* Exported macro ------------------------------------------------------------*/
/* Exported define ------------------------------------------------------------*/     
#define ACCESS_ADDRESS              (uint32_t)(0x12345678)    //(uint32_t)(0x8E89BED6)
#define HS_STARTUP_TIME             (uint16_t)(1)   /* High Speed start up time min value */
#define TX_WAKEUP_TIME              400      /* 400 us */
#define RX_WAKEUP_TIME              300      /* 300 us */
#define RX_TIMEOUT              1000000      /* 1000 ms */
#define RX_TIMEOUT_ACK           500000      /* 500 ms */

#define MAXIMUM_LL_PACKET_LENGTH    255 /* Maximum Link Layer Packet Length (user_payload + MIC)*/
#define MAXIMIM_OUTPUT_RF_POWER     0x1F
#define MAXIMUM_FHSS_CHANNEL_NUM    (uint8_t)(sizeof(fhss_channel_schema)/sizeof(uint8_t))
/* Exported functions ------------------------------------------------------- */

#endif /* MAIN_COMMON_H */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
