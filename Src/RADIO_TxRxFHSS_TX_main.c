
/******************** (C) COPYRIGHT 2022 STMicroelectronics ********************
* File Name          : RADIO_TxRxFHSS_TX_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-March-2019
* Description        : Code demonstrating a simple TX/RX scenario (transmitter side)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RADIO_TxRxFHSS/RADIO_TxRxFHSS_TX_main.c
 * @brief Code demonstrating a simple TX/RX scenario. This code implements the transmitter side.
 * Two devices are necessary to run fully this demo. The 2Mbps configuration is also supported.
 *


* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu.
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_TxRxFHSS\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_TxRxFHSS.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\RADIO_TxRxFHSS\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\RADIO_TxRxFHSS.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\RADIO\\Tx\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c TX - TX configuration
- \c TX 2M - TX configuration with 2Mbps
- \c TX_Use_OTA_ResetManager - TX configuration with OTA


* \section Board_supported Boards supported
- \c STEVAL-IDB010V1
- \c STEVAL-IDB011V1
- \c STEVAL-IDB011V2
- \c STEVAL-IDB012V1
- \c STEVAL-IDB013V1


* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB01xV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name | Description                                                                |
| JP2         |                                                                            |
----------------------------------------------------------------------------------------------------------
| USB         | USB supply power                                                            |
| BAT         | The supply voltage must be provided through battery pins.                   |


@endtable

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB01xV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | It provides the voltage to the BlueNRG-LP circuit. It must be fitted. It can be used for current measurements of the BlueNRG-LP device.                                |          
| JP2         | It is a switch between two power domains. BAT position: to provide power from battery holder; USB position: to provide power from USB connector.                       |
| JP3         | It connects the BLE_SWCLK pin of the BlueNRG-LP with the SWCLK pin of the USB_CMSISDAP. It must be fitted.                                                             |          
| JP4         | It connects the BLE_SWDIO pin of the BlueNRG-LP with the SWDIO pin of the USB_CMSISDAP. It must be fitted.                                                             |
| JP5         | It connects the BLE_RSTN pin of the BlueNRG-LP with the rest of the board (the USB_CMSISDAP and RESET push button). It must be fitted.                                 |


@endtable 

* \section Pin_settings Pin settings
@table
|            |                                                   TX                                                    |||||                                                  TX 2M                                                  |||||                                                             TX_Use_OTA_ResetManager                                                             |||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB010V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB013V1  |   STEVAL-IDB010V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB013V1  |       STEVAL-IDB010V1      |       STEVAL-IDB011V1      |       STEVAL-IDB011V2      |       STEVAL-IDB012V1      |       STEVAL-IDB013V1      |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     A0     |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |          Not Used          |            N.A.            |            N.A.            |          Not Used          |          Not Used          |
|     A1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     A10    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |            N.A.            |            N.A.            |            N.A.            |          Not Used          |          Not Used          |
|     A11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        N.A.        |          Not Used          |          Not Used          |          Not Used          |          Not Used          |            N.A.            |
|     A12    |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     A13    |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     A14    |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     A15    |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     A3     |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |            N.A.            |            N.A.            |            N.A.            |          Not Used          |            N.A.            |
|     A4     |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     A5     |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     A6     |        N.A.        |      Not Used      |      Not Used      |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |        N.A.        |        N.A.        |            N.A.            |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     A7     |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     A8     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        N.A.        |          Not Used          |          Not Used          |          Not Used          |          Not Used          |            N.A.            |
|     A9     |      Not Used      |      Not Used      |      Not Used      |        N.A.        |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        N.A.        |      Not Used      |          Not Used          |          Not Used          |          Not Used          |            N.A.            |          Not Used          |
|     B0     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     B1     |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |          Not Used          |            N.A.            |            N.A.            |          Not Used          |          Not Used          |
|     B12    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |            N.A.            |            N.A.            |            N.A.            |          Not Used          |          Not Used          |
|     B13    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |            N.A.            |            N.A.            |            N.A.            |          Not Used          |          Not Used          |
|     B14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     B15    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |            N.A.            |            N.A.            |            N.A.            |          Not Used          |          Not Used          |
|     B2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     B3     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     B4     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     B5     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     B6     |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |        N.A.        |        N.A.        |        N.A.        |      Not Used      |      Not Used      |            N.A.            |            N.A.            |            N.A.            |          Not Used          |          Not Used          |
|     B7     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     B8     |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     B9     |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |      Not Used      |      Not Used      |      Not Used      |        N.A.        |        N.A.        |          Not Used          |          Not Used          |          Not Used          |            N.A.            |            N.A.            |
|     GND    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     RST    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|    VBAT    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |

@endtable 

* \section Serial_IO Serial I/O
  The application will listen for keys typed in one node and, on return press, it will send them to the remote node.
  The remote node will listen for RF messages and it will output them in the serial port.
  In other words everything typed in one node will be visible to the other node and vice versa.
@table
| Parameter name  | Value               | Unit      |
------------------------------------------------------
| Baudrate        | 115200 [default]    | bit/sec   |
| Data bits       | 8                   | bit       |
| Parity          | None                | bit       |
| Stop bits       | 1                   | bit       |

* \section LEDs_description LEDs description
@table
|            |                                                   TX                                                    |||||                                                  TX 2M                                                  |||||                                                             TX_Use_OTA_ResetManager                                                             |||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB010V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB013V1  |   STEVAL-IDB010V1  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |   STEVAL-IDB013V1  |       STEVAL-IDB010V1      |       STEVAL-IDB011V1      |       STEVAL-IDB011V2      |       STEVAL-IDB012V1      |       STEVAL-IDB013V1      |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |
|     U5     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |          Not Used          |          Not Used          |          Not Used          |          Not Used          |          Not Used          |

@endtable

* \section Buttons_description Buttons description
@table
|                |                                                                                                TX                                                                                                 |||||                                                                                               TX 2M                                                                                               |||||                                                                                      TX_Use_OTA_ResetManager                                                                                      |||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |            STEVAL-IDB010V1           |            STEVAL-IDB011V1           |            STEVAL-IDB011V2           |            STEVAL-IDB012V1           |            STEVAL-IDB013V1           |            STEVAL-IDB010V1           |            STEVAL-IDB011V1           |            STEVAL-IDB011V2           |            STEVAL-IDB012V1           |            STEVAL-IDB013V1           |            STEVAL-IDB010V1           |            STEVAL-IDB011V1           |            STEVAL-IDB011V2           |            STEVAL-IDB012V1           |            STEVAL-IDB013V1           |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |  TX device: move to next RF channel  |
|      PUSH2     |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |               Not Used               |
|      RESET     |           Reset BlueNRG-LP           |           Reset BlueNRG-LP           |           Reset BlueNRG-LP           |           Reset BlueNRG-LPS          |           Reset BlueNRG-LPS          |           Reset BlueNRG-LP           |           Reset BlueNRG-LP           |           Reset BlueNRG-LP           |           Reset BlueNRG-LPS          |           Reset BlueNRG-LPS          |           Reset BlueNRG-LP           |           Reset BlueNRG-LP           |           Reset BlueNRG-LP           |           Reset BlueNRG-LPS          |           Reset BlueNRG-LPS          |

@endtable

* \section Usage Usage
Code demonstrating a simple TX/RX scenario. This code implements the transmitter side.
Program two devices: one with the TX configuration and the other with the RX configuration. On the TX configuration, open a serial terminal to get the result of the communication.
The transmitter sends a number of packets equals to MAX_NUM_PACKET.The communication requires ACK to confirm the packet reception.
In this example the hal_radio layer is used in order to schedule the transmissions. The callback TXCallback provided to the API HAL_RADIO_SendPacketWithAck define the behavior of the radio according to the IRQ flags.
If the IRQ_TIMEOUT occurs, the counter timeout_error_counter is incremented.
If the IRQ_CRC_ERR occurs, the counter crc_error_counter is incremented.
In the last byte of the packet is sent a counter that is increased with the number of packets sent.
Once the maximum number of packets to send is reached, the summary of the communication is printed out with the information on the timeout errors, CRC errors, and the Packet Error Rate (PER). Then after a small delay, the sequence is repeated.

This example also provides a configuration supporting the 2 Mbps feature.

**/
   

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_hal_radio_2g4.h"
#include "main_common.h"

#include "bluenrg_lp_evb_config.h"

#if ST_USE_OTA_RESET_MANAGER
#include "radio_ota.h"
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_NUM_PACKET 100

/* Private macro -------------------------------------------------------------*/
#define HI_WORD(x)  ((uint8_t)((x & 0xFF00) >> 8))
#define LO_WORD(x)  ((uint8_t)(x))

/* Private variables ---------------------------------------------------------*/  
uint16_t crc_error_counter = 0;
uint16_t packet_counter = 0;
uint16_t timeout_error_counter = 0;

uint8_t button_flag = 0;
uint8_t sendNewPacket = TRUE;

uint8_t sendData[33];
uint8_t receivedData[MAX_PACKET_LENGTH];

/* Private function prototypes -----------------------------------------------*/
uint8_t TxCallback(ActionPacket* p, ActionPacket* next);
/* Private functions ---------------------------------------------------------*/

/**
* @brief  This routine is called when a transmit event is complete. 
* @param  p: Current action packet which its transaction has been completed.
* @param  next: Next action packet which is going to be scheduled.
* @retval return value: TRUE
*/
uint8_t TxCallback(ActionPacket* p, ActionPacket* next)
{ 
  if((p->status & BLUE_STATUSREG_PREVTRANSMIT) == 0){
    
    if((p->status & BLUE_INTERRUPT1REG_RCVOK) != 0) {
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVTIMEOUT) != 0) {
      BSP_LED_Toggle(BSP_LED2);
      if(timeout_error_counter < 0xFFFF)
      {
        timeout_error_counter++;
        sendData[2] = HI_WORD(timeout_error_counter);
        sendData[3] = LO_WORD(timeout_error_counter);
      }
      //printf("Error) Timeout error!!\r\n");
    }
    else if((p->status & BLUE_INTERRUPT1REG_RCVCRCERR) != 0) {
      BSP_LED_Toggle(BSP_LED3);
      if(crc_error_counter < 0xFFFF)
      {
        crc_error_counter++;
        sendData[4] = HI_WORD(crc_error_counter);
        sendData[5] = LO_WORD(crc_error_counter);
      }
      //printf("Error) CRC error!!\r\n");
    }

    BSP_LED_Toggle(BSP_LED1);
    sendNewPacket = TRUE;
  }
  return TRUE;
}

int main(void)
{
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
#if ST_USE_OTA_RESET_MANAGER
  BSP_PB_Init(BSP_PUSH2, BUTTON_MODE_GPIO);
  if (BSP_PB_GetState(BSP_PUSH2) == SET)
  {
    while(BSP_PB_GetState(BSP_PUSH2) == SET);
  }
#endif 
  
  /* Enable UART: 115200-8bit-No Parity-1 Stop bit */
  BSP_COM_Init(NULL);
  
  uint8_t ret;

  /* Configure thr first channel */
  fhss_channel_ptr = 0;
  fhss_channel = fhss_channel_schema[fhss_channel_ptr];

  /* Build packet */
  sendData[0] = 0x02;
  sendData[1] = 5;   /* Length position is fixed */
  sendData[2] = HI_WORD(timeout_error_counter);
  sendData[3] = LO_WORD(timeout_error_counter);
  sendData[4] = HI_WORD(crc_error_counter);
  sendData[5] = LO_WORD(crc_error_counter);
  sendData[6] = fhss_channel;
  
  RADIO_Init();
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  /* Set the Network ID */
  HAL_RADIO_SetNetworkID(ACCESS_ADDRESS);
  
  /* Configures the transmit power level */
  RADIO_SetTxPower(MAXIMIM_OUTPUT_RF_POWER);
  
#ifdef TXRX_PHY_2M
  RADIO_SetPhy(STATE_MACHINE_0,PHY_2M);
#endif

#if 1
  printf("\r\n\r\n\r\n");
  printf("******************************************************\r\n");
  printf("RADIO TX Demo with FHSS\r\n");
  printf("Freq sequence(%02d):\r\n",MAXIMUM_FHSS_CHANNEL_NUM);
  for(uint8_t i=0; i<MAXIMUM_FHSS_CHANNEL_NUM;i++) {
    printf(" %02d", fhss_channel_schema[i]);
    if(MAXIMUM_FHSS_CHANNEL_NUM > 10) if((i+1)%10 == 0) printf("\r\n");
  }
  printf("\r\n");
  printf("******************************************************\r\n");
#endif

  while(1) {
    HAL_VTIMER_Tick();
    if(sendNewPacket){
      sendNewPacket = FALSE;

      printf("\r\nSend TX (idx %02d, ch %02d) :",fhss_channel_ptr, fhss_channel);
      for(volatile uint16_t i = 0; i < (sendData[1] + 2); i++) {
        printf("%02X ", sendData[i]);
      }
      printf("\r\n");

      ret =  HAL_RADIO_SendPacketWithAck(fhss_channel,TX_WAKEUP_TIME, sendData, receivedData, RX_TIMEOUT_ACK, MAXIMUM_LL_PACKET_LENGTH, TxCallback);

      if(ret != SUCCESS_0) {
        printf("ERROR %d (%d)\r\n",ret, packet_counter);
      }
      else    //Succeed TX
      { 
        //Check ACK
        if((receivedData[0] == 0xAE)  // 0xAE is ack value
          && ((receivedData[1] == 0x00))   //Length is 0
        ){
          printf("Succeed TX (Channel %02d) --> OK Ack (0x%02x, 0x%02x)\r\n", fhss_channel, receivedData[0], receivedData[1]);
          fhss_channel_ptr++;
          if(fhss_channel_ptr == MAXIMUM_FHSS_CHANNEL_NUM)  fhss_channel_ptr = 0;
          fhss_channel = fhss_channel_schema[fhss_channel_ptr];
          sendData[6] = fhss_channel;
        }
        else{
          printf("Succeed TX (Channel %02d) --> NO Ack (0x%02x, 0x%02x)\r\n", fhss_channel, receivedData[0], receivedData[1]);
        }
      }
      /*Tx delay*/
      for(volatile uint32_t i = 0; i<0x2FFFFF; i++);
    }
    
    if(button_flag == 1) {
      button_flag = 0;
      fhss_channel_ptr++;
      if(fhss_channel_ptr == MAXIMUM_FHSS_CHANNEL_NUM)  fhss_channel_ptr = 0;
      fhss_channel = fhss_channel_schema[fhss_channel_ptr];
    }

#if ST_USE_OTA_RESET_MANAGER
    if (BSP_PB_GetState(BSP_PUSH2) == SET)
    {
      while(sendNewPacket == FALSE);
      OTA_Jump_To_Reset_Manager();
    }
#endif    
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}

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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
