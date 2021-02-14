/*
 * stm32f4xx_spi_Drivers.h
 *
 *  Created on: 04-Jan-2021
 *      Author: Shubham
 */

#ifndef INC_STM32F4XX_SPI_DRIVERS_H_
#define INC_STM32F4XX_SPI_DRIVERS_H_

/*****configuring the SPIx peripherals
 *
 */
typedef struct
{
	uint8_t SPI_Devicemode;
	uint8_t SPI_Busconfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_SclkSpeed;
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t *pSPIx; //This holds the base address of SPIx(x:0,1,2) peripheral
	SPI_Config_t SPIConfig;
	uint8_t      *pTXBuffer; //To store the app. tx buffer address
	uint8_t      *pRXBuffer; //To store the app. rx buffer address
	uint32_t       TXLen; //To store the app. tx Len address
	uint32_t       RXLen; //To store the app. tx Len address
	uint8_t       TXState; //To store the app. tx state address
	uint8_t       RXState; //To store the app. rx state address
}SPI_Handle_t;

/*
 * SPI Application states
 */
#define SPI_READY         0
#define SPI_BUSY_IN_RX    1
#define SPI_BUSY_IN_TX    2

/*
 * SPI Appication possible events
 */
#define SPI_EVENT_TX_CMPLT    1
#define SPI_EVENT_RX_CMPLT    2
#define SPI_EVENT_OVR_ERR     3
#define SPI_EVENT_CRC_ERR     4
/*
 * SPI Device mode
 */
#define SPI_DEVICE_MODE_MASTER   1
#define SPI_DEVICE_MODE_SLAVE    0

/*
 * SPI Bus Config
 */

#define SPI_BUS_CONFIG_FD              1         //Full duplex
#define SPI_BUS_CONFIG_HD              2         //Half duplex
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY  3
/*
 * SPI Clk_Speed
 */

#define SPI_SCLK_SPEED_DIV2            0
#define SPI_SCLK_SPEED_DIV4            1
#define SPI_SCLK_SPEED_DIV8            2
#define SPI_SCLK_SPEED_DIV16           3
#define SPI_SCLK_SPEED_DIV32           4
#define SPI_SCLK_SPEED_DIV64           5
#define SPI_SCLK_SPEED_DIV128          6
#define SPI_SCLK_SPEED_DIV256          7

/*
 * SPI DFF
 */
#define SPI_DFF_8BITS                  0
#define SPI_DFF_16BITS                 1

/*
 * SPI COPL
 */
#define SPI_COPL_HIGH                  1
#define SPI_COPL_LOW                   0

/*
 * SPI CPHA
 */
#define SPI_CPHA_HIGH                  1
#define SPI_CPHA_LOW                   0

/*
 * SPI SSM (Slave select managment)
 */
#define SPI_SSM_EN                    1
#define SPI_SSM_DI                    0

/*
 * SPI status register flag definitions
 */
#define SPI_TXE_FLAG      (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG     (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG     (1 << SPI_SR_BSY)

//API's supported by this drivers

//Periphela clock
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//Init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and recive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t Len);
void SPI_RecevingData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_RecevingDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * other peripheral control API
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application Call back
 */
void SPI_ApplicationEventcallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F4XX_SPI_DRIVERS_H_ */
