#include "stm32f4xx_rcc_driver.h"
#include "stm32f40x.h"

uint16_t AHB_Prescalar[8] = {2,4,8,16,64,128,256,512};  //It holds the division factors of AHB Perscalar
uint16_t APB1_Prescalar[4] = {2,4,8,16};




uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clkscr, temp,ahbp,apb1p;

	clkscr = ((RCC->CFGR >> 2) & 0x3);  //System clock
	(void)clkscr;
	 if(clkscr == 0)
	 {
		 SystemClk = 16000000;  //HSI clk 16MHZ
	 }else if(clkscr == 1)
	 {
		 SystemClk = 8000000;  //HSE clk 8MHZ
	 }else if(clkscr == 2)
	 {
		 SystemClk = RCC_GetPLLOutputClk();
	 }

	 //for ahbp

	 temp = ((RCC->CFGR >> 4) & 0xF);

	 if(temp < 8)
	 {
		 ahbp = 1;
	 }else
	 {
		 ahbp = AHB_Prescalar[temp - 8];
	 }


	 //for apb1 prescalar

	 temp = ((RCC->CFGR >> 10) & 0x7);  //everything will be masked except the 3 bits thats why it is masked with 0x7

	 if(temp < 8)
	 {
	 	apb1p = 1;
	 }else
	 {
		 apb1p = APB1_Prescalar[temp - 4];
	 }

	 pclk1 = (SystemClk / ahbp) /apb1p;


	 return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk;
	uint8_t clkscr, temp,ahbp,apb2p;

	clkscr = ((RCC->CFGR >> 2) & 0x3);  //System clock
	(void)clkscr;
	 if(clkscr == 0)
	 {
		 SystemClk = 16000000;  //HSI clk 16MHZ
	 }else if(clkscr == 1)
	 {
		 SystemClk = 8000000;  //HSE clk 8MHZ
	 }else if(clkscr == 2)
	 {
		 SystemClk = RCC_GetPLLOutputClk();
	 }

	 //for ahbp

	 temp = ((RCC->CFGR >> 4) & 0xF);

	 if(temp < 8)
	 {
		 ahbp = 1;
	 }else
	 {
		 ahbp = AHB_Prescalar[temp - 8];
	 }


	 //for apb1 prescalar

	 temp = ((RCC->CFGR >> 10) & 0x7);  //everything will be masked except the 3 bits thats why it is masked with 0x7

	 if(temp < 8)
	 {
		apb2p = 1;
	 }else
	 {
		 apb2p = APB1_Prescalar[temp - 4];
	 }

	 pclk2 = (SystemClk / ahbp) /apb2p;


	 return pclk2;


}

uint32_t RCC_GetPLLOutputClk(void)
{
	return 0;
}

