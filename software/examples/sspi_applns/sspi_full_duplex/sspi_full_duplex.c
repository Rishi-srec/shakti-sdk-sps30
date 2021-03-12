/***************************************************************************
 * Project               	    		:  shakti devt board
 * Name of the file	            		:  sspi_full_duplex.c
 * Brief Description of file             :  sspi full duplex example code.
 * Name of Author    	                :  Kotteeswaran
 * Email ID                              :  kottee.1@gmail.com

 Copyright (C) 2019  IIT Madras. All rights reserved.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.

 ***************************************************************************/

#include "platform.h"
#include "plic_driver.h"
#include "log.h"

#define SSPI_SIZE 8
#define DATA_SIZE 8

#define SSPI_TEST 1
#ifdef SSPI_TEST
//SSPI
#define SSPI0_COMMCTRL	0x00010300
#define SSPI0_CLKCTRL		0x00010304
#define SSPI0_TXREG		0x00010308
#define SSPI0_RXREG		0x0001030C
#define SSPI0_INTR_EN		0x00010310
#define SSPI0_FIFOSTATUS		0x00010314
#define SSPI0_COMMSTATUS	0x00010318
#define SSPI0_INQUAL		0x0001031C

#define COMMCTRL_MOSI_OUTEN		(1<<25)
#define COMMCTRL_MISO_OUTEN		(1<<24)
#define COMMCTRL_SCLK_OUTEN		(1<<23)
#define COMMCTRL_CS_OUTEN		(1<<22)
#define COMMCTRL_RX_BITS(x)		(x<<14)
#define COMMCTRL_TX_BITS(x)		(x<<6)
#define COMMCTRL_COMM_MODE(x)	(x<<4)
#define COMMCTRL_LSB_FIRST		(1<<2)
#define COMMCTRL_SPI_EN			(1<<1)
#define COMMCTRL_MASTER_MODE	(1<<0)

#define CLKCTRL_T_CS(x)			(x<<18)
#define CLKCTRL_CS_T(x)			(x<<10)
#define CLKCTRL_PRESCALE(x)		(x<<2)
#define CLKCTRL_PHASE			(1<<1)
#define CLKCTRL_POLARITY		(1<<0)


#define SSPI1_COMMCTRL	0x00010700
#define SSPI1_CLKCTRL		0x00010704
#define SSPI1_TXREG		0x00010708
#define SSPI1_RXREG		0x0001070C
#define SSPI1_INTR_EN		0x00010710
#define SSPI1_FIFOSTATUS		0x00010714
#define SSPI1_COMMSTATUS	0x00010718
#define SSPI1_INQUAL		0x0001071C

uint32_t* spi0_commctrl		=	(const uint32_t*) SSPI0_COMMCTRL;
uint32_t* spi0_clkctrl 		=	(const uint32_t*) SSPI0_CLKCTRL;
uint8_t* spi0_tx8			=	(const uint8_t*) SSPI0_TXREG;
uint16_t* spi0_tx16			=	(const uint16_t*) SSPI0_TXREG;
uint32_t* spi0_tx32			=	(const uint32_t*) SSPI0_TXREG;
uint8_t* spi0_rx8			=	(const uint8_t*) SSPI0_RXREG;
uint16_t* spi0_rx16			=	(const uint16_t*) SSPI0_RXREG;
uint32_t* spi0_rx32			=	(const uint32_t*) SSPI0_RXREG;
uint8_t* spi0_fifostatus		=	(const uint8_t*) SSPI0_FIFOSTATUS;
uint8_t* spi0_commstatus	=	(const uint8_t*) SSPI0_COMMSTATUS;
uint32_t* spi0_inqual		=	(const uint32_t*) SSPI0_INQUAL;


uint32_t* spi1_commctrl		=	(const uint32_t*) SSPI1_COMMCTRL;
uint32_t* spi1_clkctrl 		=	(const uint32_t*) SSPI1_CLKCTRL;
uint8_t* spi1_tx8			=	(const uint8_t*) SSPI1_TXREG;
uint16_t* spi1_tx16			=	(const uint16_t*) SSPI1_TXREG;
uint32_t* spi1_tx32			=	(const uint32_t*) SSPI1_TXREG;
uint8_t* spi1_rx8			=	(const uint8_t*) SSPI1_RXREG;
uint16_t* spi1_rx16			=	(const uint16_t*) SSPI1_RXREG;
uint32_t* spi_rx32			=	(const uint32_t*) SSPI1_RXREG;
uint8_t* spi1_fifostatus		=	(const uint8_t*) SSPI1_FIFOSTATUS;
uint8_t* spi1_commstatus	=	(const uint8_t*) SSPI1_COMMSTATUS;
uint32_t* spi1_inqual		=	(const uint32_t*) SSPI1_INQUAL;
#endif

#ifdef CSPI_TEST
#define CSPI0_COMMCTRL		0x00010400
#define CSPI0_CLKCTRL			0x00010404
#define CSPI0_DATACTRLREG		0x00010408
#define CSPI0_DATAREG1		0x0001040C
#define CSPI0_DATAREG2		0x00010410
#define CSPI0_DATAREG3		0x00010414
#define CSPI0_DATAREG4		0x00010418
#define CSPI0_INTR_EN			0x0001041C
#define CSPI0_FIFO_INTR		0x00010420
#define CSPI0_COMMSTATUS		0x00010424
#define CSPI0_INQUAL			0x00010428

#define CSPI1_COMMCTRL		0x00010400
#define CSPI1_CLKCTRL			0x00010404
#define CSPI1_DATACTRLREG		0x00010408
#define CSPI1_DATAREG1		0x0001040C
#define CSPI1_DATAREG2		0x00010410
#define CSPI1_DATAREG3		0x00010414
#define CSPI1_DATAREG4		0x00010418
#define CSPI1_INTR_EN			0x0001041C
#define CSPI1_FIFO_INTR		0x00010420
#define CSPI1_COMMSTATUS		0x00010424
#define CSPI1_INQUAL			0x00010428


// end CSPI 1
#define COMMCTRL_DATA3_EN		(1<<27)
#define COMMCTRL_DATA2_EN		(1<<26)
#define COMMCTRL_DATA1_EN		(1<<25)
#define COMMCTRL_DATA0_EN		(1<<24)
#define COMMCTRL_CTRL_EN		(1<<23)
#define COMMCTRL_CTRL_OUTEN		(1<<22)
#define COMMCTRL_DATA_OUTEN		(1<<21)
#define COMMCTRL_RX_BITS(x)		(x<<13)
#define COMMCTRL_TX_BITS(x)		(x<<5)
#define COMMCTRL_RX_IMM			(1<<4)
//#define COMMCTRL_COMM_MODE		(1<<3)
#define COMMCTRL_LSB_FIRST		(1<<2)
#define COMMCTRL_SPI_EN			(1<<1)
//#define COMMCTRL_MASTER_MODE	(1<<0)

#define CLKCTRL_T_CS(x)			(x<<18)
#define CLKCTRL_CS_T(x)			(x<<10)
#define CLKCTRL_PRESCALE(x)		(x<<2)
#define CLKCTRL_PHASE			(1<<1)
#define CLKCTRL_POLARITY		(1<<0)

#define INTR_EN_DC3				(1<<9)
#define INTR_EN_DC2				(1<<8)
#define INTR_EN_DC1				(1<<7)
#define INTR_EN_DC0				(1<<6)
#define INTR_EN_CC 				(1<<5)
#define INTR_EN_FIFO_OVERRUN	(1<<4)
#define INTR_EN_FIFO_FULL		(1<<3)
#define INTR_EN_FIFO_HALF		(1<<2)
#define INTR_EN_FIFO_QUAD		(1<<1)
#define INTR_EN_FIFO_EMPTY		(1<<0)

/*
uint32_t* commctrl			=	(const uint32_t*) COMMCTRL;
uint32_t* clkctrl 			=	(const uint32_t*) CLKCTRL;
uint32_t* ctrlreg			=	(const uint32_t*) DATACTRLREG;
uint32_t* datareg1			=	(const uint32_t*) DATAREG1;
uint32_t* datareg2			=	(const uint32_t*) DATAREG2;
uint32_t* datareg3			=	(const uint32_t*) DATAREG3;
uint32_t* datareg4			=	(const uint32_t*) DATAREG4;
uint32_t* intren			=	(const uint32_t*) INTR_EN;
uint32_t* fifointr			=	(const uint32_t*) FIFO_INTR;
uint32_t* commstatus		=	(const uint32_t*) COMMSTATUS;
uint32_t* inqual			=	(const uint32_t*) INQUAL;
*/
uint32_t* spi0_commctrl			=	(const uint32_t*) CSPI_COMMCTRL;
uint32_t* spi0_clkctrl 			=	(const uint32_t*) CSPI_CLKCTRL;
uint32_t* spi0_ctrlreg			=	(const uint32_t*) CSPI_DATACTRLREG;
volatile unsigned char  * spi0_ctrlreg_8			=	(const unsigned char *) CSPI0_DATACTRLREG;
volatile unsigned short * spi0_ctrlreg_16			=	(const unsigned short  *) CSPI0_DATACTRLREG;
volatile unsigned int   * spi0_ctrlreg_32			=	(const unsigned int *) CSPI0_DATACTRLREG;
volatile unsigned char  * spi0_datareg1_8			=	(const unsigned char *) CSPI0_DATAREG1;
volatile unsigned short * spi0_datareg1_16			=	(const unsigned short  *) CSPI0_DATAREG1;
volatile unsigned int   * spi0_datareg1_32			=	(const unsigned int *) CSPI0_DATAREG1;
volatile unsigned char  * spi0_datareg2_8			=	(const unsigned char *) CSPI0_DATAREG2;
volatile unsigned short * spi0_datareg2_16			=	(const unsigned short  *) CSPI0_DATAREG2;
volatile unsigned int   * spi0_datareg2_32			=	(const unsigned int *) CSPI0_DATAREG2;
volatile unsigned char  * spi0_datareg3_8			=	(const unsigned char *) CSPI0_DATAREG3;
volatile unsigned short * spi0_datareg3_16			=	(const unsigned short  *) CSPI0_DATAREG3;
volatile unsigned int   * spi0_datareg3_32			=	(const unsigned int *) CSPI0_DATAREG3;
volatile unsigned char  * spi0_datareg4_8			=	(const unsigned char *) CSPI0_DATAREG4;
volatile unsigned short * spi0_datareg4_16			=	(const unsigned short  *) CSPI0_DATAREG4;
volatile unsigned int   * spi0_datareg4_32			=	(const unsigned int *) CSPI0_DATAREG4;
uint16_t* spi0_intren			=	(const uint16_t*) CSPI0_INTR_EN;
uint32_t* spi0_fifointr			=	(const uint32_t*) CSPI0_FIFO_INTR;
uint32_t* spi0_commstatus		=	(const uint32_t*) CSPI0_COMMSTATUS;
uint8_t*  spi0_inqual			=	(const uint8_t*) CSPI0_INQUAL;

uint32_t* spi1_commctrl			=	(const uint32_t*) CSPI1_COMMCTRL;
uint32_t* spi1_clkctrl 			=	(const uint32_t*) CSPI1_CLKCTRL;
uint32_t* spi1_ctrlreg			=	(const uint32_t*) CSPI1_DATACTRLREG;
volatile unsigned char  * spi1_ctrlreg_8			=	(const unsigned char *) CSPI1_DATACTRLREG;
volatile unsigned short * spi1_ctrlreg_16			=	(const unsigned short  *) CSPI1_DATACTRLREG;
volatile unsigned int   * spi1_ctrlreg_32			=	(const unsigned int *) CSPI1_DATACTRLREG;
volatile unsigned char  * spi1_datareg1_8			=	(const unsigned char *) CSPI1_DATAREG1;
volatile unsigned short * spi1_datareg1_16			=	(const unsigned short  *) CSPI1_DATAREG1;
volatile unsigned int   * spi1_datareg1_32			=	(const unsigned int *) CSPI1_DATAREG1;
volatile unsigned char  * spi1_datareg2_8			=	(const unsigned char *) CSPI1_DATAREG2;
volatile unsigned short * spi1_datareg2_16			=	(const unsigned short  *) CSPI1_DATAREG2;
volatile unsigned int   * spi1_datareg2_32			=	(const unsigned int *) CSPI1_DATAREG2;
volatile unsigned char  * spi1_datareg3_8			=	(const unsigned char *) CSPI1_DATAREG3;
volatile unsigned short * spi1_datareg3_16			=	(const unsigned short  *) CSPI1_DATAREG3;
volatile unsigned int   * spi1_datareg3_32			=	(const unsigned int *) CSPI1_DATAREG3;
volatile unsigned char  * spi1_datareg4_8			=	(const unsigned char *) CSPI1_DATAREG4;
volatile unsigned short * spi1_datareg4_16			=	(const unsigned short  *) CSPI1_DATAREG4;
volatile unsigned int   * spi1_datareg4_32			=	(const unsigned int *) CSPI1_DATAREG4;
uint16_t* spi1_intren			=	(const uint16_t*) CSPI1_INTR_EN;
uint32_t* spi1_fifointr			=	(const uint32_t*) CSPI1_FIFO_INTR;
uint32_t* spi1_commstatus		=	(const uint32_t*) CSPI1_COMMSTATUS;
uint8_t*  spi1_inqual			=	(const uint8_t*) CSPI1_INQUAL;

#endif


uint8_t spi0_tx[SSPI_SIZE] = {0};
uint8_t spi0_rx[SSPI_SIZE] = {0};

uint8_t spi1_tx[SSPI_SIZE] = {0};
uint8_t spi1_rx[SSPI_SIZE] = {0};

__attribute__((always_inline))
static inline void set_shakti32(uint32_t* addr, uint32_t val)
{
	*addr = val;
}

__attribute__((always_inline))
static inline void set_shakti16(uint16_t* addr, uint16_t val)
{
	*addr = val;
}

__attribute__((always_inline))                                                                      
static inline void set_shakti8(uint8_t* addr, uint8_t val)                                   
{                                                                                                   
    *addr = val;                                                                                    
} 

__attribute__((always_inline))
static inline uint32_t get_shakti32(uint32_t* addr)
{
	return *addr;
}

__attribute__((always_inline))
static inline uint16_t get_shakti16(uint16_t* addr)
{
	return *addr;
}


__attribute__((always_inline))
static inline uint8_t get_shakti8(uint8_t* addr)
{
	return *addr;
}



#ifdef CSPI_TEST
int spi_notbusy(char number)
{
	uint32_t temp_d = 0x01041041;
	while(temp_d & 0x01041041){
		waitfor(10);
		if(number == 0)
			temp_d = get_shakti8(sspi0_commstatus);
		else if(number == 1)
			temp_d = get_shakti8(sspi1_commstatus);
		else 
		{
			printf("\n Incorrect SPI number");
			return -1;
		}
		printf("\n cspi status = %x",temp_d);
	}
	printf("\n CSPI comm done \n");
	return 1;

}
#endif

#ifdef SSPI_TEST
int spi_notbusy(char number)
{
	int temp = 0x00000001;
	while(temp & 0x00000001){
		waitfor(10);
		printf("\n SSPI %x Status is %x", number, temp);
		if(number == 0)
			temp = get_shakti8(spi0_commstatus);
		else if(number == 1)
			temp = get_shakti8(spi1_commstatus);
		else 
		{
			printf("\n Incorrect SPI number");
			return -1;
		}
	}
//	printf("\n SPI comm done \n");
	return 1;
}

#endif


/** @fn main() 
 * @brief main function that runs the code
 * @details runs the code
 * @warning No warning
 * @param[in] No input parameter 
 * @param[Out] No output parameter
 */
int main()
{
	gpt_struct* instance;
	uint32_t temp = 0, retval = 0;
	uint8_t i = 0, j = 0;
	uint8_t temp_data = 0;

	printf("\n SSPI0 - SSPI1 Full Duplex Check\n");

	printf("\n Configure SSPI");
	temp = spi_notbusy(0);
	printf("SSPI0 status: %x \n", temp);

	temp = spi_notbusy(1);
	printf("SSPI1 status: %x \n", temp);

	set_shakti32(spi0_clkctrl,(CLKCTRL_PRESCALE(60)|CLKCTRL_POLARITY | CLKCTRL_PHASE));
	set_shakti8(spi0_inqual, 0x00);
	temp = spi_notbusy(0);
	printf("SSPI0 status: %x \n", temp);

//	set_shakti32(sspi1_clkctrl,(CLKCTRL_PRESCALE(30) |CLKCTRL_POLARITY | CLKCTRL_PHASE));
	set_shakti8(spi1_inqual, 0x00);
	temp = spi_notbusy(1);
	printf("SSPI1 status: %x \n", temp);

	delay_loop(2000, 1000);
#if 0
	while(1)
	{
	
			temp_data = 0x01;
				for(i = 0; i < SSPI_SIZE; i++)
				{
					sspi0_rx[i] = 0;
					sspi0_tx[i] = temp_data;
					sspi1_rx[i] = 0;
					sspi1_tx[i] = temp_data + 0x10;
					temp_data += 0x01;
				}
			if(  ( (0x2 & get_shakti8(sspi0_commstatus) ) != 0) && ( (0x2 & get_shakti8(sspi1_commstatus) ) != 0) )
			{
				printf("\n Filling Fifo data");
				for(i = 0; i < SSPI_SIZE; i++)
				{
					*sspi0_tx8 = sspi0_tx[i];
					*sspi1_tx8 = sspi0_tx[i];
				}

			}
			else
			{
				printf("\n Fifo is not empty");
			}
			set_shakti32(sspi0_commctrl,(COMMCTRL_MOSI_OUTEN | COMMCTRL_SCLK_OUTEN | \
				COMMCTRL_CS_OUTEN | COMMCTRL_TX_BITS(SSPI_SIZE * DATA_SIZE) | \
				COMMCTRL_RX_BITS(SSPI_SIZE * DATA_SIZE) | COMMCTRL_COMM_MODE(3) | \
				COMMCTRL_SPI_EN | COMMCTRL_MASTER_MODE));
			set_shakti32(sspi1_commctrl,(COMMCTRL_MISO_OUTEN | 
				COMMCTRL_TX_BITS(SSPI_SIZE * DATA_SIZE) | \
				COMMCTRL_RX_BITS(SSPI_SIZE * DATA_SIZE) | COMMCTRL_COMM_MODE(3) | \
				COMMCTRL_SPI_EN));
			temp = sspi_notbusy(0);
			temp = sspi_notbusy(1);
			printf("\n SSPI 0 Commn status: %x", *sspi0_commstatus );
			printf("\n SSPI 0 Fifo status: %x", *sspi0_fifostatus );
			printf("\n SSPI 1 Commn status: %x", *sspi1_commstatus );
			printf("\n SSPI 1 Fifo status: %x", *sspi1_fifostatus );
			
			if(  ( (0x4 & get_shakti8(sspi0_commstatus) ) != 0) && ( (0x4 & get_shakti8(sspi1_commstatus) ) != 0) )
			{
			printf("\n SSPI 0 Commn status: %x", *sspi0_commstatus );
			printf("\n SSPI 0 Fifo status: %x", *sspi0_fifostatus );
			printf("\n SSPI 1 Commn status: %x", *sspi1_commstatus );
			printf("\n SSPI 1 Fifo status: %x", *sspi1_fifostatus );

#if 0
				for(int i = 0; i < SSPI_SIZE; i++)
				{
					printf("\n i: %x", i);
					sspi0_rx[i] =  *sspi0_rx8;
					sspi1_rx[i] =  *sspi1_rx8;
					
				}
#endif
				printf("\n SSPI READ VALUES");
				
				for(int i = 0; i < SSPI_SIZE; i++)
				{
					printf("\nSSPI0[%x]: %x; SSPI1[%x]: %x", i, sspi0_rx[i], i, sspi1_rx[i]);
	//				printf("\n ADC[%x]:%x", i, adc_read[i]);
				}
			}
			else
			{
				printf("\n Didn't receive any data ");

			}
//			printf("Read values from ADC \n");
				delay_loop(2000, 1000);
		
			}
#else
	while(1)
	{
	
			temp_data = 0x01;
			for(i = 0; i < SSPI_SIZE; i++)
			{
				spi0_rx[i] = 0;
				spi0_tx[i] = temp_data;
				spi1_rx[i] = 0;
				spi1_tx[i] = temp_data + 0x10;
				temp_data += 0x01;
			}
			printf("\n Filling Fifo data");
			for(i = 0; i < SSPI_SIZE; i++)
			{
				*spi0_tx8 = spi0_tx[i];
				*spi1_tx8 = spi1_tx[i];
			}

			set_shakti32(spi1_commctrl,(COMMCTRL_MISO_OUTEN | 
				COMMCTRL_TX_BITS(SSPI_SIZE * DATA_SIZE) | \
				COMMCTRL_RX_BITS(SSPI_SIZE * DATA_SIZE) | COMMCTRL_COMM_MODE(3) | \
				COMMCTRL_SPI_EN));
			set_shakti32(spi0_commctrl,(COMMCTRL_MOSI_OUTEN | COMMCTRL_SCLK_OUTEN | \
				COMMCTRL_CS_OUTEN | COMMCTRL_TX_BITS(SSPI_SIZE * DATA_SIZE) | \
				COMMCTRL_RX_BITS(SSPI_SIZE * DATA_SIZE) | COMMCTRL_COMM_MODE(3) | \
				COMMCTRL_SPI_EN | COMMCTRL_MASTER_MODE));
			temp = spi_notbusy(0);
			temp = spi_notbusy(1);
			printf("\n SSPI 0 Commn status: %x", *spi0_commstatus );
			printf("\n SSPI 0 Fifo status: %x", *spi0_fifostatus );
			printf("\n SSPI 1 Commn status: %x", *spi1_commstatus );
			printf("\n SSPI 1 Fifo status: %x", *spi1_fifostatus );
			
			if(  ( (0x4 & get_shakti8(spi0_commstatus) ) != 0) && ( (0x4 & get_shakti8(spi1_commstatus) ) != 0) )
			{
			printf("\n SSPI 0 Commn status: %x", *spi0_commstatus );
			printf("\n SSPI 0 Fifo status: %x", *spi0_fifostatus );
			printf("\n SSPI 1 Commn status: %x", *spi1_commstatus );
			printf("\n SSPI 1 Fifo status: %x", *spi1_fifostatus );

				for(int i = 0; i < SSPI_SIZE; i++)
				{
					printf("\n i: %x", i);
					spi0_rx[i] =  *spi0_rx8;
					spi1_rx[i] =  *spi1_rx8;
					
				}
				printf("\n SSPI READ VALUES");
				
				for(int i = 0; i < SSPI_SIZE; i++)
				{
					printf("\nSSPI0[%x]: %x; SSPI1[%x]: %x", i, spi0_rx[i], i, spi1_rx[i]);
	//				printf("\n ADC[%x]:%x", i, adc_read[i]);
				}
			}
			else
			{
				printf("\n Didn't receive any data ");

			}
//			printf("Read values from ADC \n");
				delay_loop(2000, 1000);
		
			}

#endif
  return 0;
}


