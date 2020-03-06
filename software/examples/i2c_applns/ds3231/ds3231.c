/****************************************************************************
 * Project           	  : shakti devt board
 * Name of the file	      : ds3231.c
 * Name of Author         : Kotteeswaran
 * Email ID               : kottee.1@gmail.com

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

*******************************************************************************/
//#include <stdint.h>//Includes the definitions of standard input/output functions//
#include "i2c.h"
#include "log.h"
#include "uart.h"

#define UPDATE_TIME 1
#define I2C i2c_instance[1]
#define DS3231_SLAVE_ADDRESS 0XD0
#define DS3231_REG_OFFSET 0
#define DS3231_DEC_TO_HEX(decimal)  ( ( (decimal / 10 ) << 4) | (decimal % 10) )
#define DELAY_VALUE 900
#define PRESCALER_COUNT 0x1F
#define SCLK_COUNT 0x91


/** @fn read_ds3231_registers
 * @brief 
 * @details 
 * @warning 
 * @param[in] 
 * @param[Out] 
 */
int read_ds3231_registers(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned int *readTemp, unsigned char length, unsigned long delay)
{
	unsigned char read_buf[4] = {'\0'};
	int i = 0, j = 0,  k = 0, status=0;
	unsigned char temp = 0;

//Writes the slave address for write
		i2c_send_slave_address(i2c_instance, DS3231_SLAVE_ADDRESS, I2C_WRITE, 800);
//Writes the pointer to address that needs to be read
		i2c_write_data(i2c_instance, reg_offset, delay);
//Stops the I2C transaction to start reading the temperature value.
		i2c_instance->control = I2C_STOP;


//Writes the slave address for read
		i2c_send_slave_address(i2c_instance, DS3231_SLAVE_ADDRESS, I2C_READ, 800);


/* Make a dummy read as per spec of the I2C controller */
		i2c_read_data(i2c_instance, &temp, delay);

//Reads the MSB Byte of temperature [D9 - D1]
		for(i = 0; i < length; i++)
		{
			i2c_read_data(i2c_instance, &temp, delay);
			*readTemp = temp;
//			printf("\n Read Value[%02d]: %02x", i, *readTemp);
			if( i == (length - 2) )
				i2c_instance->control = I2C_NACK;
			*readTemp++;
		}
		i2c_instance->control = I2C_STOP;
		return 0;
}

/** @fn  write_ds3231_registers
 * @brief 
 * @details 
 * @warning 
 * @param[in] 
 * @param[Out] 
 */
int write_ds3231_registers(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned int *write_value, unsigned char length, unsigned long delay)
{
	int i = 0, j = 0,  k = 0, status=0;
	unsigned int temp = 0;
		i2c_send_slave_address(i2c_instance, DS3231_SLAVE_ADDRESS, I2C_WRITE, delay);
		i2c_write_data(i2c_instance, reg_offset, delay);
		for(i = 0; i < length; i++)
	{
		i2c_write_data(i2c_instance,  ( *write_value++  & 0xff) /*LM75_TEMP_REG_OFFSET */, delay);
	}
//Stops the I2C transaction to start reading the temperature value.
		i2c_instance->control = I2C_STOP;
//		delay_loop(800, 800);


			return 0;
}

/** @fn dayofweek
 * @brief 
 * @details 
 * @warning 
 * @param[in] 
 * @param[Out] 
 */
int dayofweek(int d, int m, int y)
{
    static int t[] = { 0, 3, 2, 5, 0, 3,
                       5, 1, 4, 6, 2, 4 };
    y -= m < 3;
    return ( y + y / 4 - y / 100 +
             y / 400 + t[m - 1] + d) % 7;
}

/** @fn ds3231_decimal_to_hex
 * @brief 
 * @details 
 * @warning 
 * @param[in] 
 * @param[Out] 
 */
unsigned int ds3231_decimal_to_hex(unsigned int decimal)
{
	return( ((decimal / 10 ) << 4) | (decimal % 10) );
}

/** @fn main
 * @brief 
 * @details 
 * @warning 
 * @param[in] 
 * @param[Out] 
 */
void main()
{
		int timeout;
		unsigned int tempReadValue = 0;
		unsigned long delay = 1000;
		unsigned int write_buf[7] = {0x00}, read_buf[7] = {0x00};
		unsigned char length;
		unsigned int hour, minute, second, date, month, year;
	
		hour = 11;
		minute = 46;
		second = 30;
	
		date = 25;
		month = 01;
		year = 2020;
	
		write_buf[0] = DS3231_DEC_TO_HEX(second); //Seconds
		write_buf[1] = DS3231_DEC_TO_HEX(minute); //Minutes
		write_buf[2] = DS3231_DEC_TO_HEX(hour); //Hours
	
		write_buf[4] = DS3231_DEC_TO_HEX(date); //Seconds
		write_buf[5] = DS3231_DEC_TO_HEX(month); //Minutes
		write_buf[6] = DS3231_DEC_TO_HEX( (year % 100) ); //Hours
	
		write_buf[3] = dayofweek(date, month, year) + 1;
		length = 7;
	
//	set_baud_rate(uart_instance[0], 115200);
	printf("\nHello Welcome to Shakti");

	i2c_init();

	if(config_i2c(I2C, PRESCALER_COUNT, SCLK_COUNT))
	{
		log_error("\tSomething Wrong In Initialization\n");
		return -1;
	}
	else
		log_info("\tIntilization Happened Fine\n");
#ifdef UPDATE_TIME
	write_ds3231_registers(I2C, 0x00, &write_buf[0], length, delay);
			printf("\n Write complete");
#endif
	while(1)
	{
			read_ds3231_registers(I2C, DS3231_REG_OFFSET, &read_buf[0], 7, 800);
			
			log_info("\n Date: %x-%x-20%x Day: ", read_buf[4], read_buf[5], read_buf[6]);
			
			switch(read_buf[3])
			{
				case 1:
						log_info( "   Sunday; ");
						break;

				case 2:
						log_info( "   Monday; ");
						break;

				case 3:
						log_info( "  Tuesday; ");
						break;

				case 4:
						log_info( "Wednesday; ");
						break;

				case 5:
						log_info( " Thursday; ");
						break;

				case 6:
						log_info( "   Friday; ");
						break;

				case 7:
						log_info( " Saturday; ");
						break;
			}
			log_info("Time: %x:%x:%x", read_buf[2], read_buf[1], read_buf[0]);
			delay_loop(1000, 1000);

	}
}

