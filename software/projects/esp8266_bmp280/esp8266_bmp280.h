/***************************************************************************
* Project           			:  shakti devt board
* Name of the file	     		:  esp8266_bmp280.h
* Brief Description of file     :  Header file for esp8266_bmp280.c
* Name of Author    	        :  Soutrick Roy Chowdhury
* Email ID                      :  soutrick97@gmail.com

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
/**
  * @file esp8266_bmp280.h
  * @brief sample project using bmp280 and esp8266
  * @details Header file for esp8266_bmp280.c
*/

#ifndef ESP82666_BMP280_H_
#define ESP82666_BMP280_H_

#define I2C i2c_instance[0]
#define BMP280 1
#define ESP8266 1

#define API_KEY "PPI6OVOI1A2LGXMZ"

#define BMP280_SLAVE_ADDRESS 0xEC	//Defines the Starting address of slave//
#define DELAY_VALUE 900
#define PRESCALER_COUNT 0x1F
#define SCLK_COUNT 0x91
#define BMP280_CTRL_MEANS 0xF4
#define BMP280_NORMAL_MODE 0x26
#define BMP280_STATUS_REGISTER 0xF3
#define BMP280_CONFIG_REGISTER 0xF5
#define BMP280_RESET_REGISTER 0xE0

#define BMP280_REG_DIG_T1 0x88
#define BMP280_REG_DIG_T2 0x8A
#define BMP280_REG_DIG_T3 0x8C

#define BMP280_REG_DIG_P1 0x8E
#define BMP280_REG_DIG_P2 0x90
#define BMP280_REG_DIG_P3 0x92
#define BMP280_REG_DIG_P4 0x94
#define BMP280_REG_DIG_P5 0x96
#define BMP280_REG_DIG_P6 0x98
#define BMP280_REG_DIG_P7 0x9A
#define BMP280_REG_DIG_P8 0x9C
#define BMP280_REG_DIG_P9 0x9E

#endif
