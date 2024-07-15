## Table of contents
* [General info](#general-info)
* [Features](#technologies)
* [Setup](#setup)

## General info
This project is built in STM32CubeMX version 6.5.0 and STM32CubeH7 firmware package version 1.10.0 and tested on Keil MDK 5.36.0.0 by me at 15 july 2024.
	
## Features
Fixed IP address 192.168.1.10
Code should work even when re-generating the code in STM32CubeMX
Changes in code can be find by searching for ETH_CODE keyword
Added FatFs library and SDMMC peripheral working with MDMA 
	
## Setup
First, please read information about LWIP in this link [STM32H7-LwIP-Examples](https://github.com/stm32-hotspot/STM32H7-LwIP-Examples). I've made some minor changes to that project and added FatFS support. please consider that when using FreeRTOS you should activate SDMMC+MDMA for running FatFS and turn the D-cache feature off by commenting the SCB_EnableDCache() function or you can enable cache maintenance by uncommenting ENABLE_SD_DMA_CACHE_MAINTENANCE define in sd_diskio.c. but in my project, the second method doesn't work perfectly.   
