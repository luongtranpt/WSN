/**
 * @file flash.h
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 24-07_2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef _FLASH_H_
#define _FLASH_H_

#include "main.h"



uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);



#endif /* _FLASH_H_ */

