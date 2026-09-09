/*****************************************************
**  @brief div moudle driver
**	@author zhang
**	@data 2020-10-13
**	@function  A/B = Q......Rem
*******************************************************/

#ifndef LCM32F039_CRC_H
#define LCM32F039_CRC_H
#include "lcm32f039.h"
//#include <stdint.h>
//#include <core_cm0.h>

#define MULT_CTRL_CRC_VALID_Pos 2
#define MULT_CTRL_CRC_VALID_Msk (0x1UL << MULT_CTRL_CRC_VALID_Pos)
#define MULT_CTRL_CRC_VALID_OFF (0x0UL << MULT_CTRL_CRC_VALID_Pos)
#define MULT_CTRL_CRC_VALID_ON (0x1UL << MULT_CTRL_CRC_VALID_Pos)
/************* Bit Definition for CRC Register **********/
#define CRC_CTRL_GPS_Pos 24
#define CRC_GPS_CRC8 (0x0UL << CRC_CTRL_GPS_Pos)
#define CRC_GPS_CRC16a (0x1UL << CRC_CTRL_GPS_Pos)
#define CRC_GPS_CRC16b (0x2UL << CRC_CTRL_GPS_Pos)
#define CRC_GPS_CRC32 (0x3UL << CRC_CTRL_GPS_Pos)

#define CRC_CTRL_SWAP_Pos 17
#define CRC_SWAP_00 (0x0UL << CRC_CTRL_SWAP_Pos)
#define CRC_SWAP_01 (0x1UL << CRC_CTRL_SWAP_Pos)
#define CRC_SWAP_10 (0x2UL << CRC_CTRL_SWAP_Pos)
#define CRC_SWAP_11 (0x3UL << CRC_CTRL_SWAP_Pos)

#define CRC_CTRL_LM_Pos 16
#define CRC_LM_LSB (0x0UL << CRC_CTRL_LM_Pos)
#define CRC_LM_MSB (0x1UL << CRC_CTRL_LM_Pos)

#define CRC_CTRL_INVEN_Pos 15
#define CRC_INVEN_0 (0x0UL << CRC_CTRL_INVEN_Pos)
#define CRC_INVEN_1 (0x1UL << CRC_CTRL_INVEN_Pos)

#define CRC_CTRL_BYTE_Pos 8
#define CRC_BYTE_1 (0x0UL << CRC_CTRL_BYTE_Pos)
#define CRC_BYTE_2 (0x1UL << CRC_CTRL_BYTE_Pos)
#define CRC_BYTE_3 (0x2UL << CRC_CTRL_BYTE_Pos)
#define CRC_BYTE_4 (0x3UL << CRC_CTRL_BYTE_Pos)

#define CRC_RESET_CRC_Pos 1
#define CRC_RESET_CRC_OFF (0x0UL << CRC_RESET_CRC_Pos)
#define CRC_RESET_CRC_EN (0x1UL << CRC_RESET_CRC_Pos)

#define CRC_RESET_Pos 0
#define CRC_RESET_OFF (0x0UL << CRC_RESET_Pos)
#define CRC_RESET_ON (0x1UL << CRC_RESET_Pos)

void Crc_initial_config_8(CRC_TypeDef *CRCx);
void Crc_initial_config_16a(CRC_TypeDef *CRCx);
void Crc_initial_config_16b(CRC_TypeDef *CRCx);
void Crc_initial_config_32(CRC_TypeDef *CRCx);
void Crc_write_crc_data(CRC_TypeDef *CRCx, uint32_t crc_data);
uint32_t Crc_read_result(CRC_TypeDef *CRCx);
void Crc_write_result(CRC_TypeDef *CRCx);

#endif
