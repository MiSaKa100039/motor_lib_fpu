/*****************************************************
**  @brief div moudle driver
**	@author zhang
**	@data 2020-10-13
**	@function  A/B = Q......Rem
*******************************************************/

#ifndef LCM32F039_DIV_H
#define LCM32F039_DIV_H
#include "lcm32f039.h"
#include <stdint.h>

typedef union
{
	struct
	{
		uint32_t LOW32;
		uint32_t HIGH32;
	} D;
	uint64_t Dividend64;
} D_Typedef;

typedef struct
{
	D_Typedef result64;	  //商
	uint32_t remainder64; //余数
} DIV_Result;

typedef struct
{
	uint32_t result32;	  //商
	uint32_t remainder32; //余数
} result_t;

long long Fun_32_DIV_32(int dividend, int divisor, DIV_Result *result);
long long Fun_64_DIV_32(long long dividend, int divisor, DIV_Result *result);

int Fun_32_DIV_16(int dividend, int divisor, result_t *result);
int Fun_16_DIV_16(int dividend, int divisor, result_t *result);
#endif
