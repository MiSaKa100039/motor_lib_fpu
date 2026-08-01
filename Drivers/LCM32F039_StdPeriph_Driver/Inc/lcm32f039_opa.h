#ifndef LCM32F039_OPA_H
#define LCM32F039_OPA_H
#include "lcm32f039.h"
/******************** ANACTRL Register Struct Definition *****************/
typedef struct
{
  __IO uint32_t OPA_CSR; // 0x00

} OPA_TypeDef;

typedef struct
{
  uint32_t OPA_Oppselect;
  uint32_t OPA_Opnselect;
  uint32_t OPA_Gain;
  uint32_t OPA_BiasVoltage;
  uint32_t OPA_FeedbackRes;
  uint32_t OPA_Opoto_Gpio;  //开关 OPOE   /switch
  uint32_t OPA_Opoto2_Gpio; //开关 OPOE2  /switch
  uint32_t OPA_OppRES_short;
  uint32_t OPA_OpnRES_short;
  uint32_t OPA_BiasVoltageSource;
} OPA_InitTypeDef;
/*****************  Definition for OPA GAIN **************/
#define OPA_Gain_1 (0x0UL << 8)
#define OPA_Gain_2 (0x1UL << 8)
#define OPA_Gain_6 (0x2UL << 8)
#define OPA_Gain_8 (0x3UL << 8)
#define OPA_Gain_10 (0x4UL << 8)
#define OPA_Gain_16 (0x5UL << 8)
#define OPA_Gain_20 (0x6UL << 8)
#define OPA_Gain_32 (0x7UL << 8)

/***************** JUST FOR OPA0**************/
/*****************  Definition for OP0_OPP SELECT **************/
#define OPA0_Oppselect_PA2 (0x0UL << 26)
#define OPA0_Oppselect_PF8 (0x1UL << 26) // GPIO_ANA1
#define OPA0_Oppselect_Vss (0x2UL << 26)
#define OPA0_Oppselect_PF7 (0x3UL << 26) // GPIO_ANA1
/*****************  Definition for OP0_OPN SELECT **************/
#define OPA0_Opnselect_PA3 (0x0UL << 24)
#define OPA0_Opnselect_NIN (0x2UL << 24)
#define OPA0_Opnselect_PF6 (0x3UL << 24) // GPIO_ANA1

/*****************  Definition for OP1_OPP SELECT **************/
#define OPA1_Oppselect_PF0 (0x0UL << 26) // OPA1_OPP
#define OPA1_Oppselect_PA7 (0x1UL << 26)
#define OPA1_Oppselect_Vss (0x2UL << 26)
#define OPA1_Oppselect_PA9 (0x3UL << 26) // GPIO_ANA1
/*****************  Definition for OP1_OPN SELECT **************/
#define OPA1_Opnselect_PF1 (0x0UL << 24)
#define OPA1_Opnselect_PB0 (0x1UL << 24) // GPIO_ANA1
#define OPA1_Opnselect_NIN (0x2UL << 24)
#define OPA1_Opnselect_PA10 (0x3UL << 24) // ANA1

/*****************  Definition for OP2_OPP SELECT **************/
#define OPA2_Oppselect_PF4 (0x0UL << 26)
#define OPA2_Oppselect_PB0 (0x1UL << 26) // GPIO_ana1
#define OPA2_Oppselect_Vss (0x2UL << 26)
#define OPA2_Oppselect_PB10 (0x3UL << 26) // GPIO_ana1
/*****************  Definition for OP2_OPn SELECT **************/
#define OPA2_Opnselect_PF5 (0x0UL << 24)
#define OPA2_Opnselect_PA7 (0x1UL << 24)
#define OPA2_Opnselect_NIN (0x2UL << 24)
#define OPA2_Opnselect_PB2 (0x3UL << 24) // GPIO_ana1

/*****************  Definition for OP3_OPP SELECT **************/
#define OPA3_Oppselect_PB12 (0x0UL << 26)
#define OPA3_Oppselect_PB9 (0x1UL << 26) // GPIO_ana1
#define OPA3_Oppselect_vss (0x2UL << 26)
#define OPA3_Oppselect_vss1 (0x3UL << 26) // GPIO_ana1
/*****************  Definition for OP3_OPn SELECT **************/
#define OPA3_Opnselect_PB13 (0x0UL << 24)
#define OPA3_Opnselect_PB8 (0x1UL << 24)
#define OPA3_Opnselect_NIN (0x2UL << 24)
#define OPA3_Opnselect_vss (0x3UL << 24) // GPIO_ana1

#define OPA_BiasVoltage_EN (0x1UL << 5)
#define OPA_BiasVoltage_DISEN (0x0UL << 5)
#define OPA_FeedbackRes_EN (0x1UL << 4)
#define OPA_FeedbackRes_DISEN (0x0UL << 4)

#define OPA_OPOEX_EN (0x1UL << 3)
#define OPA_OPOEX_DISEN (0x0UL << 3)

#define OPA_OPOEX2_EN (0x1UL << 7)
#define OPA_OPOEX2_DISEN (0x0UL << 7)

#define OPA_OppRES_short_EN (0x1UL << 2)
#define OPA_OppRES_short_DISEN (0x0UL << 2)
#define OPA_OpnRES_short_EN (0x1UL << 1)
#define OPA_OpnRES_short_DISEN (0x0UL << 1)

#define OPA_OPA_BiasVoltageSource_HALFVRH (0x0UL << 6)
#define OPA_OPA_BiasVoltageSource_DAC (0x1UL << 6)

/*************************OPA_NIN******************************/
#define OPA_NINselect_vss (0x0UL << 8)
#define OPA_NINselect_PA8 (0x1UL << 8)
#define OPA_NINselect_PF1 (0x2UL << 8
#define OPA_NINselect_PA6 (0x3UL << 8)


/***************** ANACTRL Declaration **************/
// ANACTRL_ADDR_STR       (0x40017C00)
#define OPA0 ((OPA_TypeDef *)(0x40017C00 + 0x00))
#define OPA1 ((OPA_TypeDef *)(0x40017C00 + 0x04))
#define OPA2 ((OPA_TypeDef *)(0x40017C00 + 0x08))
#define OPA3 ((OPA_TypeDef *)(0x40017C00 + 0x0C))
/***************** ANACTRL Function Definition **************/

void OPA0_TRIM(void);
void OPA1_TRIM(void);
void OPA2_TRIM(void);
void OPA3_TRIM(void);

uint32_t get_opa2_trim(void);
void OPA_DeInit(void);
void OPA_Init(OPA_TypeDef *opa_ptr, OPA_InitTypeDef *OPA_InitStruct);
void OPA_NINSelect(uint32_t NINSelect);
void OPA_StructInit(OPA_InitTypeDef *OPA_InitStruct);
void OPA_Cmd(OPA_TypeDef *opa_ptr, FunctionalState NewState);

uint32_t opa0_trim_FLAG(void);
uint32_t opa1_trim_FLAG(void);
uint32_t opa2_trim_FLAG(void);
uint32_t opa3_trim_FLAG(void);

void opa0_trim(uint32_t trim);
uint32_t get_opa0_trim(void);
void opa1_trim(uint32_t trim);
uint32_t get_opa1_trim(void);
void opa2_trim(uint32_t trim);
uint32_t get_opa2_trim(void);
void opa3_trim(uint32_t trim);
uint32_t get_opa3_trim(void);
#endif // __ANACTRL_H__
