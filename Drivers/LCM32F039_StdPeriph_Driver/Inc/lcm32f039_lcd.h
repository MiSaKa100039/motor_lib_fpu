#ifndef LCM32F039_LCD_H
#define LCM32F039_LCD_H
#include "lcm32f039.h"

/**
 *******************************************************************************
 ** \def group LCD Group LCD
 **
 **
 ******************************************************************************/
typedef struct
{

   uint32_t LCD_Bias;        /*偏置电平选择位*/              /*Bias level selector bit*/  
   uint32_t LCD_Brs;         /*偏置电阻选择位*/              /*Bias resistance selection bit*/
   FunctionalState LCD_Fcen; /*快速充电使能位.*/             /*Enable position of fast charging.*/
   uint32_t LCD_Crs;         /*对比度选择位*/                /*Contrast selector bit*/
   uint32_t LCD_Cclks;       /*充电时间选择位*/              /*Charging time selection bit*/
   uint32_t LCD_Scdiv;       /*LCD扫描时钟源分频控制位*/     /*LCD scanning clock source frequency division control bit*/

} LCD_InitTypeDef;
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
/*************************************
 * LCD COM口
 * ***************************************************/
#define COM0 (0x01)
#define COM1 (0x02)
#define COM2 (0x04)
#define COM3 (0x08)
#define COM4 (0x10)
#define COM5 (0x20)
/*************************************
 * LCD SEG口
 * ***************************************************/
#define SEG0 (0x00000001UL)
#define SEG1 (0x00000002UL)
#define SEG2 (0x00000004UL)
#define SEG3 (0x00000008UL)
#define SEG4 (0x00000010UL)
#define SEG5 (0x00000020UL)
#define SEG6 (0x00000040UL)
#define SEG7 (0x00000080UL)
#define SEG8 (0x00000100UL)
#define SEG9 (0x00000200UL)
#define SEG10 (0x00000400UL)
#define SEG11 (0x00000800UL)
#define SEG12 (0x00001000UL)
#define SEG13 (0x00002000UL)
#define SEG14 (0x00004000UL)
#define SEG15 (0x00008000UL)
#define SEG16 (0x00010000UL)
#define SEG17 (0x00020000UL)
#define SEG18 (0x00040000UL)
#define SEG19 (0x00080000UL)
#define SEG20 (0x00100000UL)
#define SEG21 (0x00200000UL)
#define SEG22 (0x00400000UL)
#define SEG23 (0x00800000UL)
#define SEG24 (0x01000000UL)
#define SEG25 (0x02000000UL)
#define SEG26 (0x04000000UL)
#define SEG27 (0x08000000UL)
#define SEG28 (0x10000000UL)
#define SEG29 (0x20000000UL)
#define SEG30 (0x40000000UL)
#define SEG31 (0x80000000UL)
#define SEG32 (0x80000001UL)
#define SEG33 (0x80000002UL)
/********************************************************************************
** LCD bias电平选择   /LCD Bias level selection
*****************************************************************************/
#define LCD_BIAS_2 (0X00 << 3) /*1/2 bias*/
#define LCD_BIAS_3 (0X01 << 3) /*1/3 bias*/
#define IS_LCD_BIAS(BIAS) ((BIAS == LCD_BIAS_2) || (BIAS == LCD_BIAS_3))

/**
 ******************************************************************************
 **  BRS 偏置电阻选择     /BRS bias resistance selection
 *****************************************************************************/
#define LCD_BRS_1110K (0X00UL) /*1110K */
#define LCD_BRS_496K (0X01UL)  /*496K */
#define LCD_BRS_82K (0X02UL)   /*82K */
#define LCD_BRS_20K (0X03UL)   /*20K */
#define IS_LCD_BRS(BRS) ((BRS == LCD_BRS_1110K) || \
                         (BRS == LCD_BRS_496K) ||  \
                         (BRS == LCD_BRS_82K) ||   \
                         (BRS == LCD_BRS_20K))
/**
******************************************************************************
**  LCD 对比度选择，电阻越小，对比度越高，8为最高，1为最低
**  /LCD contrast selection, the lower the resistance, the higher the contrast, 8 is the highest, 1 is the lowest
*****************************************************************************/
#define LCD_CRS_1 (0X07 << 3) /*140K */
#define LCD_CRS_2 (0X06 << 3) /*120K */
#define LCD_CRS_3 (0X05 << 3) /*100K */
#define LCD_CRS_4 (0X04 << 3) /*80K */
#define LCD_CRS_5 (0X03 << 3) /*60K */
#define LCD_CRS_6 (0X02 << 3) /*40K */
#define LCD_CRS_7 (0X01 << 3) /*20K */
#define LCD_CRS_8 (0X00 << 3) /*0K */
#define IS_LCD_CRS(CRS) ((CRS == LCD_CRS_1) || \
                         (CRS == LCD_CRS_2) || \
                         (CRS == LCD_CRS_3) || \
                         (CRS == LCD_CRS_4) || \
                         (CRS == LCD_CRS_5) || \
                         (CRS == LCD_CRS_6) || \
                         (CRS == LCD_CRS_7) || \
                         (CRS == LCD_CRS_8))
/**
 ******************************************************************************
 **  LCD 充电时间选择  /LCD charging time selection
 *****************************************************************************/
#define LCD_CCLKS_1_4 (0X00 << 6)  /*1/4 COM 周期（默认） */   /*1/4 COM Period (default) */ 
#define LCD_CCLKS_1_8 (0X01 << 6)  /*1/8 COM 周期*/           /*1/8 COM Period*/         
#define LCD_CCLKS_1_16 (0X02 << 6) /*1/16 COM 周期 */         /*1/16 COM Period */       
#define LCD_CCLKS_1_32 (0X03 << 6) /*1/32 COM 周期 */         /*1/32 COM Period */       
#define IS_LCD_CCLKS(CCLKS) ((CCLKS == LCD_CCLKS_1_4) ||  \
                             (CCLKS == LCD_CCLKS_1_8) ||  \
                             (CCLKS == LCD_CCLKS_1_16) || \
                             (CCLKS == LCD_CCLKS_1_32))

/**
 ******************************************************************************
 ** 闪烁频率选择  /Flicker frequency selection
 *****************************************************************************/
#define LCD_TK_TKFS_2HZ (0X00 << 2)    /*2Hz*/
#define LCD_TK_TKFS_1HZ (0X01 << 2)    /*1Hz*/
#define LCD_TK_TKFS_0_5HZ (0X02 << 2)  /*0.5Hz*/
#define LCD_TK_TKFS_0_25HZ (0X03 << 2) /*0.25Hz*/
#define IS_LCD_TK_TKFS (TKFS)((TKFS == LCD_TK_TKFS_2HZ) ||   \
                              (TKFS == LCD_TK_TKFS_1HZ) ||   \
                              (TKFS == LCD_TK_TKFS_0_5HZ) || \
                              (TKFS == LCD_TK_TKFS_0_25HZ))
/**
 ******************************************************************************
 **  LCD显示模式  /LCD display mode
 *****************************************************************************/
#define LCD_TK_TKMOD_SINGLE (0X00 << 6) /*单点闪烁，闪烁位置由 TKCOMS 和 TKSEGS 决定*/  /*Single point flicker, flicker position determined by TKCOMS and TKSEGS*/
#define LCD_TK_TKMOD_FULL (0X01 << 6)   /*全屏闪烁*/                                  /*Blinking in full screen*/
#define IS_LCD_TK_TKMOD(TKMOD) ((TKMOD == LCD_TK_TKMOD_SINGLE) || (TKMOD == LCD_TK_TKMOD_FULL))

#define LCDTK_IT 1

void LCD_Init(LCD_InitTypeDef *LCD_InitStruct);
void LCD_Cmd(FunctionalState NewState);
void LCD_AnalogModeTestCmd(FunctionalState NewState);
void LCD_ClearFlag(uint32_t LCD_FLAG);
void LCD_GpioSet_COM(GPIO_TypeDef *GPIOx, uint16_t GPIO_PinSource, uint32_t COMx);
void LCD_GpioSet_SEG(GPIO_TypeDef *GPIOx, uint16_t GPIO_PinSource, uint32_t SEGx);

void LCDTK_Init(uint16_t TKFSx, uint16_t TKMODx);
void LCDTK_SEG_Cmd(uint32_t SEGx, FunctionalState NewState);
void LCDTK_COM_Cmd(uint32_t COMx, FunctionalState NewState);
void LCDTK_Cmd(FunctionalState NewState);
void LCDTK_ITConfig(uint32_t LCD_FLAG, FunctionalState NewState);
void LCDTK_ClearFlag(uint32_t LCD_FLAG);

#endif /* __LCD_H__ */

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
