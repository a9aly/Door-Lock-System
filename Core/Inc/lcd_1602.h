#ifndef __LCD_1602_H
#define __LCD_1602_H

#include "stm32l4xx_hal.h"
/*LCD Defines*/
#define RS_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_11

#define EN_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_2

#define RW_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_5

#define D4_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_1

#define D5_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_6

#define D6_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_7

#define D7_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_0

#define rs(x)      x?HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,GPIO_PIN_RESET)
#define rw(x)      x?HAL_GPIO_WritePin(RW_GPIO_Port,RW_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RW_GPIO_Port,RW_Pin,GPIO_PIN_RESET)
#define en(x)      x?HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_RESET)

#define d4(x)      x?HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,GPIO_PIN_RESET)
#define d5(x)      x?HAL_GPIO_WritePin(D5_GPIO_Port,D5_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(D5_GPIO_Port,D5_Pin,GPIO_PIN_RESET)
#define d6(x)      x?HAL_GPIO_WritePin(D6_GPIO_Port,D6_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(D6_GPIO_Port,D6_Pin,GPIO_PIN_RESET)
#define d7(x)      x?HAL_GPIO_WritePin(D7_GPIO_Port,D7_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(D7_GPIO_Port,D7_Pin,GPIO_PIN_RESET)


static void lcd_send_4bit(uint8_t data);
static void lcd_send(int8_t rs,uint8_t data);
void lcd_cmd(uint8_t command);
void lcd_data(char c);
void lcd_init(void);
void lcd_clr(void);
void lcd_gotoxy(char x, char y);
void lcd_puts(char *text);
void lcd_print_start(void);
void lcd_input_menu (void);

#endif
