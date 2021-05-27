#include "lcd_1602.h"

static void delay_us(uint32_t delay){
	delay*=32;
	while(delay--);
}

static void lcd_send_4bit(uint8_t data)
{
	/* sending 4-bits to the 4 pins of lcd*/
	if(data & 0x10) d4(1); else d4(0);	// 0x10 == 0b10000
	if(data & 0x20) d5(1); else d5(0);	// 0x20 == 0b100000
	if(data & 0x40) d6(1); else d6(0); 	// 0x40 == 0b1000000
	if(data & 0x80) d7(1); else d7(0);	// 0x80 == 0b10000000
}


static void lcd_send(int8_t rs,uint8_t data)
{
	/* sending data to lcd */
	rs(rs);		/* enable rs if data is to be sent otherwise if it is a command rs is set to zero */
	rw(0);		/* set rw to zero to make the lcd read data */
	lcd_send_4bit(data); /* sending the upper 4 bits */
	en(1);		/* set enable to 1 */
	delay_us(100); /* a delay of 100 us */
	en(0);		/* get enable signal low again to receive new data */
	lcd_send_4bit(data<<4); /* send upper 4 bits of data */
	en(1); /*set enable to one*/
	delay_us(100); /* a delay of 100 us */
	en(0); /*set enable to zero to mark ending of receiving data*/
}

void lcd_data(char c)
{
	/* sending charachter to be printed on LCD screen i.e. as data not command */
	lcd_send(1,(uint8_t)c); 
}

void lcd_init(void)
{
	/* Initialize the lcd in 4-bits mode (Sending Control Commands) */
	lcd_send(0,0x33);	//upper 4 bits to initialize in 8bits mode first "That should be done by default" 
  lcd_send(0,0x32); //lower 4 bits to iniitialize in 8bits mode first
  lcd_send(0,0x28);	// iniitialize in 4 bits mode  4-bit mode (2 lines)
  lcd_send(0,0x0C); // clear the screen and turn the cursor off
  lcd_send(0,0x06); // increment the cursor "shift it to the right to appear on screen"
  lcd_send(0,0x01); // clear and display curson on screen
}

void lcd_clr(void)
{
		/* API to clear the screen */
    lcd_send(0,0x01);	//sending 0x01 to clear screen
    HAL_Delay(2);			//a delay to give chance for screen to update
}


void lcd_gotoxy(char x, char y)
{		/*send commant to move cursor in x and y direction*/
    lcd_send(0,0x80+x+(y*0x40));	/*x represents shifting locations from the screen from the left*/
																  /*y represents shifting locations up and down*/
}

void lcd_puts(char *text)
{
	/*sending string to be printed on LCD*/
    while(*text)
		{
        lcd_data(*text); /*sending charachter by charchter and ensures that sent charachter is identified as data*/
        text++;	/*point to next charachter in the string*/
    }
}

void lcd_print_start(void)
{
	/* print start menu to user */
	HAL_Delay(500);
	lcd_clr();
	lcd_gotoxy(0,0);
	lcd_puts("1 - Enter Pin");
	lcd_gotoxy(0,1);
	lcd_puts("2 - New Pin");
}

void lcd_input_menu (void)
{
	/* asking user to input pin */
	lcd_gotoxy(0,0);
	lcd_puts("Your Pin: ");
}

