#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

//���� ����������� LCD �������
#define RS       PC2
#define EN       PC3
#define LCD_PORT PORTC

//������� ������ ������� � LCD
void lcd_com(unsigned char p)                   // 'p' ���� �������
{
    LCD_PORT &= ~(1 << RS);                     // RS = 0 (������ ������)
    LCD_PORT |= (1 << EN);                      // EN = 1 (������ ������ ������� � LCD)
    LCD_PORT &= 0x0F; LCD_PORT |= (p & 0xF0);   // ������� �����
    _delay_ms(1);
    LCD_PORT &= ~(1 << EN);                     // EN = 0 (����� ������ ������� � LCD)
    _delay_ms(1);
    LCD_PORT |= (1 << EN);                      // EN = 1 (������ ������ ������� � LCD)
    LCD_PORT &= 0x0F; LCD_PORT |= (p << 4);     // ������� �����
    _delay_ms(1);
    LCD_PORT &= ~(1 << EN);                     // EN = 0 (����� ������ ������� � LCD)
    _delay_ms(1);
}


//������� ������ ������ � LCD
void lcd_dat(unsigned char p)                   // 'p' ���� ������
{
    LCD_PORT |= (1 << RS)|(1 << EN);            // RS = 1 (������ ������), EN - 1 (������ ������ ������� � LCD)
    LCD_PORT &= 0x0F; LCD_PORT |= (p & 0xF0);   // ������� �����
    _delay_us(50);
    LCD_PORT &= ~(1 << EN);                     // EN = 0 (����� ������ ������� � LCD)
    _delay_us(50);
    LCD_PORT |= (1 << EN);                      // EN = 1 (������ ������ ������� � LCD)
    LCD_PORT &= 0x0F; LCD_PORT |= (p << 4);     // ������� �����
    _delay_us(50);
    LCD_PORT &= ~(1 << EN);                     // EN = 0 (����� ������ ������� � LCD)
    _delay_us(50);
}

//������� ������� �������
void lcd_clear(void)
{
    lcd_com(0x01);
    _delay_ms(1);
}

//������� ������������� LCD
void lcd_init(void)
{
    lcd_com(0x33);   //����� 8 ���, �������� ������
    _delay_ms(10);
    lcd_com(0x32);   //����� 4 ���, �������� ������
    _delay_ms(5);
    lcd_com(0x28);   // ���� 4 ���, LCD - 2 ������
    _delay_ms(5);
    lcd_com(0x08);   // ������ ���������� �������
    _delay_ms(5);
    lcd_clear();
    lcd_com(0x06);   // ����� ������� ������
    lcd_com(0x0C);   // ��������� �������
}

//������� �������� ������� �� ������ line � ������� pos
void go_to(char pos, char line)
{
    char addr = 0x40 * line + pos;
    addr |= 0x80;
    lcd_com(addr);
}

//������� ������ ������ �� LCD, ������� � ��������� X � Y
void lcd_array( char x, char y, const char *str)
{
    go_to(x,y);
    int i = 0;
    while( str[i] != '\0' ) {          // ���� ���� �� ����� ������
        lcd_dat(str[i]);
        i++;
    };
}