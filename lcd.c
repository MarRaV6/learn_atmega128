#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

//Пины подключения LCD дисплея
#define RS       PC2
#define EN       PC3
#define LCD_PORT PORTC

//Функция записи команды в LCD
void lcd_com(unsigned char p)                   // 'p' байт команды
{
    LCD_PORT &= ~(1 << RS);                     // RS = 0 (запись команд)
    LCD_PORT |= (1 << EN);                      // EN = 1 (начало записи команды в LCD)
    LCD_PORT &= 0x0F; LCD_PORT |= (p & 0xF0);   // старший ниббл
    _delay_ms(1);
    LCD_PORT &= ~(1 << EN);                     // EN = 0 (конец записи команды в LCD)
    _delay_ms(1);
    LCD_PORT |= (1 << EN);                      // EN = 1 (начало записи команды в LCD)
    LCD_PORT &= 0x0F; LCD_PORT |= (p << 4);     // младший ниббл
    _delay_ms(1);
    LCD_PORT &= ~(1 << EN);                     // EN = 0 (конец записи команды в LCD)
    _delay_ms(1);
}


//Функция записи данных в LCD
void lcd_dat(unsigned char p)                   // 'p' байт данных
{
    LCD_PORT |= (1 << RS)|(1 << EN);            // RS = 1 (запись данных), EN - 1 (начало записи команды в LCD)
    LCD_PORT &= 0x0F; LCD_PORT |= (p & 0xF0);   // старший ниббл
    _delay_us(50);
    LCD_PORT &= ~(1 << EN);                     // EN = 0 (конец записи команды в LCD)
    _delay_us(50);
    LCD_PORT |= (1 << EN);                      // EN = 1 (начало записи команды в LCD)
    LCD_PORT &= 0x0F; LCD_PORT |= (p << 4);     // младший ниббл
    _delay_us(50);
    LCD_PORT &= ~(1 << EN);                     // EN = 0 (конец записи команды в LCD)
    _delay_us(50);
}

//Функция очистки дисплея
void lcd_clear(void)
{
    lcd_com(0x01);
    _delay_ms(1);
}

//Функция инициализации LCD
void lcd_init(void)
{
    lcd_com(0x33);   //режим 8 бит, мигающий курсор
    _delay_ms(10);
    lcd_com(0x32);   //режим 4 бит, мигающий курсор
    _delay_ms(5);
    lcd_com(0x28);   // шина 4 бит, LCD - 2 строки
    _delay_ms(5);
    lcd_com(0x08);   // полное выключение дисплея
    _delay_ms(5);
    lcd_clear();
    lcd_com(0x06);   // сдвиг курсора вправо
    lcd_com(0x0C);   // включение дисплея
}

//Функция перевода курсора на строку line и позицию pos
void go_to(char pos, char line)
{
    char addr = 0x40 * line + pos;
    addr |= 0x80;
    lcd_com(addr);
}

//Функция вывода строки на LCD, начиная с координат X и Y
void lcd_array( char x, char y, const char *str)
{
    go_to(x,y);
    int i = 0;
    while( str[i] != '\0' ) {          // цикл пока не конец строки
        lcd_dat(str[i]);
        i++;
    };
}