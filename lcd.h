#ifndef LCD_H_
#define LCD_H_
void lcd_com(unsigned char p);
void lcd_dat(unsigned char p);
void lcd_clear(void);
void lcd_init(void);
void go_to(char pos, char line);
void lcd_array( char x, char y, const char *str);
#endif