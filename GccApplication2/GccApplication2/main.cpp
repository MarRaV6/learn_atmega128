#define F_CPU 10000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

//Пины подключения LCD дисплея
#define RS       PC2
#define EN       PC3
#define LCD_PORT PORTC


// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

unsigned int read_adc(unsigned char adc_input);
void adc_init(uint8_t PIN);

//------------------------------------------------------------------------------------------------------------------
//Функция записи команды в LCD
void lcd_com(unsigned char p)					// 'p' байт команды
{
	LCD_PORT &= ~(1 << RS);						// RS = 0 (запись команд)
	LCD_PORT |= (1 << EN);						// EN = 1 (начало записи команды в LCD)
	LCD_PORT &= 0x0F; LCD_PORT |= (p & 0xF0);	// старший ниббл
	_delay_ms(1);
	LCD_PORT &= ~(1 << EN);						// EN = 0 (конец записи команды в LCD)
	_delay_ms(1);
	LCD_PORT |= (1 << EN);						// EN = 1 (начало записи команды в LCD)
	LCD_PORT &= 0x0F; LCD_PORT |= (p << 4);		// младший ниббл
	_delay_ms(1);
	LCD_PORT &= ~(1 << EN);						// EN = 0 (конец записи команды в LCD)
	_delay_ms(1);
}


//Функция записи данных в LCD
void lcd_dat(unsigned char p)					// 'p' байт данных
{
	LCD_PORT |= (1 << RS)|(1 << EN);			// RS = 1 (запись данных), EN - 1 (начало записи команды в LCD)
	LCD_PORT &= 0x0F; LCD_PORT |= (p & 0xF0);	// старший ниббл
	_delay_us(50);
	LCD_PORT &= ~(1 << EN);						// EN = 0 (конец записи команды в LCD)
	_delay_us(50);
	LCD_PORT |= (1 << EN);						// EN = 1 (начало записи команды в LCD)
	LCD_PORT &= 0x0F; LCD_PORT |= (p << 4);		// младший ниббл
	_delay_us(50);
	LCD_PORT &= ~(1 << EN);						// EN = 0 (конец записи команды в LCD)
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
	lcd_com(0x0C);	 // включение дисплея
}

//Функция перевода курсора на строку line и позицию pos
void go_to(char pos, char line)
{
	char addr = 0x40 * line + pos;
	addr |= 0x80;
	lcd_com(addr);
}

//Функция вывода строки на LCD, начиная с координат X и Y
void lcd_array( char x, char y, const char *str )
{
	go_to(x,y);
	while( *str )			//цикл пока указатель существует
	lcd_dat( *str++ );	//выводить строку
}

char upper_line[17];		//Массив для верхней строки LCD дисплея
char lower_line[17];		//Массив для нижней строки LCD дисплея
int dot_pos = 0;			//Текущая позиция точки на экране
int dot_pos_counter = 0;	//Счётчик для функционирования изменения скорости перемещения точки
int speed = 7;				//Скорость движения точки

//------------------------------------------------------------------------------------------------------------------

int main(void)
{
	// ADC initialization
	// ADC Clock frequency: 625,000 kHz
	// ADC Voltage Reference: AVCC pin
	/*ADMUX=ADC_VREF_TYPE;
	ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
	SFIOR=(0<<ACME);*/
	
	//Настройка портов
	
	DDRC = 0xFF;	//Порт C - выход (подключен LCD дисплей)
	PORTC = 0;
	
	lcd_init(); //Инициализация дисплея
	
	// глобально запретим прерывания
	cli();
	DDRA = 0x00; // порта А полностью на ввод
	PORTA = 0x00;
	
	// DDRB = 0xFF;
	DDRD = 0xFF;
	PORTD = 0x00;
	adc_init(3); // подключим АЦП к выводу PF3
	// глобально разрешим прерывания
	sei();
	//uint8_t data;
	
	bool screen_for_temp = true;
	bool sreen_for_ust = false;
	int data;
	uint8_t ust = 30;
	
	while (1) {
		int dat = read_adc(3);
		double datADC = 1023 - dat;
		data = 0.0000000268 * pow(datADC, 4) - 0.00004827 * pow(datADC, 3) + 0.031424 * pow(datADC, 2) - 8.404671 * datADC + 801.582; //fabs(datADC/3.15 + 43 - 111);
		
		if (PINA & (1<<0)) {
			sreen_for_ust = true;
			screen_for_temp = false;
		};
		
		if (PINA & (1<<1)) {
			sreen_for_ust = false;
			screen_for_temp = true;
		};
		
		if (PINA & (1<<2)) {
			ust = ust + 1;
		};
		
		if (PINA & (1<<3)) {
			ust = ust - 1;
		};
		
		if (screen_for_temp)
		{
			upper_line[0] = 'T';
			upper_line[1] = 'E';
			upper_line[2] = 'M';
			upper_line[3] = 'P';
			upper_line[4] = ' ';
			upper_line[5] = '=';
			upper_line[6] = ' ';
			upper_line[7] = data/100+0x30;
			upper_line[8] = (data/10)%10+0x30; //1024/1000=10.24/10=24
			upper_line[9] = (data)%10+0x30;
		};
		 
		if (sreen_for_ust) {
			upper_line[0] = 'U';
			upper_line[1] = 'S';
			upper_line[2] = 'T';
			upper_line[3] = ' ';
			upper_line[4] = '=';
			upper_line[5] = ' ';
			upper_line[6] = ust/100+0x30;
			upper_line[7] = (ust/10)%10+0x30; //1024/1000=10.24/10=24
			upper_line[8] = ust%10+0x30;
			upper_line[9] = ' ';
		}
		
		
		lower_line[0] = ' ';
		lower_line[1] = datADC/1000+0x30;
		lower_line[2] = ((int)datADC/100)%10+0x30;
		lower_line[3] = ((int)datADC/10)%10+0x30;//%10+0x30;
		lower_line[4] = ((int)datADC)%10+0x30;
		
		lcd_array(1,0,upper_line);	//Вывод массива верхней строки на дисплей
		lcd_array(0,1,lower_line);	//Вывод массива нижней строки на дисплей
		
		_delay_ms(100);
	}
}

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
	ADMUX=adc_input | ADC_VREF_TYPE;
	// Delay needed for the stabilization of the ADC input voltage
	_delay_ms(10);
	// Start the AD conversion
	ADCSRA|=(1<<ADSC);
	// Wait for the AD conversion to complete
	while ((ADCSRA & (1<<ADIF))==0);
	ADCSRA|=(1<<ADIF);
	return ADCW;
}

// PIN - номер порта F от 0 до 7
void adc_init(uint8_t PIN) {
	DDRF &= ~(1<<PIN); // вывод PF с номером PIN настроить на ввод
	// левое смещение, 8 битный АЦП
	// устанавливаем источник АЦП пин с номером PIN
	ADMUX |= (0<<REFS1) | (1<<REFS0) |(1<<ADLAR) | (PIN<<MUX0);
	ADCSRA |= (1<<ADEN) | // разрешение работы АЦП
	(1<<ADSC) | // запуск АЦП
	(1<<ADFR) | // автоматический режим АЦП
	(1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	// предделитель АЦП 128
}

