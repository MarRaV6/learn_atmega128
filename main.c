/*
 * SampleWork.c
 *
 * Created: 20.03.2018 17:19:28
 * Author : x_dea
 */

#define F_CPU 10000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "kalman.h"

#define true 1
#define false 0

//время для работы в секундах

// Коэффициенты PID
#define Kp  1.5
#define Ki  0.25
#define Kd  0 // 5
#define MAX_I 50

// АЦП
// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))
#define ADC_PIN 3

// Размер таблицы для сплайна
#define TABLE_LEN 11

// ШИМ
#define TIMER_TOP 0x3FF
#define PWM_OCR OCR1A
#define MAX_PWM_PRC 75
#define MIN_PWM_PRC 0

// Serial
#define BOUDRATE 9600
#define SERIAL_BUFF_SIZE 50

// Максимальная температура уставки
#define MAX_TARGET 120
#define SCR_LEN 17

//Пины подключения LCD дисплея
#define RS       PC2
#define EN       PC3
#define LCD_PORT PORTC

void lcd_com(unsigned char p);
void lcd_dat(unsigned char p);
void lcd_clear(void);
void lcd_init(void);
void go_to(char pos, char line);
void lcd_array( char x, char y, const char *str);
void clear_line(char* str, int size);

uint16_t read_adc(uint8_t adc_input);
void adc_init(uint8_t);

void InitStruct(void);
double S3x(double datADC1);
void spline_progonka(void);

void usart0_init(uint16_t baud);
void usart0_send(uint8_t data);
void usart_print(char * str);
void usart_println(char * str);

typedef struct DependResist{
    int  code; // значение кода ацп
    int  temp; // соотвествующее значение температуры для кода
} DependResist;

enum Screen {
    screenTemp = 0,
    screenPWM = 1,
    screenDebug = 2
};

uint64_t timeSeconds = 0;   // время в секундах
uint64_t timeMillis = 0;    // время в милисекундах

// kalman_t k;

//------------------------------------------------------------------------------------------------------------------

void Timer_Init()
{
    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 7,816 kHz
    // Mode: Fast PWM top=0x03FF
    // OC1A output: Non-Inverted PWM
    // OC1B output: Disconnected
    // OC1C output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer Period: 0,13101 s
    // Output Pulse(s):
    // OC1A Period: 0,13101 s Width: 0 us
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    // Compare C Match Interrupt: Off
    TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (1<<WGM11) | (1<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
    TCNT1H=0x00;
    TCNT1L=0x00;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;
    OCR1CH=0x00;
    OCR1CL=0x00;
    
    
    // Timer/Counter 3 initialization
    // Clock source: System Clock
    // Clock value: 10000,000 kHz
    // Mode: CTC top=OCR3A
    // OC3A output: Disconnected
    // OC3B output: Disconnected
    // OC3C output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer Period: 0,9989 ms
    // Timer3 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: On
    // Compare B Match Interrupt: Off
    // Compare C Match Interrupt: Off
    TCCR3A=(0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
    TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (1<<WGM32) | (0<<CS32) | (0<<CS31) | (1<<CS30);
    TCNT3H=0x00;
    TCNT3L=0x00;
    ICR3H=0x00;
    ICR3L=0x00;
    OCR3AH=0x27;
    OCR3AL=0x04;
    OCR3BH=0x00;
    OCR3BL=0x00;
    OCR3CH=0x00;
    OCR3CL=0x00;
    
    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);
    ETIMSK=(0<<TICIE3) | (1<<OCIE3A) | (0<<OCIE3B) | (0<<TOIE3) | (0<<OCIE3C) | (0<<OCIE1C);
}

// прерывание по 3 таймеру 
ISR(TIMER3_COMPA_vect) {    
    timeMillis++;  // считаем мс для нашей программы
    
    // если  счетчик времени не стал равной 1 мс, то крутим локальный счетчик
    if (timeMillis % 1000 == 0) {
        timeSeconds++;  // увеличиваем счетчик времени для секунд 
    }
}

void preparations(void){
    // кнопки
    DDRA = 0x00;
    PORTA = 0x00;

    // только PB5 на ШИМ
    DDRB = 0xFF;
    PORTB = 0x00;
    
    DDRC = 0xFF;    //Порт C - выход (подключен LCD дисплей)
    PORTC = 0x00;

    lcd_init(); //Инициализация дисплея
    
    // не используем
    DDRD = 0xFF;
    PORTD = 0x00;
    
    adc_init(ADC_PIN); // подключим АЦП к выводу PF3
}


int main(void) {
    // глобально запретим прерывания
    cli();
    preparations();
    Timer_Init();
    usart0_init(9600);
    sei();  // глобально разрешим прерывания

    InitStruct();

    char upper_line[SCR_LEN];        //Массив для верхней строки LCD дисплея
    char lower_line[SCR_LEN];        //Массив для нижней строки LCD дисплея

    uint16_t tempADC;
    double temp;               // реальная температура
    int16_t target = 30;        // целевая температура
    int16_t pwm_load = 0;       // мера скважности

    // PID
    double epsOld = 0, eps = 0;
    double U = 0, P = 0, I = 0, D = 0;

    enum Screen screen = screenTemp;  // текущий экран
    
    uint64_t lastDisplayTime = 0;

    while (1) {        
        if ((timeMillis - lastDisplayTime) >= 100) {
            lastDisplayTime = timeMillis;

            if (PINA & (1<<1)) {  // A1
               if (screen < screenDebug) {
                   screen++;
               }                
            } else if (PINA & (1<<0)) {  // A0
               if (screen > screenTemp) {
                   screen--;
               }
            };

            tempADC = 1023 - read_adc(ADC_PIN);
            // temp = 0.0000000268 * pow(tempADC, 4) - 0.00004827 * pow(tempADC, 3) + 0.031424 * pow(tempADC, 2) - 8.404671 * tempADC + 801.582; //полином для преобразования температуры
            temp = S3x(tempADC);
            
            char serial_buff[SERIAL_BUFF_SIZE];
            snprintf(serial_buff, SERIAL_BUFF_SIZE, "%i;%i;%0.2f", (int)timeMillis, target, temp);
            usart_println(serial_buff);
        
            clear_line(upper_line, SCR_LEN);
            clear_line(lower_line, SCR_LEN);

            switch (screen) {
                case screenTemp: {
                    if (PINA & (1<<2)) {
                        target = (target + 1) < MAX_TARGET ? target + 1 : target;
                        } else if (PINA & (1<<3)) {
                        target = (target - 1) > 0 ? target - 1 : target;
                    };                        
                        
                    snprintf(upper_line, SCR_LEN, "T: %0.2f (%i)", temp, tempADC);
                    snprintf(lower_line, SCR_LEN, "TARG: %i", target);
                        
                    lcd_clear();
                } break;
                case screenPWM: {
                    snprintf(upper_line, SCR_LEN, "%i%% E:%0.2f", pwm_load, eps);
                    snprintf(lower_line, SCR_LEN, "%i,%i,%0.1f", (int)P, (int)I, D);
                        
                    lcd_clear();
                } break;
                case screenDebug: {
                    snprintf(upper_line, SCR_LEN, "sec: %i", (int)timeSeconds);
                    lcd_clear();
                }
            }            
        
            lcd_array(1,0, upper_line);
            lcd_array(1,1, lower_line);
                
            eps = target - temp;
            P = Kp * eps;
            I += Ki * eps;
            D = Kd * (eps - epsOld);
            U = P + I + D;          
            epsOld = eps;
            
            if (fabs(I) > MAX_I) {
                I = MAX_I * ((I > 0) ? 1 : -1);
            }
            pwm_load = (int) U;
            pwm_load = (pwm_load > MAX_PWM_PRC) ? MAX_PWM_PRC : (pwm_load < MIN_PWM_PRC) ? MIN_PWM_PRC : pwm_load;
            
            if (eps < 0) pwm_load = MIN_PWM_PRC;   // в случае превышения сразу выключим обогревание

            PWM_OCR = (uint16_t)(pwm_load * 10.23);  // изменим широту импулься PWM
        
            //kalman_filter()           
        }
    }
}

// Read the AD conversion result
uint16_t read_adc(uint8_t adc_input){
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

//--------------------------------------------------------------------
//

DependResist dependResist[(TABLE_LEN+1)]; // создаем калибровочную таблицу на max_n
float splineL[TABLE_LEN];

void Spline_progonka(void){
    float mu[TABLE_LEN];

    splineL[1] = 0;
    mu[1] = 0;

    for (uint8_t i = 2; i < TABLE_LEN; i++){
        float sig = (dependResist[i].code - dependResist[i-1].code ) / (dependResist[i+1].code - dependResist[i-1].code);
        float p = sig * splineL[i-1]+2;
        splineL[i] = (sig - 1) / p;
        mu[i] = (dependResist[i+1].temp - dependResist[i].temp) / (dependResist[i+1].code - dependResist[i].code) -
                (dependResist[i].temp - dependResist[i-1].temp) / (dependResist[i].code - dependResist[i-1].code);
        mu[i] = (6* mu[i] / ( dependResist[i+1].code - dependResist[i-1].code) - sig * mu[i-1]) / p;
    }
    splineL[TABLE_LEN] = 0;
    for (int i = TABLE_LEN-1; i > 0; i--){
        splineL[i] = splineL[i] * splineL[i+1] + mu[i];
    }
}

double S3x(double datADC1){
    Spline_progonka();

    int klo = 1;
    int khi = TABLE_LEN;
    int k = TABLE_LEN;

    while (k>1){
        k = khi - klo;
        if (dependResist[k].code > datADC1){
            khi = k;
        } else {
            klo = k;
        }
        k = khi - klo;
    }

    int h = dependResist[khi].code - dependResist[klo].code;
    float a = (dependResist[khi].code - datADC1) / h;
    float b = (datADC1 - dependResist[klo].code) / h;

    return ( a * dependResist[klo].temp + b * dependResist[khi].temp +
                ((( pow(a,3) - a) * splineL[klo] + (pow(b,3) - b) * splineL[khi])) * (h * h) / 6  );
}

void InitStruct(void){
    int tempCode[TABLE_LEN] = {300, 303, 322, 330, 337, 357, 370, 386, 395, 409, 424};
    int tempT[TABLE_LEN]    = { 28,  30,  35,  40,  42,  50,  55,  60,  65,  70,  75};
    for (uint8_t index = 1; index <= TABLE_LEN; index++){
        dependResist[index].code = tempCode[index];
        dependResist[index].temp = tempT[index];
    }
}

//--------------------------------------------------------------------
// Функции usart

void usart0_init(uint16_t baud){
    UCSR0B =    (1<<RXCIE0) |
    (1<<RXEN0) |
    (1<<TXEN0) |
    (0<<UCSZ02);
    UCSR0C =    (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00);

    uint16_t speed = (F_CPU / 16) / baud - 1;

    UBRR0H = (speed >> 8) & 0xFF;

    UBRR0L = speed & 0xFF;
}

void usart0_send(uint8_t data) {
    while (!(UCSR0A & (1<<UDRE0))) {}

    UDR0 = data;
}

void usart_print(char * str) {
    while (*str != 0) {
        usart0_send(*str);

        str++;
    }
}

void usart_println(char * str) {
    usart_print(str);
    usart0_send('\r');
    usart0_send('\n');
}

void clear_line(char* str, int size) {
    memset(str, ' ', size);
}

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