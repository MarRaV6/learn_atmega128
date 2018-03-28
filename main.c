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

#define true 1
#define false 0

//Пины подключения LCD дисплея
#define RS       PC2
#define EN       PC3
#define LCD_PORT PORTC

// Коэффициенты PID
#define Kp  1
#define Ki  0.5
#define Kd  0.1

// АЦП
// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))
#define ADC_PIN 3

// Размер таблицы для сплайна
#define TABLE_LEN 11

// ШИМ
#define TIMER_TOP 0x3FF
#define PWM_OCR OCR1A

// Максимальная температура уставки
#define MAX_TARGET 120
#define SCR_LEN 17

uint16_t read_adc(uint8_t adc_input);
void adc_init(uint8_t);

void intToChars(char *str, uint16_t valueInt);

void InitStruct(void);
int S3x(uint16_t datADC1);
void spline_progonka(void);

typedef struct DependResist{
    int  code; // значение кода ацп
    int  temp; // соотвествующее значение температуры для кода
} DependResist;

enum Screen {
    screenTemp = 0,
    screenTarget = 1,
    screenPWM = 2
};

//------------------------------------------------------------------------------------------------------------------
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
void lcd_array( char x, char y, const char *str, int size)
{
    go_to(x,y);
    int i = 0;
    while( str[i] != EOF ) {          // цикл пока не конец строки
        lcd_dat(str[i]);
        i++;
    };
}

void clear_line(char* str, int size) {
    memset(str, ' ', size);
}

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

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);
    ETIMSK=(0<<TICIE3) | (0<<OCIE3A) | (0<<OCIE3B) | (0<<TOIE3) | (0<<OCIE3C) | (0<<OCIE1C);
};

void preparations(void){
    DDRC = 0xFF;    //Порт C - выход (подключен LCD дисплей)
    PORTC = 0x00;

    // кнопки
    DDRA = 0x00;
    PORTA = 0x00;

    // только PB5 на ШИМ
    DDRB = 0xFF;
    PORTB = 0x00;

    // не используем
    DDRD = 0xFF;
    PORTD = 0x00;

    lcd_init(); //Инициализация дисплея
    adc_init(ADC_PIN); // подключим АЦП к выводу PF3
}

int main(void) {
    // глобально запретим прерывания
    cli();
    preparations();
    sei();  // глобально разрешим прерывания

    Timer_Init();
    InitStruct();

    char upper_line[SCR_LEN];        //Массив для верхней строки LCD дисплея
    char lower_line[SCR_LEN];        //Массив для нижней строки LCD дисплея

    uint16_t tempADC;
    int temp;               // реальная температура
    int target = 30;        // целевая температура
    int pwm_load = 0;       // мера скважности
    char buff[5];           // временный буфер для вывода чисел

    // PID
    int eps = 0, epsOld = 0;
    float U = 0, P = 0, I = 0, D = 0;

    enum Screen screen = screenTemp;  // текущий экран

    while (1) {
        tempADC = 1023 - read_adc(ADC_PIN);

        // temp = 0.0000000268 * pow(tempADC, 4) - 0.00004827 * pow(tempADC, 3) + 0.031424 * pow(tempADC, 2) - 8.404671 * tempADC + 801.582; //полином для преобразования температуры
        temp = S3x(tempADC);

        clear_line(upper_line, SCR_LEN);
        clear_line(lower_line, SCR_LEN);

        // переключение экранов
        if (PINA & (1<<0)) {  // A0
            if (screen < screenPWM) {
                screen++;
            }
        } else if (PINA & (1<<1)) {  // A1
            if (screen > screenTemp) {
                screen--;
            }
        };

        clear_line(buff, sizeof buff);

        switch (screen) {
            case screenTemp: {
                intToChars(buff, temp);
                snprintf(upper_line, SCR_LEN, "TEMP = %s", buff);

                clear_line(buff, sizeof buff);

                intToChars(buff, tempADC);
                snprintf(lower_line, SCR_LEN, "ADC = %s", buff);

            } break;
            case screenTarget: {
                if (PINA & (1<<2)) {
                    target = (target + 1) < MAX_TARGET ? target + 1 : target;
                } else if (PINA & (1<<3)) {
                    target = (target - 1) > 0 ? target - 1 : target;
                };

                intToChars(buff, target);
                snprintf(upper_line, SCR_LEN, "TARG = %s", buff);

                clear_line(lower_line, SCR_LEN);

            } break;
            case screenPWM: {
                if (PINA & (1<<2)) {
                    pwm_load = (pwm_load + 1) < 100 ? pwm_load + 1 : pwm_load;
                } else if (PINA & (1<<3)) {
                    pwm_load = (pwm_load - 1) > 0 ? pwm_load - 1 : pwm_load;
                };

                intToChars(buff, pwm_load);
                snprintf(upper_line, SCR_LEN, "PWM = %s", buff);

                clear_line(lower_line, SCR_LEN);
            }
        }

        lcd_array(1,0, upper_line, SCR_LEN);
        lcd_array(1,1, lower_line, SCR_LEN);

        eps = target - temp;
        P = Kp * eps;
        I = I - Ki * eps;
        D = Kd * (epsOld - eps);
        U = P + I + D;
        epsOld = eps;

        pwm_load += U;  // вот тут округления с float до int

        if (pwm_load > 75) pwm_load = 75;   // ограничим ШИМ сверху
        if (pwm_load < 0) pwm_load = 0;     // и снизу
        if (eps < 0) pwm_load = 0;          // в случае превышения сразу выключим

        PWM_OCR = (uint16_t)(pwm_load * 10.23);  // изменим широту импулься PWM

        _delay_ms(100);  // убрать
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
//функция для перевода числа в строку
void intToChars(char *str, uint16_t valueInt){
    char valueChar[5];

    uint8_t flagMinus = valueInt < 0;
    str[0] = flagMinus ? '-' : '+';
    int countInt = (valueInt == 0) ? 1 : 0;  // длина будующей строки

    while(valueInt != 0){
        valueChar[countInt++] = valueInt % 10 + 0x30;
        valueInt /= 10;
    }

    int j = 1;
    while(countInt >= 0){
        str[j++] = valueChar[--countInt];
    }
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

int S3x(uint16_t datADC1){
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

    int s3x = (int) ( a * dependResist[klo].temp + b * dependResist[khi].temp +
                ((( pow(a,3) - a) * splineL[klo] + (pow(b,3) - b) * splineL[khi])) * (h * h) / 6  );
    return s3x;
}

void InitStruct(void){
    dependResist[1].code  = 300; dependResist[1].temp  = 28;
    dependResist[2].code  = 303; dependResist[2].temp  = 30;
    dependResist[3].code  = 322; dependResist[3].temp  = 35;
    dependResist[4].code  = 330; dependResist[4].temp  = 40;
    dependResist[5].code  = 337; dependResist[5].temp  = 42;
    dependResist[6].code  = 357; dependResist[6].temp  = 50;
    dependResist[7].code  = 370; dependResist[7].temp  = 55;
    dependResist[8].code  = 386; dependResist[8].temp  = 60;
    dependResist[9].code  = 395; dependResist[9].temp  = 65;
    dependResist[10].code = 409; dependResist[10].temp = 70;
    dependResist[11].code = 424; dependResist[11].temp = 75;
}
