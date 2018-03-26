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

//Пины подключения LCD дисплея
#define RS       PC2
#define EN       PC3
#define LCD_PORT PORTC


// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))
// ???
#define max_n 11

unsigned int read_adc(unsigned char adc_input);
void adc_init(uint8_t);

void InitStruct(void);
int S3x(double datADC1);
void spline_progonka(void);

typedef struct DependResist{    
    int  code; // значение кода ацп 
    int  temp; // соотвествующее значение температуры для кода
} DependResist;


DependResist dependResist[(max_n+1)]; // создаем калибровочную таблицу на max_n значений

uint64_t time = 0;  // наше собственное время, 64бит должно хватить на пару миллиардов лет


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
void lcd_array( char x, char y, const char *str )
{
    go_to(x,y);
    while( *str )           //цикл пока указатель существует
    lcd_dat( *str++ );  //выводить строку
}

char upper_line[17];        //Массив для верхней строки LCD дисплея
char lower_line[17];        //Массив для нижней строки LCD дисплея
int dot_pos = 0;            //Текущая позиция точки на экране
int dot_pos_counter = 0;    //Счётчик для функционирования изменения скорости перемещения точки
int speed = 7;              //Скорость движения точки

//------------------------------------------------------------------------------------------------------------------

void Timer_Init()
{
    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 9,766 kHz
    // Mode: CTC top=OCR1A
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // OC1C output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer Period: 0,1024 ms
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    // Compare C Match Interrupt: Off
    TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (1<<WGM11) | (0<<WGM10);
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
    
    TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);
    ETIMSK=(0<<TICIE3) | (0<<OCIE3A) | (0<<OCIE3B) | (0<<TOIE3) | (0<<OCIE3C) | (0<<OCIE1C);
    
    EICRA=(0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
    EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
    EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);

    
    TIMSK |= (1<<TOIE1); // разрешаем прерывание по переполнению таймера
    TCNT1 = 64456;        // выставляем начальное значение TCNT1
    
    // USART0 initialization
    // USART0 disabled
    UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
};

// Timer1 overflow interrupt service routine
//interrupt [TIM1_OVF] void timer1_ovf_isr(void)
ISR(TIMER1_OVF_vect)
{
    // просигналим портом, что наш таймер вообще работает
    if (PORTB == 1) {
        PORTB = 0;
    }
    else {
        PORTB = 1;
    }
    
    time++;  // инкрементировать наш собственный счетчик времени, по которому из главного цикла делать ШИМ
}


int main(void) {
    DDRC = 0xFF;    //Порт C - выход (подключен LCD дисплей)
    PORTC = 0;
    
    // глобально запретим прерывания
    cli();
    lcd_init(); //Инициализация дисплея
    
    DDRA = 0x00; // порта А полностью на ввод
    PORTA = 0x00;
    
    DDRB = 0xFF;
    PORTB = 0x00;
    
    DDRD = 0xFF;
    PORTD = 0x00;
    
    adc_init(3); // подключим АЦП к выводу PF3
    sei();  // глобально разрешим прерывания
    
    int data;
    char screen_for_temp = 'T';
    char sreen_for_ust = 'F';
    uint8_t ust = 30;
    
    Timer_Init();
    InitStruct();
    
    while (1) {
        float datADC = 1023 - read_adc(3);
        
        // float datADC = 470;
        // data = 0.0000000268 * pow(datADC, 4) - 0.00004827 * pow(datADC, 3) + 0.031424 * pow(datADC, 2) - 8.404671 * datADC + 801.582; //полином для преобразования температуры
        
        data = S3x(datADC); 
        
        // переключение экранов
        // todo при переключении экранов обнулять upper_line и lower_line, иначе могут быть артефакты
        if (PINA & (1<<0)) {
            screen_for_temp = 'F';
            sreen_for_ust   = 'T';
        } 
        else if (PINA & (1<<1)) {
            screen_for_temp = 'T';
            sreen_for_ust   = 'F';
        };
        
        if (PINA & (1<<2)) {
            ust = ust + 1;
        } else if (PINA & (1<<3)) {
            ust = ust - 1;
        };

        if (screen_for_temp == 'T'){
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
        
        if (sreen_for_ust == 'T'){
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
        
        lcd_array(1,0,upper_line);  //Вывод массива верхней строки на дисплей
        lcd_array(0,1,lower_line);  //Вывод массива нижней строки на дисплей
        
        _delay_ms(100);
        
        // OCR1A = (uint16_t)(0.2 * 0x3FF);
        // OCR1AL = 0x3FF;
        OCR1A = 64000;  // значение таймера при котором сработает прерывание
    }
}

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input){
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


float l[max_n];

void Spline_progonka(void){
    float mu[max_n];

    l[1] = 0;
    mu[1] = 0;
    
    for (uint8_t i = 2; i < max_n; i++){    
        float sig = (dependResist[i].code - dependResist[i-1].code ) / (dependResist[i+1].code - dependResist[i-1].code);   
        float p = sig * l[i-1]+2;
        l[i] = (sig - 1) / p;
        mu[i] = (dependResist[i+1].temp - dependResist[i].temp) / (dependResist[i+1].code - dependResist[i].code) - 
                (dependResist[i].temp - dependResist[i-1].temp) / (dependResist[i].code - dependResist[i-1].code);
        mu[i] = (6* mu[i] / ( dependResist[i+1].code - dependResist[i-1].code) - sig * mu[i-1]) / p;
    }   
    l[max_n] = 0;       
    for (int i = max_n-1; i > 0; i--){
        l[i] = l[i] * l[i+1] + mu[i];
    }
}

int S3x(double datADC1){
    Spline_progonka();
    
    int klo = 1;
    int khi = max_n;
    int k = max_n;

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
                ((( pow(a,3) - a) * l[klo] + (pow(b,3) - b) * l[khi])) * (h * h) / 6  );    
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