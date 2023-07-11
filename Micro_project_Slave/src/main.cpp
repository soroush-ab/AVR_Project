#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define LCD_Port PORTC
#define ctrl PORTA

#define en PA0
#define rw PA1  // for select write or read opration
#define rs PA2  // for select Data or command

#define Vref 5

#define SelPort PD4
#define RedLed PD3
#define BlueLed PD2

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

char Key[16] = {83, 111, 114, 111, 117, 115, 104, 65, 98, 100, 111, 108, 108, 97, 104, 105};

char Password[] = "1234";
bool ShowPass = false;
bool access = false;
bool isPer = true;
char InPass[20];
int charNum = 0;

// persian charector
char cR[] = {0,0,2,2,4,8,0,0}; 
char cZ[] = {2,0,2,3,4,8,0,0};
char cM[] = {0,14,10,30,0,0,0,0};
char cO[] = {0,0,14,10,15,2,14,0};
char cF[] = {4,0,14,10,30,0,0,0};
char cGH[] = {5,0,7,21,23,20,28,0};
char cNA[] = {0,8,8,10,8,10,14,0};

// LCD functions
void LCD_CMD(char cmd);
void initial_LCD();
void LCD_Write(char chr);
void PrintLCD(char *str);
void lcd_custom_char(char location, char *data);
void ShowPass_Per();

// password checking
void SpecialChar(char chr);
void ShowPasswordToggle();
void checkADC();

// usart functions
void UART_init(long USART_BAUDRATE);
unsigned char UART_RxChar();
void UART_TxChar(char ch);
void UART_SendString(char *str);

// pwn functions
void PWM_init();
void checkLight(int light);
void checkTemp(int temp);

// encrypt and 
char encrypt(char *key, char data);
char decrypt(char *key, char data);

int main() {
    initial_LCD();
    UART_init(9600);
    LCD_CMD(0x0C);
    PWM_init();

    // test
    ShowPass_Per();

    // PrintLCD("pass: ");

    while (1) {
        if(!access){
            char MSData = UART_RxChar();
            SpecialChar(MSData);
        }
        else
            checkADC();
        }

}

void PWM_init()
{
    // phase currect pwm, clear on compare, clk no prescale
	TCCR0 = (1<<WGM00) | (1<<COM01) | (1<<CS00);
    TCCR2 = (1<<WGM20) | (1<<COM21) | (1<<CS20);
	DDRB |= (1<<PB3);  // set oc0 output
    DDRD |= (1<<PD7) | (1<<SelPort) | (1<<RedLed) | (1<<BlueLed);  
    // set oc2 output and port d4, d3, d2 output for select temp motor, Redled, blueled

    OCR0 = 0;
    OCR2 = 0;

}

void UART_init(long USART_BAUDRATE)
{
	UCSRB |= (1 << RXEN) | (1 << TXEN);	/* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit char size */
	UBRRL = BAUD_PRESCALE;			/* Load lower 8-bits of the baud rate */
	UBRRH = (BAUD_PRESCALE >> 8);		/* Load upper 8-bits*/
}

unsigned char UART_RxChar()
{
	while ((UCSRA & (1 << RXC)) == 0);/* Wait till data is received */
	return decrypt(Key,UDR);			/* Return the byte*/
}

void UART_TxChar(char ch)
{
    ch = encrypt(Key, ch);
	while (! (UCSRA & (1<<UDRE)));	/* Wait for empty transmit buffer*/
	UDR = ch ;
}

void UART_SendString(char *str)
{
	unsigned char j=0;
	
	while (str[j]!=0)		/* Send string till null */
	{
		UART_TxChar(str[j]);	
		j++;
	}
}

void initial_LCD() {
    DDRC = 0xff;
    DDRA |= (1 << en) | (1 << rs) | (1 << rw);

    LCD_CMD(0x38);  // 8-bit mode
    _delay_ms(1);

    LCD_CMD(0x01);  // cmd to clear lcd
    _delay_ms(1);

    LCD_CMD(0x02);  // return home
    _delay_ms(1);

    LCD_CMD(0x06);
    _delay_ms(1);

    LCD_CMD(0x80);  // Force cursor to the beginning
    _delay_ms(1);

    return;
}

void lcd_set_cursor(unsigned char row, unsigned char col) {
    unsigned char address;
    if (row == 0) {
        address = col;
    } else {
        address = 0x40 + col;
    }
    LCD_CMD(0x80 + address);
}

void ShowPass_Per(){
    int coursor = 15;
    LCD_CMD(0x01);

    lcd_custom_char(0, cR);
    lcd_custom_char(1, cM);
    lcd_custom_char(2, cZ);

    lcd_set_cursor(0,coursor--);
    LCD_Write(0);
    lcd_set_cursor(0,coursor--);
    LCD_Write(1);
    lcd_set_cursor(0,coursor--);
    LCD_Write(2);
    lcd_set_cursor(0,coursor--);
    LCD_Write(':');

    if(ShowPass)
        for (int i = charNum-1 ; i>=0 ; i--){
            lcd_set_cursor(0,coursor--);
            LCD_Write(InPass[i]);
        }
    
    else
        for (int i = 0; i < charNum; i++)
        {
            lcd_set_cursor(0, coursor--);
            LCD_Write('*');
        }

}

void AccessPer(bool Acc){
    int coursor = 15;
    LCD_CMD(0x01);

    lcd_custom_char(0, cM);
    lcd_custom_char(1, cO);
    lcd_custom_char(2, cF);
    lcd_custom_char(3, cGH);
    //lcd_custom_char(5, cNA);

     if(!Acc){
        lcd_set_cursor(0,coursor--);
        LCD_Write(5);
    }

    lcd_set_cursor(0,coursor--);
    LCD_Write(0);
    lcd_set_cursor(0,coursor--);
    LCD_Write(1);
    lcd_set_cursor(0,coursor--);
    LCD_Write(2);
    lcd_set_cursor(0,coursor--);
    LCD_Write(3);

}

void LCD_CMD(char cmd) {
    LCD_Port = cmd;

    ctrl &= ~(1 << rs);
    ctrl &= ~(1 << rw);
    ctrl |= (1 << en);

    _delay_ms(10);

    ctrl &= ~(1 << en);

    return;
}

void LCD_Write(char chr) {
    LCD_Port = chr;

    ctrl |= (1 << rs);
    ctrl &= ~(1 << rw);
    ctrl |= (1 << en);

    _delay_ms(1);

    ctrl &= ~(1 << en);

    return;
}

void lcd_custom_char(char location, char *data)
{
    unsigned char i;
    if(location<8)
    {
     LCD_CMD(0x40 + (location*8));  /* Command 0x40 and onwards forces 
                                       the device to point CGRAM address */
       for(i=0;i<8;i++)  /* Write 8 byte for generation of 1 character */
           LCD_Write(data[i]);      
    }   
}

void PrintLCD(char *str) {
    for (int i = 0; str[i] != '\0'; i++)
        LCD_Write(str[i]);
}

void ShowPasswordToggle() {
    //clear lcd
    LCD_CMD(0x01);

    if(! isPer){

         PrintLCD("pass: ");

        if (ShowPass) {
            for (int i = 0; i < charNum; i++)
                LCD_Write('*');

            _delay_ms(5);

            ShowPass = false;
        } else {
            PrintLCD(InPass);
            ShowPass = true;
        }
    }
    else{
        ShowPass = !ShowPass;
        ShowPass_Per();
    }
}

bool checkPass(){
  for (int i = 0; Password[i] != '\0'; i++) {

                if (Password[i] != InPass[i] || InPass[i] == '\0') {
                    LCD_CMD(0x01);

                    // reset password
                    InPass[0] = '\0';
                    charNum = 0;

                    if(!isPer){
                       PrintLCD("Wrong password");
                       _delay_ms(50);
                        LCD_CMD(0x01);
                       PrintLCD("pass: ");
                    }

                    else{
                        AccessPer(false);
                        _delay_ms(50);
                        LCD_CMD(0x01);
                        ShowPass_Per();
                    }

                    UART_TxChar('n');

                    return false;
                }
            }
            LCD_CMD(0x01);

            if(!isPer)
                PrintLCD("Access is granted");

            else
                AccessPer(true);

            _delay_ms(50);

            UART_TxChar('g');
            
            LCD_CMD(0x01);
            return true;
}

void DeleteChar(){
  if (charNum != 0) {
                // delete last char
                InPass[--charNum] = '\0';
                LCD_CMD(0x01);

                if(!isPer){

                    PrintLCD("pass: ");

                    if (!ShowPass) {
                        for (int i = 0; i < charNum; i++)
                            LCD_Write('*');

                        _delay_ms(5);
                    } else
                        PrintLCD(InPass);
                }

                else{
                    ShowPass_Per();
                }

            }
}

void AddChr(char chr){
  if (chr <= '9' && chr >= '0' && !access) {
                InPass[charNum++] = chr;
                InPass[charNum] = '\0';

                if(!isPer){

                    if (ShowPass)
                        LCD_Write(chr);

                    else
                        LCD_Write('*');
                }
                else{
                    ShowPass_Per();
                }
            }
}

void SpecialChar(char chr) {
    switch (chr) {
        case 'p':
            ShowPasswordToggle();
            UART_TxChar('n');
            break;

        case 'l':
            LCD_CMD(0x01);
            PrintLCD("pass: ");

            if (isPer){
                if (!ShowPass) {
                    for (int i = 0; i < charNum; i++)
                        LCD_Write('*');

                    _delay_ms(5);
                } else
                    PrintLCD(InPass);
                
                isPer = false;
            }

            else{
                isPer = true;
                ShowPass_Per();
            }

            UART_TxChar('n');
            break;


        case '*':
            access = checkPass();
            break;

        case '#':
            DeleteChar();
            UART_TxChar('n');
            break;

        default:
            AddChr(chr);
            UART_TxChar('n');
            break;
    }
}

void checkTemp(int temp){
    int duty = 0;

    if(temp>=25 && temp<55){
        PORTD &= ~(1<<SelPort);
        duty = 50;
        duty += (int((temp-25)/5))*10;
    }

    else if(temp>0 && temp<=20){
        PORTD |= (1<<SelPort);
        duty = 100;
        duty -= (int(temp/5))*25;
    }

    else if(temp>=55){
        PORTD |= (1<<RedLed);
        _delay_ms(10);
        PORTD &= ~(1<<RedLed);
    }

    else if(temp==0){
        PORTD |= (1<<BlueLed);
        _delay_ms(10);
        PORTD &= ~(1<<BlueLed);
    }

    OCR2 = duty*2.55;
}

void checkLight(int light){
    int duty = 0;
    if(light>=0 && light<=100){
        duty = 100;
        duty -= (int(light / 25)) * 25;
    }

    OCR0 = duty*2.55;
}

void checkADC(){

    unsigned char adc_value1, adc_value2;
    float voltage;
    unsigned int temp, light;
    
    // Receive ADC values through UART
    adc_value1 = UART_RxChar();
    _delay_ms(50);
    adc_value2 = UART_RxChar();  

    temp = adc_value1; 
    voltage = (temp) * Vref * 100 / 1024 ;
    temp = voltage;

    char strtemp[10];
    sprintf(strtemp, "temp: %d", int(temp));

    light = adc_value2;
    voltage = light * Vref * 100 / 1024;
    light = voltage;

    char strlight[10];
    sprintf(strlight, "light: %d", int(light));

    PrintLCD(strtemp);
    LCD_CMD(0xC0);
    PrintLCD(strlight);

    checkTemp(temp);
    checkLight(light);

     _delay_ms(5);

    LCD_CMD(0x01);
}

char encrypt(char *key, char data){
    char res = data;
    for (int i = 0; i < 16; i++)
    {
        res ^= key[i];
    }

    return res;
    
}

char decrypt(char *key, char data){
    char res = data;
    for (int i = 15; i >=0; i--)
    {
        res ^= key[i];
    }

    return res;
    
}