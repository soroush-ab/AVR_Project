#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ADC0 0 
#define ADC1 1 
#define ADC2 2 
#define ADC3 3 
#define ADC4 4 
#define ADC5 5 
#define ADC6 6
#define ADC7 7

#define Temp ADC0
#define Light ADC1

#define Vref 5.0

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define KEY_PORT    PORTC
#define KEY_DDR     DDRC
#define KEY_PIN     PINC
 
#define C1  4
#define C2  5
#define C3  6

#define PassToggle PD3

const unsigned char Delay = 5;
bool PressKey = false;
bool access = false;

char Key[] = {83,111,114,111,117,115,104,65,98,100,111,108,108,97,104,105};

// ADC function
void ADC_Init();
int ADC_Read(unsigned char input_channel);

// keypad functions
void keypad_init(void);
char key_scan(void);
unsigned char key_pressed(void);
unsigned char key_released(void);

// usart functions
void UART_init(long USART_BAUDRATE);
unsigned char UART_RxChar();
void UART_TxChar(char ch);
void UART_SendString(char *str);

// Password Hide
void PasswordLanToggle_Init();
ISR(INT0_vect){
	if(!access)
		UART_TxChar('l');

	while (PD2 == 0);	
}

ISR(INT1_vect){
	// send data to slave to toggle password.
	if(!access)
		UART_TxChar('p'); 
	
	while (PD3 == 0);	
}

// encrypt and 
char encrypt(char *key, char data);
char decrypt(char *key, char data);

int main(){
	ADC_Init();
	keypad_init();
	UART_init(9600);
	PasswordLanToggle_Init();

	while (1){
		if(!access){
			char keydata = key_scan();
			if(PressKey){
				UART_TxChar(keydata);
				PressKey = false;
				char check = UART_RxChar();

				if( check == 'g'){
					// access granted
					access = true;
				}
			}
		}

		else{
			int temp, light;
			temp = ADC_Read(ADC0);
			_delay_ms(10);
			light = ADC_Read(ADC1);
			_delay_ms(10);

			UART_TxChar(temp);
			_delay_ms(50);
			UART_TxChar(light);
			_delay_ms(30);
		}
	}	
}

void ADC_Init(){
	DDRA=0x00;			/* Make ADC port as input */
	ADCSRA = 0x87;			/* Enable ADC, fr/128  */
	ADMUX = 0x40;
}

void adc_inputChannel(unsigned char input_channel)
{
	ADMUX &= 0xC0; // clear
	
	switch(input_channel)
	{
		case ADC0:
			ADMUX |= 0x00;
			break;
		case ADC1:
			ADMUX |= (1<<MUX0);
			break;
		case ADC2:
			ADMUX |= (1<<MUX1);
			break;
		case ADC3:
			ADMUX |= (1<<MUX0) | (1<<MUX1);
			break;
		case ADC4:
			ADMUX |= (1<<MUX2);
			break;
		case ADC5:
			ADMUX |= (1<<MUX0) | (1<<MUX2);
			break;
		case ADC6:
			ADMUX |= (1<<MUX1) | (1<<MUX2);
			break;
		case ADC7:
			//ADMUX |= 0x07;
			ADMUX |= (1<<MUX0) | (1<<MUX1) | (1<<MUX2);
			break;
		default:
			ADMUX |= (0x1F & input_channel);
	}
	return;
}

int ADC_Read(unsigned char input_channel){

  int LSB, MSB, analogReading;

  adc_inputChannel(input_channel);
  ADCSRA |= (1<<ADSC);    // start conversion
  while((ADCSRA & (1<<ADIF)) == 0);   // wait untill conversion complete
  //or while((ADCSRA & (1<<ADSC)) == 1);
  LSB = (int)ADCL;
  MSB = (int)ADCH;
  analogReading = LSB + (MSB*256);

  //uint8_t res = (ADCL + (ADCH<<8));
//   temp = uint8_t(voltage*100);
  return analogReading;  
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
	return decrypt(Key, UDR);			/* Return the byte*/
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

unsigned char key_released() {  
    KEY_PORT = 0xf0;
    _delay_us(Delay);              
    if((KEY_PIN & 0xf0) == 0xf0)
        return 1;
    else
        return 0;
}

unsigned char key_pressed() {
    KEY_PORT = 0xf0;
    _delay_us(Delay);
    if( (KEY_PIN & 0xf0) != 0xf0 ) { // User presses some key
        return 1;
    }
    return 0;
}

char key_scan() {

	char table [] = {'1','2','3',
					 '4','5','6',
					 '7','8','9',
					 '*','0','#',}; 
    unsigned char i, key; 
    if(key_pressed()){
		PressKey = true;
        for(i = 0; i < 4; i++){
            KEY_PORT = ~(1 << i); 
            _delay_us(Delay);
 
            if(((KEY_PIN >> C1) & 1) == 0)     key = table[i*3];
 
            if(((KEY_PIN >> C2) & 1) == 0)     key = table[i*3+1];
 
            if(((KEY_PIN >> C3) & 1) == 0)     key = table[i*3+2];            
        }              
        while(!key_released());     
        return key;              
    }
    
    else 
        return 255;
 
}

void keypad_init(){
    KEY_DDR = 0x0f;
    KEY_PORT = 0xf0;
}

void PasswordLanToggle_Init(){
	DDRD = 0x00; 				// port D is input for key pad and password toggle switch.
	PORTD |= (1<<PassToggle) | (1<<PD2);	// enable pullup for toggle switch and change language
	GICR |= (1 << INT1) | (1 << INT0);		// set INT1
	MCUCR |= (1 << ISC11) | (1 << ISC01);		// falling edge
	sei();
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