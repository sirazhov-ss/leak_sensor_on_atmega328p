#define F_CPU 16000000UL
#define BAUDRATE (F_CPU/(16*4800L))-1
#define WAIT 100
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int stop_key = 0;
int leak;
int lb;
int counter;
int send = 0;


void PIN_Init(void){
	DDRD &= ~((1<<PD3) | (1<<PD2));
	DDRD |= (1<<PD4) | (1<<PD5);
	PORTD &= ~((1<<PD5) | (1<<PD4) | (1<<PD3) | (1<<PD2));
	SMCR |= 1<<SE;
	SMCR |= 1<<SM1;
//	EIMSK |= 1<<INT1;
//	EICRA &= ~((1<<ISC11) | (1<<ISC10));
}

void UART_Init(unsigned int baud){
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	UCSR0B |= (1<<RXCIE0);
}


void UART_Transmit(unsigned char data){
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

unsigned char UART_Receive(void){
	while (!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void send_message(char* message){
	int length = strlen(message);
	for (int i = 0; i < length; i++) {
		UART_Transmit(message[i]);
	};
	UART_Transmit('\n');
}

void BT_ON(void){
	PORTD |= ((1<<PD5) | (1<<PD4));
	
}

void BT_OFF(void){
	PORTD &= ~((1<<PD5) | (1<<PD4));
}

int get_state(int bit){
	int state = -1;
	if (~PIND & (1<<bit)) {
		state = 1;
	}
	else {
		state = 0;
	}
	return state;
}

ISR(INT0_vect){
	EIMSK &= ~(1<<INT0);
	SMCR &= ~(1<<SE);
	send = 1;
	stop_key = 0;
	counter = WAIT;
	BT_ON();
}

ISR(INT1_vect){
	EIMSK &= ~(1<<INT1);
	SMCR &= ~(1<<SE);
	send = 1;
	lb = 1;
	stop_key = 0;
	counter = WAIT;
	BT_ON();
}

ISR(WDT_vect){
	if (PIND & (1<<PD2)) {
		MCUCR &= ~(1<<WDRF);
		WDTCSR &= ~((1<<WDE) | (1<<WDIE));
		send = 1;
		stop_key = 0;
		counter = WAIT;
		BT_ON();
	}
	else {
		for (int i=0; i<4; i++){
			PORTD ^= 1<<PD5;
			_delay_ms(200);
		}
	}
	asm("wdr");
}

ISR(USART_RX_vect){
	stop_key = UDR0;
}

int main(void)
{
	PIN_Init();
	UART_Init(BAUDRATE);
	sei();
	char* response[2] = {"No leak!", "Leak is detected!"};
//	lb = get_state(3);
	lb = 0;
	send = 1;
	BT_ON();
	while (1){
		if (send == 1) {
			leak = get_state(2);
			if (stop_key != '1' && counter == WAIT) {
				if (lb == 1) {
					send_message("Low battery!");
				}
				send_message(response[leak]);
				if (counter <= 0) {
					counter = WAIT;
				}
			}
			counter--;
			if (stop_key == '1') {
				BT_OFF();
				leak = -1;
				send = 0;
				lb = 0;
				if (~PIND & (1<<PD2)) {
					MCUCR |= 1<<WDRF;
					WDTCSR |= (1<<WDCE) | (1<<WDE);
					WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
					SMCR |= 1<<SE;
					SMCR |= 1<<SM1;
				}
				else {
					EIMSK |= 1<<INT0;
					EICRA &= ~((1<<ISC01) | (1<<ISC00));
					SMCR |= 1<<SE;
					SMCR |= 1<<SM1;
				}
				
			}
		}
		asm("sleep");
	}
}