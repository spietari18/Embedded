/* slave.c
 *
 * Created: 09/03/2022
 * Author: Samuli, Robert
 * Using Arduino UNO as slave
 * Atmega328p
 */

/*
#define ALARM_OFF 0
#define ALARM_ON 1
#define ALARM_TRIGGER 2
#define PW_OK 3
#define PW_WRONG 4
#define PW_TIMEOUT 5
*/

#define F_CPU 16000000UL
#define FOSC 16000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)

#define SLAVE_ADDRESS 85 // 0b1010101

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <util/delay.h>
#include "alarmstates.h"
#include "lcd.h"
#include <avr/interrupt.h>

ISR
(TIMER1_COMPA_vect)
{
    TCNT1 = 0; // reset timer counter
}


static void
USART_init(uint16_t ubrr) // unsigned int
{
    /* Set baud rate in the USART Baud Rate Registers (UBRR) */
    UBRR0H = (unsigned char) (ubrr >> 8);
    UBRR0L = (unsigned char) ubrr;
    
    /* Enable receiver and transmitter on RX0 and TX0 */
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //NOTE: the ATmega328p has 1 UART: 0
    
    /* Set frame format: 8 bit data, 2 stop bit */
    UCSR0C |= (1 << USBS0) | (3 << UCSZ00);
    
}

static void
USART_Transmit(unsigned char data, FILE *stream)
{
    /* Wait until the transmit buffer is empty*/
    while(!(UCSR0A & (1 << UDRE0)))
    {
        ;
    }
    
    /* Put the data into a buffer, then send/transmit the data */
    UDR0 = data;
}

static char
USART_Receive(FILE *stream)
{
    /* Wait until the transmit buffer is empty*/
    while(!(UCSR0A & (1 << UDRE0)))
    {
        ;
    }
    
    /* Get the received data from the buffer */
    return UDR0;
}

int main(void) {
    // SETUP
    // DEFINE VARIABLES
    char twi_receive_data = '9';
    uint8_t twi_status = 0;
    
    int message = 0;
    int last_message = 0;
    int alarm = 0;
    
    // Initialize lcd
    lcd_init(LCD_DISP_ON);
    lcd_clrscr();

    // Initialize buzzer pwm
    /* set up the ports and pins */
    DDRB |= (1 << PB1); // OC1A is located in digital pin 9
    
    // Enable interrupts
    sei();

     /* set up the 16-bit timer/counter1, mode 9 used */
    TCCR1B  = 0; // reset timer/counter 1
    TCNT1   = 0;
    TCCR1A |= (1 << 6); // set compare output mode to toggle

    // mode 9 phase correct
    TCCR1A |= (1 << 0); // set register A WGM[1:0] bits
    
    TCCR1B |= (1 << 4); // set register B WBM[3:2] bits

    TIMSK1 |= (1 << 1); // enable compare match A interrupt
    OCR1A = 2462;

    
    
    // Initialize USART
    FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
    FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

    // initialize the UART with 9600 BAUD
    USART_init(MYUBRR);

     // redirect the stdin and stdout to UART functions
    stdout = &uart_output;
    stdin = &uart_input;

    // INITIALIZE TWI
    // slave address
    TWAR = 0b10101010; // same as 170 DEC
    // Slave address 85 + TWI General Call Recognition Enable Bit '0' LSB ---> 170
    
    // Initialize TWI slave
    TWCR |= (1 << TWEA) | (1 << TWEN);
    TWCR &= ~(1 << TWSTA) & ~(1 << TWSTO);

    while(1) {
        // LOOP
        if (alarm == 1) {
            //printf("Alarm should beep\n");
            TCCR1B |= (1 << 0);     // Start timer, prescaler 0
            //_delay_ms(250);
            //TCCR1B |= (0 << 0);
        } else {
            TCCR1B &= 0b11111000;   // Stop timer
        }
        // LISTEN FOR MASTER
        if ((TWCR & (1 << TWINT))){
            printf("Receiving...\n");
            // get TWI status
            twi_status = (TWSR & 0xF8);	
            
            // "clear" TWINT and generate acknowledgment ACK (TWEA)
            TWCR |=  (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            
            // wait for the the TWINT to set
            while(!(TWCR & (1 << TWINT)))
            {
                ;
            }
            
            // get TWI status
            twi_status = (TWSR & 0xF8);

            // if status indicates that previous response was either slave address or general call and ACK was returned
            // store the data register value to twi_receive_data
            if((twi_status == 0x80) || (twi_status == 0x90))
            {
                twi_receive_data = TWDR;
            }
            else if((twi_status == 0x88) || (twi_status == 0x98))        
            {           
                // if status indicates that previous response was either slave address or general call and NOT ACK was returned
                // store the data register value to twi_receive_data
                twi_receive_data = TWDR;
            }            
            else if(twi_status == 0xA0)
            {
                // Stop condition or repeated start was received
                // Clear interrupt flag
                TWCR |= (1 << TWINT);
            }
            printf("Received: %c\n", twi_receive_data);
            message = twi_receive_data - '0';
        }

        // IF MESSAGE
        if(message != last_message){
            printf("Got new message!\n");
            printf("%d\n", message);
            last_message = message;
            // STATE MACHINE TO HANDLE SIGNALS
            switch(message) {
                case 0: // ALARM_OFF
                    lcd_gotoxy(0,0);
                    lcd_puts("Disarmed       ");
                    alarm = 0;
                    break;

                case 1: // ALARM_ON
                    lcd_gotoxy(0,0);
                    lcd_puts("Armed          ");
                    alarm = 0;
                    break;

                case 2: // ALARM_TRIGGER
                    lcd_gotoxy(0,0);
                    lcd_puts("Alarm triggered");
                    alarm = 1;
                    break;
                
                case 3: // PW_OK
                    lcd_gotoxy(0,1);
                    lcd_puts("Password: Ok    ");
                    break;

                case 5: // PW_TIMEOUT
                    lcd_gotoxy(0,1);
                    lcd_puts("Password Timeout");
                    break;

                case 4: // PW_WRONG:
                    lcd_gotoxy(0,1);
                    lcd_puts("Password: Wrong ");
                    break;

                default:
                    printf("None of these options...\n");
                    break;
            }
            //_delay_ms(100);
        }
    }
}