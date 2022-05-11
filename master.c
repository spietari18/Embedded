/* master.c
 *
 * Created: 09/03/2022
 * Author: Samuli, Robert
 * Using Arduino MEGA as master
 * ATmega2560
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include "alarmstates.h"
#include "keypad/keypad.h"

#define F_CPU 16000000UL
#define FOSC 16000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)

#define SLAVE_ADDRESS 85 // 0b1010101

// Global variables
int pw_index = 0;        // Password input arrays index

static void
USART_init(uint16_t ubrr)
{
    /* Set baud rate in the USART Baud Rate Registers (UBRR) */
    UBRR0H = (unsigned char) (ubrr >> 8);
    UBRR0L = (unsigned char) ubrr;
    
    /* Enable receiver and transmitter on RX0 and TX0 */
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //NOTE: the ATmega2560 has 4 UARTs: 0,1,2,3
    
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
    
    /* Puts the data into a buffer, then sends/transmits the data */
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

int 
TWI_transmit(char twi_send_data)
{
    // Initialize variables
    uint8_t twi_index = 0;
    uint8_t twi_status = 0;

    // Start transmission by sending START condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); 
    // TWCR = (1 << 7) | (1 << 5) | (1 << 2); 
    
    // wait for the TWINT to set
    while (!(TWCR & (1 << TWINT))) 
    {
        ;    
    }
    
    // read the status from TWI status register, 0xF8 is used to mask prescaler bits so that 
    // only the status bits are read
    twi_status = (TWSR & 0xF8); 
    
    // print the status bits to the UART for monitoring
    itoa(twi_status, test_char_array, 16);
    printf(test_char_array);
    printf(" ");
    
    // Send slave address and write command to enter MT mode
    TWDR = 0b10101010; // load slave address and write command
    // Slave address = 85 + write bit '0' as a LSB ---> 170 
    
    // clear TWINT to start transmitting the slave address + write command
    TWCR = (1 << TWINT) | (1 << TWEN); 
    // TWCR = (1 << 7) | (1 << 2);
    
    // wait for the TWINT to set
    while (!(TWCR & ( 1<< TWINT)))
    {
        ;
    }
    
    // read the status from TWI status register, 0xF8 is used to mask prescaler bits so that
    // only the status bits are read
    twi_status = (TWSR & 0xF8);
    
    itoa(twi_status, test_char_array, 16);
    printf(test_char_array);
    printf(" ");
    
    // transmit data to the slave
    for(int8_t twi_data_index = 0; twi_data_index < sizeof(twi_send_data); twi_data_index++)
    {
                
        TWDR = twi_send_data[twi_data_index]; // load data 
        
        // "clear" TWINT to start transmitting the data
        TWCR = (1 << TWINT) | (1 << TWEN);
        // TWCR = (1 << 7) | (1 << 2);
        
        // wait for the TWINT to set
        while (!(TWCR & ( 1<< TWINT)))
        {
            ;
        }

        // read the status from TWI status register, 0xF8 is used to mask prescaler bits so that
        // only the status bits are read
        twi_status = (TWSR & 0xF8);
        itoa(twi_status, test_char_array, 16);
        printf(test_char_array);
        printf(" ");
        
    }
    
    // stop transmission by sending STOP
    TWCR = (1 << TWINT) | (1 << TWSTO) |(1 << TWEN);
    //TWCR = (1 << 7) | (1 << 4) |(1 << 2);
    printf("\n");
    
    return 0;
}

ISR (TIMER3_COMPA_vect) {
    pw_index = 0;        // Reset password input arrays index

    // Send PW_TIMEOUT to slave

    printf("PW_TIMEOUT\n");

}

// Interrupt for PIR sensor
ISR (INT3_vect) {
    // Send ALARM_TRIGGER to slave

    printf("ALARM TRIGGERED");
}

int main(void) {
    // SETUP
    // DEFINE VARIABLES
    char PW_INPUT[10];      // Array to save password inputted
    char PASSWORD[9];       // Array for password

    // SETUP PINS
    // PIR
    DDRD &= ~(1 << PD3);        // PIN 18 TX1

    // Initialize keypad
    KEYPAD_Init();

    // READ PASSWORD FROM EEPROM

    // Initialize TWI 
    // set SCL frequency to 400 kHz, using equation in p. 242 of ATmega2560 datasheet
    TWSR = 0x00; // TWI status register prescaler value set to 1
    TWBR = 0x03; // TWI bit rate register.
    TWCR |= (1 << TWEN); // enable TWI

    // Enable interrupts
    sei();

    // INITIALIZE INTERRUPT FOR MOTION SENSOR
    // Clear interrupt register
    EIMSK = 0b00000000;
    // Set trigger to rising edge
    ISC31 = 1;
    ISC30 = 1;
    // Enable interrupt 3
    EIMSK = 0b00001000;
    SREG = 1;

    // INITIALIZE INTERRUPT FOR KEYPAD TIMEOUT
    DDRE |= (1 << PE3); // OC3A is located in digital pin 5
    // Reset timer 3
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;

    // Initialize timer
    TCCR3A = 0x02;          // CTC
    TCCR3B = 0b00000101;    // 1024 prescaling
    OCR3A = 78125;          // TOP value
    TIMSK3 |= (1 << 1);     // Enable compare match A interrupt

    // Setup the stream functions for UART
    FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
    FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

    // initialize the UART with 9600 BAUD
    USART_init(MYUBRR);
    
    // redirect the stdin and stdout to UART functions
    stdout = &uart_output;
    stdin = &uart_input;

    printf("DONE!\n");
    while(1) {
        // LOOP

        // Read from keypad, save to array
        PW_INPUT[pw_index] = KEYPAD_GetKey();

        if (pw_index == 0) {
            // Initialize timer after first keypress is registered

        }

        // Increase index by one
        pw_index++;

        // IF SUBMIT
        printf("Submit pressed!\n");
        if (PW_INPUT[pw_index -1] == "#") {
            // Compare password
            for (int i = 0; i < pw_index -1; i++) {
                if (PW_INPUT[i] != PASSWORD[i]) {
                    // Password wrong
                    // Send PW_WRONG to slave

                    printf("PW_WRONG!\n");

                    break;
                }
                // Last index of password
                else if (i == pw_index -2) {
                    // Password correct
                    // Send PW_OK to slave

                    printf("PW_OK!\n");

                    // Send ALARM_OFF to slave
                    
                    printf("ALARM_OFF\n");
                }
            }
            // Clear inputted password by setting index to 0
            pw_index = 0;
        }

        // If clear
        if (PW_INPUT[pw_index -1] == "D") {
            pw_index = 0;
            printf("PW_CLEAR!");
        }
    }
}