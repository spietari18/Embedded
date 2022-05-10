/* slave.c
 *
 * Created: 09/03/2022
 * Author: Samuli, Robert
 * Using Arduino MEGA as slave
 * Atmega2560
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include "alarmstates.h"

#include "lcd.h" // lcd header file made by Peter Fleury


int main(void) {
    // SETUP
    // DEFINE VARIABLES
    lcd_init(LCD_DISP_ON);
    lcd_clrscr();


    int alarm = 0;
    // SETUP PINS
    
    // INITIALIZE I2C OR SPI

    while(1) {
        // LOOP

        // LISTEN FOR MASTER

        // IF MESSAGE
            lcd_clrscr()
            // STATE MACHINE TO HANDLE SIGNALS
            switch(message) {
                case ALARM_OFF:
                    lcd_puts("Disarmed");
                    alarm = 0;
                    break;

                case ALARM_ON:
                    lcd_puts("Armed");
                    break;

                case ALARM_TRIGGER:
                    lcd_puts("Alarm triggered");
                    alarm = 1;
                    break;
                
                case PW_OK:
                    lcd_puts("Password Accepted");
                    break;

                case PW_TIMEOUT:
                    lcd_puts("Timeout");
                    break;

                case PW_WRONG:
                    lcd_puts("Password Declined");
                    break;

                default:
            }

    }
}