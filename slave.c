/* slave.c
 *
 * Created: 09/03/2022
 * Author: Samuli, Robert
 * Using Arduino MEGA as slave
 * Atmega2560
 */

#include <avr/io.h>
#include "alarmstates.h";

int main(void) {
    // SETUP
    // DEFINE VARIABLES

    // SETUP PINS
    
    // INITIALIZE I2C OR SPI

    while(1) {
        // LOOP

        // LISTEN FOR MASTER

        // IF MESSAGE

            // STATE MACHINE TO HANDLE SIGNALS
            switch(message) {
                case ALARM_OFF:
                    break;

                case ALARM_ON:
                    break;

                case ALARM_TRIGGER:
                    break;
                
                case PW_OK:
                    break;

                case PW_TIMEOUT:
                    break;

                case PW_WRONG:
                    break;

                default:
            }

    }
}