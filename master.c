/* master.c
 *
 * Created: 09/03/2022
 * Author: Samuli, Robert
 * Using Arduino UNO as master
 * ATmega328p
 */

#include <avr/io.h>

int main(void) {
    // SETUP
    // DEFINE VARIABLES

    // SETUP PINS

    // READ PASSWORD FROM EEPROM

    // INITIALIZE I2C OR SPI

    // INITIALIZE INTERRUPT FOR MOTION SENSOR

    while(1) {
        // LOOP

        // READ FROM KEYPAD

            // IF TIMEOUT

                // SEND PW_TIMEOUT TO SLAVE

        // IF SUBMIT

            // COMPARE PASSWORD

                // CORRECT

                    // ALARM OFF

                    // SEND PW_OK TO SLAVE


    }
}