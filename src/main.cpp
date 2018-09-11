// #include <Arduino.h>

// void setup() {
//     // put your setup code here, to run once:
// }

// void loop() {
//     // put your main code here, to run repeatedly:
// }

// // alternative to the setup loop paradigm:
// int main ()
//   {
//   init ();  // used for things like millis(), micros(), and delay(), 
//             // This will also affect analogRead(), analogWrite(), and a few other things

//   Serial.begin (115200);
//   Serial.println ("Hello, world");
//   Serial.flush (); // let serial printing finish
//                     // otherwise interrupts will be turned off and the most recent serial prints may not be shown.
//   }  // end of main

//   /**
//  * Copyright (C) PlatformIO <contact@platformio.org>
//  * See LICENSE for details.
//  */

#include <avr/io.h>
#include <util/delay.h>
// https://www.arduino.cc/en/Reference/PortManipulation
// DDR: determines I or O
// PORT: determines HIGH or LOW
// PIN: reads the state of INPUT pins set to input with pinMode(). (Read only)
int main(void)
{
    // make the LED pin an output for PORTB5 (pin 13)
    DDRB = 1 << PORTB5;

    while (1)
    {
        _delay_ms(1000);

        // toggle the LED
        PORTB ^= 1 << PORTB5;
    }

    return 0;
}