# finis

Finis is a demonstration of a discrete [Moore](https://en.wikipedia.org/wiki/Moore_machine)
[finite state machine](https://en.wikipedia.org/wiki/Finite-state_machine) running on a
AVR microcontroller.

## Hardware Layout

For this demonstration I am using a simple layout with an ATmega328P microcontroller
some LEDs, NPN Transistors and a Photoresistor. The ATmega uses an external
Crystal with 16 MHz clock rate. Our application shows the brightness detected by
the photoresistor with different colored LEDs. We need four of them (red, green,
yellow, blue). I use NPN transistors to drive the LEDs. The following schematic
should help you to rebuild my layout.

![finis schematic](https://crapp.github.io/finis/finis_schematic.png)

## State Machine

I have written a Table-Driven State Machine in C. This State Machine has 3 main
states and 3 transitional states. The input is derived from the value read by the ADC.
Connected to the ADC is a photoresistor. The ambient light conditions are divided in
three ranges.

Here is the state machine definition table

| State     | Input Bright | Input Dim | Input Dark |
| --------- | ------------ | --------- | ---------- |
| GREEN     | GREEN        | TO_RED    | TO_YELLOW  |
| RED       | TO_GREEN     | RED       | TO_YELLOW  |
| YELLOW    | TO_GREEN     | TO_RED    | YELLOW     |
| TO_GREEN  | GREEN        | GREEN     | GREEN      |
| TO_RED    | RED          | RED       | RED        |
| TO_YELLOW | YELLOW       | YELLOW    | YELLOW     |

The neat thing about using a table is this can be translated directly into code.
Have a look at the `struct s_type` and the global arrays FSM in `finis.c`

This implementation currently uses `_delay_ms()` to control the FSM tick frequency.
Depending on what your application does it might be better to use a Timer register.

## License

```
finis state machine example for 8bit avr
Copyright © 2016 Christian Rapp
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

3. Neither the name of the organization nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY  EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL yourname BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

